// FadePose — minimal ARKit pose streamer (PHONE_POSE_TDD Fix pass).
// Create via ios/FadePose/project.yml (XcodeGen) or by hand — see README.
//
// Wire (UDP :8463):
//   START <track>
//   POSE <t_ms> <x> <y> <z> <qw> <qx> <qy> <qz> <track>
//   STOP
// Camera frames are never sent — ARKit uses them only for VIO.

import ARKit
import Network
import SwiftUI
import UIKit

enum TrackCode: Int {
    case normal = 0
    case limited = 1
    case relocalizing = 2
}

final class PoseStreamer: NSObject, ObservableObject, ARSessionDelegate {
    @Published var status = "idle"
    @Published var streaming = false
    @Published var trackingLabel = "—"
    @Published var trackingNormal = false
    /// While held, the arm holds its position and only the cut angle moves.
    @Published var tiltOnly = false

    private let session = ARSession()
    private var connection: NWConnection?
    private var origin: simd_float4x4?
    private var lastSend = Date.distantPast
    private let sendInterval: TimeInterval = 0.02  // 50 Hz
    private var sessionRunning = false
    private var startSent = false
    /// Position delta frozen while tiltOnly is held, so the arm parks.
    private var frozenDelta: SIMD3<Float>?
    /// Offset that keeps position continuous after releasing tiltOnly —
    /// without it, letting go would snap the arm to wherever the hand
    /// drifted while locked.
    private var posOffset = SIMD3<Float>(repeating: 0)

    override init() {
        super.init()
        session.delegate = self
        session.delegateQueue = .main
    }

    /// Begin ARKit warmup as soon as the UI appears (independent of UDP).
    func startSession() {
        guard !sessionRunning else { return }
        guard ARWorldTrackingConfiguration.isSupported else {
            status = "ARKit world tracking not supported"
            return
        }
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        sessionRunning = true
        status = "AR warming up — wait for tracking: normal"
    }

    func connect(host: String, port: UInt16) {
        connection?.cancel()
        let nwPort = NWEndpoint.Port(rawValue: port)!
        let conn = NWConnection(host: NWEndpoint.Host(host), port: nwPort, using: .udp)
        connection = conn
        conn.start(queue: .main)
        status = "UDP target \(host):\(port) (connectionless)"
    }

    func startStreaming() {
        guard trackingNormal else {
            status = "tracking not normal — wait"
            return
        }
        guard connection != nil else {
            status = "set host/port and Connect UDP first"
            return
        }
        origin = nil
        startSent = false
        frozenDelta = nil
        posOffset = SIMD3<Float>(repeating: 0)
        streaming = true
        status = "streaming"
        UIApplication.shared.isIdleTimerDisabled = true
    }

    func stopStreaming() {
        streaming = false
        startSent = false
        frozenDelta = nil
        posOffset = SIMD3<Float>(repeating: 0)
        sendLine("STOP")
        origin = nil
        status = "stopped"
        UIApplication.shared.isIdleTimerDisabled = false
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let track = Self.trackCode(for: frame.camera.trackingState)
        trackingLabel = trackLabel(track)
        trackingNormal = (track == .normal)

        // Always update tracking during warmup — do not gate on streaming.
        guard streaming else { return }

        let now = Date()
        guard now.timeIntervalSince(lastSend) >= sendInterval else { return }
        lastSend = now

        if track != .normal {
            return
        }

        let t = frame.camera.transform
        if !startSent {
            // START and origin capture on the first normal frame after Start.
            sendLine("START \(TrackCode.normal.rawValue)")
            origin = t
            startSent = true
        }
        guard let o = origin else { return }

        let rel = simd_mul(simd_inverse(o), t)

        // Position must be sent in a GRAVITY-ALIGNED frame. inverse(o)*t
        // expresses movement in the origin CAMERA's own axes, so holding the
        // phone anything but upright-and-forward permutes up/right/forward —
        // moving up would yaw the arm, moving right would raise it.
        // ARKit's world +Y is always up, so take the world delta and remove
        // only the origin's heading, which keeps "forward" meaning the way
        // you were facing when you pressed Start.
        var dw = t.columns.3 - o.columns.3
        let dw3 = SIMD3<Float>(dw.x, dw.y, dw.z) - posOffset
        if tiltOnly {
            // hold the arm where it is; only the wrist keeps tracking
            if frozenDelta == nil { frozenDelta = dw3 }
            let f = frozenDelta!
            dw = SIMD4<Float>(f.x, f.y, f.z, dw.w)
        } else {
            if let f = frozenDelta {
                // released: re-anchor so the arm resumes from where it sits
                posOffset = dw3 - f
                frozenDelta = nil
            }
            let d = SIMD3<Float>(dw.x, dw.y, dw.z) - posOffset
            dw = SIMD4<Float>(d.x, d.y, d.z, dw.w)
        }
        var fwd = SIMD3<Float>(-o.columns.2.x, -o.columns.2.y, -o.columns.2.z)
        if hypotf(fwd.x, fwd.z) < 0.15 {
            // camera is pointing straight up or down (phone lying flat), so
            // it has no usable heading: use the phone's top edge instead
            fwd = SIMD3<Float>(o.columns.1.x, o.columns.1.y, o.columns.1.z)
        }
        let heading = atan2f(fwd.x, -fwd.z)
        let ca = cosf(heading), sa = sinf(heading)
        let x = dw.x * ca + dw.z * sa
        let y = dw.y
        let z = -dw.x * sa + dw.z * ca

        // The cut angle must be gravity-referenced too. simd_quatf(rel)
        // measures rotation in the origin CAMERA's frame, so with the phone
        // tilted — a normal clipper grip — turning it horizontally leaks into
        // pitch: at a 40 deg grip a 90 deg turn swings the cut angle 30 deg,
        // most of its range. Send the elevation of the phone's forward axis
        // as a pure pitch rotation instead; yaw about gravity cannot change
        // it. The server still differences this against Start, so tilt
        // re-zeros there, and pure pitch rotations difference cleanly.
        let fwdNow = SIMD3<Float>(-t.columns.2.x, -t.columns.2.y, -t.columns.2.z)
        let elevation = asinf(max(-1.0, min(1.0, fwdNow.y)))
        let q = simd_quatf(angle: elevation, axis: SIMD3<Float>(1, 0, 0))
        _ = rel
        let tMs = Int(frame.timestamp * 1000) % 1_000_000_000
        let line = String(
            format: "POSE %d %.5f %.5f %.5f %.5f %.5f %.5f %.5f %d",
            tMs, x, y, z, q.vector.w, q.vector.x, q.vector.y, q.vector.z, track.rawValue
        )
        sendLine(line)
    }

    private static func trackCode(for state: ARCamera.TrackingState) -> TrackCode {
        switch state {
        case .normal:
            return .normal
        case .limited(let reason):
            if case .relocalizing = reason { return .relocalizing }
            return .limited
        case .notAvailable:
            return .limited
        @unknown default:
            return .limited
        }
    }

    private func trackLabel(_ track: TrackCode) -> String {
        switch track {
        case .normal: return "normal"
        case .limited: return "limited"
        case .relocalizing: return "relocalizing"
        }
    }

    private func sendLine(_ line: String) {
        guard let connection, let data = line.data(using: .utf8) else { return }
        connection.send(content: data, completion: .contentProcessed { _ in })
    }
}

@main
struct FadePoseApp: App {
    var body: some Scene {
        WindowGroup { ContentView() }
    }
}


// MARK: - Cut library API

/// One saved cut, as the server returns it.
struct Cut: Identifiable, Decodable, Equatable {
    let id: String
    let name: String
    let duration_s: Double
    let n_samples: Int
}

private struct SignInReply: Decodable {
    struct U: Decodable { let id: String; let handle: String }
    let user: U
    let takes: [Cut]
}
private struct TakesReply: Decodable { let takes: [Cut] }
private struct SaveReply: Decodable { let take: Cut; let takes: [Cut] }
private struct ApiErrorBody: Decodable { let error: String }

/// Talks to pose_server's dashboard API over HTTP. The pose stream stays on
/// UDP; this is the slow path — signing in, listing and replaying cuts.
@MainActor
final class Library: ObservableObject {
    @Published var handle = ""
    @Published var userId = ""
    @Published var cuts: [Cut] = []
    @Published var busy = false
    @Published var note = ""
    @Published var recording = false

    /// Host is shared with the UDP stream; the API sits on the next port up,
    /// matching pose_server's defaults (8463 UDP, 8464 HTTP).
    var host = "172.20.10.2"
    var httpPort: UInt16 = 8464

    var signedIn: Bool { !userId.isEmpty }

    private func post<T: Decodable>(_ path: String, _ body: [String: Any],
                                    as type: T.Type) async throws -> T {
        var payload = body
        payload["user_id"] = userId
        var req = URLRequest(url: URL(string: "http://\(host):\(httpPort)\(path)")!)
        req.httpMethod = "POST"
        req.setValue("application/json", forHTTPHeaderField: "Content-Type")
        req.httpBody = try JSONSerialization.data(withJSONObject: payload)
        req.timeoutInterval = 8
        let (data, resp) = try await URLSession.shared.data(for: req)
        let code = (resp as? HTTPURLResponse)?.statusCode ?? 0
        if !(200..<300).contains(code) {
            let msg = (try? JSONDecoder().decode(ApiErrorBody.self, from: data))?.error
            throw NSError(domain: "FadeNinja", code: code,
                          userInfo: [NSLocalizedDescriptionKey: msg ?? "HTTP \(code)"])
        }
        return try JSONDecoder().decode(T.self, from: data)
    }

    private func run(_ label: String, _ work: @escaping () async throws -> Void) {
        busy = true
        Task {
            do { try await work() }
            catch { note = "\(label): \(error.localizedDescription)" }
            busy = false
        }
    }

    func signIn(handle h: String) {
        run("sign in") {
            let r = try await self.post("/api/signin", ["handle": h], as: SignInReply.self)
            self.userId = r.user.id
            self.handle = r.user.handle
            self.cuts = r.takes
            self.note = "signed in as \(r.user.handle)"
        }
    }

    func signOut() {
        userId = ""; handle = ""; cuts = []; recording = false; note = ""
    }

    func refresh() {
        guard signedIn else { return }
        run("load cuts") {
            self.cuts = try await self.post("/api/takes", [:], as: TakesReply.self).takes
        }
    }

    func startRecording() {
        run("record") {
            _ = try await self.post("/api/record/start", [:], as: [String: Bool].self)
            self.recording = true
            self.note = "recording — move the phone"
        }
    }

    func saveRecording(name: String) {
        run("save") {
            let r = try await self.post("/api/record/stop", ["name": name], as: SaveReply.self)
            self.cuts = r.takes
            self.recording = false
            self.note = "saved \(r.take.name) (\(String(format: "%.1f", r.take.duration_s))s)"
        }
    }

    func replay(_ cut: Cut) {
        run("replay") {
            _ = try await self.post("/api/replay", ["take_id": cut.id],
                                    as: [String: AnyCodableStub].self)
            self.note = "replaying \(cut.name)"
        }
    }

    func stopReplay() {
        run("stop") {
            _ = try await self.post("/api/replay/stop", [:], as: [String: AnyCodableStub?].self)
            self.note = "replay stopped"
        }
    }
}

/// Minimal stand-in so replies we do not read still decode.
struct AnyCodableStub: Decodable {
    init(from decoder: Decoder) throws { _ = try? decoder.singleValueContainer() }
}

// MARK: - Look

/// One place for the palette so every screen agrees: a dark shop interior,
/// barber red for anything that acts, steel blue for anything that measures.
enum Ink {
    static let bg        = Color(red: 0.055, green: 0.060, blue: 0.075)
    static let bgLift    = Color(red: 0.094, green: 0.102, blue: 0.125)
    static let card      = Color(red: 0.118, green: 0.129, blue: 0.157)
    static let stroke    = Color.white.opacity(0.08)
    static let text      = Color(red: 0.960, green: 0.945, blue: 0.918)
    static let dim       = Color(red: 0.569, green: 0.596, blue: 0.639)
    static let red       = Color(red: 0.902, green: 0.239, blue: 0.298)
    static let blue      = Color(red: 0.290, green: 0.620, blue: 1.000)
    static let green     = Color(red: 0.247, green: 0.784, blue: 0.443)
    static let amber     = Color(red: 0.965, green: 0.694, blue: 0.180)

    static var backdrop: some View {
        LinearGradient(colors: [bgLift, bg], startPoint: .top, endPoint: .bottom)
            .ignoresSafeArea()
    }
}

private func tap(_ style: UIImpactFeedbackGenerator.FeedbackStyle = .medium) {
    UIImpactFeedbackGenerator(style: style).impactOccurred()
}

/// A card. Everything that groups content uses this, so the app has one
/// surface treatment rather than five.
struct Panel<Content: View>: View {
    var title: String? = nil
    @ViewBuilder var content: Content

    var body: some View {
        VStack(alignment: .leading, spacing: 14) {
            if let title {
                Text(title.uppercased())
                    .font(.system(size: 11, weight: .bold, design: .rounded))
                    .tracking(1.4)
                    .foregroundStyle(Ink.dim)
            }
            content
        }
        .padding(18)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Ink.card, in: RoundedRectangle(cornerRadius: 20, style: .continuous))
        .overlay(RoundedRectangle(cornerRadius: 20, style: .continuous)
            .stroke(Ink.stroke, lineWidth: 1))
    }
}

struct StatusDot: View {
    let label: String
    let on: Bool
    var color: Color = Ink.green

    var body: some View {
        HStack(spacing: 6) {
            Circle().fill(on ? color : Ink.dim.opacity(0.45))
                .frame(width: 7, height: 7)
                .shadow(color: on ? color.opacity(0.8) : .clear, radius: 4)
            Text(label)
                .font(.system(size: 11, weight: .semibold, design: .rounded))
                .foregroundStyle(on ? Ink.text : Ink.dim)
        }
        .padding(.horizontal, 10).padding(.vertical, 6)
        .background(Capsule().fill(Color.white.opacity(0.05)))
    }
}

// MARK: - Sign in

struct SignInView: View {
    @ObservedObject var library: Library
    @Binding var host: String
    @State private var handle = ""
    @FocusState private var focused: Bool

    var body: some View {
        ZStack {
            Ink.backdrop
            ScrollView {
                VStack(spacing: 26) {
                    Spacer(minLength: 50)

                    ZStack {
                        Circle()
                            .fill(RadialGradient(colors: [Ink.red.opacity(0.35), .clear],
                                                 center: .center, startRadius: 4, endRadius: 96))
                            .frame(width: 190, height: 190)
                        Image(systemName: "scissors")
                            .font(.system(size: 62, weight: .semibold))
                            .foregroundStyle(Ink.red)
                            .rotationEffect(.degrees(-20))
                    }

                    VStack(spacing: 7) {
                        Text("FADE NINJA")
                            .font(.system(size: 36, weight: .heavy, design: .rounded))
                            .tracking(1.5)
                            .foregroundStyle(Ink.text)
                        Text("the cut you can't reach")
                            .font(.system(size: 15, weight: .medium, design: .rounded))
                            .foregroundStyle(Ink.dim)
                    }

                    VStack(spacing: 14) {
                        field("Robot address", text: $host, icon: "wifi",
                              keyboard: .decimalPad)
                        field("Your handle", text: $handle, icon: "person.fill")
                            .focused($focused)
                    }

                    Button {
                        tap()
                        library.host = host
                        library.signIn(handle: handle)
                    } label: {
                        HStack(spacing: 8) {
                            if library.busy { ProgressView().tint(.white) }
                            Text(library.busy ? "Signing in" : "Enter the shop")
                                .font(.system(size: 17, weight: .bold, design: .rounded))
                        }
                        .frame(maxWidth: .infinity).padding(.vertical, 16)
                        .background(
                            LinearGradient(colors: [Ink.red, Ink.red.opacity(0.75)],
                                           startPoint: .top, endPoint: .bottom),
                            in: RoundedRectangle(cornerRadius: 16, style: .continuous))
                        .foregroundStyle(.white)
                        .shadow(color: Ink.red.opacity(0.4), radius: 14, y: 6)
                    }
                    .disabled(handle.trimmingCharacters(in: .whitespaces).count < 2
                              || library.busy)
                    .opacity(handle.trimmingCharacters(in: .whitespaces).count < 2 ? 0.45 : 1)

                    Text("Just a handle — no password. It only decides whose cuts are whose.")
                        .font(.system(size: 12, design: .rounded))
                        .foregroundStyle(Ink.dim)
                        .multilineTextAlignment(.center)
                        .padding(.horizontal, 20)

                    if !library.note.isEmpty {
                        Text(library.note)
                            .font(.system(size: 12, weight: .medium, design: .rounded))
                            .foregroundStyle(Ink.amber)
                            .multilineTextAlignment(.center)
                    }
                    Spacer(minLength: 30)
                }
                .padding(.horizontal, 24)
            }
        }
        .onTapGesture { focused = false }
    }

    private func field(_ label: String, text: Binding<String>, icon: String,
                       keyboard: UIKeyboardType = .default) -> some View {
        HStack(spacing: 12) {
            Image(systemName: icon)
                .font(.system(size: 15))
                .foregroundStyle(Ink.dim)
                .frame(width: 20)
            VStack(alignment: .leading, spacing: 2) {
                Text(label.uppercased())
                    .font(.system(size: 9.5, weight: .bold, design: .rounded))
                    .tracking(1.2)
                    .foregroundStyle(Ink.dim)
                TextField("", text: text)
                    .font(.system(size: 17, weight: .medium, design: .rounded))
                    .foregroundStyle(Ink.text)
                    .autocapitalization(.none)
                    .autocorrectionDisabled()
                    .keyboardType(keyboard)
            }
        }
        .padding(.horizontal, 16).padding(.vertical, 12)
        .background(Ink.card, in: RoundedRectangle(cornerRadius: 14, style: .continuous))
        .overlay(RoundedRectangle(cornerRadius: 14, style: .continuous)
            .stroke(Ink.stroke, lineWidth: 1))
    }
}

// MARK: - Remote

struct RemoteView: View {
    @ObservedObject var streamer: PoseStreamer
    @ObservedObject var library: Library
    @Binding var host: String
    @Binding var portText: String
    @State private var takeName = ""
    @State private var elapsed = 0
    private let tick = Timer.publish(every: 1, on: .main, in: .common).autoconnect()

    private var canDrive: Bool { streamer.streaming }

    var body: some View {
        ScrollView {
            VStack(spacing: 16) {
                connection
                driveButton
                lockButton
                recording
                if !library.note.isEmpty {
                    Text(library.note)
                        .font(.system(size: 12, weight: .medium, design: .rounded))
                        .foregroundStyle(Ink.amber)
                        .multilineTextAlignment(.center)
                        .padding(.horizontal, 8)
                }
                Text("The camera is used only to track where the phone is. Frames never leave the device.")
                    .font(.system(size: 11, design: .rounded))
                    .foregroundStyle(Ink.dim.opacity(0.8))
                    .multilineTextAlignment(.center)
                    .padding(.horizontal, 16).padding(.top, 4)
            }
            .padding(16)
        }
        .onReceive(tick) { _ in if library.recording { elapsed += 1 } }
        .onChange(of: library.recording) { rec in if rec { elapsed = 0 } }
    }

    // ---- connection
    private var connection: some View {
        Panel(title: "Connection") {
            HStack(spacing: 8) {
                StatusDot(label: "LINKED", on: streamer.streaming, color: Ink.green)
                StatusDot(label: streamer.trackingNormal ? "TRACKING" : "WARMING UP",
                          on: streamer.trackingNormal, color: Ink.blue)
                Spacer()
            }
            HStack(spacing: 10) {
                TextField("host", text: $host)
                    .font(.system(size: 15, weight: .medium, design: .monospaced))
                    .foregroundStyle(Ink.text)
                    .autocapitalization(.none)
                    .keyboardType(.decimalPad)
                TextField("port", text: $portText)
                    .font(.system(size: 15, weight: .medium, design: .monospaced))
                    .foregroundStyle(Ink.text)
                    .keyboardType(.numberPad)
                    .frame(width: 66)
                Button {
                    tap(.light)
                    library.host = host
                    library.httpPort = (UInt16(portText) ?? 8463) &+ 1
                    streamer.connect(host: host, port: UInt16(portText) ?? 8463)
                } label: {
                    Image(systemName: "arrow.triangle.2.circlepath")
                        .font(.system(size: 15, weight: .bold))
                        .foregroundStyle(Ink.blue)
                        .padding(9)
                        .background(Ink.blue.opacity(0.15), in: Circle())
                }
            }
            .padding(.horizontal, 12).padding(.vertical, 10)
            .background(Ink.bg, in: RoundedRectangle(cornerRadius: 12, style: .continuous))

            Text(streamer.status)
                .font(.system(size: 11, design: .rounded))
                .foregroundStyle(Ink.dim)
        }
    }

    // ---- the primary action
    private var driveButton: some View {
        Button {
            tap(.heavy)
            if streamer.streaming { streamer.stopStreaming() }
            else { streamer.startStreaming() }
        } label: {
            HStack(spacing: 12) {
                Image(systemName: streamer.streaming ? "stop.fill" : "play.fill")
                    .font(.system(size: 20, weight: .bold))
                Text(streamer.streaming ? "Stop driving" : "Start driving")
                    .font(.system(size: 19, weight: .bold, design: .rounded))
            }
            .frame(maxWidth: .infinity).padding(.vertical, 20)
            .background(
                LinearGradient(colors: streamer.streaming
                               ? [Ink.red, Ink.red.opacity(0.72)]
                               : [Ink.green, Ink.green.opacity(0.72)],
                               startPoint: .top, endPoint: .bottom),
                in: RoundedRectangle(cornerRadius: 20, style: .continuous))
            .foregroundStyle(.white)
            .shadow(color: (streamer.streaming ? Ink.red : Ink.green).opacity(0.35),
                    radius: 16, y: 7)
        }
        .disabled(!streamer.streaming && !streamer.trackingNormal)
        .opacity(!streamer.streaming && !streamer.trackingNormal ? 0.4 : 1)
    }

    // ---- hold to lock
    private var lockButton: some View {
        VStack(spacing: 8) {
            ZStack {
                RoundedRectangle(cornerRadius: 22, style: .continuous)
                    .fill(streamer.tiltOnly ? Ink.amber : Ink.card)
                RoundedRectangle(cornerRadius: 22, style: .continuous)
                    .stroke(streamer.tiltOnly ? Ink.amber : Ink.stroke, lineWidth: 1)
                VStack(spacing: 7) {
                    Image(systemName: streamer.tiltOnly ? "lock.fill" : "lock.open")
                        .font(.system(size: 26, weight: .semibold))
                    Text(streamer.tiltOnly ? "POSITION LOCKED" : "HOLD TO LOCK POSITION")
                        .font(.system(size: 14, weight: .bold, design: .rounded))
                        .tracking(0.6)
                }
                .foregroundStyle(streamer.tiltOnly ? Color.black : Ink.text)
            }
            .frame(height: 118)
            .scaleEffect(streamer.tiltOnly ? 0.98 : 1)
            .animation(.easeOut(duration: 0.12), value: streamer.tiltOnly)
            .gesture(DragGesture(minimumDistance: 0)
                .onChanged { _ in
                    if canDrive && !streamer.tiltOnly { tap(.rigid); streamer.tiltOnly = true }
                }
                .onEnded { _ in streamer.tiltOnly = false })
            .opacity(canDrive ? 1 : 0.4)

            Text("Parks the arm so you can reposition your hand — only the blade angle keeps moving.")
                .font(.system(size: 11, design: .rounded))
                .foregroundStyle(Ink.dim)
                .multilineTextAlignment(.center)
        }
    }

    // ---- recording
    private var recording: some View {
        Panel(title: "Record a cut") {
            if library.recording {
                HStack(spacing: 10) {
                    Circle().fill(Ink.red).frame(width: 11, height: 11)
                        .opacity(elapsed % 2 == 0 ? 1 : 0.28)
                        .animation(.easeInOut(duration: 0.5), value: elapsed)
                    Text(String(format: "%01d:%02d", elapsed / 60, elapsed % 60))
                        .font(.system(size: 30, weight: .bold, design: .monospaced))
                        .foregroundStyle(Ink.text)
                    Spacer()
                    Text("REC").font(.system(size: 12, weight: .heavy, design: .rounded))
                        .foregroundStyle(Ink.red)
                }
            }

            TextField("", text: $takeName, prompt: Text("Name this cut")
                .foregroundColor(Ink.dim))
                .font(.system(size: 16, weight: .medium, design: .rounded))
                .foregroundStyle(Ink.text)
                .padding(.horizontal, 14).padding(.vertical, 12)
                .background(Ink.bg, in: RoundedRectangle(cornerRadius: 12, style: .continuous))

            HStack(spacing: 10) {
                Button {
                    tap(); library.startRecording()
                } label: {
                    Label("Record", systemImage: "record.circle")
                        .font(.system(size: 15, weight: .bold, design: .rounded))
                        .frame(maxWidth: .infinity).padding(.vertical, 14)
                        .background(Ink.red.opacity(library.recording ? 0.25 : 1),
                                    in: RoundedRectangle(cornerRadius: 14, style: .continuous))
                        .foregroundStyle(library.recording ? Ink.dim : .white)
                }
                .disabled(library.recording || !canDrive || library.busy)

                Button {
                    tap(); library.saveRecording(name: takeName); takeName = ""
                } label: {
                    Label("Save", systemImage: "square.and.arrow.down")
                        .font(.system(size: 15, weight: .bold, design: .rounded))
                        .frame(maxWidth: .infinity).padding(.vertical, 14)
                        .background(library.recording ? Ink.blue : Ink.card,
                                    in: RoundedRectangle(cornerRadius: 14, style: .continuous))
                        .foregroundStyle(library.recording ? .white : Ink.dim)
                }
                .disabled(!library.recording || library.busy)
            }

            if !canDrive {
                Text("Press Start driving first.")
                    .font(.system(size: 11, design: .rounded))
                    .foregroundStyle(Ink.dim)
            }
        }
    }
}

// MARK: - Replay

struct ReplayView: View {
    @ObservedObject var library: Library
    @State private var playingId: String? = nil

    var body: some View {
        ScrollView {
            VStack(spacing: 14) {
                if library.cuts.isEmpty {
                    empty
                } else {
                    ForEach(library.cuts) { cut in card(cut) }
                    Button {
                        tap(.light); playingId = nil; library.stopReplay()
                    } label: {
                        Label("Stop replay", systemImage: "stop.circle")
                            .font(.system(size: 15, weight: .semibold, design: .rounded))
                            .frame(maxWidth: .infinity).padding(.vertical, 14)
                            .background(Ink.card, in: RoundedRectangle(cornerRadius: 14,
                                                                       style: .continuous))
                            .foregroundStyle(Ink.text)
                    }
                    .disabled(library.busy)
                }
            }
            .padding(16)
        }
        .refreshable { library.refresh() }
    }

    private var empty: some View {
        VStack(spacing: 14) {
            Image(systemName: "waveform.path")
                .font(.system(size: 46, weight: .light))
                .foregroundStyle(Ink.dim.opacity(0.6))
            Text("No cuts yet")
                .font(.system(size: 19, weight: .bold, design: .rounded))
                .foregroundStyle(Ink.text)
            Text("Record one on the Remote tab and it shows up here, ready to run again.")
                .font(.system(size: 13, design: .rounded))
                .foregroundStyle(Ink.dim)
                .multilineTextAlignment(.center)
                .padding(.horizontal, 30)
        }
        .padding(.vertical, 70)
    }

    private func card(_ cut: Cut) -> some View {
        let playing = playingId == cut.id
        return HStack(spacing: 14) {
            ZStack {
                RoundedRectangle(cornerRadius: 14, style: .continuous)
                    .fill(playing ? Ink.blue.opacity(0.22) : Color.white.opacity(0.05))
                Image(systemName: playing ? "waveform" : "scissors")
                    .font(.system(size: 20, weight: .semibold))
                    .foregroundStyle(playing ? Ink.blue : Ink.dim)
            }
            .frame(width: 50, height: 50)

            VStack(alignment: .leading, spacing: 3) {
                Text(cut.name)
                    .font(.system(size: 17, weight: .semibold, design: .rounded))
                    .foregroundStyle(Ink.text)
                    .lineLimit(1)
                Text(String(format: "%.1fs · %d samples", cut.duration_s, cut.n_samples))
                    .font(.system(size: 12, weight: .medium, design: .monospaced))
                    .foregroundStyle(Ink.dim)
            }
            Spacer(minLength: 6)

            Button {
                tap(); playingId = cut.id; library.replay(cut)
            } label: {
                Image(systemName: "play.fill")
                    .font(.system(size: 15, weight: .bold))
                    .foregroundStyle(.white)
                    .frame(width: 42, height: 42)
                    .background(
                        LinearGradient(colors: [Ink.blue, Ink.blue.opacity(0.7)],
                                       startPoint: .top, endPoint: .bottom),
                        in: Circle())
                    .shadow(color: Ink.blue.opacity(0.4), radius: 8, y: 3)
            }
            .disabled(library.busy)
        }
        .padding(14)
        .background(Ink.card, in: RoundedRectangle(cornerRadius: 20, style: .continuous))
        .overlay(RoundedRectangle(cornerRadius: 20, style: .continuous)
            .stroke(playing ? Ink.blue.opacity(0.55) : Ink.stroke, lineWidth: 1))
    }
}

// MARK: - Shell

struct ContentView: View {
    @StateObject private var streamer = PoseStreamer()
    @StateObject private var library = Library()
    @AppStorage("host") private var host = "172.20.10.2"
    @AppStorage("port") private var portText = "8463"
    @AppStorage("handle") private var savedHandle = ""
    @AppStorage("userId") private var savedUserId = ""
    @State private var tab = 0

    var body: some View {
        Group {
            if library.signedIn { shell } else {
                SignInView(library: library, host: $host)
            }
        }
        .preferredColorScheme(.dark)
        .onAppear {
            streamer.startSession()
            library.host = host
            library.httpPort = (UInt16(portText) ?? 8463) &+ 1
            if !savedUserId.isEmpty {
                library.userId = savedUserId
                library.handle = savedHandle
                library.refresh()
            }
        }
        // single-parameter form: the two-parameter onChange is iOS 17+
        .onChange(of: library.userId) { id in
            savedUserId = id
            savedHandle = library.handle
        }
    }

    private var shell: some View {
        ZStack {
            Ink.backdrop
            VStack(spacing: 0) {
                header
                TabView(selection: $tab) {
                    RemoteView(streamer: streamer, library: library,
                               host: $host, portText: $portText)
                        .tabItem { Label("Remote", systemImage: "dot.radiowaves.left.and.right") }
                        .tag(0)
                    ReplayView(library: library)
                        .tabItem { Label("My cuts", systemImage: "list.bullet.rectangle") }
                        .tag(1)
                }
                .tint(Ink.red)
            }
        }
    }

    private var header: some View {
        HStack(spacing: 12) {
            Image(systemName: "scissors")
                .font(.system(size: 17, weight: .bold))
                .foregroundStyle(Ink.red)
                .rotationEffect(.degrees(-20))
            VStack(alignment: .leading, spacing: 1) {
                Text("FADE NINJA")
                    .font(.system(size: 15, weight: .heavy, design: .rounded))
                    .tracking(1.1)
                    .foregroundStyle(Ink.text)
                Text("@\(library.handle)")
                    .font(.system(size: 11, weight: .medium, design: .rounded))
                    .foregroundStyle(Ink.dim)
            }
            Spacer()
            Menu {
                Button("Refresh cuts") { library.refresh() }
                Button("Sign out", role: .destructive) {
                    library.signOut(); savedUserId = ""; savedHandle = ""
                }
            } label: {
                Image(systemName: "ellipsis.circle")
                    .font(.system(size: 19))
                    .foregroundStyle(Ink.dim)
            }
        }
        .padding(.horizontal, 18).padding(.top, 6).padding(.bottom, 12)
    }
}
