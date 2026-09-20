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

// MARK: - Sign in

struct SignInView: View {
    @ObservedObject var library: Library
    @Binding var host: String
    @State private var handle = ""

    var body: some View {
        VStack(spacing: 22) {
            Spacer()
            Text("FADE NINJA").font(.system(size: 34, weight: .heavy))
            Text("your cuts, saved and replayable")
                .font(.subheadline).foregroundStyle(.secondary)

            VStack(alignment: .leading, spacing: 8) {
                Text("ROBOT ADDRESS").font(.caption2.bold())
                    .foregroundStyle(.secondary)
                TextField("host", text: $host)
                    .textFieldStyle(.roundedBorder)
                    .autocapitalization(.none)
                    .keyboardType(.decimalPad)
            }

            VStack(alignment: .leading, spacing: 8) {
                Text("YOUR HANDLE").font(.caption2.bold())
                    .foregroundStyle(.secondary)
                TextField("e.g. saswath", text: $handle)
                    .textFieldStyle(.roundedBorder)
                    .autocapitalization(.none)
                    .autocorrectionDisabled()
            }

            Button {
                library.host = host
                library.signIn(handle: handle)
            } label: {
                Text(library.busy ? "Signing in…" : "Sign in")
                    .frame(maxWidth: .infinity).padding(.vertical, 6)
            }
            .buttonStyle(.borderedProminent)
            .disabled(handle.trimmingCharacters(in: .whitespaces).count < 2 || library.busy)

            Text("A handle is all we store — no password. It only decides whose cuts are whose.")
                .font(.caption).foregroundStyle(.secondary)
                .multilineTextAlignment(.center)

            if !library.note.isEmpty {
                Text(library.note).font(.caption).foregroundStyle(.orange)
                    .multilineTextAlignment(.center)
            }
            Spacer()
        }
        .padding(24)
    }
}

// MARK: - Cut library

struct LibraryView: View {
    @ObservedObject var library: Library
    @ObservedObject var streamer: PoseStreamer
    @State private var takeName = ""

    var body: some View {
        VStack(alignment: .leading, spacing: 12) {
            HStack {
                Text("MY CUTS").font(.caption.bold()).foregroundStyle(.secondary)
                Spacer()
                Button("Refresh") { library.refresh() }.font(.caption)
            }

            TextField("name this cut", text: $takeName)
                .textFieldStyle(.roundedBorder)

            HStack(spacing: 10) {
                Button(library.recording ? "Recording…" : "Start recording") {
                    library.startRecording()
                }
                .buttonStyle(.borderedProminent).tint(.red)
                .disabled(library.recording || !streamer.streaming || library.busy)

                Button("Save cut") { library.saveRecording(name: takeName); takeName = "" }
                    .buttonStyle(.bordered)
                    .disabled(!library.recording || library.busy)
            }

            if !streamer.streaming {
                Text("Press Start above before recording.")
                    .font(.caption).foregroundStyle(.secondary)
            }

            if library.cuts.isEmpty {
                Text("No cuts yet. Record one.")
                    .font(.caption).foregroundStyle(.secondary).padding(.top, 4)
            } else {
                ForEach(library.cuts) { cut in
                    HStack {
                        VStack(alignment: .leading, spacing: 2) {
                            Text(cut.name).font(.body.weight(.medium))
                            Text("\(String(format: "%.1f", cut.duration_s))s · \(cut.n_samples) samples")
                                .font(.caption2).foregroundStyle(.secondary)
                        }
                        Spacer()
                        Button("Replay") { library.replay(cut) }
                            .buttonStyle(.bordered).font(.caption)
                            .disabled(library.busy)
                    }
                    .padding(.vertical, 6)
                    Divider()
                }
                Button("Stop replay") { library.stopReplay() }
                    .font(.caption).disabled(library.busy)
            }
        }
    }
}

// MARK: - Main screen

struct ContentView: View {
    @StateObject private var streamer = PoseStreamer()
    @StateObject private var library = Library()
    @AppStorage("host") private var host = "172.20.10.2"
    @AppStorage("port") private var portText = "8463"
    @AppStorage("handle") private var savedHandle = ""
    @AppStorage("userId") private var savedUserId = ""

    var body: some View {
        Group {
            if library.signedIn {
                controls
            } else {
                SignInView(library: library, host: $host)
            }
        }
        .onAppear {
            streamer.startSession()
            library.host = host
            library.httpPort = (UInt16(portText) ?? 8463) &+ 1
            if !savedUserId.isEmpty {          // stay signed in across launches
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

    private var controls: some View {
        ScrollView {
            VStack(spacing: 16) {
                HStack {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("FADE NINJA").font(.headline.bold())
                        Text("@\(library.handle)").font(.caption)
                            .foregroundStyle(.secondary)
                    }
                    Spacer()
                    Button("Sign out") {
                        library.signOut(); savedUserId = ""; savedHandle = ""
                    }.font(.caption)
                }

                Text(streamer.status).font(.caption).foregroundStyle(.secondary)
                Text("tracking: \(streamer.trackingLabel)")
                    .font(.caption)
                    .foregroundStyle(streamer.trackingNormal ? .green : .orange)

                HStack {
                    TextField("host", text: $host)
                        .textFieldStyle(.roundedBorder)
                        .autocapitalization(.none)
                    TextField("port", text: $portText)
                        .textFieldStyle(.roundedBorder)
                        .frame(width: 80)
                }

                Button("Connect UDP") {
                    library.host = host
                    library.httpPort = (UInt16(portText) ?? 8463) &+ 1
                    streamer.connect(host: host, port: UInt16(portText) ?? 8463)
                }

                Text(streamer.tiltOnly ? "POSITION LOCKED — tilt only"
                                       : "HOLD TO LOCK POSITION")
                    .font(.system(size: 15, weight: .bold))
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 22)
                    .background(streamer.tiltOnly ? Color.orange.opacity(0.85)
                                                  : Color.gray.opacity(0.22))
                    .foregroundColor(streamer.tiltOnly ? .black : .primary)
                    .cornerRadius(14)
                    .gesture(
                        DragGesture(minimumDistance: 0)
                            .onChanged { _ in if streamer.streaming { streamer.tiltOnly = true } }
                            .onEnded { _ in streamer.tiltOnly = false }
                    )
                    .opacity(streamer.streaming ? 1 : 0.45)

                Button(streamer.streaming ? "Stop" : "Start") {
                    if streamer.streaming { streamer.stopStreaming() }
                    else { streamer.startStreaming() }
                }
                .buttonStyle(.borderedProminent)
                .tint(streamer.streaming ? .red : .green)
                .disabled(!streamer.streaming && !streamer.trackingNormal)

                Divider()
                LibraryView(library: library, streamer: streamer)

                if !library.note.isEmpty {
                    Text(library.note).font(.caption).foregroundStyle(.orange)
                        .multilineTextAlignment(.center)
                }
                Text("Camera is used only for ARKit tracking. Frames are never sent.")
                    .font(.caption2).foregroundStyle(.secondary)
                    .multilineTextAlignment(.center)
            }
            .padding()
        }
    }
}
