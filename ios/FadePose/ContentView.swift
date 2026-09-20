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

    private let session = ARSession()
    private var connection: NWConnection?
    private var origin: simd_float4x4?
    private var lastSend = Date.distantPast
    private let sendInterval: TimeInterval = 0.02  // 50 Hz
    private var sessionRunning = false
    private var startSent = false

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
        streaming = true
        status = "streaming"
        UIApplication.shared.isIdleTimerDisabled = true
    }

    func stopStreaming() {
        streaming = false
        startSent = false
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
        let dw = t.columns.3 - o.columns.3
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

        // orientation stays relative to Start, so tilt re-zeros when you
        // press Start and the cut angle is measured from however you hold it
        let q = simd_quatf(rel)
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

struct ContentView: View {
    @StateObject private var streamer = PoseStreamer()
    @AppStorage("host") private var host = "172.20.10.2"
    @AppStorage("port") private var portText = "8463"

    var body: some View {
        VStack(spacing: 16) {
            Text("FadePose").font(.largeTitle.bold())
            Text(streamer.status).foregroundStyle(.secondary)
            Text("tracking: \(streamer.trackingLabel)")
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
                streamer.connect(host: host, port: UInt16(portText) ?? 8463)
            }

            Button(streamer.streaming ? "Stop" : "Start") {
                if streamer.streaming { streamer.stopStreaming() }
                else { streamer.startStreaming() }
            }
            .buttonStyle(.borderedProminent)
            .tint(streamer.streaming ? .red : .green)
            .disabled(!streamer.streaming && !streamer.trackingNormal)

            Text("Camera is used only for ARKit tracking. Frames are never sent.")
                .font(.caption)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
        }
        .padding()
        .onAppear { streamer.startSession() }
    }
}
