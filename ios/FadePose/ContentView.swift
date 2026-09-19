// FadePose — minimal ARKit pose streamer (PHONE_POSE_TDD Phase 6).
// Create an Xcode iOS App (SwiftUI), replace ContentView with this file,
// delete the generated *App.swift (this file declares @main), and add
// Info keys from Info-keys.plist.txt.
//
// Sends UDP datagrams to pose_server (:8463):
//   START
//   POSE <t_ms> <x> <y> <z> <qw> <qx> <qy> <qz> <track>
//   STOP
// Camera frames are never sent — ARKit uses them internally for VIO only.

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

    private let session = ARSession()
    private var connection: NWConnection?
    private var origin: simd_float4x4?
    private var lastSend = Date.distantPast
    private let sendInterval: TimeInterval = 0.02  // 50 Hz

    override init() {
        super.init()
        session.delegate = self
    }

    func connect(host: String, port: UInt16) {
        connection?.cancel()
        let nwPort = NWEndpoint.Port(rawValue: port)!
        let conn = NWConnection(host: NWEndpoint.Host(host), port: nwPort, using: .udp)
        connection = conn
        conn.start(queue: .main)
        status = "connected \(host):\(port)"
    }

    func startStreaming() {
        guard let state = session.currentFrame?.camera.trackingState,
              case .normal = state else {
            status = "tracking not normal — wait"
            return
        }
        let config = ARWorldTrackingConfiguration()
        config.worldAlignment = .gravity
        session.run(config, options: [.resetTracking, .removeExistingAnchors])
        origin = nil
        sendLine("START")
        streaming = true
        status = "streaming"
        UIApplication.shared.isIdleTimerDisabled = true
    }

    func stopStreaming() {
        streaming = false
        sendLine("STOP")
        session.pause()
        origin = nil
        status = "stopped"
        UIApplication.shared.isIdleTimerDisabled = false
    }

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        guard streaming else { return }
        let now = Date()
        guard now.timeIntervalSince(lastSend) >= sendInterval else { return }
        lastSend = now

        let track: TrackCode
        switch frame.camera.trackingState {
        case .normal: track = .normal
        case .limited: track = .limited
        case .notAvailable: track = .limited
        @unknown default: track = .limited
        }
        trackingLabel = "\(track)"

        if track != .normal { return }

        let t = frame.camera.transform
        if origin == nil {
            origin = t
        }
        let o = origin!
        // Relative translation in ARKit world (metres)
        let rel = simd_mul(simd_inverse(o), t)
        let x = rel.columns.3.x
        let y = rel.columns.3.y
        let z = rel.columns.3.z
        let q = simd_quatf(rel)
        let tMs = Int(frame.timestamp * 1000) % 1_000_000_000
        let line = String(
            format: "POSE %d %.5f %.5f %.5f %.5f %.5f %.5f %.5f %d",
            tMs, x, y, z, q.vector.w, q.vector.x, q.vector.y, q.vector.z, track.rawValue
        )
        sendLine(line)
    }

    private func sendLine(_ line: String) {
        guard let connection, let data = (line + "\n").data(using: .utf8) else { return }
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

            Text("Camera is used only for ARKit tracking. Frames are never sent.")
                .font(.caption)
                .foregroundStyle(.secondary)
                .multilineTextAlignment(.center)
        }
        .padding()
    }
}
