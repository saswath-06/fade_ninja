//  Fade Pendant — the iPhone as a teach glove.
//
//  The phone is a CONTROLLER, not a sensor. It streams its orientation over
//  UDP at 50 Hz; the laptop turns that into rate commands for the arm and
//  records where the ARM actually went. So this app's accuracy does not
//  matter much — drift is absorbed by rate control and the barber's eye.
//
//  No camera, no ARKit: gravity-referenced pitch and roll from CoreMotion
//  are drift-free and work in any lighting.
//
//  Wire format, one datagram per sample:
//      PEND <seq> <t_ms> <pitch> <roll> <slider> <cut><rec>

import SwiftUI
import UIKit
import CoreMotion
import Network

// MARK: - Pendant

final class Pendant: ObservableObject {
    @Published var pitch: Double = 0        // degrees, + nose up
    @Published var roll: Double = 0         // degrees, + right
    @Published var slider: Double = 0       // -1...1, thumb control
    @Published var cutting = false
    @Published var recording = false
    @Published var status = "not connected"
    @Published var connected = false
    @Published var sent = 0

    private let motion = CMMotionManager()
    private var conn: NWConnection?
    private var reference: CMAttitude?
    private var seq: UInt32 = 0
    private var started: Date?

    static let sampleRate = 50.0

    // MARK: connection

    func connect(host: String, port: UInt16) {
        conn?.cancel()
        guard let p = NWEndpoint.Port(rawValue: port) else {
            status = "bad port"; return
        }
        let c = NWConnection(host: NWEndpoint.Host(host), port: p, using: .udp)
        c.stateUpdateHandler = { [weak self] state in
            DispatchQueue.main.async {
                switch state {
                case .ready:
                    self?.connected = true
                    self?.status = "streaming to \(host):\(port)"
                    self?.startMotion()
                case .failed(let err):
                    self?.connected = false
                    self?.status = "failed: \(err.localizedDescription)"
                case .waiting(let err):
                    self?.status = "waiting: \(err.localizedDescription)"
                default:
                    break
                }
            }
        }
        conn = c
        status = "connecting…"
        c.start(queue: .global(qos: .userInteractive))
    }

    func disconnect() {
        motion.stopDeviceMotionUpdates()
        conn?.cancel()
        conn = nil
        connected = false
        cutting = false
        recording = false
        status = "not connected"
    }

    // MARK: motion

    private func startMotion() {
        guard motion.isDeviceMotionAvailable else {
            status = "no motion sensors"; return
        }
        started = Date()
        motion.deviceMotionUpdateInterval = 1.0 / Pendant.sampleRate
        // .xArbitraryZVertical: gravity-referenced, no magnetometer, so no
        // compass weirdness near motors.
        motion.startDeviceMotionUpdates(using: .xArbitraryZVertical,
                                        to: .main) { [weak self] dm, _ in
            guard let self, let dm else { return }
            let attitude = dm.attitude.copy() as! CMAttitude
            if let ref = self.reference { attitude.multiply(byInverseOf: ref) }
            self.pitch = attitude.pitch * 180 / .pi
            self.roll  = attitude.roll  * 180 / .pi
            self.send()
        }
    }

    /// Make the current pose neutral. Hold the phone how you'd hold clippers,
    /// then tap — handedness and grip stop mattering.
    func recenter() {
        reference = motion.deviceMotion?.attitude.copy() as? CMAttitude
        pitch = 0; roll = 0
    }

    // MARK: transmit

    private func send() {
        guard let conn, connected else { return }
        seq &+= 1
        let ms = Int((started.map { Date().timeIntervalSince($0) } ?? 0) * 1000)
        let line = String(format: "PEND %u %d %.2f %.2f %.3f %d%d",
                          seq, ms, pitch, roll, slider,
                          cutting ? 1 : 0, recording ? 1 : 0)
        conn.send(content: line.data(using: .utf8),
                  completion: .contentProcessed { _ in })
        if seq % 25 == 0 { sent = Int(seq) }
    }
}

// MARK: - Spring-back vertical slider (phi rate)

struct RateSlider: View {
    @Binding var value: Double
    private let height: CGFloat = 230

    var body: some View {
        GeometryReader { geo in
            ZStack {
                Capsule().fill(Color.white.opacity(0.07))
                Rectangle().fill(Color.white.opacity(0.18))
                    .frame(height: 1)
                Circle()
                    .fill(value == 0 ? Color.gray.opacity(0.55)
                                     : Color(red: 0.43, green: 0.66, blue: 0.86))
                    .frame(width: 54, height: 54)
                    .offset(y: -CGFloat(value) * (geo.size.height / 2 - 30))
                    .animation(value == 0 ? .spring(response: 0.25) : nil,
                               value: value)
            }
            .contentShape(Rectangle())
            .gesture(
                DragGesture(minimumDistance: 0)
                    .onChanged { g in
                        let half = geo.size.height / 2
                        let dy = half - g.location.y
                        value = max(-1, min(1, Double(dy / (half - 30))))
                    }
                    .onEnded { _ in value = 0 }   // springs back: it's a rate
            )
        }
        .frame(width: 74, height: height)
    }
}

// MARK: - UI

struct ContentView: View {
    @StateObject private var pendant = Pendant()
    @AppStorage("host") private var host = "192.168.2.1"
    @AppStorage("port") private var portText = "8470"

    private let ink = Color(red: 0.93, green: 0.94, blue: 0.95)
    private let muted = Color(red: 0.60, green: 0.63, blue: 0.67)
    private let panel = Color(red: 0.12, green: 0.13, blue: 0.15)

    var body: some View {
        ZStack {
            Color(red: 0.08, green: 0.09, blue: 0.10).ignoresSafeArea()
            VStack(spacing: 14) {

                // header ------------------------------------------------
                HStack {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("FADE PENDANT")
                            .font(.system(size: 19, weight: .heavy, design: .default))
                            .foregroundColor(ink)
                        Text(pendant.status)
                            .font(.system(size: 11, design: .monospaced))
                            .foregroundColor(pendant.connected ? .green : muted)
                    }
                    Spacer()
                    Button(pendant.connected ? "Stop" : "Connect") {
                        if pendant.connected { pendant.disconnect() }
                        else { pendant.connect(host: host,
                                               port: UInt16(portText) ?? 8470) }
                    }
                    .font(.system(size: 14, weight: .semibold))
                    .padding(.horizontal, 16).padding(.vertical, 9)
                    .background(pendant.connected ? Color.red.opacity(0.8)
                                                  : Color.blue.opacity(0.85))
                    .foregroundColor(.white)
                    .clipShape(Capsule())
                }

                // address -----------------------------------------------
                HStack(spacing: 8) {
                    TextField("laptop IP", text: $host)
                        .keyboardType(.decimalPad)
                        .textFieldStyle(.plain)
                        .font(.system(size: 15, design: .monospaced))
                        .foregroundColor(ink)
                        .padding(10).background(panel).cornerRadius(9)
                    TextField("port", text: $portText)
                        .keyboardType(.numberPad)
                        .textFieldStyle(.plain)
                        .font(.system(size: 15, design: .monospaced))
                        .foregroundColor(ink)
                        .frame(width: 78)
                        .padding(10).background(panel).cornerRadius(9)
                }

                // live attitude -----------------------------------------
                HStack(spacing: 10) {
                    readout("TILT", pendant.pitch, "clipper / length")
                    readout("TWIST", pendant.roll, "around head")
                }

                Spacer(minLength: 4)

                // controls ----------------------------------------------
                HStack(alignment: .center, spacing: 18) {
                    VStack(spacing: 10) {
                        Button(action: { pendant.recenter() }) {
                            Text("RECENTER")
                                .font(.system(size: 13, weight: .bold))
                                .frame(maxWidth: .infinity)
                                .padding(.vertical, 13)
                                .background(panel).foregroundColor(ink)
                                .cornerRadius(10)
                        }
                        Button(action: { pendant.recording.toggle() }) {
                            HStack(spacing: 7) {
                                Circle().fill(pendant.recording ? Color.red : muted)
                                    .frame(width: 11, height: 11)
                                Text(pendant.recording ? "RECORDING" : "RECORD")
                                    .font(.system(size: 13, weight: .bold))
                            }
                            .frame(maxWidth: .infinity)
                            .padding(.vertical, 13)
                            .background(pendant.recording ? Color.red.opacity(0.22)
                                                          : panel)
                            .foregroundColor(ink)
                            .cornerRadius(10)
                        }
                        .disabled(!pendant.connected)

                        Text("HEIGHT")
                            .font(.system(size: 10, weight: .bold))
                            .foregroundColor(muted).padding(.top, 2)
                    }

                    RateSlider(value: $pendant.slider)
                }

                // cut ---------------------------------------------------
                Text(pendant.cutting ? "CLIPPER RUNNING" : "HOLD TO CUT")
                    .font(.system(size: 17, weight: .heavy))
                    .foregroundColor(pendant.cutting ? .white : ink)
                    .frame(maxWidth: .infinity)
                    .padding(.vertical, 26)
                    .background(pendant.cutting ? Color(red: 0.85, green: 0.31, blue: 0.31)
                                                : panel)
                    .cornerRadius(16)
                    .gesture(
                        DragGesture(minimumDistance: 0)
                            .onChanged { _ in if pendant.connected { pendant.cutting = true } }
                            .onEnded { _ in pendant.cutting = false }
                    )
                    .opacity(pendant.connected ? 1 : 0.45)
            }
            .padding(18)
        }
        .preferredColorScheme(.dark)
        .statusBarHidden(true)
        .onAppear { UIApplication.shared.isIdleTimerDisabled = true }
        .onDisappear { UIApplication.shared.isIdleTimerDisabled = false }
    }

    private func readout(_ label: String, _ value: Double,
                         _ sub: String) -> some View {
        VStack(alignment: .leading, spacing: 2) {
            Text(label).font(.system(size: 10, weight: .bold))
                .foregroundColor(muted)
            Text(String(format: "%+.0f°", value))
                .font(.system(size: 30, weight: .medium, design: .monospaced))
                .foregroundColor(ink)
            Text(sub).font(.system(size: 10)).foregroundColor(muted)
        }
        .frame(maxWidth: .infinity, alignment: .leading)
        .padding(12).background(panel).cornerRadius(11)
    }
}

@main
struct FadePendantApp: App {
    var body: some Scene { WindowGroup { ContentView() } }
}
