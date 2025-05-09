//
//  ViewController.swift
//  CameraWebSocketV1
//
//  Created by Miguel Domingo on 3/12/25.
//

import UIKit
import AVFoundation

class ViewController: UIViewController {
    
    var cameraController = CameraController()
    let webSocketManager = WebSocketManager()
    var recordingTimer: Timer?
    var recordingStartTime: Date?
    var currentCommand: String?
    var commandReceivedAt: Date?
    
    @IBOutlet weak var recordButton: UIButton!
    @IBOutlet weak var flashView: UIView!
    @IBOutlet weak var recordingTimerLabel: UILabel!
    @IBOutlet weak var buttonStackView: UIStackView!
    @IBOutlet weak var timer: UILabel!
    @IBOutlet weak var commandStatusLabel: UILabel!
    @IBOutlet weak var connectionOverlayView: UIView!
    @IBOutlet weak var connectionStatusLabel: UILabel!
    @IBOutlet weak var skipButton: UIButton!

    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        connectionOverlayView.isHidden = false
        connectionStatusLabel.text = "Connecting to WebSocket..."
        cameraController.attachPreview(to: cameraPreviewView)
        
        // Listen for "picture" command from WebSockets
        NotificationCenter.default.addObserver(self, selector: #selector(triggerPhotoCapture), name: Notification.Name("CapturePhoto"), object: nil)
        
        // Zoom command
        NotificationCenter.default.addObserver(self, selector: #selector(handleZoomIn), name: Notification.Name("ZoomIn"), object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(handleZoomOut), name: Notification.Name("ZoomOut"), object: nil)
        
        //Video command
        NotificationCenter.default.addObserver(self, selector: #selector(toggleVideoRecording), name: Notification.Name("ToggleVideo"), object: nil)
        
        //Reconnect
        NotificationCenter.default.addObserver(
                self,
                selector: #selector(appDidBecomeActive),
                name: UIApplication.didBecomeActiveNotification,
                object: nil)
        
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(hideConnectionOverlay),
            name: .WebSocketConnected,
            object: nil
        )
        
        //Command Status
      /*  NotificationCenter.default.addObserver(
                self,
                selector: #selector(handleWebSocketCommand(_:)),
                name: Notification.Name("WebSocketCommandReceived"),
                object: nil)
        
        // Existing command received listener
            NotificationCenter.default.addObserver(
                self,
                selector: #selector(handleWebSocketCommand(_:)),
                name: Notification.Name("WebSocketCommandReceived"),
                object: nil
            )
        
        
            //New: listen for completion
            NotificationCenter.default.addObserver(
                self,
                selector: #selector(handleCommandCompleted),
                name: Notification.Name("CommandCompleted"),
                object: nil
            )
        
        
        //Command Starting
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(showLatency(_:)),
            name: Notification.Name("CommandStarting"),
            object: nil
        )
        */
        
        //Command Starting
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(showCommandLatency(_:)),
            name: Notification.Name("CommandStarting"),
            object: nil
        )
        
        UIDevice.current.beginGeneratingDeviceOrientationNotifications()
        
        //Rotation
        NotificationCenter.default.addObserver(
            self,
            selector: #selector(deviceDidRotate),
            name: UIDevice.orientationDidChangeNotification,
            object: nil
        )
        
        buttonStackView.transform = CGAffineTransform(rotationAngle: .pi/2)
        timer.transform = CGAffineTransform(rotationAngle: .pi/2)
        commandStatusLabel.transform = CGAffineTransform(rotationAngle: .pi/2)
        

    }
    
    override func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        cameraController.previewLayer?.frame = cameraPreviewView.bounds
        if let connection = cameraController.previewLayer?.connection {
            if connection.isVideoOrientationSupported {
                connection.videoOrientation = getCurrentOrientation()
            }
        }
        
    }
    
    func getCurrentOrientation() -> AVCaptureVideoOrientation {
        switch UIDevice.current.orientation {
        case .landscapeLeft:
            return .landscapeRight
        case .landscapeRight:
            return .landscapeLeft
        case .portraitUpsideDown:
            return .portraitUpsideDown
        default:
            return .portrait
        }
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        NotificationCenter.default.addObserver(self, selector: #selector(orientationDidChange), name: UIDevice.orientationDidChangeNotification, object: nil)
    }
    
    
    var lastKnownOrientation: UIDeviceOrientation = .portrait
    
    @objc func deviceDidRotate() {
            let orientation = UIDevice.current.orientation

                switch orientation {
                case .portrait:
                    //print("Portrait (raw: \(orientation.rawValue))")
                    break
                case .portraitUpsideDown:
                    //print("Upside Down (raw: \(orientation.rawValue))")
                    break
                case .landscapeLeft:
                    //print("Landscape Left (raw: \(orientation.rawValue))")
                    break
                case .landscapeRight:
                    //print("Landscape Right (raw: \(orientation.rawValue))")
                    break
                case .faceUp:
                    //print("Face Up (raw: \(orientation.rawValue)) — Ignored")
                    break
                case .faceDown:
                    //print("Face Down (raw: \(orientation.rawValue)) — Ignored")
                    break
                case .unknown:
                    //print("Unknown orientation (raw: \(orientation.rawValue)) — Ignored")
                    break
                default:
                    break
                }
    }


    
    @objc func orientationDidChange() {
        if let connection = cameraController.previewLayer?.connection, connection.isVideoOrientationSupported {
            connection.videoOrientation = getCurrentOrientation()
        }
    }
    
    @IBOutlet weak var cameraPreviewView: UIView!
    
    @objc func triggerPhotoCapture() {
        cameraController.capturePhoto(withOrientation: lastKnownOrientation)
        animateFlash()
    }
    
    @IBAction func capturePhoto(_sender: UIButton) {
        cameraController.capturePhoto(withOrientation: lastKnownOrientation)
        animateFlash()
        //print("Take Photo button pressed")
        
    }
    
    @IBAction func videoRecordTapped(_sender: UIButton) {
        toggleVideoRecording()
    }
    
    @IBAction func toggleZoomIn(_sender: UIButton) {
        cameraController.zoom(by: 0.5)
    }
    
    @IBAction func toggleZoomOut(_sender: UIButton) {
        cameraController.zoom(by: -0.5)
    }
    
    @IBAction func skipButtonPressed(_ sender: UIButton) {
        hideConnectionOverlay()
    }
    
    @objc func appDidBecomeActive() {
        //print("App became active, reconnecting WebSocket...")
        webSocketManager.connectWebSocket()
    }
    
    
    
    
    // Handling zoom functions
    @objc func handleZoomIn() {
        cameraController.zoom(by: 0.5)
    }
    
    @objc func handleZoomOut() {
        cameraController.zoom(by: -0.5)
    }
    
    //Handling video recording function
    @objc func toggleVideoRecording() {
        if cameraController.isRecording {
                cameraController.stopVideoRecording()
                stopRecordingTimer()
                recordButton.setTitle("⏺ Record", for: .normal)
                recordButton.backgroundColor = .systemRed
            } else {
                cameraController.startVideoRecording()
                startRecordingTimer()
                recordButton.setTitle("⏹ Stop", for: .normal)
                recordButton.backgroundColor = .darkGray
            }
        
    }
    
    //Nice little flash animation when taking picture
    func animateFlash() {
        DispatchQueue.main.async {
            UIView.animate(withDuration: 0.1, animations: {
                self.flashView.alpha = 1
            }) { _ in
                UIView.animate(withDuration: 0.1, delay: 0.05, options: [], animations: {
                    self.flashView.alpha = 0
                }, completion: nil)
            }
        }
    }

    
    //Starts Record Timer
    func startRecordingTimer() {
        recordingStartTime = Date()
        recordingTimerLabel.text = "00:00"
        recordingTimerLabel.isHidden = false

        recordingTimer = Timer.scheduledTimer(withTimeInterval: 1.0, repeats: true) { [weak self] _ in
            self?.updateRecordingTimer()
        }
    }
    
    //Stops Record Timer
    func stopRecordingTimer() {
        recordingTimer?.invalidate()
        recordingTimer = nil
        recordingStartTime = nil
        recordingTimerLabel.text = "00:00"
        recordingTimerLabel.isHidden = true
    }
    
    //Updates Timer
    func updateRecordingTimer() {
        guard let startTime = recordingStartTime else { return }

        let elapsed = Int(Date().timeIntervalSince(startTime))
        let minutes = elapsed / 60
        let seconds = elapsed % 60
        recordingTimerLabel.text = String(format: "%02d:%02d", minutes, seconds)
    }
    
/*
    //Handler for Command Status
    @objc func handleWebSocketCommand(_ notification: Notification) {
        guard let command = notification.object as? String else { return }

        currentCommand = command
        commandReceivedAt = Date()

        commandStatusLabel.text = "\(command)..."
    }
  */
    
    //Helper Function for Connection Blur
    @objc func hideConnectionOverlay() {
        UIView.animate(withDuration: 0.3) {
            self.connectionOverlayView.alpha = 0
        } completion: { _ in
            self.connectionOverlayView.isHidden = true
        }
    }
    
    @IBAction func skipConnectionTapped(_ sender: UIButton) {
        hideConnectionOverlay()
    }
    
    @objc func showCommandLatency(_ notification: Notification) {
        guard let (command, ms) = CommandLatencyTracker.shared.calculateLatency() else {
                print("[CALC] Missing data")
                return
            }
            let formatted = String(format: "%.1f", ms)
            commandStatusLabel.text = "⚡️ \(command) — \(formatted) ms"

            DispatchQueue.main.asyncAfter(deadline: .now() + 5) {
                self.commandStatusLabel.text = ""
            }
        }
    
/*
    @objc func showLatency(_ notification: Notification) {
        guard let command = notification.object as? String else { return }

        let now = Date()

        // Save timestamp first
        if currentCommand == nil {
            currentCommand = command
            commandReceivedAt = now
            print("📥 Received \(command) at \(now)")
            return // We'll get called again if "CommandStarting" fires again too early
        }

        guard let start = commandReceivedAt else { return }

        let ms = Int(now.timeIntervalSince(start) * 1000)
        print("⚡️ Command '\(command)' latency: \(ms) ms")

        commandStatusLabel.text = "⚡️ \(command) — \(ms) ms"

        DispatchQueue.main.asyncAfter(deadline: .now() + 2) {
            self.commandStatusLabel.text = ""
        }

        currentCommand = nil
        commandReceivedAt = nil
    }

    
    func markCommandCompleted() {
        guard let command = currentCommand, let start = commandReceivedAt else { return }

            let now = Date()
            let latency = now.timeIntervalSince(start) * 1000
            let ms = Int(latency)

        print("Command '\(command)' latency: \(ms) ms (start: \(start), now: \(now))")
            commandStatusLabel.text = " \(command) — \(ms) ms"

        DispatchQueue.main.asyncAfter(deadline: .now() + 2) {
            self.commandStatusLabel.text = ""
        }
        
        currentCommand = nil
        commandReceivedAt = nil
    }
    
    @objc func handleCommandCompleted() {
        markCommandCompleted()
    }
    
    */
    
}

