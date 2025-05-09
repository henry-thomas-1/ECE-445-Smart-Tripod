//
//  File.swift
//  CameraWebSocketV1
//
//  Created by Miguel Domingo on 3/12/25.
//

import Foundation

extension Notification.Name {
    static let CommandStarting = Notification.Name("CommandStarting")
}

extension Notification.Name {
    static let WebSocketConnected = Notification.Name("WebSocketConnected")
}

class WebSocketManager {
    var webSocketTask: URLSessionWebSocketTask?
    var lastCommandTimestamp: Date?
    var lastCommandName: String?
    
    func connectWebSocket() {
        
        if webSocketTask?.state == .running {
                print("WebSocket already running — skipping reconnect")
                return
            }
        
        webSocketTask?.cancel(with: .goingAway, reason: nil)
        
        print("🌐 Attempting WebSocket connection...")
        guard let url = URL(string: "ws://100.70.11.11:9876/") else {
        //guard let url = URL(string: "ws://192.168.4.1/ws") else {
            print("invalid websocket url")
            return
        }
        
        webSocketTask = URLSession.shared.webSocketTask(with: url)
        webSocketTask?.resume()
        
        listenForMessages()
        webSocketTask?.sendPing { error in
                DispatchQueue.main.async {
                    if let error = error {
                        print("❌ WebSocket ping failed: \(error.localizedDescription)")
                    } else {
                        print("✅ WebSocket connected and responsive")
                        NotificationCenter.default.post(name: .WebSocketConnected, object: nil)
                    }
                }
            }
        
    }
    
    func listenForMessages() {
        webSocketTask?.receive { [weak self] result in
            switch result {
            case .failure(let error):
                print("WebSocket receive error: \(error.localizedDescription)")
                self?.listenForMessages() // Retry

            case .success(let message):
                print("Message received")

                switch message {
                case .string(let text):
                    print("Text: \(text)")

                    // Record the command and timestamp immediately
                    CommandLatencyTracker.shared.record(command: text)

                    // Notify the ViewController to calculate and display latency
                    DispatchQueue.main.async {
                        NotificationCenter.default.post(name: .CommandStarting, object: text)

                        // Perform the actual command
                        switch text {
                        case "picture":
                            print("Posting CapturePhoto")
                            NotificationCenter.default.post(name: Notification.Name("CapturePhoto"), object: nil)

                        case "video":
                            print("Posting ToggleVideo")
                            NotificationCenter.default.post(name: Notification.Name("ToggleVideo"), object: nil)

                        case "zoom-in":
                            print("Posting ZoomIn")
                            NotificationCenter.default.post(name: Notification.Name("ZoomIn"), object: nil)

                        case "zoom-out":
                            print("Posting ZoomOut")
                            NotificationCenter.default.post(name: Notification.Name("ZoomOut"), object: nil)

                        default:
                            print("Unknown command: \(text)")
                        }
                    }

                default:
                    print("Received non-string WebSocket message")
                }

                //Continue listening
                self?.listenForMessages()
            }
        }
    }

}
    
    /*    func listenForMessages() {
     webSocketTask?.receive { [weak self] result in
     switch result {
     case .failure(let error):
     print ("WebSocket receive error: \(error)")
     DispatchQueue.main.asyncAfter(deadline: .now() + 3) {
     self?.connectWebSocket()
     case .success(let message):
     switch message {
     case .string(let text):
     print("Received: \(text)")
     switch text {
     case "picture" :
     NotificationCenter.default.post(name: Notification.Name("CapturePhoto"), object: nil)
     case "zoom-in":
     NotificationCenter.default.post(name: Notification.Name("ZoomIn"), object:nil)
     case "zoom-out":
     NotificationCenter.default.post(name: Notification.Name("ZoomOut"), object:nil)
     case "video":
     NotificationCenter.default.post(name: Notification.Name("ToggleVideo"), object: nil)
     default:
     print("unrecognized command")
     }
     default:
     break
     }
     }
     
     self?.listenForMessages()
     }
     }
*/

