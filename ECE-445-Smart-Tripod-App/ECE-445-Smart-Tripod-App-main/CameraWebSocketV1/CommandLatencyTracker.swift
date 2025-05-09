//
//  CommandLatencyTracker.swift
//  CameraWebSocketV1
//
//  Created by Miguel Domingo on 4/29/25.
//

import Foundation

class CommandLatencyTracker {
    static let shared = CommandLatencyTracker()

    private var timestamp: Date?
    private var command: String?

    func record(command: String) {
        self.command = command
        self.timestamp = Date()
        print("[RECORD] '\(command)' at \(self.timestamp!)")
    }

    func calculateLatency() -> (command: String, ms: Double)? {
        guard let command = command, let timestamp = timestamp else {
            print("[CALC] Missing data")
            return nil
        }
        let now = Date()
            let latency = now.timeIntervalSince(timestamp) * 1000 // ms as Double
            print("[CALC] '\(command)' - \(latency) ms (from \(timestamp) to \(now))")

            // Reset after calculation
            self.command = nil
            self.timestamp = nil

            return (command, latency)
    }
}
