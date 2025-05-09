//
//  CameraController.swift
//  CameraWebSocketV1
//
//  Created by Miguel Domingo on 3/12/25.
//

import AVFoundation
import UIKit

class CameraController: NSObject {
    
    let session = AVCaptureSession()
    var output = AVCapturePhotoOutput()
    var movieOutput = AVCaptureMovieFileOutput()
    var isRecording = false
    var captureDevice: AVCaptureDevice?
    var previewLayer: AVCaptureVideoPreviewLayer?
    
    override init() {
        super.init()
        setupCamera()
    }
    
    
    func setupCamera() {
        session.sessionPreset = .photo
        guard let device = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back) else {
            print("No camera available")
            return
        }
        captureDevice = device
        
        do {
            let input = try AVCaptureDeviceInput(device: device)
            session.addInput(input)
            session.addOutput(output)
            
            //adding video support
            if session.canAddOutput(movieOutput) {
                session.addOutput(movieOutput)
            }
            
        } catch {
            print("Camera setup error: \(error)")
        }
    }
    
    func attachPreview(to view: UIView) {
        previewLayer = AVCaptureVideoPreviewLayer(session: session)
        previewLayer?.videoGravity = .resizeAspect
        previewLayer?.frame = view.bounds
        view.layer.addSublayer(previewLayer!)
        
        DispatchQueue.main.async {
            self.previewLayer?.frame = view.bounds
        }
        session.startRunning()
    }
    
    func capturePhoto(withOrientation deviceOrientation: UIDeviceOrientation) {
        let settings = AVCapturePhotoSettings()

        let videoOrientation = getCurrentVideoOrientation(from: deviceOrientation)

            if let photoOutputConnection = output.connection(with: .video),
               photoOutputConnection.isVideoOrientationSupported {
                photoOutputConnection.videoOrientation = videoOrientation
                //print("📸 Applied video orientation:", videoOrientation.rawValue)
            }

        output.capturePhoto(with: settings, delegate: self)
    }

    
    // Zoom Comtrol
    func zoom(by delta: CGFloat) {
        guard let device = captureDevice else { return }
        do {
            try device.lockForConfiguration()
            let currentZoom = device.videoZoomFactor
            let maxZoom = device.activeFormat.videoMaxZoomFactor
            let newZoom = min(max(currentZoom + delta, 1.0), min(maxZoom, 5.0)) //Clamped to 5x right now for safety (change later)
            device.videoZoomFactor = newZoom
            device.unlockForConfiguration()
            print("zoom set to \(newZoom)x")
        } catch {
            print ("Zoom error: \(error)")
        }
    }
    
    // Video Recording
    func startVideoRecording() {
        guard !isRecording else { return }
        
        let filename = UUID().uuidString + ".mov"
        let tempURL = FileManager.default.temporaryDirectory.appendingPathComponent(filename)
        
        print("Starting recording to: \(tempURL.path)")
        
        if let connection = movieOutput.connection(with: .video) {
            connection.videoOrientation = .portrait
        }
        
        movieOutput.startRecording(to: tempURL, recordingDelegate: self)
        isRecording = true
        
    }
    
    func stopVideoRecording() {
        guard isRecording else { return }

        movieOutput.stopRecording()
        // isRecording will be set to false in the delegate
    }
    
}

//saving photo to Photos App
extension CameraController: AVCapturePhotoCaptureDelegate {
    func photoOutput(_ output: AVCapturePhotoOutput, didFinishProcessingPhoto photo: AVCapturePhoto, error: Error?) {
        guard let data = photo.fileDataRepresentation(),
              let image = UIImage(data: data) else {
            print("Failed to capture image")
            return
        }

        // Debug: log orientation
            //print("Captured image orientation:", image.imageOrientation.rawValue)

            // Flatten it to portrait orientation (no EXIF rotation issues)

            // Save to Photos app
            UIImageWriteToSavedPhotosAlbum(image, nil, nil, nil)

            //print("Photo saved with correct orientation")
        
    }

}

//saving video to Photos App
extension CameraController: AVCaptureFileOutputRecordingDelegate {
    func fileOutput(_ output: AVCaptureFileOutput, didFinishRecordingTo outputFileURL: URL, from connections: [AVCaptureConnection], error: Error?) {
        isRecording = false
        
        if let error = error {
            print("Recording error: \(error)")
            return
        }
        
        UISaveVideoAtPathToSavedPhotosAlbum(outputFileURL.path, nil, nil, nil)
        print("Video saved to Photos")
    }
}

//Helper for fixing the saved images to portrait just like photos app ( CURRENTLY NOT WORKING )

func normalizeToPortraitUp(_ image: UIImage) -> UIImage {
    let renderer = UIGraphicsImageRenderer(size: image.size)
    return renderer.image { _ in
        image.draw(in: CGRect(origin: .zero, size: image.size))
    }
}

func getCurrentVideoOrientation(from orientation: UIDeviceOrientation) -> AVCaptureVideoOrientation {
    switch orientation {
    case .portrait: return .portrait
    case .portraitUpsideDown: return .portraitUpsideDown
    case .landscapeLeft: return .landscapeRight
    case .landscapeRight: return .landscapeLeft
    default: return .portrait
    }
}



    /*func forcePortraitOrientation(_ image: UIImage) -> UIImage {
    guard let cgImage = image.cgImage else { return image }

    let originalSize = image.size
    let isLandscape = originalSize.width > originalSize.height

    // Always use portrait canvas (e.g., 1080x1920 or similar ratio)
    let canvasSize = CGSize(width: min(originalSize.width, originalSize.height),
                            height: max(originalSize.width, originalSize.height))

    let renderer = UIGraphicsImageRenderer(size: canvasSize)

    let newImage = renderer.image { context in
        context.cgContext.setFillColor(UIColor.black.cgColor) // optional black background like Photos app
        context.cgContext.fill(CGRect(origin: .zero, size: canvasSize))

        var drawRect = CGRect(origin: .zero, size: originalSize)

        if isLandscape {
            // Rotate into portrait
            context.cgContext.translateBy(x: canvasSize.width / 2, y: canvasSize.height / 2)
            context.cgContext.rotate(by: -.pi / 2)
            drawRect = CGRect(x: -originalSize.height / 2,
                              y: -originalSize.width / 2,
                              width: originalSize.height,
                              height: originalSize.width)
        } else {
            // Just center portrait image
            drawRect = CGRect(x: (canvasSize.width - originalSize.width) / 2,
                              y: (canvasSize.height - originalSize.height) / 2,
                              width: originalSize.width,
                              height: originalSize.height)
        }

        context.cgContext.draw(cgImage, in: drawRect)
    }

    return newImage
}
     */

