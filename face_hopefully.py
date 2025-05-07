#!/usr/bin/env python3

import cv2
import numpy as np
import time
import os
import signal
import sys
import RPi.GPIO as GPIO

# Define GPIOs for movement
GPIO_LEFT = 6
GPIO_RIGHT = 5
GPIO_UP = 26
GPIO_DOWN = 13

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(GPIO_LEFT, GPIO.OUT)
GPIO.setup(GPIO_RIGHT, GPIO.OUT)
GPIO.setup(GPIO_UP, GPIO.OUT)
GPIO.setup(GPIO_DOWN, GPIO.OUT)

# Ensure all start HIGH
GPIO.output(GPIO_LEFT, GPIO.HIGH)
GPIO.output(GPIO_RIGHT, GPIO.HIGH)
GPIO.output(GPIO_UP, GPIO.HIGH)
GPIO.output(GPIO_DOWN, GPIO.HIGH)

# Track previous GPIO states
prev_left_active = False
prev_right_active = False
prev_up_active = False
prev_down_active = False

# Create log directory if it doesn't exist
LOG_DIR = os.path.expanduser("~/opencv_logs")
os.makedirs(LOG_DIR, exist_ok=True)
LOG_FILE = os.path.join(LOG_DIR, "opencv_processor.log")

def log_message(message):
    """Write log messages to file with timestamp"""
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a") as f:
        f.write(f"{timestamp} - {message}\n")
    print(f"{timestamp} - {message}")

def signal_handler(sig, frame):
    """Handle clean shutdown on SIGINT/SIGTERM"""
    log_message("Received shutdown signal. Closing OpenCV and exiting.")
    if 'cap' in globals() and cap is not None:
        cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()
    sys.exit(0)

def main():
    global cap, prev_left_active, prev_right_active, prev_up_active, prev_down_active

    # Register signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize log file
    with open(LOG_FILE, "w") as f:
        f.write("")

    log_message("Starting OpenCV video processing with KCF tracking")

    # Light performance parameters - adjust these values based on your Pi's capability
    detection_scale = 0.5  # Process at half resolution for detection
    detection_interval = 30  # Only detect faces every 30th frame
    verification_interval = 60  # Verify face tracking every 60th frame
    
    # Use original camera initialization that works on your setup
    max_retries = 10
    retry_count = 0
    cap = None

    while retry_count < max_retries:
        try:
            cap = cv2.VideoCapture(0)  # /dev/video0
            if cap.isOpened():
                log_message("Video device opened successfully")
                break
            else:
                retry_count += 1
                log_message(f"Failed to open video device. Retry {retry_count}/{max_retries}...")
                time.sleep(2)
        except Exception as e:
            retry_count += 1
            log_message(f"Error opening video device: {str(e)}. Retry {retry_count}/{max_retries}...")
            time.sleep(2)

    if not cap or not cap.isOpened():
        log_message("Failed to open video device after multiple retries. Exiting.")
        return

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    log_message(f"Capture resolution: {width}x{height}")

    # Center region for triggering movement
    rect_width = int(width / 5)
    rect_height = int(height / 4)
    rect_x = int((width - rect_width) / 2)
    rect_y = int((height - rect_height) / 2)

    # Load Haar cascade face detector
    try:
        cascade_path = "/home/tripod/opencv_haar/haarcascade_frontalface_default.xml"
        face_cascade = cv2.CascadeClassifier(cascade_path)
    except Exception as e:
        log_message(f"Error loading cascade: {e}")
        return
    if face_cascade.empty():
        log_message("Failed to load Haar cascade for face detection.")
        return

    cv2.namedWindow('Processed UxPlay Feed', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Processed UxPlay Feed', 640, 456)

    # KCF tracker variables
    tracker = None
    tracking_box = None
    tracking_confidence = 1.0
    tracking_active = False
    tracking_lost_count = 0
    
    frame_count = 0
    start_time = time.time()
    last_detection_time = time.time()

    try:
        while True:
            # Get frame
            ret, frame = cap.read()
            if not ret:
                log_message("Failed to grab frame, retrying...")
                time.sleep(0.1)
                continue

            frame_count += 1
            
            # Calculate FPS every 100 frames
            if frame_count % 100 == 0:
                elapsed_time = time.time() - start_time
                fps = 100 / elapsed_time
                log_message(f"Processing at {fps:.2f} FPS")
                start_time = time.time()

            # Initialize face position variables
            face_x, face_y, face_w, face_h = 0, 0, 0, 0
            face_detected = False
            tracking_successful = False

            # Face detection (runs periodically or if tracking is not active)
            run_detection = not tracking_active or frame_count % detection_interval == 0
            
            if run_detection:
                # Convert to grayscale and resize for faster processing
                small_gray = cv2.cvtColor(
                    cv2.resize(frame, (0, 0), fx=detection_scale, fy=detection_scale), 
                    cv2.COLOR_BGR2GRAY
                )
                
                # Detect faces
                faces = face_cascade.detectMultiScale(
                    small_gray,
                    scaleFactor=1.2,
                    minNeighbors=5,
                    minSize=(int(30*detection_scale), int(30*detection_scale))
                )
                
                # Scale coordinates back to original size
                if len(faces) > 0:
                    faces = faces / detection_scale
                    faces = faces.astype(np.int32)
                    face_x, face_y, face_w, face_h = faces[0]
                    face_detected = True
                    last_detection_time = time.time()
                    
                    # Initialize or reinitialize tracker with new detection
                    if tracker is None or not tracking_active:
                        log_message("Initializing KCF tracker with new face detection")
                        tracker = cv2.TrackerKCF_create()
                        tracking_box = (face_x, face_y, face_w, face_h)
                        tracker.init(frame, tracking_box)
                        tracking_active = True
                        tracking_confidence = 1.0
                        tracking_lost_count = 0
            
            # Run face verification periodically to ensure we're still tracking a face
            elif tracking_active and frame_count % verification_interval == 0:
                # Convert to grayscale and resize for faster processing
                small_gray = cv2.cvtColor(
                    cv2.resize(frame, (0, 0), fx=detection_scale, fy=detection_scale), 
                    cv2.COLOR_BGR2GRAY
                )
                
                # Create region of interest from tracking box
                if tracking_box:
                    tx, ty, tw, th = [int(v) for v in tracking_box]
                    # Scale down for verification
                    tx_s, ty_s = int(tx * detection_scale), int(ty * detection_scale)
                    tw_s, th_s = int(tw * detection_scale), int(th * detection_scale)
                    
                    # Ensure ROI is within bounds
                    if (ty_s >= 0 and tx_s >= 0 and 
                        ty_s + th_s <= small_gray.shape[0] and 
                        tx_s + tw_s <= small_gray.shape[1]):
                        
                        # Detect faces in ROI
                        faces = face_cascade.detectMultiScale(
                            small_gray[ty_s:ty_s+th_s, tx_s:tx_s+tw_s],
                            scaleFactor=1.1,
                            minNeighbors=4,
                            minSize=(int(20*detection_scale), int(20*detection_scale))
                        )
                        
                        if len(faces) == 0:
                            log_message("Verification failed - no face in tracking region")
                            tracking_confidence *= 0.5
                            if tracking_confidence < 0.3:
                                tracking_active = False
                                tracker = None
                        else:
                            tracking_confidence = 1.0
                            last_detection_time = time.time()
            
            # Apply tracker if active
            if tracking_active:
                success, box = tracker.update(frame)
                if success:
                    tracking_box = box
                    face_x, face_y, face_w, face_h = [int(v) for v in box]
                    tracking_successful = True
                    tracking_lost_count = 0
                else:
                    tracking_lost_count += 1
                    if tracking_lost_count > 10:
                        log_message("Lost tracking for too many frames, resetting")
                        tracking_active = False
                        tracker = None
            
            # If we have a valid face position (either from detection or tracking)
            if face_detected or tracking_successful:
                # Draw Rectangle around face
                color = (255, 0, 0) if face_detected else (0, 255, 0)  # Blue for detection, Green for tracking
                cv2.rectangle(frame, (face_x, face_y), (face_x + face_w, face_y + face_h), color, 2)
                
                # Calculate center of face
                cx, cy = face_x + face_w // 2, face_y + face_h // 2
                cv2.circle(frame, (cx, cy), 3, (0, 255, 255), -1)
                
                # Check if face is outside center box and control GPIOs
                left_active = cx < rect_x
                right_active = cx > rect_x + rect_width
                up_active = cy < rect_y
                down_active = cy > rect_y + rect_height
                
                # Only update GPIO if state has changed
                if left_active != prev_left_active:
                    GPIO.output(GPIO_LEFT, GPIO.LOW if left_active else GPIO.HIGH)
                    prev_left_active = left_active
                    
                if right_active != prev_right_active:
                    GPIO.output(GPIO_RIGHT, GPIO.LOW if right_active else GPIO.HIGH)
                    prev_right_active = right_active
                    
                if up_active != prev_up_active:
                    GPIO.output(GPIO_UP, GPIO.LOW if up_active else GPIO.HIGH)
                    prev_up_active = up_active
                    
                if down_active != prev_down_active:
                    GPIO.output(GPIO_DOWN, GPIO.LOW if down_active else GPIO.HIGH)
                    prev_down_active = down_active
                
                # Draw direction indicators
                if left_active:
                    cv2.arrowedLine(frame, (rect_x - 5, height // 2), (rect_x - 15, height // 2), 
                                   color=(0,255,255), thickness=2, tipLength=0.2)
                if right_active:
                    cv2.arrowedLine(frame, (rect_x + rect_width + 5, height // 2), 
                                   (rect_x + rect_width + 15, height // 2), 
                                   color=(0,255,255), thickness=2, tipLength=0.2)
                if up_active:
                    cv2.arrowedLine(frame, (width // 2, rect_y - 5), (width // 2, rect_y - 15), 
                                   color=(0,255,255), thickness=2, tipLength=0.2)
                if down_active:
                    cv2.arrowedLine(frame, (width // 2, rect_y + rect_height + 5), 
                                   (width // 2, rect_y + rect_height + 15), 
                                   color=(0,255,255), thickness=2, tipLength=0.2)
                
                # Add position and tracking info text
                track_mode = "Detection" if face_detected else f"Tracking (conf: {tracking_confidence:.2f})"
                cv2.putText(
                    frame,
                    f"{track_mode}: ({cx}, {cy})",
                    (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    1
                )
                
                # Check if we haven't had a successful detection in a while (30 seconds)
                if time.time() - last_detection_time > 30:
                    log_message("No face detection for 30 seconds, resetting tracker")
                    tracking_active = False
                    tracker = None
            else:
                # Reset all GPIOs to inactive if no face detected or tracked
                if prev_left_active or prev_right_active or prev_up_active or prev_down_active:
                    GPIO.output(GPIO_LEFT, GPIO.HIGH)
                    GPIO.output(GPIO_RIGHT, GPIO.HIGH)
                    GPIO.output(GPIO_UP, GPIO.HIGH)
                    GPIO.output(GPIO_DOWN, GPIO.HIGH)
                    prev_left_active = prev_right_active = prev_up_active = prev_down_active = False
                
                # Reset the tracker if we've been lost for a while
                if tracking_active and tracking_lost_count > 10:
                    tracking_active = False
                    tracker = None

            # Always draw center rectangle
            cv2.rectangle(
                frame,
                (rect_x, rect_y),
                (rect_x + rect_width, rect_y + rect_height),
                (0, 255, 0),
                2
            )
            
            # Add tracking status
            status = "ACTIVE" if tracking_active else "SEARCHING"
            cv2.putText(
                frame,
                f"Status: {status}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                1
            )
            
            # Show output
            cv2.imshow('Processed UxPlay Feed', frame)

            # Exit on 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except Exception as e:
        log_message(f"Error in video processing loop: {str(e)}")

    finally:
        if cap:
            cap.release()
        cv2.destroyAllWindows()
        log_message("OpenCV processing stopped")
        GPIO.cleanup()

if __name__ == "__main__":
    main()