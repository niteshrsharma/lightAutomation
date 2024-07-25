import cv2
import mediapipe as mp
import serial
import time

# Set up the serial connection to the Arduino
try:
    arduino = serial.Serial('COM4', 9600, timeout=1)  # Adjust 'COM3' and baud rate as needed
    time.sleep(2)  # Wait for the connection to initialize
except serial.SerialException as e:
    print("Error opening serial port")
    exit(1)

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands

def detect_hand_gesture(frame, hands):
    # Convert the image to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Process the frame with MediaPipe Hands
    results = hands.process(rgb_frame)
    
    # Check if landmarks are detected
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Check thumb openness
            thumb_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].y
            
            # Check index finger openness
            index_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_DIP].y
            
            # Check middle finger openness
            middle_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.MIDDLE_FINGER_DIP].y
            
            # Check ring finger openness
            ring_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.RING_FINGER_DIP].y
            
            # Check pinky finger openness
            pinky_is_open = hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_TIP].y < hand_landmarks.landmark[mp_hands.HandLandmark.PINKY_DIP].y
            
            # Determine gesture based on finger openness
            if thumb_is_open and index_is_open and middle_is_open and ring_is_open and pinky_is_open:
                return "led on"  # All fingers open (thumbs-up gesture)
            else:
                return "led off"  # Any finger closed (thumbs-down gesture)
    
    return None

def main():
    cap = cv2.VideoCapture(0)  # Open the default camera (index 0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    try:
        with mp_hands.Hands() as hands:
            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                # Flip the frame horizontally for a more natural mirror effect
                frame = cv2.flip(frame, 1)

                # Detect hand gesture
                gesture = detect_hand_gesture(frame, hands)

                if gesture == "led on":
                    arduino.write(b'1')  # Send '1' to Arduino to turn the LED on
                    cv2.putText(frame, "(LED ON)", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                elif gesture == "led off":
                    arduino.write(b'0')  # Send '0' to Arduino to turn the LED off
                    cv2.putText(frame, "(LED OFF)", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                # Display the frame
                cv2.imshow('Hand Gesture Control', frame)

                # Check for user input to quit (press 'q')
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        print("Program terminated.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        arduino.close()

if __name__ == "__main__":
    main()

