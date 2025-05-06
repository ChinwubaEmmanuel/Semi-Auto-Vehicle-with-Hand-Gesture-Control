import cv2
import mediapipe as mp
import joblib
import numpy as np
import serial
from collections import deque
import pandas as pd

# Load model
model = joblib.load('gesture_model_2.pkl')

# MediaPipe used for hand recog
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

# Recognition using cam
cap = cv2.VideoCapture(0)

# Bluetooth (outgoing for HC-09 is COM9, incoming is COM11) [change bluetooth setting on pc to advanced, doesnt find it otherwise, not sure why]
bt = serial.Serial('COM9', 9600) # Baudrate for data mode is 9600

# Sliding window of last 5 predictions, so not as glitchy
recent_preds = deque(maxlen=5)  
stable_prediction = ""          # Final prediction to display

# Gestures to transmit
gesture_to_index = {
    'stop': 0,
    'go': 1,
    'three_sixty_degrees': 2,
    'left_90': 3,
    'right_90': 4,
    'forward': 5
}

# So it can send first gesture
prev_gesture_index = 9

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Flip and convert to rgb
    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    # Display hand landmarks
    if result.multi_hand_landmarks:
        for hand in result.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

            # Predict gesture
            landmarks = []
            for lm in hand.landmark:
                landmarks.extend([lm.x, lm.y, lm.z])
           
            if len(landmarks) == 63:
                landmark_data = pd.DataFrame([landmarks], columns=[str(i) for i in range(63)])
                pred = model.predict(landmark_data)[0]
                recent_preds.append(pred)

                # Only update when its stable
                if len(recent_preds) == recent_preds.maxlen and len(set(recent_preds)) == 1:
                    stable_prediction = pred

    # Display gesture on frame
    if stable_prediction:
        cv2.putText(frame, f"Gesture: {stable_prediction}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
        # Transmit gesture to bt module, only if its different from previous recog [Uart2 used, don't want to overload FIFO]
        if stable_prediction in gesture_to_index:
            gesture_index = gesture_to_index[stable_prediction]

            if prev_gesture_index != gesture_index:
                bt.write(f"{gesture_index}".encode())
                print("Sent!")
                prev_gesture_index = gesture_index

    cv2.imshow("Hand Gesture Recognition", frame)
    # quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
