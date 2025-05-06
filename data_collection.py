import cv2
import mediapipe as mp
import csv
import os

# Gestures to be recognized
gestures = ['stop', 'go', 'three_sixty_degrees', 'left_90', 'right_90', 'forward']
# Create folder with hand landmarks
output_folder = 'vehicle_gesture_data'
os.makedirs(output_folder, exist_ok=True)

# Mediapipe for hand recog
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(max_num_hands=1)
mp_draw = mp.solutions.drawing_utils

# Recognition using cam
cap = cv2.VideoCapture(0)
current_label = gestures[5]
sample_count = 0
max_samples = 30  # Number of samples for each gesture

file = open(f'{output_folder}/{current_label}.csv', 'w', newline='')
writer = csv.writer(file)

print(f"Collecting for gesture: {current_label}")
print("'s' to save sample | 'q' to quit")

# Super loop
while True:
    ret, frame = cap.read()
    if not ret:
        break
    # Flip, convert to rgb
    frame = cv2.flip(frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    # Display hand landmarks
    if result.multi_hand_landmarks:
        for hand in result.multi_hand_landmarks:
            mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

    # Show frame
    cv2.putText(frame, f"Label: {current_label} | Count: {sample_count}", (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1)
    cv2.imshow("Collecting Gesture Data", frame)

    # Read key presses
    key = cv2.waitKey(1) & 0xFF

    # Save sample
    if key == ord('s') and current_label and result.multi_hand_landmarks:
        landmarks = []
        for lm in result.multi_hand_landmarks[0].landmark:
            landmarks.extend([lm.x, lm.y, lm.z])
        writer.writerow(landmarks)
        sample_count += 1
        print(f"Saved sample {sample_count} for '{current_label}'")

    # End when all samples collected
    if sample_count >= max_samples:
        print("Done collecting!")
        break

    # quit
    if key == ord('q'):
        break

file.close()
cap.release()
cv2.destroyAllWindows()
