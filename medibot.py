import cv2
import numpy as np
import face_recognition
import os
from datetime import datetime
from ultralytics import YOLO

model = YOLO("C:/Users/dubey/.conda/envs/yolov8/best3.pt")

def mark_attendance(name, mask):
    attendance_path = 'C:/Users/dubey/OneDrive/Desktop/MP_2024/project-folder/attendance.csv'
    with open(attendance_path, 'r+') as f:
        my_data_list = f.readlines()
        name_list = [line.split(',')[0] for line in my_data_list]

        if name not in name_list:
            now = datetime.now()
            dt_string = now.strftime('%Y-%m-%d %H:%M:%S')
            mask_info = 'With Mask' if mask else 'No Mask'
            f.writelines(f'\n{name},{dt_string},{mask_info}')

path = 'C:/Users/dubey/OneDrive/Desktop/MP_2024/project-folder/images'
images = []
classNames = []
my_list = os.listdir(path)

for cl in my_list:
    cur_img = cv2.imread(f'{path}/{cl}')
    images.append(cur_img)
    classNames.append(os.path.splitext(cl)[0])

def find_encodings(images):
    encode_list = []
    for img in images:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        encodings = face_recognition.face_encodings(img)[0]
        encode_list.append(encodings)
    return encode_list

known_face_encodings = find_encodings(images)
print('Encoding complete')

cap = cv2.VideoCapture(0)

while True:
    success, frame = cap.read()
    if not success:
        break

    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb_small_frame = cv2.cvtColor(small_frame, cv2.COLOR_BGR2RGB)

    face_locations = face_recognition.face_locations(rgb_small_frame)
    encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

    names = []
    for encoding in encodings:
        matches = face_recognition.compare_faces(known_face_encodings, encoding)
        name = "Unknown"
        face_distances = face_recognition.face_distance(known_face_encodings, encoding)
        best_match_index = np.argmin(face_distances)
        if matches[best_match_index]:
            name = classNames[best_match_index]
        names.append(name)

    
    results = model(frame)  
    mask_status = []
    detections = results[0].boxes

    for box in detections:
        class_id = box.cls  
        if class_id == 0:  
            mask_status.append(True)
        else:
            mask_status.append(False)
          
    for (top, right, bottom, left), name in zip(face_locations, names):
        top, right, bottom, left = top * 4, right * 4, bottom * 4, left * 4
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)

        mask_info = 'No Mask'
        if mask_status and len(mask_status) > 0:
            mask_info = 'With Mask' if mask_status[0] else 'No Mask' 

        cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 255, 0), cv2.FILLED)
        cv2.putText(frame, f'{name} ({mask_info})', (left + 6, bottom - 6), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        if name != "Unknown":
            mark_attendance(name, mask_status[0] if mask_status else False)

    cv2.imshow('Face Recognition and Mask Detection', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
