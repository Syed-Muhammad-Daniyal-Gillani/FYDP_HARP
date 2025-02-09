import cv2
import os
import mediapipe as mp
import numpy as np

# Initialize MediaPipe FaceMesh
mp_face_mesh = mp.solutions.face_mesh
mp_drawing = mp.solutions.drawing_utils
face_mesh = mp_face_mesh.FaceMesh(static_image_mode=False, max_num_faces=1, refine_landmarks=True)
video_in = None

def initialize_camera(camera_index):
    global video_in
    if video_in is None:
        video_in = cv2.VideoCapture(camera_index)
        if not video_in.isOpened():
            raise ValueError(f"Camera at index {camera_index} could not be opened.")

def release_camera():
    global video_in
    if video_in is not None:
        video_in.release()
        video_in = None

def getVideo(width_, height_):
    success, img = video_in.read()
    img_in = cv2.resize(img, (width_, height_))
    return img_in

def detect_face(img, frame_width, frame_height):
    """
    Detects faces in an image and calculates normalized center coordinates.
    """
    faceCascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = faceCascade.detectMultiScale(imgGray, scaleFactor=1.2, minNeighbors=6, minSize=(50, 50))
    largest_face_area = 0
    normalized_x, normalized_y = 0.0, 0.0

    for (x, y, w, h) in faces:
        area = w * h
        if area > largest_face_area:  # Only consider the largest face
            largest_face_area = area
            normalized_x = ((x + w / 2) - frame_width / 2) / (frame_width / 2)
            normalized_y = ((y + h / 2) - frame_height / 2) / (frame_height / 2)
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return img, (round(normalized_x, 2), round(normalized_y, 2)), largest_face_area

def detect_emotion(face_roi):
    """
    Detects emotions based on facial landmarks using MediaPipe.
    """
    if face_roi is None:
        return "No face detected"

    # Convert to RGB for MediaPipe
    rgb_face = cv2.cvtColor(face_roi, cv2.COLOR_BGR2RGB)
    results = face_mesh.process(rgb_face)

    if results.multi_face_landmarks:
        for face_landmarks in results.multi_face_landmarks:
            landmarks = [(int(l.x * face_roi.shape[1]), int(l.y * face_roi.shape[0])) for l in face_landmarks.landmark]

            # Key facial points
            left_eyebrow = landmarks[70][1]  # Top of left eyebrow
            right_eyebrow = landmarks[295][1]  # Top of right eyebrow
            mouth_open = landmarks[13][1] - landmarks[14][1]  # Mouth openness
            left_lip = landmarks[61][1]  # Left lip corner
            right_lip = landmarks[291][1]  # Right lip corner
            lip_center = landmarks[0][1]  # Center of lips

            # Emotion detection logic
            if mouth_open > 10:  # Surprise
                return "Surprised"
            elif left_eyebrow < landmarks[159][1] and right_eyebrow < landmarks[386][1]:  # Angry
                return "Angry"
            elif left_lip < lip_center and right_lip < lip_center:  # Smile detection
                return "Happy"
            elif mouth_open < 2 and abs(left_lip - right_lip) < 3:  # No expression
                return "Neutral"
            else:
                return "Confused"

    return "Neutral"