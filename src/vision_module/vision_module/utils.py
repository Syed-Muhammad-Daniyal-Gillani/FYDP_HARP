import cv2
import os
from fer import FER

detector = FER()
video_in = None
# cascade_path = os.path.join(os.path.abspath(__file__), "haarcascade_frontalface_default.xml")

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
    Detects emotions from a face region.
    """
    if face_roi is None:
        return "No face detected"

    results = detector.detect_emotions(face_roi)
    if results:
        emotion, score = max(results[0]["emotions"].items(), key=lambda item: item[1])
        return emotion
    return "Neutral"  # Default if no emotion is detected