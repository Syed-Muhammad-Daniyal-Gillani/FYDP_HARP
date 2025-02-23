import cv2
import os
from ament_index_python.packages import get_package_share_directory
# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
# os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
# import mediapipe as mp
import numpy as np
from deepface import DeepFace ## pip install deepface
video_in = None

def get_cascade_path():
    # Get the path to the installed package
    package_share_directory = get_package_share_directory('vision_module')
    # Construct the path to the haarcascade file inside the installed package
    cascade_path = os.path.join(package_share_directory, 'resource', 'haarcascade_frontalface_default.xml')
    return cascade_path


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
    faceCascade = cv2.CascadeClassifier(get_cascade_path())

    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Detect faces
    faces = faceCascade.detectMultiScale(imgGray, scaleFactor=1.2, minNeighbors=6, minSize=(50, 50))
    largest_face_area = 0
    normalized_x, normalized_y = 0.0, 0.0
    face_roi = None

    for (x, y, w, h) in faces:
        area = w * h
        if area > largest_face_area:  # Only consider the largest face
            largest_face_area = area
            normalized_x = ((x + w / 2) - frame_width / 2) / (frame_width / 2)
            normalized_y = ((y + h / 2) - frame_height / 2) / (frame_height / 2)
            face_roi = img[y:y + h, x:x + w]  # Crop the face region
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
    return img, (round(normalized_x, 2), round(normalized_y, 2)), largest_face_area, face_roi

def detect_emotion(face_roi):
    detected_emotion = DeepFace.analyze(face_roi, actions = ['emotion'], enforce_detection= False)
    return detected_emotion[0]["dominant_emotion"]