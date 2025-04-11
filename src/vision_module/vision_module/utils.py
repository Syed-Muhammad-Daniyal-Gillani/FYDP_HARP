import cv2
import os
from ament_index_python.packages import get_package_share_directory
# os.environ["CUDA_VISIBLE_DEVICES"] = "-1"
# os.environ["TF_CPP_MIN_LOG_LEVEL"] = "2"
# import mediapipe as mp
import numpy as np
import glob
from deepface import DeepFace ## pip install deepface
video_in = None

def get_cascade_path():
    # Get the path to the installed package
    package_share_directory = get_package_share_directory('vision_module')
    # Construct the path to the haarcascade file inside the installed package
    cascade_path = os.path.join(package_share_directory, 'resource', 'haarcascade_frontalface_default.xml')
    return cascade_path


def initialize_camera():
    global video_in
    if video_in is None:
        video_in = get_camera()
        if not video_in.isOpened():
            raise ValueError(f"Camera could not be opened.")

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

def get_camera():
    # Use pixel format detection to find the correct camera
    video_device = get_video_device()
    
    if video_device:
        return cv2.VideoCapture(video_device)
    else:
        return cv2.VideoCapture(0)  # Fallback to internal cam (video0)

def get_video_device():
    # Search for all video devices (usually /dev/video*)
    video_devices = glob.glob('/dev/video*')

    # Exclude the internal camera (usually /dev/video0)
    video_devices = [dev for dev in video_devices if dev != '/dev/video0']

    for video_device in video_devices:
        cap = cv2.VideoCapture(video_device)

        if cap.isOpened():
            # Check the pixel format (FOURCC)
            pixel_format = int(cap.get(cv2.CAP_PROP_FOURCC))
            pixel_format_str = fourcc_to_str(pixel_format)
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            
            # Print the format and resolution to help identify the camera
            print(f"Device: {video_device}, Format: {pixel_format_str}, Resolution: {width}x{height}")

            # Check if the pixel format matches your requirements (e.g., YUYV or BGR)
            if pixel_format_str in ['YUYV', 'MJPG', 'BGR']:
                print(f"Selected camera: {video_device}")
                cap.release()  # Release after checking
                return video_device  # Return the valid video device

            cap.release()  # Release if it's not the correct format

    return None  # If no valid device found, return None

def fourcc_to_str(fourcc):
    """
    Converts a FOURCC code to a string.
    """
    return ''.join([chr((fourcc >> 8 * i) & 0xFF) for i in range(4)])