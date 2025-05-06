import time
import cv2
import threading
import queue
import numpy as np
from typing import List, NamedTuple
from tflite_runtime.interpreter import Interpreter
from ament_index_python.packages import get_package_share_directory
import os

resource_path = os.path.join(get_package_share_directory('vision_module'), 'resource')
model_path = os.path.join(resource_path, '2.tflite')
label_path = os.path.join(resource_path, 'kinetics600_label_map.txt')

# Visualization parameters
ROW_SIZE = 20  # pixels
LEFT_MARGIN = 24  # pixels
TEXT_COLOR = (0, 0, 255)  # red
FONT_SIZE = 1.3
FONT_THICKNESS = 2
MODEL_FPS = 24  # Target input FPS
MODEL_FPS_ERROR_RANGE = 0.1  # Acceptable fps error
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

class VideoClassifierOptions(NamedTuple):
    label_allow_list: List[str] = None
    label_deny_list: List[str] = None
    max_results: int = 1
    num_threads: int = 6
    score_threshold: float = 0.0

class Category(NamedTuple):
    label: str
    score: float

class VideoClassifier:
    MODEL_INPUT_SIGNATURE_NAME = 'image'
    MODEL_OUTPUT_SIGNATURE_NAME = 'logits'
    MODEL_INPUT_MEAN = 0
    MODEL_INPUT_STD = 255
    
    def __init__(self,
                 model_path: str,
                 label_file: str,
                 options: VideoClassifierOptions = VideoClassifierOptions()):
        # Load TFLite signature runner
        interpreter = Interpreter(model_path=model_path, num_threads=options.num_threads)
        self._signature = interpreter.get_signature_runner()
        self._options = options
        
        # Load labels
        with open(label_file, 'r') as f:
            self._label_list = [line.strip() for line in f]
            
        # Determine model input size
        input_details = self._signature.get_input_details()[self.MODEL_INPUT_SIGNATURE_NAME]
        shape = input_details['shape']
        # Remove batch dimensions (1's)
        real_shape = np.delete(shape, np.where(shape == 1))
        self._input_height, self._input_width = real_shape[0], real_shape[1]
        
        # Initialize internal LSTM-like states
        self.clear()
        
    def clear(self):
        # Initialize internal states to zeros
        states = {}
        for name, spec in self._signature.get_input_details().items():
            if name == self.MODEL_INPUT_SIGNATURE_NAME:
                continue
            states[name] = np.zeros(spec['shape'], dtype=spec['dtype'])
        self._internal_states = states
        
    def _preprocess(self, cam_frame: np.ndarray) -> np.ndarray:
        # Resize and normalize
        img = cv2.resize(cam_frame, (self._input_width, self._input_height))
        tensor = np.expand_dims(np.expand_dims(img, axis=0), axis=0)  # [1,1,H,W,3]
        return (tensor.astype(np.float32) - self.MODEL_INPUT_MEAN) / self.MODEL_INPUT_STD
        
    def classify(self, cam_frame: np.ndarray) -> List[Category]:
        # Preprocess cam_frame
        tensor = self._preprocess(cam_frame)
        
        # Run inference with internal states
        outputs = self._signature(**self._internal_states, image=tensor)
        logits = outputs.pop(self.MODEL_OUTPUT_SIGNATURE_NAME)
        self._internal_states = outputs
        
        # Softmax
        exp_logits = np.exp(np.squeeze(logits, axis=0))
        probs = exp_logits / np.sum(exp_logits)
        
        # Sort and wrap in Category
        idxs = np.argsort(-probs)
        results = [Category(label=self._label_list[i], score=float(probs[i])) for i in idxs]
        
        # Apply allow/deny and threshold
        if self._options.label_deny_list:
            results = [r for r in results if r.label not in self._options.label_deny_list]
        if self._options.label_allow_list:
            results = [r for r in results if r.label in self._options.label_allow_list]
        results = [r for r in results if r.score >= self._options.score_threshold]
        
        return results[:self._options.max_results]

# Initialize default options and classifier
options = VideoClassifierOptions(num_threads=4, max_results=1, score_threshold=0.4)
classifier = VideoClassifier(model_path, label_path, options)

def run(model_path: str, label_path: str, max_results: int,
        num_threads: int, camera_id: int, width: int, height: int) -> None:
    options = VideoClassifierOptions(num_threads=num_threads, max_results=max_results, score_threshold=0.4)
    classifier = VideoClassifier(model_path, label_path, options)
    
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    
    if not cap.isOpened():
        print("ERROR: Cannot open camera.")
        return
        
    categories = []
    fps = 0.0
    time_per_infer = 0.0
    last_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        frame = cv2.flip(frame, 1)
        now = time.time()
        
        if (now - last_time) * MODEL_FPS >= (1 - MODEL_FPS_ERROR_RANGE):
            fps = 1.0 / ((now - last_time) + 1e-8)
            last_time = now
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            start = time.time()
            categories = classifier.classify(rgb)
            time_per_infer = time.time() - start
            
            if all(category.score < 0.5 for category in categories):
                classifier.clear()
                
            for cat in categories:
                print(f"Detected: {cat.label} ({cat.score:.2f})")
                
        disp = frame.copy()
        cv2.putText(disp, f"FPS: {fps:.1f}", (LEFT_MARGIN, ROW_SIZE),
                    cv2.FONT_HERSHEY_PLAIN, FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)
        cv2.putText(disp, f"Infer: {int(time_per_infer * 1000)}ms",
                    (LEFT_MARGIN, 2 * ROW_SIZE), cv2.FONT_HERSHEY_PLAIN,
                    FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)
                    
        for idx, cat in enumerate(categories):
            cv2.putText(disp, f"{cat.label} ({cat.score:.2f})",
                        (LEFT_MARGIN, (idx + 3) * ROW_SIZE), cv2.FONT_HERSHEY_PLAIN,
                        FONT_SIZE, TEXT_COLOR, FONT_THICKNESS)
                        
        cv2.imshow('video_classification', disp)
        if cv2.waitKey(1) & 0xFF == 27:
            break
            
    cap.release()
    cv2.destroyAllWindows()

def classify_single_frame(frame):
    """
    Classify a single frame and return the classification results
    """
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    start = time.time()
    categories = classifier.classify(rgb)
    time_per_infer = time.time() - start
    
    return categories, time_per_infer