import time
import cv2
import threading
import queue
import numpy as np
from typing import List, NamedTuple
from tflite_runtime.interpreter import Interpreter

# Visualization parameters
_ROW_SIZE = 20  # pixels
_LEFT_MARGIN = 24  # pixels
_TEXT_COLOR = (0, 0, 255)  # red
_FONT_SIZE = 1.3
_FONT_THICKNESS = 2
_MODEL_FPS = 24  # Target input FPS
_MODEL_FPS_ERROR_RANGE = 0.1  # Acceptable fps error
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
class VideoClassifierOptions(NamedTuple):
    label_allow_list: List[str] = None
    label_deny_list: List[str] = None
    max_results: int = 5
    num_threads: int = 6
    score_threshold: float = 0.0

class Category(NamedTuple):
    label: str
    score: float

class VideoClassifier:
    _MODEL_INPUT_SIGNATURE_NAME = 'image'
    _MODEL_OUTPUT_SIGNATURE_NAME = 'logits'
    _MODEL_INPUT_MEAN = 0
    _MODEL_INPUT_STD = 255

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
        input_details = self._signature.get_input_details()[self._MODEL_INPUT_SIGNATURE_NAME]
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
            if name == self._MODEL_INPUT_SIGNATURE_NAME:
                continue
            states[name] = np.zeros(spec['shape'], dtype=spec['dtype'])
        self._internal_states = states

    def _preprocess(self, cam_frame: np.ndarray) -> np.ndarray:
        # Resize and normalize
        img = cv2.resize(cam_frame, (self._input_width, self._input_height))
        tensor = np.expand_dims(np.expand_dims(img, axis=0), axis=0)  # [1,1,H,W,3]
        return (tensor.astype(np.float32) - self._MODEL_INPUT_MEAN) / self._MODEL_INPUT_STD

    def classify(self, cam_frame: np.ndarray) -> List[Category]:
        # Preprocess cam_frame
        tensor = self._preprocess(cam_frame)
        # Run inference with internal states
        outputs = self._signature(**self._internal_states, image=tensor)
        logits = outputs.pop(self._MODEL_OUTPUT_SIGNATURE_NAME)
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


def run(model_path: str, label_path: str, max_results: int,
        num_threads: int, camera_frame):
    """Classify a single frame and return top label + score (no threading)."""

    # Setup classifier (consider moving this OUTSIDE the function and initializing once for speed)
    options = VideoClassifierOptions(num_threads=num_threads, max_results=max_results)
    classifier = VideoClassifier(model_path, label_path, options)

    # Flip and preprocess frame
    frame = cv2.flip(camera_frame, 1)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Classify frame
    categories = classifier.classify(rgb)

    # Pick top category (if any)
    if categories:
        top_cat = categories[0]
        label = top_cat.label
        score = top_cat.score

        # Optional: Debug print
        print(f"Detected: {label} ({score:.2f})")

        return label, score
    else:
        return "", 0.0
