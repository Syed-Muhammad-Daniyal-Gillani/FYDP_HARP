#!/usr/bin/env python3
# Unified threaded video classification script

import argparse
import sys
import time
import cv2
import threading
import queue
import numpy as np
from typing import List, NamedTuple

# Try to import TFLite Interpreter
try:
    from ai_edge_litert.interpreter import Interpreter
except ImportError:
    from tflite_runtime.interpreter import Interpreter

# Visualization parameters
_ROW_SIZE = 20  # pixels
_LEFT_MARGIN = 24  # pixels
_TEXT_COLOR = (0, 0, 255)  # red
_FONT_SIZE = 1.3
_FONT_THICKNESS = 2
_MODEL_FPS = 24  # Target input FPS
_MODEL_FPS_ERROR_RANGE = 0.1  # Acceptable fps error

class VideoClassifierOptions(NamedTuple):
    label_allow_list: List[str] = None
    label_deny_list: List[str] = None
    max_results: int = 5
    num_threads: int = 4
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

    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        # Resize and normalize
        img = cv2.resize(frame, (self._input_width, self._input_height))
        tensor = np.expand_dims(np.expand_dims(img, axis=0), axis=0)  # [1,1,H,W,3]
        return (tensor.astype(np.float32) - self._MODEL_INPUT_MEAN) / self._MODEL_INPUT_STD

    def classify(self, frame: np.ndarray) -> List[Category]:
        # Preprocess frame
        tensor = self._preprocess(frame)
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
        num_threads: int, camera_id: int, width: int, height: int) -> None:
    # Setup classifier
    options = VideoClassifierOptions(num_threads=num_threads, max_results=max_results)
    classifier = VideoClassifier(model_path, label_path, options)

    # Thread-safe queue for frames
    frame_queue = queue.Queue(maxsize=5)
    stop_event = threading.Event()

    # Shared variables
    categories = []
    fps = 0.0
    time_per_infer = 0.0
    last_time = time.time()

    def capture_frames():
        cap = cv2.VideoCapture(camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not cap.isOpened():
            print("ERROR: Cannot open camera.")
            stop_event.set()
            return
        while not stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                continue
            frame = cv2.flip(frame, 1)
            # Non-blocking enqueue (drop if full)
            try:
                frame_queue.put_nowait(frame)
            except queue.Full:
                pass

            # Display with latest inference info
            disp = frame.copy()
            cv2.putText(disp, f"FPS: {fps:.1f}", (_LEFT_MARGIN, _ROW_SIZE),
                        cv2.FONT_HERSHEY_PLAIN, _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
            cv2.putText(disp, f"Infer: {int(time_per_infer*1000)}ms",
                        (_LEFT_MARGIN, 2*_ROW_SIZE), cv2.FONT_HERSHEY_PLAIN,
                        _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
            for idx, cat in enumerate(categories):
                cv2.putText(disp, f"{cat.label} ({cat.score:.2f})",
                            (_LEFT_MARGIN, (idx+3)*_ROW_SIZE), cv2.FONT_HERSHEY_PLAIN,
                            _FONT_SIZE, _TEXT_COLOR, _FONT_THICKNESS)
            cv2.imshow('video_classification', disp)
            if cv2.waitKey(1) & 0xFF == 27:
                stop_event.set()
                break
        cap.release()
        cv2.destroyAllWindows()

    def inference_loop():
        nonlocal categories, fps, time_per_infer, last_time
        while not stop_event.is_set():
            try:
                frame = frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            now = time.time()
            if (now - last_time) * _MODEL_FPS >= (1 - _MODEL_FPS_ERROR_RANGE):
                fps = 1.0 / ((now - last_time) + 1e-8)
                last_time = now
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                start = time.time()
                categories = classifier.classify(rgb)
                if all(category.score < 0.5 for category in categories):
                    classifier.clear()
                time_per_infer = time.time() - start
                for cat in categories:
                    print(f"Detected: {cat.label} ({cat.score:.2f})") #return this shit

    # Start threads
    t1 = threading.Thread(target=capture_frames)
    t2 = threading.Thread(target=inference_loop)
    t1.start()
    t2.start()
    t1.join()
    t2.join()


def main():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--model', default='2.tflite', help='TFLite model path')
    parser.add_argument('--label', default='kinetics600_label_map.txt', help='Label file path')
    parser.add_argument('--maxResults', type=int, default=1, help='Max results')
    parser.add_argument('--numThreads', type=int, default=4, help='Interpreter threads')
    parser.add_argument('--cameraId', type=int, default=0, help='Camera ID')
    parser.add_argument('--frameWidth', type=int, default=640, help='Frame width')
    parser.add_argument('--frameHeight', type=int, default=480, help='Frame height')
    args = parser.parse_args()

    run(args.model, args.label, args.maxResults,
        args.numThreads, args.cameraId, args.frameWidth, args.frameHeight)

if __name__ == '__main__':
    main()
