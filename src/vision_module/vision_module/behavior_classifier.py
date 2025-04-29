# vision_module/behavior_recognition.py
import cv2
import numpy as np
import queue
import threading
import time
from typing import List, NamedTuple
from tflite_runtime.interpreter import Interpreter


class Category(NamedTuple):
    label: str
    score: float


class VideoClassifierOptions(NamedTuple):
    label_allow_list: List[str] = None
    label_deny_list: List[str] = None
    max_results: int = 2
    num_threads: int = 4
    score_threshold: float = 0.5


class BehaviorRecognizer:
    def __init__(self, model_path, label_path, camera_id=0, width=640, height=480):
        self.options = VideoClassifierOptions()
        self.classifier = VideoClassifier(model_path, label_path, self.options)
        self.camera_id = camera_id
        self.width = width
        self.height = height

        self.frame_queue = queue.Queue(maxsize=5)
        self.stop_event = threading.Event()
        self.behavior_label = "none"
        self.score = 0.0

        self.capture_thread = threading.Thread(target=self._capture_frames)
        self.infer_thread = threading.Thread(target=self._inference_loop)

        self.capture_thread.start()
        self.infer_thread.start()

    def get_behavior(self):
        return self.behavior_label, self.score

    def stop(self):
        self.stop_event.set()
        self.capture_thread.join()
        self.infer_thread.join()

    def _capture_frames(self):
        cap = cv2.VideoCapture(self.camera_id)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        while not self.stop_event.is_set():
            ret, frame = cap.read()
            if not ret:
                continue
            try:
                self.frame_queue.put_nowait(frame)
            except queue.Full:
                pass
        cap.release()

    def _inference_loop(self):
        last_time = time.time()
        while not self.stop_event.is_set():
            try:
                frame = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            now = time.time()
            if (now - last_time) > 0.041:  # ~24 FPS
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                categories = self.classifier.classify(rgb)
                if categories:
                    top = categories[0]
                    self.behavior_label = top.label
                    self.score = top.score
                else:
                    self.behavior_label = "none"
                    self.score = 0.0
                last_time = now


class VideoClassifier:
    _MODEL_INPUT_SIGNATURE_NAME = 'image'
    _MODEL_OUTPUT_SIGNATURE_NAME = 'logits'
    _MODEL_INPUT_MEAN = 0
    _MODEL_INPUT_STD = 255

    def __init__(self, model_path, label_file, options: VideoClassifierOptions):
        interpreter = Interpreter(model_path=model_path, num_threads=options.num_threads)
        self._signature = interpreter.get_signature_runner()
        self._options = options
        with open(label_file, 'r') as f:
            self._label_list = [line.strip() for line in f]
        input_details = self._signature.get_input_details()[self._MODEL_INPUT_SIGNATURE_NAME]
        shape = np.delete(input_details['shape'], np.where(input_details['shape'] == 1))
        self._input_height, self._input_width = shape[0], shape[1]
        self.clear()

    def clear(self):
        self._internal_states = {
            name: np.zeros(spec['shape'], dtype=spec['dtype'])
            for name, spec in self._signature.get_input_details().items()
            if name != self._MODEL_INPUT_SIGNATURE_NAME
        }

    def _preprocess(self, frame):
        img = cv2.resize(frame, (self._input_width, self._input_height))
        tensor = np.expand_dims(np.expand_dims(img, axis=0), axis=0)
        return (tensor.astype(np.float32) - self._MODEL_INPUT_MEAN) / self._MODEL_INPUT_STD

    def classify(self, frame):
        tensor = self._preprocess(frame)
        outputs = self._signature(**self._internal_states, image=tensor)
        logits = outputs.pop(self._MODEL_OUTPUT_SIGNATURE_NAME)
        self._internal_states = outputs
        probs = np.exp(np.squeeze(logits, axis=0))
        probs /= np.sum(probs)
        idxs = np.argsort(-probs)
        results = [Category(label=self._label_list[i], score=float(probs[i])) for i in idxs]
        results = [r for r in results if r.score >= self._options.score_threshold]
        return results[:self._options.max_results]
