# harp_assistant/harp_node.py

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import requests
import sounddevice as sd
import numpy as np
import speech_recognition as sr
from piper.voice import PiperVoice

AUDIO_FILE = "input.wav"
RECOGNIZER = sr.Recognizer()
MIC = sr.Microphone(device_index=None)
OPEN_API = "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"

class speech_node(Node):
    def __init__(self):
        super().__init__('harp_node')

        self.publisher_ = self.create_publisher(String, 'harp_response', 10)
        self.subscription = self.create_subscription(String, 'harp_trigger', self.trigger_callback, 10)

        self.model_path, self.config_path = self.get_piper_model_and_config()
        self.piper_voice = PiperVoice.load(self.model_path, self.config_path)
        self.get_logger().info("🧠 HARP ROS 2 Node Initialized.")

        # ✅ Timer to auto-run once
        self.has_run = False
        self.timer = self.create_timer(1.5, self.run_once)

    def run_once(self):
        if not self.has_run:
            self.get_logger().info("🚀 Starting auto interaction...")
            self.listen_and_respond()
            self.has_run = True
            self.timer.cancel()

    def listen_and_respond(self):
        prompt = self.listen()
        if prompt:
            response = self.chat_with_llm(prompt)
            self.speak(response)
            msg = String()
            msg.data = response
            self.publisher_.publish(msg)

    def get_piper_model_and_config(self):
        model_dir = os.path.expanduser("~/piper_data")
        model_path = config_path = None

        for root, _, files in os.walk(model_dir):
            for file in files:
                if file.endswith(".onnx"):
                    model_path = os.path.join(root, file)
                    for f in files:
                        if f.endswith(".json"):
                            config_path = os.path.join(root, f)
                            break
                    break
        if not model_path or not config_path:
            raise FileNotFoundError("No Piper model/config found in ~/piper_data")
        return model_path, config_path

    def trigger_callback(self, msg):
        self.get_logger().info("🎤 Listening triggered...")
        self.listen_and_respond()

    def listen(self):
        try:
            with MIC as source:
                self.get_logger().info("🎤 Speak now...")
                RECOGNIZER.adjust_for_ambient_noise(source, duration=0.5)
                audio = RECOGNIZER.listen(source)

            with open(AUDIO_FILE, "wb") as f:
                f.write(audio.get_wav_data())

            subprocess.run([
                "whisper", AUDIO_FILE, "--model", "small", "--language", "en", "--output_format", "txt"
            ])

            with open("input.txt", "r") as f:
                text = f.read().strip()
            self.get_logger().info(f"📝 Transcribed: {text}")
            return text
        except Exception as e:
            self.get_logger().error(f"❌ Listening Error: {e}")
            return ""

    def chat_with_llm(self, prompt):
        headers = {
            "Authorization": f"Bearer {OPEN_API}",
            "Content-Type": "application/json",
        }

        data = {
            "model": "mistralai/mistral-7b-instruct",
            "messages": [
                {"role": "system", "content": (
                    "You are HARP (Humanoid Assistive Robot Professional), a healthcare assistant. "
                    "Be brief, empathetic, helpful, and always answer clearly. "
                    "Use simple language unless talking to medical professionals."
                )},
                {"role": "user", "content": prompt}
            ]
        }

        try:
            response = requests.post("https://openrouter.ai/api/v1/chat/completions", headers=headers, json=data)
            content = response.json()['choices'][0]['message']['content']
            self.get_logger().info(f"🤖 Response: {content}")
            return content
        except Exception as e:
            self.get_logger().error(f"❌ LLM Error: {e}")
            return "Sorry, I'm having trouble thinking right now."

    def speak(self, text):
        try:
            stream = sd.OutputStream(samplerate=self.piper_voice.config.sample_rate, channels=1, dtype='int16')
            stream.start()
            for audio_bytes in self.piper_voice.synthesize_stream_raw(text):
                int_data = np.frombuffer(audio_bytes, dtype=np.int16)
                stream.write(int_data)
            stream.stop()
            stream.close()
            self.get_logger().info("🗣️ Spoken response finished.")
        except Exception as e:
            self.get_logger().error(f"❌ TTS Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = speech_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
