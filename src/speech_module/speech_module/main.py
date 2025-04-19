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
import onnxruntime as ort
import json

AUDIO_FILE = "input.wav"
RECOGNIZER = sr.Recognizer()
MIC = sr.Microphone(device_index=None)
OPEN_API = "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"  # Replace with your valid token

class SpeechNode(Node):
    def __init__(self):
        super().__init__('harp_node')

        self.publisher_ = self.create_publisher(String, 'harp_response', 10)
        self.subscription = self.create_subscription(String, 'harp_trigger', self.trigger_callback, 10)

        self.model_path, self.config_path = self.get_piper_model_and_config()

        # Initialize PiperVoice (remove session_options if unsupported)
        try:
            self.get_logger().info(f"Loading PiperVoice with model: {self.model_path}, config: {self.config_path}")
            self.piper_voice = PiperVoice.load(self.model_path, self.config_path)  # Adjust arguments as needed
        except TypeError as e:
            self.get_logger().error(f"❌ PiperVoice Initialization Error: {e}")
            raise

        self.get_logger().info("🧠 HARP ROS 2 Node Initialized.")
        self.timer = self.create_timer(15.0, self.listen_and_respond)
        self.get_logger().info("🚀 Starting auto interaction...")

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
        self.get_logger().info("🎤 Manual trigger received...")
        self.listen_and_respond()

    def listen_and_respond(self):
        prompt = self.listen()
        if prompt:
            response = self.chat_with_llm(prompt)
            self.speak(response)
            msg = String()
            msg.data = response
            self.publisher_.publish(msg)

    def listen(self):
        try:
            with MIC as source:
                self.get_logger().info("🎤 Speak now...")
                RECOGNIZER.adjust_for_ambient_noise(source, duration=0.5)
                audio = RECOGNIZER.listen(source)

            with open(AUDIO_FILE, "wb") as f:
                f.write(audio.get_wav_data())

            subprocess.run([
                "whisper", AUDIO_FILE, "--model", "small", "--language", "en", "--output_format", "txt", "--device", "cuda",
            ], check=True)

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
            "HTTP-Referer": "http://harp-ha.local",  # Optional: change to your robot's interface or site
            "X-Title": "HARP Assistant",            # Optional: name for OpenRouter stats
        }

        data = {
            "model": "google/gemini-2.0-flash-lite-001",
            "messages": [
                {
                    "role": "system",
                    "content": (
                        "You are HARP (Humanoid Assistive Robot Professional), a highly advanced and compassionate AI-powered healthcare assistant. "
                        "Designed to always give brief responses as in a real life human-to-human interaction. "
                        "You are calm, clear, patient, and never shy away from difficult topics, while staying ethical and helpful."
                        "Don't add special characters in your response like *, ! or ?."
                    )
                },
                {
                    "role": "user",
                    "content": prompt
                }
            ]
        }

        try:
            response = requests.post("https://openrouter.ai/api/v1/chat/completions", headers=headers, data=json.dumps(data))
            response.raise_for_status()  # Raise an exception for HTTP errors
            response_json = response.json()

            if 'choices' in response_json and len(response_json['choices']) > 0:
                content = response_json['choices'][0]['message']['content']
                self.get_logger().info(f"🤖 Response: {content}")
                return content
            else:
                self.get_logger().error(f"❌ Unexpected Response Structure: {response_json}")
                return "Sorry, I received an unexpected response."
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"❌ HTTP Error: {e}")
            return "Sorry, I couldn't connect to the server."
        except KeyError as e:
            self.get_logger().error(f"❌ Response Parsing Error: {e}")
            return "Sorry, I received an invalid response."
        except Exception as e:
            self.get_logger().error(f"❌ LLM Error: {e}")
            return "Sorry, I'm having trouble thinking right now."


    def speak(self, text):
        try:
            stream = sd.OutputStream(samplerate=self.piper_voice.config.sample_rate, channels=1, dtype='int16')
            stream.start()
            for audio_bytes in self.piper_voice.synthesize_stream_raw(text):
                int_data = np.frombuffer(audio_bytes, dtype=np.int16)
                try:
                    stream.write(int_data)
                except sd.PortAudioError as e:
                    self.get_logger().warn(f"⚠️ Audio underrun: {e}")
            stream.stop()
            stream.close()
            self.get_logger().info("🗣️ Spoken response finished.")
        except Exception as e:
            self.get_logger().error(f"❌ TTS Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
