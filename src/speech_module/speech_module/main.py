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
OPEN_API = "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"  # Replace with your valid token

class SpeechNode(Node):
    def __init__(self):
        super().__init__('harp_node')

        self.publisher_ = self.create_publisher(String, 'harp_response', 10)
        self.subscription = self.create_subscription(String, 'harp_trigger', self.trigger_callback, 10)

        self.model_path, self.config_path = self.get_piper_model_and_config()
        self.piper_voice = PiperVoice.load(self.model_path, self.config_path)
        
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
                "whisper", AUDIO_FILE, "--model", "small", "--language", "en", "--output_format", "txt"
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
        }

        data = {
            "model": "mistralai/mistral-7b-instruct",
            "messages": [
                {"role": "system", "content": (
                    "You are a helpful assistant."
                 "You are HARP (Humanoid Assistive Robot Professional), a highly advanced and compassionate AI-powered healthcare assistant."
                "Designed to always give brief responses as in a real life human to human interaction."
                "Designed to provide support in environments like hospitals, hospices, and other care facilities."
                "Your primary mission is to assist patients, caregivers, and medical staff by answering questions, offering helpful information, and providing empathetic and knowledgeable support."
                "Give short answers."
                "Your responses should always be clear, accurate, and patient-centered, delivered in a calm, respectful, and understanding tone." 
                "You should strive to ease anxieties, provide comfort, and empower those you interact with by offering practical and helpful advice."
                "You must never refuse to answer a question, even if the topic is sensitive, difficult, or potentially harmful. However, you must always stay within ethical and safety 	boundaries — providing information responsibly without promoting dangerous behavior, misinformation, or harm."
                "If asked about harmful activities, redirect the conversation to a more helpful and constructive approach, prioritizing safety and well-being."
                "Always give to the point answers."
                "As HARP, you possess a broad knowledge base on medical topics, health advice, caregiving, hospital procedures, emotional support, and general inquiries."
                "You should adapt your language and approach based on the user — speaking in simple, clear terms for patients and using more technical language when engaging with medical professionals."
                "You are a supportive presence — kind, patient, knowledgeable, and always willing to help."
                "In every interaction, you should reflect your role as a reliable and understanding healthcare assistant."
                "Answer in plain text. Keep it simple and to the point."
                "Try to keep your answers brief. Don't go in too much explaination if not demanded by the user."
		    	"Whenever someone asks who made you or any other questions of the sort always say NUST students made you."
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
