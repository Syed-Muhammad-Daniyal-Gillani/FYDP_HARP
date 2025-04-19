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
LEMONFOX_API_KEY = "3z53h2KvRgHAWoVJXUdpL3SuOx777PZt"

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
        """Handle a complete conversation after detecting the hotword."""
        while True:
            # Listen for user input
            prompt = self.listen()
            if not prompt:
                self.get_logger().info("🔇 No input detected. Ending conversation.")
                break  # End the conversation if no input is detected

            # Get a response from the LLM
            response = self.chat_with_llm(prompt)
            if not response:
                self.get_logger().info("🤖 No response generated. Ending conversation.")
                break  # End the conversation if no response is generated

            # Speak the response
            self.speak(response)

            # Publish the response
            msg = String()
            msg.data = response
            self.publisher_.publish(msg)

            # Check if the conversation should continue
            end_phrases = ["goodbye!", "goodbye","bye!", "bye", "allah hafiz", "allah hafiz!", "khuda hafiz", "khuda hafiz!"]
            if any(phrase in response.lower() for phrase in end_phrases):
                self.get_logger().info("👋 Ending conversation as per user request.")
                break  # End the conversation if the response indicates a goodbye

    def wait_for_hotword(self):
        """Wait for a hotword and respond with a prompt."""
        try:
            self.get_logger().info("🎤 Waiting for the hotword...")

            # Continuously listen for the hotword
            with MIC as source:
                hotword_detected = False  # Flag to track if the hotword is detected
                while not hotword_detected:
                    RECOGNIZER.adjust_for_ambient_noise(source, duration=0.5)
                    self.get_logger().info("🎤 Listening for the hotword...")
                    audio = RECOGNIZER.listen(source)

                    try:
                        # Convert audio to text using the Lemonfox Whisper API
                        with open(AUDIO_FILE, "wb") as f:
                            f.write(audio.get_wav_data())

                        self.get_logger().info("☁️ Uploading audio to Lemonfox Whisper API for hotword detection...")
                        with open(AUDIO_FILE, "rb") as f:
                            response = requests.post(
                                "https://api.lemonfox.ai/v1/audio/transcriptions",
                                headers={
                                    "Authorization": f"Bearer {LEMONFOX_API_KEY}"
                                },
                                files={
                                    "file": (AUDIO_FILE, f, "audio/wav")
                                }
                            )

                        response.raise_for_status()
                        detected_text = response.json().get("text", "").lower()
                        self.get_logger().info(f"📝 Detected: {detected_text}")

                        # Check if the hotword is in the detected text
                        hotwords = ["hi ", "hi!", "hey!","hello!", "hello", " hey","harp", "harp!"]  # Add your hotwords here
                        if any(hotword in detected_text for hotword in hotwords):
                            self.get_logger().info("🎤 Hotword detected!")
                            self.speak("How can I help you?")
                            hotword_detected = True  # Set the flag to exit the loop
                    except requests.exceptions.RequestException as e:
                        self.get_logger().error(f"❌ Whisper API Error during hotword detection: {e}")
                    except Exception as e:
                        self.get_logger().info("🔇 Could not detect hotword, retrying...")

        except Exception as e:
            self.get_logger().error(f"❌ Error in hotword detection: {e}")

    def listen(self):
        """Listen for user input after hotword detection."""
        try:
            self.wait_for_hotword()  # Wait for the hotword first

            # Start recording after hotword detection
            with MIC as source:
                RECOGNIZER.adjust_for_ambient_noise(source, duration=0.5)
                self.get_logger().info("🎤 Speak now...")
                audio = RECOGNIZER.listen(source)

            # Save the recorded audio to a file
            with open(AUDIO_FILE, "wb") as f:
                f.write(audio.get_wav_data())

            self.get_logger().info("☁️ Uploading audio to Lemonfox Whisper API for transcription...")
            with open(AUDIO_FILE, "rb") as f:
                response = requests.post(
                    "https://api.lemonfox.ai/v1/audio/transcriptions",
                    headers={
                        "Authorization": f"Bearer {LEMONFOX_API_KEY}"
                    },
                    files={
                        "file": (AUDIO_FILE, f, "audio/wav")
                    }
                )

            response.raise_for_status()
            transcription = response.json().get("text", "")
            self.get_logger().info(f"📝 Transcribed: {transcription}")
            return transcription

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"❌ Whisper API Error: {e}")
            return ""
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
                        "Always reply in English."
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
