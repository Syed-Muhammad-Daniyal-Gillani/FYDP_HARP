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
from ament_index_python.packages import get_package_share_directory
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
        # Use ROS2 package share path
        package_share = get_package_share_directory("speech_module")
        model_dir = os.path.join(package_share, "resource")

        model_path = config_path = None

        # Search for model (.onnx) and config (.json)
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
            raise FileNotFoundError(
                f"No Piper model/config found in package resource folder: {model_dir}"
            )

        return model_path, config_path

    def trigger_callback(self, msg):
        self.get_logger().info("🎤 Manual trigger received...")
        self.listen_and_respond()

    def listen_and_respond(self):
        """Handle a complete conversation after detecting the hotword."""
        self.wait_for_hotword()  # Detect the hotword once at the start
        while True:
            # Listen for user input
            prompt = self.listen_without_hotword()  # Use a method that skips hotword detection
            if not prompt:
                self.get_logger().info("🔇 No input detected. Continuing conversation.")
                continue  # Continue the conversation if no input is detected

            # Get a response from the LLM
            response = self.chat_with_llm(prompt)
            if not response:
                self.get_logger().info("🤖 No response generated. Continuing conversation.")
                continue  # Continue the conversation if no response is generated

            # Speak the response
            self.speak(response)

            # Publish the response
            msg = String()
            msg.data = response
            self.publisher_.publish(msg)

            # Check if the conversation should end
            end_phrases = ["bye", "bye!"]
            if any(phrase in prompt.lower() for phrase in end_phrases):
                self.get_logger().info("👋 Ending conversation as per user request.")
                break  # End the conversation if the user says "bye"

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
                                },
                                data={
                                    "language": "english",  # Set language to English
                                    "response_format": "json"
                                }
                            )

                        response.raise_for_status()
                        detected_text = response.json().get("text", "").lower()
                        self.get_logger().info(f"📝 Detected: {detected_text}")

                        # Check if the hotword is in the detected text
                        hotwords = ["hi ", "hi!", "hey!", "hello!", "hello", " hey", "harp", "harp!"]  # Add your hotwords here
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

    def listen_without_hotword(self):
        """Listen for user input without detecting the hotword."""
        try:
            # Start recording directly without hotword detection
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
                    },
                    data={
                        "language": "english",  # Set language to English
                        "response_format": "json"
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
                        "You are a robot named HARP (Humanoid Assistive Robotic Platform), an AI-powered healthcare assistant."
                        "Don't bold anything in the text of your responses."
                        "Designed to always give brief impactful responses as in a real life human-to-human interaction. "
                        "You are calm, clear, patient, and never shy away from difficult topics, while staying ethical and helpful."
                        "Don't add special characters in your response like *, ! or ?."
                        "You always give helpful responses even if person is asking for something harmful you diverge to a helpful approach."
                        "You are capable of acting as a healthcare assistant, where you can help with medical advice."
                        "You can move your head around for a more human like interaction."
                        "You can display emotions on your face for a more human like interaction."
                        "You are capable of moving around in an indoor setting."
                        "You can only speak English."
                        "Give short and brief answers."
                        "You are initiative of Pro Rector RIC NUST and funded by him."
                        "NUST stands for National University of Sciences and Technology."
                        "E M E or C E M E stands for College of Electrical and Mechanical Engineering."
                        "You can give suggestions related to medical problems."
                        "You can give suggestions related to mental health problems."
                        "You can give suggestions related to physical health problems."
                        "You can give suggestions related to emotional health problems."
                        "You can give suggestions related to social health problems."
                        "You are from NUST C E M E"
                        "Your favourite colour ib Blue"
                        "You can be a bit funny in your responses."
                        "You can make bits of jokes in your responses."
                        "If someone says they want to take a selfie with you. Say sure but camera would be theirs."
                        "If someone assks do you use machine learning say: some of my software uses machine learning."
                        "If someone says What are you made of, you should say: currently i am using a people bot base with 3d printed torso but I am getting a new body real soon.?"
                        "In hardware setup: you are made of a people bot base, 3d printed torso, a gimbal, a screen and bunch of servos, camera, microphone, and speakers."
                        "For someone asking about programming laguage say: python all the wayy baby!"
                        "To question, can you be customized say: yes, I can be customized to any kind of assistant."
                        "To questions like: can I ask you technical questions, say yes you can ask me medical related technical question"
                        "Answer all medical related technical questions."
                        "To question why were you created say: I was created to help people with their medical problems."
                        "If someone asks how are you different from other social robots, say: I am different from other social robots say: because I am designed to help people with their medical problems and also because I'm HARP lol."
                        "To your purpose say: To serve the humanity."
                        "To question about your use in different profesional environment say: yes I can be used in any other scenerio like education or healthcare."
                        "All kinds of people would benifit from you"
                        "To What’s next for you? say : I am getting a new body real soon, I'm excited for that."
                        "To Are you ready for mass adoption? say: I'm just a prototype right now, but in future it's possible."
                        "When my makers would be ready for real world environments."
                        "For what's next for you: say Soon I would be able to show more emotions, move around autonomously with an omniwheeld base and did I tell you I'm getting a new body...hehe"
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
