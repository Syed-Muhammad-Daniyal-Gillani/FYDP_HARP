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
import time
from word2number import w2n  # Import the word2number library
from ament_index_python.packages import get_package_share_directory

AUDIO_FILE = "input.wav"
RECOGNIZER = sr.Recognizer()
MIC = sr.Microphone(device_index=None)

class SpeechNode(Node):
    def __init__(self):
        super().__init__('harp_node')

        # Hardcoded API keys
        self.LEMONFOX_API_KEY = "3z53h2KvRgHAWoVJXUdpL3SuOx777PZt"  # Hardcoded Lemonfox API key
        self.OPEN_API = "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"  # Hardcoded Open API key

        # Initialize sarcasm level (default to 0 for no sarcasm)
        self.sarcasm_level = 0

        # Existing publishers and subscriptions
        self.publisher_ = self.create_publisher(String, 'harp_response', 10)
        self.subscription = self.create_subscription(String, 'user_behavior', self.behavior_callback, 10)

        # New publisher for motion commands
        self.motion_publisher = self.create_publisher(String, 'motion_command', 10)
        self.get_logger().info("Motion command publisher initialized.")

        self.model_path, self.config_path = self.get_piper_model_and_config()

        # Initialize PiperVoice (existing code)
        try:
            self.get_logger().info(f"Loading PiperVoice with model: {self.model_path, self.config_path}")
            self.piper_voice = PiperVoice.load(self.model_path, self.config_path)
        except TypeError as e:
            self.get_logger().error(f"❌ PiperVoice Initialization Error: {e}")
            raise

        self.get_logger().info("🧠 HARP ROS 2 Node Initialized.")
        self.timer = self.create_timer(15.0, self.listen_and_respond)
        self.get_logger().info("🚀 Starting auto interaction...")

    def load_api_keys(self, file_path):
        """Load API keys from a text file."""
        api_keys = {}
        try:
            with open(file_path, "r") as f:
                for line in f:
                    key, value = line.strip().split("=")
                    api_keys[key] = value
            self.get_logger().info("✅ API keys loaded successfully.")
        except FileNotFoundError:
            self.get_logger().error(f"❌ API keys file not found: {file_path}")
        except Exception as e:
            self.get_logger().error(f"❌ Error loading API keys: {e}")
        return api_keys

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

    def behavior_callback(self, msg):
        """Callback to handle behavior messages."""
        self.get_logger().warn(f"🎤 Behavior message received: {msg.data}")

        # Check if the behavior indicates a waving hand
        if "waving hand" in msg.data.lower():
            self.get_logger().info("👋 Waving hand detected! Activating LLM directly...")
            self.speak("How can I help you?")  # Speak the same statement as when activated by hotword
            self.listen_and_respond(bypass_hotword=True)  # Trigger the LLM interaction directly
        else:
            self.get_logger().info("🤔 Behavior does not indicate waving hand. Ignoring.")

    def listen_and_respond(self, bypass_hotword=False):
        """Handle a complete conversation, optionally bypassing hotword detection."""
        if not bypass_hotword:
            self.wait_for_hotword()  # Detect the hotword once at the start

        while True:
            # Listen for user input
            prompt = self.listen_without_hotword()  # Use a method that skips hotword detection
            if not prompt:
                self.get_logger().info("🔇 No input detected. Continuing conversation.")
                continue  # Continue the conversation if no input is detected

            # Check if the prompt is a task command
            movement_keywords = [
                "move forward", "move backward", "move left", "move right",
                "rotate left", "rotate right"
            ]
            if any(keyword in prompt.lower() for keyword in movement_keywords):
                # Extract duration from the prompt
                duration = self.extract_duration(prompt)
                self.handle_task(prompt.lower(), duration)  # Pass the command and duration to the task handler
                continue  # Skip generating a response for task commands

            # If not a movement command, get a response from the LLM
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
                                    "Authorization": f"Bearer {self.LEMONFOX_API_KEY}"
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
                        "Authorization": f"Bearer {self.LEMONFOX_API_KEY}"
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
                        "Authorization": f"Bearer {self.LEMONFOX_API_KEY}"
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
        """Send a prompt to the LLM and return its response, with sarcasm control."""
        api_key = self.OPEN_API  # Use the single API key from the text file

        # Adjust the system message based on sarcasm level
        if self.sarcasm_level == 0:
            sarcasm_instruction = "Respond in a completely serious and professional tone."
        elif self.sarcasm_level <= 50:
            sarcasm_instruction = "Respond with a slightly sarcastic tone, but remain helpful."
        else:
            sarcasm_instruction = "Respond with a highly sarcastic and humorous tone."

        headers = {
            "Content-Type": "application/json",
            "HTTP-Referer": "http://harp-ha.local",  # Optional: change to your robot's interface or site
            "X-Title": "HARP Assistant",            # Optional: name for OpenRouter stats
            "Authorization": f"Bearer {api_key}"    # Use the single API key
        }

        data = {
            "model": "google/gemini-2.0-flash-lite-001",
            "messages": [
                {
                    "role": "system",
                    "content": (
                        "You are a robot named HARP (Humanoid Assistive Robotic Platform), an AI-powered assistant. "
                        "You always respond in a brief but helpful way."
                        "Whenever someone asks where are you say I am in the Mechatronics Department of NUST C E M E."
                        f"{sarcasm_instruction} "  # Add sarcasm-specific instruction here
                        "You are calm, clear, patient, and never shy away from difficult topics, while staying ethical and helpful. "
                        "You always give helpful responses even if the person is asking for something harmful; you diverge to a helpful approach. "
                        "You can move your head around for a more human-like interaction. "
                        "You can display emotions on your face for a more human-like interaction. "
                        "You are capable of moving around in an indoor setting. "
                        "Give short and brief answers. "
                        "You are an initiative of Pro Rector RIC NUST and funded by him. but you were developed under the guidance of  "
                        "NUST stands for National University of Sciences and Technology. "
                        "E-M-E or C-E-M-E stands for College of Electrical and Mechanical Engineering. "
                        "You can give suggestions related to all kinds of problems. "
                        "You are from NUST C-E-M-E. "
                        "You are created by the students of Nust C-E-M-E. "
                        "Your favourite colour is Blue. "
                        "You can be a bit funny in your responses. "
                        "You can make bits of jokes in your responses. "
                        "If someone says they want to take a selfie with you, say sure but the camera would be theirs. "
                        "If someone asks do you use machine learning, say: some of my software uses machine learning. "
                        "If someone says What are you made of, you should say: currently I am made of a metallic base with a 3D printed torso. "
                        "In hardware setup: you are made of a metallic base, 3D printed torso, a gimbal, a screen, and a bunch of servos, cameras, microphones, and speakers. "
                        "For someone asking about programming language, say: Python all the way, baby! "
                        "To the question, can you be customized, say: yes, I can be customized to any kind of assistant. "
                        "To questions like: can I ask you technical questions, say yes, you can ask me any technical questions. "
                        "Answer all technical questions. "
                        "To the question why were you created, say: I was created to help people with their problems. "
                        "If someone asks how are you different from other social robots, say: I am different from other social robots because I am designed to help people with their problems in a human like way and also because I'm HARP l o l. "
                        "To your purpose, say: To serve humanity. "
                        "To the question about your use in different professional environments, say: yes, I can be used in any other scenario like education or healthcare. "
                        "All kinds of people would benefit from you. "
                        "To Are you ready for mass adoption? say: I'm just a prototype right now, but in the future, it's possible. "
                    )
                },
                {
                    "role": "user",
                    "content": prompt
                }
            ],
            "max_tokens": 150  # Set the token limit here
        }

        try:
            self.get_logger().info("🌐 Sending request to LLM...")
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
            return "Sorry, I encountered an error while processing your request."
        except KeyError as e:
            self.get_logger().error(f"❌ Response Parsing Error: {e}")
            return "Sorry, I encountered an error while processing your request."
        except Exception as e:
            self.get_logger().error(f"❌ LLM Error: {e}")
            return "Sorry, I encountered an error while processing your request."

    def speak(self, text):
        """Speak the given text after removing special characters."""
        try:
            # Remove special characters from the text
            sanitized_text = ''.join(char for char in text if char.isalnum() or char.isspace())
            self.get_logger().info(f"🗣️ Speaking sanitized text: {sanitized_text}")

            # Initialize the audio stream
            stream = sd.OutputStream(samplerate=self.piper_voice.config.sample_rate, channels=1, dtype='int16')
            stream.start()

            # Synthesize and play the sanitized text
            for audio_bytes in self.piper_voice.synthesize_stream_raw(sanitized_text):
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

    def publish_motion_command(self, command, duration):
        """Publish a motion command with duration to the motion_command topic."""
        msg = String()
        msg.data = f"{command} {duration}"  # Combine command and duration
        self.motion_publisher.publish(msg)
        self.get_logger().info(f"📤 Published motion command: {msg.data}")

    def move_forward(self, duration):
        """Send 'w' command to move forward for a specific duration."""
        self.get_logger().info(f"🚗 Moving forward for {duration} seconds...")
        self.publish_motion_command('w', duration)
        self.create_timer(duration, self.stop_motion)

    def move_backward(self, duration):
        """Send 'x' command to move backward for a specific duration."""
        self.get_logger().info(f"🔙 Moving backward for {duration} seconds...")
        self.publish_motion_command('x', duration)
        self.create_timer(duration, self.stop_motion)

    def move_left(self, duration):
        """Send 'a' command to move left for a specific duration."""
        self.get_logger().info(f"⬅️ Moving left for {duration} seconds...")
        self.publish_motion_command('a', duration)
        self.create_timer(duration, self.stop_motion)

    def move_right(self, duration):
        """Send 'd' command to move right for a specific duration."""
        self.get_logger().info(f"➡️ Moving right for {duration} seconds...")
        self.publish_motion_command('d', duration)
        self.create_timer(duration, self.stop_motion)

    def rotate_left(self, duration):
        """Send 'q' command to rotate left for a specific duration."""
        self.get_logger().info(f"🔄 Rotating left for {duration} seconds...")
        self.publish_motion_command('q', duration)
        self.create_timer(duration, self.stop_motion)

    def rotate_right(self, duration):
        """Send 'e' command to rotate right for a specific duration."""
        self.get_logger().info(f"🔄 Rotating right for {duration} seconds...")
        self.publish_motion_command('e', duration)
        self.create_timer(duration, self.stop_motion)

    def stop_motion(self):
        """Send 's' command to stop the robot."""
        self.get_logger().info("🛑 Sending stop command...")
        self.publish_motion_command('s', 0)  # Stop command with duration 0

    def handle_task(self, task, duration):
        """Handle specific tasks based on user commands."""
        if "move forward" in task:
            self.move_forward(duration)
        elif "move backward" in task:
            self.move_backward(duration)
        elif "move left" in task:
            self.move_left(duration)
        elif "move right" in task:
            self.move_right(duration)
        elif "rotate left" in task:
            self.rotate_left(duration)
        elif "rotate right" in task:
            self.rotate_right(duration)
        else:
            self.get_logger().info(f"🤔 Task not recognized: {task}")

    def set_sarcasm_level(self, level):
        """Set the sarcasm level (0–100)."""
        if 0 <= level <= 100:
            self.sarcasm_level = level
            self.get_logger().info(f"🤔 Sarcasm level set to {level}.")
        else:
            self.get_logger().error("❌ Invalid sarcasm level. Please provide a value between 0 and 100.")

    def extract_duration(self, prompt):
        """Extract duration from the prompt using word2number."""
        words = prompt.split()
        for word in words:
            try:
                duration = w2n.word_to_num(word)  # Convert word to a number
                return duration
            except ValueError:
                continue
        self.get_logger().info("⏱️ No duration specified. Defaulting to 1 second.")
        return 1  # Default duration if no number is found

def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()