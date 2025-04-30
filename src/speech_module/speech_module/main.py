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
        self.wait_for_hotword()  # Detect the hotword once at the start
        while True:
            # Listen for user input
            prompt = self.listen_without_hotword()  # Use a method that skips hotword detection
            if not prompt:
                self.get_logger().info("🔇 No input detected. Continuing conversation.")
                continue  # Continue the conversation if no input is detected

            # Check if the prompt is a task command
            if any(keyword in prompt.lower() for keyword in [
                "move straight", "move right", "move left", "move backwards",
                "strafe front right diagonal", "strafe front left diagonal",
                "strafe back left diagonal", "strafe back right diagonal", "turn backwards"
            ]):
                self.handle_task(prompt.lower())  # Pass the command to the task handler
                continue  # Skip generating a response for task commands

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
        """Send a prompt to the LLM and return its response, with fallback for multiple API keys."""
        api_keys = [
            "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"  # Replace with your valid keys
        ]

        headers_template = {
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
                        "You are a robot named HARP (Humanoid Assistive Robotic Platform), an AI-powered assistant."
                        "Designed to always give brief impactful responses as in a real life human-to-human interaction. "
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
            ]
        }

        for api_key in api_keys:
            headers = headers_template.copy()
            headers["Authorization"] = f"Bearer {api_key}"

            try:
                self.get_logger().info(f"🌐 Attempting request with API key: {api_key[:10]}...")  # Log partial key for debugging
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
                self.get_logger().error(f"❌ HTTP Error with API key {api_key[:10]}: {e}")
                continue  # Try the next API key
            except KeyError as e:
                self.get_logger().error(f"❌ Response Parsing Error with API key {api_key[:10]}: {e}")
                continue  # Try the next API key
            except Exception as e:
                self.get_logger().error(f"❌ LLM Error with API key {api_key[:10]}: {e}")
                continue  # Try the next API key

        # If all API keys fail
        self.get_logger().error("❌ All API keys failed.")
        return "Sorry, I'm having trouble thinking right now."

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

    def move_straight(self, duration):
        """Simulate moving the robot straight."""
        self.get_logger().info(f"🚗 Moving straight for {duration} seconds...")
        # Add motor control logic here to move the robot straight
        time.sleep(duration)  # Simulate the movement duration
        self.get_logger().info("✅ Finished moving straight.")

    def move_right(self, duration):
        """Simulate moving the robot to the right."""
        self.get_logger().info(f"➡️ Moving right for {duration} seconds...")
        # Add motor control logic here to move the robot to the right
        time.sleep(duration)  # Simulate the movement duration
        self.get_logger().info("✅ Finished moving right.")

    def move_left(self, duration):
        """Simulate moving the robot to the left."""
        self.get_logger().info(f"⬅️ Moving left for {duration} seconds...")
        # Add motor control logic here to move the robot to the left
        time.sleep(duration)  # Simulate the movement duration
        self.get_logger().info("✅ Finished moving left.")

    def move_backwards(self, duration):
        """Simulate moving the robot backwards."""
        self.get_logger().info(f"🔙 Moving backwards for {duration} seconds...")
        # Add motor control logic here to move the robot backwards
        time.sleep(duration)  # Simulate the movement duration
        self.get_logger().info("✅ Finished moving backwards.")

    def strafe_front_right_diagonal(self, duration):
        """Simulate strafing front-right diagonally."""
        self.get_logger().info(f"↗️ Strafing front-right for {duration} seconds...")
        # Add motor control logic here to strafe front-right diagonally
        time.sleep(duration)  # Simulate the movement duration
        self.get_logger().info("✅ Finished strafing front-right.")

    def strafe_front_left_diagonal(self, duration):
        """Simulate strafing front-left diagonally."""
        self.get_logger().info(f"↖️ Strafing front-left for {duration} seconds...")
        # Add motor control logic here to strafe front-left diagonally
        time.sleep(duration)  # Simulate the movement duration
        self.get_logger().info("✅ Finished strafing front-left.")

    def strafe_back_left_diagonal(self, duration):
        """Simulate strafing back-left diagonally."""
        self.get_logger().info(f"↙️ Strafing back-left for {duration} seconds...")
        # Add motor control logic here to strafe back-left diagonally
        time.sleep(duration)  # Simulate the movement duration
        self.get_logger().info("✅ Finished strafing back-left.")

    def strafe_back_right_diagonal(self, duration):
        """Simulate strafing back-right diagonally."""
        self.get_logger().info(f"↘️ Strafing back-right for {duration} seconds...")
        # Add motor control logic here to strafe back-right diagonally
        time.sleep(duration)  # Simulate the movement duration
        self.get_logger().info("✅ Finished strafing back-right.")

    def turn_backwards(self, duration):
        """Simulate turning backwards."""
        self.get_logger().info(f"🔄 Turning backwards for {duration} seconds...")
        # Add motor control logic here to turn the robot backwards
        time.sleep(duration)  # Simulate the movement duration
        self.get_logger().info("✅ Finished turning backwards.")

    def handle_task(self, task):
        """Handle specific tasks based on user commands."""
        try:
            # Extract the duration from the command
            duration_text = task.split("for")[1].strip().split()[0]  # Extract the duration part
            try:
                # Try converting spelled-out numbers to digits
                duration = w2n.word_to_num(duration_text)
            except ValueError:
                # If conversion fails, assume it's already a digit
                duration = int(duration_text)
        except (IndexError, ValueError):
            self.get_logger().error("❌ Invalid duration specified in the command.")
            return

        # Match the task to the corresponding movement function
        if "move straight" in task:
            self.move_straight(duration)
        elif "move right" in task:
            self.move_right(duration)
        elif "move left" in task:
            self.move_left(duration)
        elif "move backwards" in task:
            self.move_backwards(duration)
        elif "strafe front right diagonal" in task:
            self.strafe_front_right_diagonal(duration)
        elif "strafe front left diagonal" in task:
            self.strafe_front_left_diagonal(duration)
        elif "strafe back left diagonal" in task:
            self.strafe_back_left_diagonal(duration)
        elif "strafe back right diagonal" in task:
            self.strafe_back_right_diagonal(duration)
        elif "turn backwards" in task:
            self.turn_backwards(duration)
        else:
            self.get_logger().info(f"🤔 Task not recognized: {task}")

def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()