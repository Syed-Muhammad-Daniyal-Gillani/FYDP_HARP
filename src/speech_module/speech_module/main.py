import os
import sounddevice as sd
import soundfile as sf
import speech_recognition as sr
import google.generativeai as genai
import tempfile
import numpy as np
from piper.voice import PiperVoice
import subprocess
import requests
# Configuration
# GEMINI_API_KEY = "AIzaSyDOpE4p5xyZPdKPtXjrJyPuCMrQ_P7w-2k"
# genai.configure(api_key=GEMINI_API_KEY)
# model = genai.GenerativeModel('gemini-1.5-pro')
open_api="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"
# Audio settings
AUDIO_FILE = "input.wav"
RECOGNIZER = sr.Recognizer()
MIC = sr.Microphone(device_index=None)  # Set device_index if needed

def get_piper_model_and_config():
    model_dir = os.path.expanduser("~/piper_data")
    model_path = None
    config_path = None

    for root, _, files in os.walk(model_dir):
        for file in files:
            if file.endswith(".onnx"):
                model_path = os.path.join(root, file)
                # Look for any .json file in the same directory
                for f in files:
                    if f.endswith(".json"):
                        config_path = os.path.join(root, f)
                        break
                break  # Stop after first valid model

    if not model_path or not config_path:
        raise FileNotFoundError("No valid Piper model (.onnx) and config (.json) found in ~/piper_data.")

    return model_path, config_path


model_path, config_path = get_piper_model_and_config()
piper_voice = PiperVoice.load(model_path, config_path)


# Speak function using Piper
def speak(text):
    print("🗣️ Generating speech with Piper...")
    try:
        stream = sd.OutputStream(samplerate=piper_voice.config.sample_rate, channels=1, dtype='int16')
        stream.start()
        for audio_bytes in piper_voice.synthesize_stream_raw(text):
            int_data = np.frombuffer(audio_bytes, dtype=np.int16)
            stream.write(int_data)
        stream.stop()
        stream.close()
        print("✅ Speech played successfully.")
    except Exception as e:
        print("❌ Piper TTS Error:", e)

# Listen and transcribe

def listen():
    try:
        print("🎤 Waiting for your voice...")
        with MIC as source:
            RECOGNIZER.adjust_for_ambient_noise(source, duration=0.5)
            audio = RECOGNIZER.listen(source)
        print("✅ Voice captured, saving...")

        with open(AUDIO_FILE, "wb") as f:
            f.write(audio.get_wav_data())

        # Run Whisper for transcription
        subprocess.run([
            "whisper", AUDIO_FILE, "--model", "small", "--language", "en", "--output_format", "txt"
        ])
        with open("input.txt", "r") as f:
            text = f.read().strip()
        print(f"📝 You said: {text}")
        return text
    except Exception as e:
        print("❌ Error during listening or transcription:", e)
        return ""

# LLM Chat with Gemini
def chat_with_llm(prompt):
    try:
        headers = {
            "Authorization": f"Bearer {open_api}",
            "Content-Type": "application/json",
        }

        data = {
            "model": "mistralai/mistral-7b-instruct",
            "messages": [
                {"role": "system", "content": "You are a helpful assistant."
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
			},
                {"role": "user", "content": prompt}
            ]
        }

        response = requests.post("https://openrouter.ai/api/v1/chat/completions", headers=headers, json=data)

        print(response.json()['choices'][0]['message']['content'])
        return response.json()['choices'][0]['message']['content']

    except Exception as e:
        print("❌ LLM Error:", e)
        return "I'm having trouble thinking right now."

# Main loop
if __name__ == "__main__":
    print("🧠 HARP Assistant Ready. Say something!")
    while True:
        prompt = listen()
        if prompt.lower() in ["exit", "quit", "stop"]:
            print("👋 Goodbye!")
            break
        if prompt:
            response = chat_with_llm(prompt)
            speak(response)
