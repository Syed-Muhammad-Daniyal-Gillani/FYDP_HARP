import ollama
import whisper
import ChatTTS
import torchaudio
import torch
import sounddevice as sd
import numpy as np
import wave
from ollama import Client

# Record Audio Function
def record_audio(filename="input.wav", duration=5, samplerate=16000):
    print("Listening...")
    try:
        audio_data = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype=np.int16)
        sd.wait()

        if np.max(np.abs(audio_data)) < 500:
            print("No significant audio detected. Try speaking louder.")
            return None

        with wave.open(filename, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(samplerate)
            wf.writeframes(audio_data.tobytes())

        return filename
    except Exception as e:
        print(f"Error recording audio: {e}")
        return None

# Whisper STT
def transcribe_audio(filename="input.wav"):
    try:
        model = whisper.load_model("small", device="cpu")
        result = model.transcribe(filename)
        return result["text"].strip()
    except Exception as e:
        print(f"Error in speech recognition: {e}")
        return None

# Initialize ChatTTS once
chat_tts = ChatTTS.Chat()
chat_tts.load(compile=False)  # compile=True for speedup (optional)

# Text-to-Speech using ChatTTS
def speak(text):
    try:
        texts = [text]
        wavs = chat_tts.infer(texts)
        waveform_np = wavs[0]

        # Ensure shape [1, N] for mono audio (1 channel)
        if waveform_np.ndim == 1:
            waveform_np = waveform_np[np.newaxis, :]
        elif waveform_np.shape[0] > waveform_np.shape[1]:
            waveform_np = waveform_np.T

        waveform = torch.from_numpy(waveform_np).float()

        # Save the audio properly
        torchaudio.save("output.wav", waveform, 24000, encoding="PCM_S", bits_per_sample=16)

        # Play using sounddevice
        sd.play(wavs[0], 24000)
        sd.wait()
    except Exception as e:
        print(f"Error in TTS: {e}")


# Query the local LLM
def ask_llm(user_input):
    prompt = f"""
    User: {user_input}

    Assistant:
    """
    client = Client(
        host='http://localhost:11434',
        headers={'x-some-header': 'some-value'}
    )

    stream = ollama.chat(model="mistral", messages=[{"role": "user", "content": prompt}], stream=True)

    response_text = ""
    print("\nAssistant: ", end="")

    for chunk in stream:
        if "message" in chunk and "content" in chunk["message"]:
            text = chunk["message"]["content"]
            response_text += text
            print(text, end="", flush=True)

    print("\n")
    return response_text

# Main loop
def main():
    print("\nAssistant is active. Speak something...")

    while True:
        audio_file = record_audio()
        if not audio_file:
            print("Retrying...")
            continue

        user_input = transcribe_audio(audio_file)
        if not user_input:
            print("Could not understand audio. Try again.")
            continue

        print(f"\nYou: {user_input}")

        if user_input.lower() in ["quit", "exit", "stop"]:
            print("Goodbye!")
            break

        response = ask_llm(user_input)
        speak(response)


if __name__ == "__main__":
    main()
