import ollama
# import pdfplumber
import whisper
import pyttsx3
import sounddevice as sd
import numpy as np
import wave
from ollama import Client
# # Load Personality from PDF
# pdf_path = "/home/mak/Documents/local_llm_code/personality.pdf"
# def load_personality_from_pdf(pdf_path):
#     """Extracts text from the Personality.pdf file."""
#     try:
#         with pdfplumber.open(pdf_path) as pdf:
#             text = "\n".join([page.extract_text() for page in pdf.pages if page.extract_text()])
#         return text.strip()
#     except Exception as e:
#         print(f"Error loading personality document: {e}")
#         return ""

# Record Audio Function
def record_audio(filename="input.wav", duration=5, samplerate=16000):
    """Records audio from the microphone and saves it to a WAV file."""
    print("Listening...")
    try:
        audio_data = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype=np.int16)
        sd.wait()  # Wait until recording is finished

        # Check if the audio is silent (all zeros)
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


# Whisper Speech-to-Text Function
def transcribe_audio(filename="input.wav"):
    """Transcribes recorded speech using OpenAI's Whisper model."""
    try:
        # model = whisper.load_model("small", device="cuda") uncomment for Cuda enabled
        model = whisper.load_model("small", device="cpu")
	 # Use 'base', 'small', 'medium', or 'large'
        result = model.transcribe(filename)
        return result["text"].strip()
    except Exception as e:
        print(f"Error in speech recognition: {e}")
        return None

# Text-to-Speech Function
def speak(text):
    """Converts text to speech and plays it."""
    engine = pyttsx3.init()
    engine.setProperty("rate", 170)  # Adjust speech speed
    engine.say(text)
    engine.runAndWait()

# LLM Query Function
# def ask_llm(user_input, personality_text): uncomment for Personality
def ask_llm(user_input):
    """Queries the local Mistral LLM with a personality-based prompt."""
    #uncomment for Personality
    # prompt = f"""
    # {personality_text}

    # User: {user_input}
    
    # Assistant:
    # """
    prompt = f"""
    User: {user_input}
    
    Assistant:
    """
    client = Client(
    host='http://localhost:11434',
    headers={'x-some-header': 'some-value'}
    )
    
    # Stream response from Ollama
    stream = ollama.chat(model="mistral", messages=[{"role": "user", "content": prompt}], stream=True)
    
    response_text = ""
    print("\nAssistant: ", end="")

    for chunk in stream:
        if "message" in chunk and "content" in chunk["message"]:
            text = chunk["message"]["content"]
            response_text += text
            print(text, end="", flush=True)  # Live output
    
    print("\n")  # Ensure newline after response
    return response_text

# Main Loop
def main():
    # pdf_path = "/home/mak/Documents/local_llm_code/personality.pdf"
    # personality_text = load_personality_from_pdf(pdf_path)

    # if not personality_text:
    #     print("Failed to load personality profile. Exiting...")
    #     return

    print("\nAssistant is active. Speak something...")

    while True:
        audio_file = record_audio()
        if not audio_file:
            print("Retrying...")
            continue  # Try recording again if no valid audio

        user_input = transcribe_audio(audio_file)
        if not user_input:
            print("Could not understand audio. Try again.")
            continue  # Retry if no transcription

        print(f"\nYou: {user_input}")

        if user_input.lower() in ["quit", "exit", "stop"]:
            print("Goodbye!")
            break

        # response = ask_llm(user_input, personality_text)
        response = ask_llm(user_input)
        speak(response)  # Convert response to speech


if __name__ == "__main__":
    main()
