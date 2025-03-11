# # # from speech_recognition import Microphone

# # # print("Available audio devices:")
# # # for i, mic_name in enumerate(Microphone.list_microphone_names()):
# # #     print(f"{i}: {mic_name}")

# import sounddevice as sd
# import numpy as np

# def test_mic(device_index=4):
#     duration = 5  # seconds
#     samplerate = 48000
#     print("Recording...")
#     audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='int16', device=device_index)
#     sd.wait()
#     print("Playback...")
#     sd.play(audio, samplerate)
#     sd.wait()

# # test_mic(4)  # Change to your actual device index

# import sounddevice as sd

# print(sd.query_devices())


# import sounddevice as sd

# devices = sd.query_devices()
# for i, device in enumerate(devices):
#     print(f"{i}: {device['name']} - Default sample rate: {device.get('default_samplerate', 'N/A')}")



# import sounddevice as sd
# print(sd.query_devices())
# print(sd.default.samplerate)

# import sounddevice as sd
# import numpy as np

# def test_mic():
#     duration = 5  # seconds
#     samplerate = 48000
#     print("Recording...")
#     audio = sd.rec(int(duration * samplerate), samplerate=samplerate, channels=1, dtype='int16')
#     sd.wait()
#     print("Playback...")
#     sd.play(audio, samplerate)
#     sd.wait()

# test_mic()

# import sounddevice as sd
# print(sd.query_devices())

# import pyaudio
# p = pyaudio.PyAudio()
# for i in range(p.get_device_count()):
#     print(p.get_device_info_by_index(i))

# import pyaudio

# p = pyaudio.PyAudio()

# # Use index 4 — the device with input channels
# stream = p.open(format=pyaudio.paInt16,
#                 channels=1,
#                 rate=48000,
#                 input=True,
#                 input_device_index=4,  # <- This is the key part
#                 frames_per_buffer=1024)

# print("Listening...")
# while True:
#     data = stream.read(1024)
#     # process audio data here...


# import pyaudio

# p = pyaudio.PyAudio()

# # Device index for the mic (HDA Intel PCH: CX11970 Analog)
# device_index = 4

# stream = p.open(format=pyaudio.paInt16,
#                 channels=1,
#                 rate=48000,
#                 input=True,
#                 input_device_index=device_index,
#                 frames_per_buffer=2048)  # Larger buffer size

# print("Listening...")
# try:
#     while True:
#         try:
#             data = stream.read(2048, exception_on_overflow=False)  # Avoid overflow errors
#             # Process audio data here...
#         except OSError as e:
#             print(f"Stream read error: {e}")
# except KeyboardInterrupt:
#     print("Stopping...")
# finally:
#     stream.stop_stream()
#     stream.close()
#     p.terminate()

# import pyaudio
# import numpy as np

# p = pyaudio.PyAudio()

# # Device index for the mic (HDA Intel PCH: CX11970 Analog)
# device_index = 4

# stream = p.open(format=pyaudio.paInt16,
#                 channels=1,
#                 rate=48000,
#                 input=True,
#                 frames_per_buffer=2048)

# print("Listening...")
# try:
#     while True:
#         try:
#             data = stream.read(2048, exception_on_overflow=False)
#             audio_data = np.frombuffer(data, dtype=np.int16)
#             volume = np.linalg.norm(audio_data)  # Simple volume measure
#             print(f"Volume: {volume:.2f}")
#         except OSError as e:
#             print(f"Stream read error: {e}")
# except KeyboardInterrupt:
#     print("Stopping...")
# finally:
#     stream.stop_stream()
#     stream.close()
#     p.terminate()

# import pyaudio
# import numpy as np

# p = pyaudio.PyAudio()

# # Try "default" device for now
# stream = p.open(format=pyaudio.paInt16,
#                 channels=1,
#                 rate=48000,
#                 input=True,
#                 frames_per_buffer=2048)

# print("Listening...")
# try:
#     while True:
#         try:
#             data = stream.read(2048, exception_on_overflow=False)
#             audio_data = np.frombuffer(data, dtype=np.int16)
#             volume = np.linalg.norm(audio_data)
#             if volume > 1000:  # Set a lower threshold for detecting sound
#                 print(f"Volume: {volume:.2f}")
#         except OSError as e:
#             print(f"Stream read error: {e}")
# except KeyboardInterrupt:
#     print("Stopping...")
# finally:
#     stream.stop_stream()
#     stream.close()
#     p.terminate()


# import pyaudio
# import numpy as np

# p = pyaudio.PyAudio()

# # Try "default" device for now
# stream = p.open(format=pyaudio.paInt16,
#                 channels=1,
#                 rate=48000,
#                 input=True,
#                 frames_per_buffer=2048)

# print("Listening...")
# try:
#     while True:
#         try:
#             data = stream.read(2048, exception_on_overflow=False)
#             audio_data = np.frombuffer(data, dtype=np.int16)
#             volume = np.linalg.norm(audio_data)
#             if volume > 1000:  # Set a lower threshold for detecting sound
#                 print(f"Volume: {volume:.2f}")
#         except OSError as e:
#             print(f"Stream read error: {e}")
# except KeyboardInterrupt:
#     print("Stopping...")
# finally:
#     stream.stop_stream()
#     stream.close()
#     p.terminate()

import pyaudio
import time

p = pyaudio.PyAudio()

# Use the device name from pactl output
device_name = "alsa_input.pci-0000_00_1f.3.analog-stereo"

# Find the device index
device_index = 4
for i in range(p.get_device_count()):
    info = p.get_device_info_by_index(i)
    if device_name in info.get('name', ''):
        device_index = i
        break

if device_index is None:
    print("Could not find the device!")
    exit(1)

stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=48000,
                input=True,
                input_device_index=device_index,
                frames_per_buffer=2048)

print("Listening...")

try:
    while True:
        data = stream.read(2048, exception_on_overflow=False)
        volume = max(abs(int.from_bytes(data[i:i+2], 'little', signed=True)) for i in range(0, len(data), 2))
        print(f"Volume: {volume}")
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Stopping...")
finally:
    stream.stop_stream()
    stream.close()
    p.terminate()
