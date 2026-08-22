import pyaudio
import audioop
import sys

def test_mic():
    p = pyaudio.PyAudio()
    
    # Using your exact Logitech USB Mic (index 3)
    try:
        stream = p.open(format=pyaudio.paInt16,
                        channels=1,
                        rate=16000,
                        input=True,
                        input_device_index=3,
                        frames_per_buffer=1024)
    except Exception as e:
        print(f"Failed to open microphone: {e}")
        sys.exit(1)

    print("\n🎤 MIC TEST ACTIVE! Speak normally. (Press Ctrl+C to stop)")
    print("-" * 50)

    try:
        while True:
            # Read a chunk of audio
            data = stream.read(1024, exception_on_overflow=False)
            
            # Calculate the RMS (volume energy)
            volume = audioop.rms(data, 2)
            
            # Create a visual bar that scales with the volume
            bar = "█" * int(volume / 50)
            
            # Print the exact number and the visual bar, overwriting the same line
            print(f"\rVolume: {volume:5} | {bar}".ljust(80), end="")
            
    except KeyboardInterrupt:
        print("\n\nClosing mic...")
        stream.stop_stream()
        stream.close()
        p.terminate()

if __name__ == "__main__":
    test_mic()