import os
import json
import pyaudio
from vosk import Model, KaldiRecognizer

MODEL_PATH = "/home/darwin/darwin/Linux/project/gwenski/react_on_object_detect/vosk-model-en"
CMD_FILE = "/tmp/darwin_voice_cmd.txt"
MUTE_FLAG = "/tmp/darwin_speaking"

def listen_loop():
    if not os.path.exists(MODEL_PATH):
        print(f"[VOICE] FATAL: Vosk model not found at {MODEL_PATH}")
        return

    print("[VOICE] Loading English Neural STT Model...")
    model = Model(MODEL_PATH)
    
    # Changed "stop" to two-word triggers: "robot stop" and "emergency stop"
    grammar = '["hello", "hi", "hey", "begin", "wake up", "robot stop", "sleep", "shut down", "stand up", "center", "open left", "close left", "open right", "close right", "hold", "grab", "take", "release", "let go", "close both", "close hands", "go forward", "go backward", "step left", "step right", "[unk]"]'
    
    rec = KaldiRecognizer(model, 16000, grammar)

    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, 
                    channels=1, 
                    rate=16000, 
                    input=True, 
                    frames_per_buffer=8000)
    stream.start_stream()

    print("[VOICE] Microphones hot. Vosk is listening offline...")

    while True:
        try:
            data = stream.read(4000, exception_on_overflow=False)
            
            # --- MUTE CHECK ---
            # If the robot's speaker is playing, flush audio buffer and ignore
            if os.path.exists(MUTE_FLAG):
                rec.Reset()
                continue

            if rec.AcceptWaveform(data):
                result = json.loads(rec.Result())
                text = result.get("text", "")
                
                # Ignore empty strings and unknown noise tokens
                if text and text != "[unk]":
                    print(f"[VOICE] Heard: '{text}'")
                    with open(CMD_FILE, "w") as f:
                        f.write(text)
                        
        except Exception as e:
            print(f"[VOICE] Error in audio stream: {e}")

if __name__ == '__main__':
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    if os.path.exists(MUTE_FLAG):
        os.remove(MUTE_FLAG)
    listen_loop()