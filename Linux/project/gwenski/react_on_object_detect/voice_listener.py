import speech_recognition as sr
import os
import sys
import signal

CMD_FILE = "/tmp/darwin_voice_cmd.txt"

# --- Safely close the microphone when killed ---
def handle_shutdown(signum, frame):
    print("\n[VOICE LISTENER] Shutting down safely, releasing microphone hardware...")
    sys.exit(0)

# Tell Python to run the shutdown function if it gets a kill command
signal.signal(signal.SIGINT, handle_shutdown)
signal.signal(signal.SIGTERM, handle_shutdown)
# -----------------------------------------------

def listen_loop():
    r = sr.Recognizer()
    
    # Sensitivity settings
    r.energy_threshold = 400 
    r.dynamic_energy_threshold = False 
    r.pause_threshold = 0.5 

    mic = sr.Microphone()  //device_index=3
    print("Ready to receive commands.")

    while True:
        try:
            with mic as source:
                print("\n--> LISTENING... (Speak now)")
                audio = r.listen(source, timeout=3, phrase_time_limit=3)
            
            print("    [Processing audio...]")
            text = r.recognize_google(audio).lower()
            
            if any(word in text for word in ["release", "drop", "start", "go"]):
                print(f">>> COMMAND TRIGGERED: {text} <<<")
                with open(CMD_FILE, "w") as f:
                    f.write(text)
                    f.flush()
                    os.fsync(f.fileno()) 
                    
        except sr.WaitTimeoutError:
            pass
        except sr.UnknownValueError:
            pass
        except sr.RequestError as e:
            print(f"    [API Error]: {e}")
        except Exception as e:
            print(f"    [Error]: {e}")

if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    listen_loop()