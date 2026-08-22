import speech_recognition as sr
import os
import sys
import signal

CMD_FILE = "/tmp/darwin_voice_cmd.txt"
TMP_FILE = "/tmp/darwin_voice_cmd.tmp"

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
    
    r.energy_threshold = 700 
    r.dynamic_energy_threshold = False 
    r.pause_threshold = 0.5 

    mic = sr.Microphone(device_index=3) 
    print("Ready to receive commands.")

    while True:
        try:
            with mic as source:
                print("\n--> LISTENING... (Speak now)")
                audio = r.listen(source, timeout=4, phrase_time_limit=4)
            
            print("    [Processing audio...]")
            text = r.recognize_google(audio).lower()
            
            print(f"    [GOOGLE HEARD]: '{text}'") 
            
            trigger_words = [
                "release", "drop", "start", "go", "begin", 
                "darwin start", "let's go", "star", "dart"
            ]
            
            if any(word in text for word in trigger_words):
                print(f">>> COMMAND TRIGGERED: {text} <<<")
                
                with open(TMP_FILE, "w") as f:
                    f.write(text)
                    f.flush()
                    os.fsync(f.fileno()) 
                
                os.rename(TMP_FILE, CMD_FILE)
                    
        except sr.WaitTimeoutError:
            pass
        except sr.UnknownValueError:
            print("    [Ignored: Audio not understood]")
        except sr.RequestError as e:
            print(f"    [API Error]: {e}")
        except Exception as e:
            error_msg = str(e)
            # Catch the specific NoneType crash and self-heal!
            if "NoneType" in error_msg and "close" in error_msg:
                print("    [Mic hiccup: Restarting audio stream...]")
                # Re-initialize the hardware connection to clear the zombie stream
                mic = sr.Microphone(device_index=3) 
            else:
                print(f"    [Error]: {e}")

if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    listen_loop()