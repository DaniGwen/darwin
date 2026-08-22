import speech_recognition as sr
import os
import sys
import signal
import time

CMD_FILE = "/tmp/darwin_voice_cmd.txt"
TMP_FILE = "/tmp/darwin_voice_cmd.tmp"

# Flag to keep the loop running
keep_running = True

# --- Graceful Shutdown Handler ---
def handle_shutdown(signum, frame):
    global keep_running
    print("\n[VOICE LISTENER] Caught stop signal! Safely closing ALSA stream...")
    # Setting this to False tells the inner loop to stop, 
    # which cleanly releases the "with mic as source:" block!
    keep_running = False

signal.signal(signal.SIGINT, handle_shutdown)
signal.signal(signal.SIGTERM, handle_shutdown)
# ---------------------------------

def listen_loop():
    global keep_running
    r = sr.Recognizer()
    
    # Start at 600 (above room noise, below servo noise)
    r.energy_threshold = 600
    # Let it slowly adapt to the room over time
    r.dynamic_energy_threshold = True 
    r.pause_threshold = 0.5 

    print("Ready to receive commands.")

    try:
        # Using native sample rate for crystal clear AI understanding
        mic = sr.Microphone(device_index=3)
        
        with mic as source:
            print("    [Hardware connected and locked successfully]")
            
            while keep_running:
                try:
                    print("\n--> LISTENING... (Speak now)")
                    # Shortened timeout to 2 seconds so it checks keep_running frequently
                    audio = r.listen(source, timeout=2, phrase_time_limit=4)
                
                    # If Ctrl+C was pressed while we were listening, exit immediately!
                    if not keep_running:
                        break

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
                    pass # Timeout just lets the loop spin to check keep_running
                except sr.UnknownValueError:
                    print("    [Ignored: Audio not understood]")
                except sr.RequestError as e:
                    print(f"    [API Error]: {e}")
                except Exception as e:
                    if keep_running:
                        raise RuntimeError(f"Stream corrupted: {e}")
                    
    except Exception as e:
        if keep_running:
            print(f"\n    [CRITICAL] ALSA audio driver choked. Rebooting Python process... {e}")
            time.sleep(1.5) 
            os.execv(sys.executable, [sys.executable] + sys.argv)

if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    if os.path.exists(TMP_FILE):
        os.remove(TMP_FILE)
    listen_loop()
    
    # Exits only after the "with mic" block is formally closed!
    sys.exit(0)