import speech_recognition as sr
import os
import sys
import signal
import time

CMD_FILE = "/tmp/darwin_voice_cmd.txt"
TMP_FILE = "/tmp/darwin_voice_cmd.tmp"

def handle_shutdown(signum, frame):
    print("\n[VOICE LISTENER] Shutting down safely, releasing microphone hardware...")
    sys.exit(0)

signal.signal(signal.SIGINT, handle_shutdown)
signal.signal(signal.SIGTERM, handle_shutdown)

def listen_loop():
    r = sr.Recognizer()
    
    r.energy_threshold = 700 
    r.dynamic_energy_threshold = False 
    r.pause_threshold = 0.5 

    print("Ready to receive commands.")

    try:
        # THE FIX: Removed chunk_size=4096! It defaults back to 1024, which the Pi's USB can handle.
        mic = sr.Microphone(device_index=3, sample_rate=16000)
        
        with mic as source:
            print("    [Hardware connected and locked successfully]")
            
            while True:
                try:
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
                    # If PyAudio crashes due to USB limits, trigger the reboot
                    raise RuntimeError(f"Stream corrupted: {e}")
                    
    except Exception as e:
        # Ignore the meaningless NoneType error text and focus on the reboot
        print("\n    [CRITICAL] ALSA audio driver choked on USB bandwidth. Rebooting Python process...")
        time.sleep(1.5) # Give the Linux kernel a moment to un-jam the USB port
        os.execv(sys.executable, [sys.executable] + sys.argv)

if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    if os.path.exists(TMP_FILE):
        os.remove(TMP_FILE)
    listen_loop()