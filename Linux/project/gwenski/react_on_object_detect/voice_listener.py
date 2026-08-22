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
    
    r.dynamic_energy_threshold = True 
    r.pause_threshold = 0.5 

    print("Ready to receive commands.")

    try:
        # THE CURE: We MUST keep 16000Hz to prevent the Pi's USB bus from suffocating.
        # Google's API actually prefers 16000Hz for speech recognition anyway!
        mic = sr.Microphone(device_index=3, sample_rate=16000)
        
        with mic as source:
            print("    [Hardware connected and locked successfully]")
            
            # THE NEW MAGIC: Let Python calculate the exact volume threshold automatically!
            print("    [Calibrating to room noise for 2 seconds. Please stay quiet...]")
            r.adjust_for_ambient_noise(source, duration=2)
            print(f"    [Calibration complete. Perfect threshold found: {int(r.energy_threshold)}]")
            
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
                    raise RuntimeError(f"Stream corrupted: {e}")
                    
    except Exception as e:
        print(f"\n    [CRITICAL] ALSA audio driver choked. Rebooting Python process... {e}")
        time.sleep(1.5) 
        os.execv(sys.executable, [sys.executable] + sys.argv)

if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    if os.path.exists(TMP_FILE):
        os.remove(TMP_FILE)
    listen_loop()