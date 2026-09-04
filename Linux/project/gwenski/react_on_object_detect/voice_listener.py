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
    keep_running = False

signal.signal(signal.SIGINT, handle_shutdown)
signal.signal(signal.SIGTERM, handle_shutdown)
# ---------------------------------

def listen_loop():
    global keep_running
    r = sr.Recognizer()
    
    # Start at 600 (above room noise, below servo noise)
    r.energy_threshold = 600
    r.dynamic_energy_threshold = True 
    r.pause_threshold = 0.5 

    print("Ready to receive commands.")

    try:
        mic = sr.Microphone(device_index=4)
        
        with mic as source:
            print("    [Calibrating ambient noise... please stay quiet for 2 seconds]")
            # --- TWEAK 1: Calibrate to the room's actual background noise ---
            r.adjust_for_ambient_noise(source, duration=2)
            print("    [Hardware connected and locked successfully]")
            
            while keep_running:
                try:
                    print("\n--> LISTENING... (Speak now)")
                    audio = r.listen(source, timeout=2, phrase_time_limit=4)
                
                    if not keep_running:
                        break

                    print("    [Processing audio...]")
                    
                    # --- TWEAK 2: Lock the language model to prevent regional phonetic drift ---
                    text = r.recognize_google(audio, language="bg-BG").lower()
                    
                    print(f"    [GOOGLE HEARD]: '{text}'") 
                    
                    # --- TWEAK 3: Add the exact phonetic mistakes Google makes ---
                    trigger_words = [
                        # Startup
                        "старт", "тръгвай", "започни",
                        # Greetings
                        "здравей", "здрасти", "хей",
                        # Stop / Sleep
                        "спри", "заспи", "изключи",
                        # Stand / Reset
                        "изправи се", "център", 
                        # Independent Grippers
                        "отвори лявата", "затвори лявата", 
                        "отвори дясната", "затвори дясната",
                        # Hold / Catch
                        "дръж", "хвани", "вземи",
                        # General Close Gripper
                        "затвори", "затвори всички",
                        # Release
                        "пусни", "остави"
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
                    if keep_running:
                        raise RuntimeError(f"Stream corrupted: {e}")
                    
    except Exception as e:
        if keep_running:
            print(f"\n    [CRITICAL] ALSA audio driver choked. Rebooting Python process... {e}")
            for _ in range(15):
                if not keep_running:
                    print("    [Abort] Shutdown signal received during reboot. Exiting instead.")
                    sys.exit(0)
                time.sleep(0.1) 
            
            if keep_running:
                os.execv(sys.executable, [sys.executable] + sys.argv)

if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    if os.path.exists(TMP_FILE):
        os.remove(TMP_FILE)
    listen_loop()
    
    sys.exit(0)