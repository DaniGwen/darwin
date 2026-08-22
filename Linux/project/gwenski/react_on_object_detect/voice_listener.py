import speech_recognition as sr
import os
import sys
import signal
import time # Needed for a tiny safety delay

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

    # =======================================================
    # OUTER LOOP: Creates and repairs the hardware connection
    # =======================================================
    while True:
        try:
            mic = sr.Microphone(device_index=3, chunk_size=4096)
            
            with mic as source:
                print("    [Hardware connected and locked successfully]")
                
                # =======================================================
                # INNER LOOP: Listens continuously without turning mic off
                # =======================================================
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
                        # No one spoke. Loop around and keep the mic open!
                        pass
                    except sr.UnknownValueError:
                        print("    [Ignored: Audio not understood]")
                    except sr.RequestError as e:
                        print(f"    [API Error]: {e}")
                    except Exception as e:
                        error_msg = str(e)
                        print(f"    [Error]: {error_msg}")
                        
                        # THE MAGIC FIX: If the stream dies, BREAK the inner loop!
                        if "Audio source must be entered" in error_msg or "NoneType" in error_msg or "closed" in error_msg:
                            print("    [Mic stream died! Forcing a hard reset...]")
                            break # This kicks it back out to the Outer Loop to reboot the mic!
                            
        except Exception as e:
            print(f"    [Hardware Error]: {e}")
            time.sleep(1) # If the hardware completely vanishes, wait 1 sec before retrying to avoid spam

if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    if os.path.exists(TMP_FILE):
        os.remove(TMP_FILE)
    listen_loop()