import speech_recognition as sr
import os
import sys

CMD_FILE = "/tmp/darwin_voice_cmd.txt"

def listen_loop():
    r = sr.Recognizer()
    
    r.energy_threshold = 20 
    r.dynamic_energy_threshold = False 

    mic = sr.Microphone(device_index=3) 
    
    print("Ready to receive commands. (Press Ctrl+C to safely quit)")

    # Wrap the entire loop in a try/except block to catch Ctrl+C
    try:
        while True:
            try:
                with mic as source:
                    print("Listening for command...")
                    audio = r.listen(source, timeout=5, phrase_time_limit=4)
                    
                    text = r.recognize_google(audio).lower()
                    print(f" Heard: {text}")
                    
                    if "release" in text or "drop" in text:
                        print("Command: release")
                        with open(CMD_FILE, "w") as f:
                            f.write("release")
                            # Force the file to save instantly so C++ reacts faster
                            f.flush()
                            os.fsync(f.fileno()) 
                            
            except sr.WaitTimeoutError:
                pass # Removed the print spam
            except sr.UnknownValueError:
                print("Could not understand audio")
            except sr.RequestError as e:
                print(f"API Error: {e}")
            except Exception as e:
                print(f"Error: {e}")
                
    except KeyboardInterrupt:
        # MAGIC FIX: When you press Ctrl+C, this block cleanly closes the microphone!
        print("\nClosing audio streams and releasing microphone...")
        sys.exit(0)
            
if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    listen_loop()