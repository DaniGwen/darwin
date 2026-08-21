import speech_recognition as sr
import os

CMD_FILE = "/tmp/darwin_voice_cmd.txt"

def listen_loop():
    r = sr.Recognizer()
    
    # 1. Hardcoded to 100. (Drop to 50 or 10 if you still have to speak loudly)
    r.energy_threshold = 100 
    
    # 2. Lock it so it never tries to auto-adjust and freeze the camera
    r.dynamic_energy_threshold = False 

    mic = sr.Microphone(device_index=3) 
    
    print("Ready to receive commands.")

    while True:
        try:
            with mic as source:
                print("Listening for command...")
                # 3. Timeout prevents infinite hanging if the room is quiet
                audio = r.listen(source, timeout=5, phrase_time_limit=4)
                
                text = r.recognize_google(audio).lower()
                print(f" Heard: {text}")
                
                if "release" in text or "drop" in text:
                    print("Command: release")
                    with open(CMD_FILE, "w") as f:
                        f.write("release")
                        
        except sr.WaitTimeoutError:
            print("Silence detected, restarting listen loop...")
            pass
        except sr.UnknownValueError:
            print("Could not understand audio")
            pass
        except sr.RequestError as e:
            print(f"Could not request results; {e}")
        except Exception as e:
            print(f"Error: {e}")
            
if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    listen_loop()