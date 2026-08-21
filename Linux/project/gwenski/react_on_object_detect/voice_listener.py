import speech_recognition as sr
import os

CMD_FILE = "/tmp/darwin_voice_cmd.txt"

def listen_loop():
    r = sr.Recognizer()
    
    # 1. Hardcode the microphone sensitivity to bypass the freeze
    r.energy_threshold = 3500 
    r.dynamic_energy_threshold = True 

    # 2. Use your confirmed Logitech USB Mic index
    mic = sr.Microphone(device_index=3) 
    
    print("Ready to receive commands.")
            
    while True:
        try:
            with mic as source:
                print("Listening for command...")
                audio = r.listen(source, phrase_time_limit=4)
                
                # Convert speech to text
                text = r.recognize_google(audio).lower()
                print(f" Heard: {text}")
                
                # Check for trigger words
                if "release" in text or "drop" in text:
                    print("Command: release")
                    with open(CMD_FILE, "w") as f:
                        f.write("release")
                        
        except sr.UnknownValueError:
            print("Could not understand audio")
        except sr.RequestError as e:
            print(f"Could not request results; {e}")
        except Exception as e:
            print(f"Error: {e}")
            
if __name__ == "__main__":
    if os.path.exists(CMD_FILE):
        os.remove(CMD_FILE)
    listen_loop()