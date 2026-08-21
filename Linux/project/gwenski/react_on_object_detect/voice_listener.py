import speech_recognition as sr
import os

CMD_FILE = "/tmp/darwin_voice_cmd.txt"

def listen_loop():
    r = sr.Recognizer()
    
    # 1. Lower threshold drastically so even quiet speech triggers it
    r.energy_threshold = 50 
    # 2. Turn OFF dynamic adjustment so it stays at 300
    r.dynamic_energy_threshold = False 

    mic = sr.Microphone(device_index=3) 
    
    print("Ready to receive commands.")
            
    while True:
        try:
            with mic as source:
                print("Listening for command...")
                # 3. Add timeout=5. If it hears nothing for 5 seconds, it restarts.
                audio = r.listen(source, timeout=5, phrase_time_limit=4)
                
                text = r.recognize_google(audio).lower()
                print(f" Heard: {text}")
                
                if "release" in text or "drop" in text:
                    print("Command: release")
                    with open(CMD_FILE, "w") as f:
                        f.write("release")
                        
        except sr.WaitTimeoutError:
            # This triggers if 5 seconds pass without hearing anything loud enough
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