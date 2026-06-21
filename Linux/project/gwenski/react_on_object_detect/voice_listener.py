import speech_recognition as sr
import os
import time

CMD_FILE = "/tmp/darwin_voice_cmd.txt"

def listen_loop():
    r = sr.Recognizer()
    mic = sr.Microphone()
    with mic as source:
            print("Adjusting for background noise...")
            r.adjust_for_ambient_noise(source, duration=2)
            print("Ready to receive commands.")
            
    while True:
        try:
            with mic as source:
                print("Listening for command...")
                audio = r.listen(source, phrase_time_limit=4)
                text = r.recognize_google(audio).lower()
                print(f" Heard: {text}")
                
                if "release" in text or "drop" in text:
                    print("Command: release")
                    with open(CMD_FILE, "w") as f:
                        f.write("release")
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