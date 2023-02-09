import pyttsx3

def text_to_speech(speech):
    engine = pyttsx3.init()
    # engine.setProperty("rate", 178)
    engine.say(speech)
    engine.runAndWait()


text_to_speech("hello this is a test")