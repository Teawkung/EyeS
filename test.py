
import speech_recognition as sr
r = sr.Recognizer()
print('Start')
with sr.Microphone() as source:
    while True:
        print('Listening')
        r.adjust_for_ambient_noise(source)
        audio = r.listen(source)
        try:
            print('You said : '+r.recognize_google(audio))
        except:
            print('Error!')