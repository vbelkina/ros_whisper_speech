# ros_whisper_speech

## software requirements: 

- python3
- ROS noetic

 Packages: 

- `pip install SpeechRecognition`
- `pip install openai-whisper`
- `pip install pyttsx3`
- `pip install espeak`

might need some packages for the sound system…

## to install package:

in catkin_ws/src:

`git clone https://github.com/vbelkina/ros_whisper_speech.git` 

run: 

`catkin_make`

## errors

Warnings that I'm getting and not sure how to fix, but the program works: 

`ALSA lib pcm_oss.c:377:(_snd_pcm_oss_open) Unknown field port
ALSA lib pcm_oss.c:377:(_snd_pcm_oss_open) Unknown field port
ALSA lib pcm_usb_stream.c:486:(_snd_pcm_usb_stream_open) Invalid type for card
ALSA lib pcm_usb_stream.c:486:(_snd_pcm_usb_stream_open) Invalid type for card
ALSA lib pcm_dmix.c:1089:(snd_pcm_dmix_open) unable to open slave
ALSA lib pcm_oss.c:377:(_snd_pcm_oss_open) Unknown field port
ALSA lib pcm_oss.c:377:(_snd_pcm_oss_open) Unknown field port
ALSA lib pcm_usb_stream.c:486:(_snd_pcm_usb_stream_open) Invalid type for card
ALSA lib pcm_usb_stream.c:486:(_snd_pcm_usb_stream_open) Invalid type for card
ALSA lib pcm_dmix.c:1089:(snd_pcm_dmix_open) unable to open slave`