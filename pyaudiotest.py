# pyaudio test bluetooth headset

import pyaudio

p = pyaudio.PyAudio()

for i in range(p.get_device_count()):
    if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
        print(p.get_device_info_by_index(i))

p.terminate()

