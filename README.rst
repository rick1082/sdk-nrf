Dual gateway BIS demo
########################

This is a prototype project for showing the concept of a BIS headset switches audio streams
between two BIS gateways

============
Causion:
============
* This is a concept-proof project which based on NCS v1.9.99-dev1, which doesn't involve standard LE audio profiles like BAP/BASS/PBP but using BIS ISO channel for audio stream directly
* We won't provide further support/debug/upgrade for this project in the current stage
* Please ake sure you are able to build nrf5340 audio project from NCS v1.9.99-dev1 which using LC3 codec before checkout this demo. The full documentation can be found in here: http://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.99-dev1/nrf/applications/nrf5340_audio/README.html

============
Setup the demo:
============
1. Setup the NCS enviornment for nrf5340 audio project on NCS v1.9.99-dev1

2. Prepare 3 nRF5340 audio DKs

3. Modify the `CONFIG_BT_DEVICE_NAME` in `nrf/applications/nrf5340_audio/overlay-gateway.conf` to "NRF5340_AUDIO_DEV_1" and then build and  program gateway project to the first nRF5340 audio DK

4. Modify the `CONFIG_BT_DEVICE_NAME` in `nrf/applications/nrf5340_audio/overlay-gateway.conf` to "NRF5340_AUDIO_DEV_2" and the build and program gateway project to the second nRF5340 audio DK

5. Program the headset to the third nRF5340 audio DK

6. Press button 4 on the headset for switch the audio stream between gateway 1 and gateway 2

7. The default setting of audio interface for gateway is USB, user can change the setting in `nrf/applications/nrf5340_audio/prj.conf` for changing the interface to I2S, or the on board PDM microphone. For using the PDM microphone, please make sure `CONFIG_AUDIO_SOURCE_I2S=y`
