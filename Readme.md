The sketch tested on Wemos chip
The code assumes connection to WiFi and Blynk service

After power up the device , search for WiFi network called WebIR....
Connect to it (you might need to go to airplane mode on some smartphones ) , open a browser on address http://192.168.2.1
Fill up all the form details
Apon submit the device will restart and connect to your home network.

Once connected following urls are supported :

http://<ip>/dash - main screen
http://<ip>/setup - configure IR codes - need A/C remote ready , the device will record signal
http://<ip>/clean - hard reset the device
  
[utils library](https://github.com/rolo-repo/arduino-utils) - should be installed as prerequisite

the Blynk application can be downloaded from following link 

1. Download Blynk App: http://j.mp/blynk_Android or http://j.mp/blynk_iOS
2. Touch the QR-code icon and point the camera to the code below
3. Enjoy my app!

![The App](https://github.com/rolo-repo/remote_air_conditioner/blob/master/AC_ControllerApp.png)
