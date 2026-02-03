THIS PROJECT IS WORKING IN PROGRESS
Following the hardware required: 
  -  ESP32-S3 WROOM N16R8 with camera (OV5640) module
  -  TOF VL53L0X
  -  433mhz TX FS1000A
  -  433mhz RX mx-rm-5v
  -  
High level explanation of the utilization loop:
Whenever the ToF reads a near object, camera will take a picture, picture will be uploaded to remote server via WiFi, if the response from the remote is positive, a 433mhz signal will be emitted via TX.
If the object is still near, it will continue to take pictures and read response from remote server.
The RX module will only work first time, when the esp32 will need to capture a signal.

