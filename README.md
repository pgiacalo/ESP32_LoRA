A program that sends and receives wireless (bidirectional) messages between 2 ESP32 boards using LoRA devices.
This c code relies on the esp-idf library and espressif tools only. No Arduino libraries are used or required. 

The wireless communication is NOT 100% reliable, since the wireless signals from the 2 ESP32 boards may 
be sent simultaneouly and thus interfere with eachother. 

A future update to this code will add logic for receiver acknowledgements and transmission resends, if delivery fails.

The comments at the top of the code include a wiring diagram showing how to connect the ESP32 to the LoRA module.

To build, flash and run this code using espressif tools:

cd ESP32_LoRA  
. ~/esp/esp-idf/export.sh  
idf.py -p <usb_port> clean build flash monitor  

Once running, the console will print the messages sent and received. 
