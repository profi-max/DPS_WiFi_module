# DPS WiFi module
Based on the ESP8266 (ESP-12) module for communicate over Wifi with RD (Riden) power supplies:
 DPS5020 DPS5015 DPS5005 DPS8005 DPS3012 DPS3005 DPS3003
Supported both stock Chinese and alternative firmware.

## DPS alternative firmware
Allow you control your wifi communication.
https://profimaxblog.ru/dps_update/
<image src="/Pictures/dps45_mainscreen.jpg" alt="DPS">


## How to flash ESP8266 (3 ways)
1. Use finished firmware files from the folder **Binary**. Upload Flash Download Tools  from official page: https://www.espressif.com/en/support/download/other-tools
Connect USB cable to your development board, run the application, select COM port and bin-file. When download is completed push "RST" (Reset) button to restart your dev board.
2. Use Arduino IDE. Open the sketch from **Arduino** folder. Select NodeMCU 1.0 board. Install WiFiManager by tzapu library. Compile and upload the sketch to your dev board.
3. Use Visual Studio Code + Platformio. Open project from **VSCode** folder. Build and upload the project.
<details>
<summary>Click to view Flash Download Tools</summary>
<image src="/Pictures/flasher.jpg" alt="Flasher">
</details>

## Schematics
The development board is needed only for flashing and debugging. After debugging, you can connect a module without a development board.
In the NodeMcu dev board "FLASH" button connected to GPIO0, so you can use "FLASH" button as "WiFi Reset".
You can use any board with ESP8266. 
<details>
<summary>I use this one: Click to view the  board</summary>
Aliexpress page: https://aliexpress.ru/item/4000550036826.html
<image src="/Pictures/NodeMCU.jpg" alt="NodeMCU">
</details>
<image src="/Pictures/dps_NodeMcu.jpg" alt="DPS and NodeMCU">
<image src="/Pictures/dps_esp12.jpg" alt="DPS and ESP-12">

## How it works
The project is Modbus RTU to Modbus TCP bridge. Now your DPS power supply can communicate with DPSmaster application over WiFi.
In the DPSmaster application select "TCP" and enter correct ESP8266's local IP address.
DPSmaster officail page: https://profimaxblog.ru/dpsmaster/
![DPSmaster](/Pictures/DPSmaster.jpg)

## Video & Photo Gallery
<details>
<summary>Click to view the Gallery</summary>
https://youtube.com/shorts/995Rk9Xic3o

https://youtube.com/shorts/P--5Z4uEhjc

![DPSview](/Pictures/IMG_4732.JPG)
![DPSview](/Pictures/IMG_4735.JPG)
![DPSview](/Pictures/IMG_4733.JPG)
![DPSview](/Pictures/IMG_4736.JPG)
</details>

## How to establish a connection with Wi-Fi Router.
Push "WiFi Reset" button for 3 seconds. The ESP8266 module switch to access point mode. Open WiFi settings on your phone. Choose "DPS TCP bridge", no password needed. Then click "Configure WiFi", choose your router, enter password and click "Save". Now the ESP8266 module is in the station mode and connected to your wifi router.
For DPS alternative firmware you can use "Communication" menu "Wifi reset - Yes/No" item instead optional "WiFi Reset" button.
<image src="/Pictures/WifiManager.jpg" alt="WiFiManager">

## How to find out the local Wi-Fi IP address of ESP8266.
Open the router's web page in a browser. View the list of clients.
For DPS alternative firmware: Go to "Parameters" menu. Press the "SET" button. The IP address will be displayed for 3 seconds.

## How to use with DPS Chinese firmware
Comment the line like this:
```
// comment the line below if your DPSxxxx device has chinesse firmware
//#define DPS_ALTERNATIVE_FIRMWARE
```
Enter correct baud rate:
```
Serial.begin(9600);
```
## How to debug
The software serial over UART0/USB is used at 115200 speed. Unfortunately software serial is not a reliable tool.
```
// uncomment the line below if you need debug via USB/UART0
#define MB_DEBUG
```
