# X68KFDPi2W
X68KFDPi2W - Floppies drive emulator All-In-One for Sharp X68000

The design is an evolution of my previous X68KFDPico, and allows the two drives of the X68000 to be emulated both externally and internally connected. The engine of the emulator are two Raspberry Pico, one ‘normal’ to handle the special floppy signals and the other ‘W’ to emulate the floppies, SDcard image management and a simple web server to choose the disks to be inserted in the floppies.
Ejection and insertion of the disks can be done via the two keys in the X68KFDPi2W or by connecting the cables of the original floppy keys (in case they are not present internally). The same for the internal LEDs, which are however replicated in the emulator (only the green one, the red LED is managed only for the internal one).
Floppies images format supported are "HDM" and "HFE". 
Writing to floppy is not supported yes (may be in future...)

A special thanks to Adafruit team for their great "Floppy library", from wich i starded for floppy emulation part.

![ScreenShot](https://raw.githubusercontent.com/aotta/X68KFDPi2W/main/pictures/X68KFDIPi2W_Board.jpg)
![ScreenShot](https://raw.githubusercontent.com/aotta/X68KFDPi2W/main/pictures/X68KFDIPi2W_HFE.jpg)
![ScreenShot](https://raw.githubusercontent.com/aotta/X68KFDPi2W/main/pictures/X68KFDIPi2W_LED.jpg)
![ScreenShot](https://raw.githubusercontent.com/aotta/X68KFDPi2W/main/pictures/X68KFDIPi2W_LED2.jpg)

You need to add a "wifi.cfg" in root of your SDcard, with this 4 text row:
SSID
WIFI PASSWORD
Default floppy in drive 0
Default floppy in drive 1

The 2 lines related to the default floppys can be omitted and changed everytime via Web server.
![ScreenShot](https://raw.githubusercontent.com/aotta/X68KFDPi2W/main/pictures/X68KFDIPi2W_WEB.jpg)

Gerbers file are provided for the PCB, add you pico clone, and flash the firmware ".uf2" in the Pico and in the Pico W by connecting it while pressing button on Pico and drop it in the opened windows on PC.

For any comments or questions about the projects, please use the official thread in the forum: https://nfggames.com/forum2/index.php?topic=7512.0



