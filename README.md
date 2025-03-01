# X68KFDPi2W
X68KFDPi2W - Floppies drive emulator All-In-One for Sharp X68000

The design is an evolution of my previous X68KFDPico, and allows the two drives of the X68000 to be emulated both externally and internally connected. The engine of the emulator are two Raspberry Pico, one ‘normal’ to handle the special floppy signals and the other ‘W’ to emulate the floppies, SDcard image management and a simple web server to choose the disks to be inserted in the floppies.
Ejection and insertion of the disks can be done via the two keys in the X68KFDPi2W or by connecting the cables of the original floppy keys (in case they are not present internally). The same for the internal LEDs, which are however replicated in the emulator (only the green one, the red LED is managed only for the internal one).
Floppies images format supported are "HDM" and "HFE". 
Writing to floppy is not supported yes (may be in future...)

A special thanks to Adafruit team for their great "Floppy library", from wich i starded for floppy emulation part.

![ScreenShot](https://raw.githubusercontent.com/aotta/X68KFDPi2W/main/pictures/X68KFDPi2W_Board.jpg)

**WARNING!** "purple" Pico has not the same pinout of original Raspberry "green" ones, you MUST use the clone or you may damage your hardware.
Also note that the battery used is a RECHARGEABLE LIR2032, if you want to use a NON reachargeable battery you must add a diode in circuit!!!

Tested only on PAL consoles so far, feel free to send comments and feedback on AtariAge thread:
https://forums.atariage.com/topic/374297-picoa10400-preview/



Also added a Raspberry Pico 2 version, relative files are named Pico2A10400. It works but consider its smaller flash size for roms (3mb):

![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/Pico2A10400.jpg)

Gerbers file are provided for the PCB, add you pico clone, and flash the firmware ".uf2" in the Pico by connecting it while pressing button on Pico and drop it in the opened windows on PC.
After flashed with firmware, and every time you have to change your ROMS repository, you can simply connect the Pico to PC and drag&drop "BIN" files  into.

**NOTE 2** Due to different timing of PicoA10400 and the Atari consoles, that can't be resetted, the flashcart MUST BE POWERED ON (with POWER SWITCH ON CART) BEFORE POWERING THE CONSOLE!!! Also, some games and ALL A7800 GAMES NEEDS THAT THE CONSOLE IS POWERED OFF THEN POWERED ON TO START!!!!

Even if the diode should protect your console, **DO NOT CONNECT PICO WHILE INSERTED IN A POWERED ON CONSOLE!**

19th january 2025: added Pico 10400 Alternative Version by XAD, with improvements in pcb and shell: https://www.nightfallcrew.com/17/01/2025/picoa10400-flashcart-for-atari-2600-7800/
![ScreenShot](https://raw.githubusercontent.com/aotta/PicoA10400/main/pictures/picoA10400_08.jpg)

