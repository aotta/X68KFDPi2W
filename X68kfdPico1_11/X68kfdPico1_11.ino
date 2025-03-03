/*
 * X68KFDINO - External FD adapter for Sharp X68000
 * by Andrea Ottaviani june 2023
 * 
 * 
Access/Write strobe :

/OPTION_SEL_X : "Chip select" for the extra functions of drive DSX (X: 0-3)

Input / functions signals :

/EJECT_FUNC : If this signal is low when OPTION_SEL_X is asserted, the disk will be ejected.
/LOCK_FUNC (Eject MSK?): If this signal is low when OPTION_SEL_X is asserted, the disk will be locked (eject button off). If this signal is high when OPTION_SEL_X is asserted the disk will be unlocked (eject button on).
/BLINK_FUNC : If this signal is low when OPTION_SEL_X is asserted, the LED blink. If this signal is high when OPTION_SEL_X is asserted, the LED stop blinking.

Output signals (drive status):

/DISK_IN_DRIVE : when OPTION_SEL_X is asserted : Low = a disk in the drive. High = No disk.
/INSERT_FAULT : when OPTION_SEL_X is asserted : Low = a disk insert failure occured . High = No problem. This status is cleared at the rising edge of OPTION_SEL_X.

Output signal (Interrupt line):

/INT : Interrupt line : This signal is activated at each disk status change (inserted/ejected). The interrupt is cleared at the rising edge of /OPTION_SEL_X. (Note : more than one drive can activate this line in the same time -> Softwares scan all disk drive at each /INT assertion).

Note 1: For proper operation all output should be able to drive 5/150=33mA since 150 ohms pull-up are used.
Note 2: The X68000 /OPTION_SEL_X assertion duration is ~2.5us per access. This can depend of the software, the CPU speed.
 */


//#define WAIT_SERIAL

#ifdef USE_TINYUSB
#include <Adafruit_TinyUSB.h>
#endif

//Cablage
#define OPT0                2  // Set D2 as OPT0 Read strobe
#define OPT1                3  // Set D3 as OPT1 Read strobe
#define OPT2                4  // Set D4 as OPT2 Read strobe
#define OPT3                5  // Set D5 as OPT3 Read strobe
#define Eject               6  // Set D6 as Eject Read strobe
#define Inserted            7  // Set D7 as Inserted Read strobe
#define FDDINT              8  // Set D8 as FD Interrupt Read strobe
#define Error               9  // Set D9 as Error Read strobe
#define EjectMSK           10  // Set D10 as EjectMSK Read strobe
#define Motor              11  // Set D11 as Motor Read strobe
#define LEDFD0             12  // Set D12 as LED FD0 output (Green LED)
#define LEDFD1             13  // Set D13 as LED FD1 output (Green LED)
#define LED_FD0_RED        14  // Set D14 as LED FD0 output (Red LED)
#define LED_FD1_RED        15  // Set D15 as LED FD1 output (Red LED)
#define BT_FD0             16  // Set D16 as Button FD0 Read strobe
#define BT_FD1             17  // Set D17 as Button FD1 Read strobe
#define LED_BLINK          18  // Set A0 as LED (Blink) output
#define BT_FD0_INT         19  // Set D19 as Button FD0 Read strobe from internal floppy
#define BT_FD1_INT         20  // Set D20 as Button FD1 Read strobe from internal floppy
#define DF0_EN             21  // Set D20 as Button FD1 Read strobe from internal floppy
#define DF1_EN             22  // Set D20 as Button FD1 Read strobe from internal floppy
#define INTLED             25  // Set D20 as Button FD1 Read strobe from internal floppy
#define INTEXT             26  // Set D20 as Button FD1 Read strobe from internal floppy

void setup() {
  pinMode(OPT0, INPUT_PULLUP);
  pinMode(OPT1, INPUT_PULLUP);
  pinMode(OPT2, INPUT_PULLUP);
  pinMode(OPT3, INPUT_PULLUP);
  pinMode(Eject, INPUT_PULLUP);
  pinMode(BT_FD0, INPUT);
  pinMode(BT_FD1, INPUT);
  pinMode(LED_BLINK, INPUT_PULLUP); // Blink
  pinMode(EjectMSK, INPUT_PULLUP); // LOCK (Red led)
  pinMode(Motor, INPUT_PULLUP); // 
  pinMode(Inserted, OUTPUT);
  pinMode(Error, OUTPUT);
  pinMode(FDDINT, OUTPUT);
  pinMode(LEDFD0, OUTPUT);
  pinMode(LEDFD1, OUTPUT);
  pinMode(LED_FD0_RED, OUTPUT); // used for LED LOCK, Red light
  pinMode(LED_FD1_RED, OUTPUT); // used for LED LOCK, Red light
  pinMode(BT_FD0_INT, OUTPUT);
  pinMode(BT_FD1_INT, OUTPUT);
  pinMode(DF0_EN, OUTPUT);
  pinMode(DF1_EN, OUTPUT);
  pinMode(INTLED,OUTPUT);
  pinMode(INTEXT,INPUT);


#if defined(WAIT_SERIAL)
 Serial.begin(115200);
  while (!Serial) {
  }
  Serial.println("Serial connected");
#endif


}

void loop() {
  unsigned char DF0=0; // 0=eject, 1=insert, 2=lock, 3=blink
  unsigned char DF1=0; // 0=eject, 1=insert, 2=lock, 3=blink
  unsigned char bitBlink0=0;
  unsigned char bitBlink1=0;
  unsigned char OPTA;
  unsigned char OPTB;
  

  unsigned short int BlinkCounter=0;
  if (digitalRead(INTEXT)) { //poivia
  //  if (0) {  //debug 0 force drive 0/1, 1 force drive 2/3
      OPTA=OPT0;
      OPTB=OPT1;
      Serial.print("Drive 0/1");
    } else {
      OPTA=OPT2;
      OPTB=OPT3;
      Serial.print("Drive 2/3");
    }


  // put your main code here, to run repeatedly:
  Serial.println("Start");
  
  while(1) {
    BlinkCounter++;
    if (BlinkCounter>=29999) BlinkCounter=0;
    
    if (digitalRead(BT_FD0)) {
        digitalWrite(FDDINT,LOW);
       if (DF0==1) {
        DF0=0;
       } else {
        bitBlink0=0;
        DF0=1;
        digitalWrite(INTLED,LOW);
         }
    Serial.print("DF0:");Serial.println(DF0);
    delay(200);
    }
   
  if (digitalRead(BT_FD1)) {
      digitalWrite(FDDINT,LOW);
        if (DF1==1) {
        DF1=0;
        } else {
        bitBlink1=0;
        DF1=1;
           digitalWrite(INTLED,LOW);
       }
    Serial.print("DF1:");Serial.println(DF1);
    delay(200);
    }
   

////////////////// OPT 0 //////////////////
  if (!digitalReadFast(OPTA)) {
    //Serial.print("optA");

    if (!digitalReadFast(Eject)) {   // era Eject
       digitalWriteFast(Inserted,HIGH);
       DF0=0;
       digitalWriteFast(FDDINT,LOW);  // not remove!! era HIGH     
       Serial.println("Ej 0-");
       delay(4);
       digitalWrite(INTLED,HIGH);
    }    
    
    if (!digitalReadFast(EjectMSK)) {
        digitalWriteFast(LED_FD0_RED,HIGH);
        digitalWriteFast(LEDFD0,LOW);
        Serial.println("EjMSK 0-");
    } else {
        digitalWriteFast(LED_FD0_RED,LOW);  
    }

    if (!digitalReadFast(LED_BLINK)) {
       bitBlink0=1;
      }   

    if (!DF0) {
        digitalWriteFast(Inserted,HIGH);  // for TT!!  
    //    digitalWriteFast(FDDINT,HIGH);
    //    digitalWriteFast(Error,HIGH);  
        digitalWriteFast(DF0_EN,LOW);
    } else {
        digitalWriteFast(Inserted,LOW);
      //  digitalWriteFast(FDDINT,HIGH);  // not remove!! era HIGH
      //  digitalWriteFast(Error,HIGH);  
        digitalWriteFast(DF0_EN,HIGH);
      }
  
  digitalWriteFast(FDDINT,HIGH);  // not remove!! era HIGH
      
 }  // opt 0


////////////////// OPT 1 //////////////////
 if (!digitalReadFast(OPTB)) {
   // Serial.print("optB");
   
    if (!(digitalReadFast(Eject))) { // era eject
      Serial.print("Ej 1-"); 
      digitalWriteFast(Inserted,HIGH);   
      digitalWriteFast(FDDINT,LOW);  // not remove!! era HIGH
      DF1=0;
      delay(4);
      digitalWrite(INTLED,HIGH);
    }
      
    if (!digitalReadFast(EjectMSK)) {
      digitalWriteFast(LED_FD1_RED,HIGH);
      digitalWriteFast(LEDFD1,LOW);
      Serial.print("EjMSK 1-");
      } else {
        digitalWriteFast(LED_FD1_RED,LOW);  
    }

    if (!digitalReadFast(LED_BLINK)) {
       bitBlink1=1;
      }   

    if (!DF1) {
       digitalWriteFast(Inserted,HIGH);  // for TT!!  
     //   digitalWriteFast(FDDINT,HIGH);  //NOT REMOVE FOR EJECT DRIVE LOW!!!
     //   digitalWriteFast(Error,HIGH);  
        digitalWriteFast(DF1_EN,LOW);
      } else {
        digitalWriteFast(Inserted,LOW);
      //  digitalWriteFast(FDDINT,HIGH);  // not remove!! era HIGH
      //  digitalWriteFast(Error,HIGH);
        digitalWriteFast(DF1_EN,HIGH);  
      }
    
    digitalWriteFast(FDDINT,HIGH);  // not remove!! era HIGH
   
  } //optB

  if (!DF0) {
        digitalWriteFast(LEDFD0,LOW);
        digitalWriteFast(BT_FD0_INT,LOW);
      } else {
         digitalWriteFast(LEDFD0,HIGH);
         digitalWriteFast(BT_FD0_INT,HIGH);
      }
  if (!DF1) {
        digitalWriteFast(LEDFD1,LOW);
        digitalWriteFast(BT_FD1_INT,LOW);
      } else {
         digitalWriteFast(LEDFD1,HIGH);
         digitalWriteFast(BT_FD1_INT,HIGH);
      }

  if (bitBlink0) {
       delay(10);
      if (BlinkCounter>=3000) {
        digitalWriteFast(LEDFD0,LOW);
      } else {
        digitalWriteFast(LEDFD0,HIGH);
      }
    }
  if (bitBlink1) {
      //Serial.print("b1"); // do not remove, needed for timing
       delay(10);
    if (BlinkCounter>=3000) {
        digitalWriteFast(LEDFD1,LOW);
      } else {
        digitalWriteFast(LEDFD1,HIGH);
      }
    }

 } //while
 
}
