/////////////////////////////////////////////////////////////////////////////////////////
// X68KFDPI2W a floppy emulator AIO for Sharp X68000 by Andrea Ottaviani (c) 02/2025   //
// thanks to Adafruit for its great floppy library !!                                  
//
/////////////////////////////////////////////////////////////////////////////////////////

//#define WAIT_SERIAL
//#define WAIT_WIFI
bool HFE0_On=false,HFE1_On=false;   

#include "drive.pio.h"

#include "SdFat.h"
#include "pitches.h"
// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 1
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <LEAmDNS.h>
#include <StreamString.h>

#define DENSITY_PIN    0  // IDC 2
#define SELECT_PIN3    1  // IDC 6  DRIVE SELECT DS3
#define INDEX_PIN      2  // IDC 8   // output from PicoFloppy
#define SELECT_PIN0    3  // IDC 10 DRIVE SELECT DS0
#define SELECT_PIN1    4  // IDC 12 DRIVE SELECT DS1
#define SELECT_PIN2    5  // IDC 14 DRIVE SELECT DS2
#define MOTOR_PIN      6  // IDC 16
#define DIR_PIN        7  // IDC 18
#define STEP_PIN       8  // IDC 20
#define WRDATA_PIN     9  // IDC 22 //input for reading from PC when writing SD
#define WRGATE_PIN    10  // IDC 24 // input for writing on disk
#define TRK0_PIN      11  // IDC 26 // output from PicoFloppy
#define PROT_PIN      12  // IDC 28 // output from PicoFloppy
#define READ_PIN      13  // IDC 30 // output from PicoFloppy
#define SIDE_PIN      14  // IDC 32
#define READY_PIN     15  // IDC 34  // output from PicoFloppy
#define BUZZER_PIN    20  // Output pin for sound
#define DF0_EN        21  // Info for drive A enabled from other PICO
#define DF1_EN        22  // Info for drive B enabled from other PICO
#define INTEXT        26  // Read from jumper - FD0-1 when HIGH, FD2-3 when LOW
#define LVC245_EN     27  // Enable output to FD bus when HIGH


char ssid[40];
char password[40];
char filename0[128];
char filename1[128];
int curtrack0=0;
int curtrack1=0;
bool subdirOn=false;
bool refreshOn=false;
int SELECT_PINA, SELECT_PINB;
int numdrv0,numdrv1;

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(40)

// Try to select the best SD card configuration.
//#if HAS_SDIO_CLASS
//#define SD_CONFIG SdioConfig(FIFO_SDIO)
#if  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

#if SD_FAT_TYPE == 0
SdFat SD;
File dir;
File file0;
File file1;
File file2;
File tempfile;
#elif SD_FAT_TYPE == 1
SdFat32 SD;
File32 dir;
File32 file0;
File32 file1;
File32 file2;
File32 tempfile;
#elif SD_FAT_TYPE == 2
SdExFat SD;
ExFile dir;
ExFile file0;
ExFile file1;
ExFile file2;
ExFile tempfile;
#elif SD_FAT_TYPE == 3
SdFs SD;
FsFile dir;
FsFile file0;
FsFile file1;
FsFile file2;
FsFile tempfile;
#else  // SD_FAT_TYPE
#error invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE
char curDir[512]="/";
WebServer server(80);
const int led = LED_BUILTIN;

#define DEBUG_PRINTF(...) Serial.printf(__VA_ARGS__)
#define DEBUG_ASSERT(x)                                                        \
  do {                                                                         \
    if (!(x)) {                                                                \
      Serial.printf(__FILE__ ":%d: Assert fail: " #x "\n", __LINE__);          \
    }                                                                          \
  } while (0)
#include "mfm_impl.h"

enum {
  max_flux_bits = 166667 // 300RPM (200ms rotational time), 1us bit times era 200000  
};
enum { max_flux_count_long = (max_flux_bits + 31) / 32 };

// Data shared between the two CPU cores
volatile int fluxout0; // side number 0/1 or -1 if no flux should be generated
volatile int fluxin0; // side number 0/1 or -1 if no flux should be generated
volatile int fluxout1; // side number 0/1 or -1 if no flux should be generated
volatile int fluxin1; // side number 0/1 or -1 if no flux should be generated
volatile size_t flux_count_long =
    max_flux_count_long; // in units of uint32_ts (longs)
volatile uint32_t flux_data0[2]
                           [max_flux_count_long]; // one track of flux data for
                                                  // both sides of the disk
volatile uint32_t flux_data1[2]
                           [max_flux_count_long]; // one track of flux data for
                                                  // both sides of the disk
volatile uint32_t flux_datain0[2]
                           [max_flux_count_long]; // one track of flux data for
                                                  // both sides of the disk
volatile int wrote0=0;
volatile int writeon0=0;

struct floppy_format_info_t {
  uint8_t cylinders, sectors, sides; // number of sides may be 1 or 2
  uint16_t bit_time_ns;
  size_t flux_count_bit;
  uint8_t n; // sector size is 128<<n
  bool is_fm;
};

const struct floppy_format_info_t format_info[] = {
    {80, 18, 2, 1000, 200000, 2, false}, // 3.5" 1440kB, 300RPM
    {80, 9, 2, 2000, 100000, 2, false},  // 3.5" 720kB, 300RPM

    {80, 15, 2, 1000, 166667, 2, false}, // 5.25" 1200kB, 360RPM
    
    {77, 8, 2, 1000, 166667, 3, false}, // 5.25" 1232kB, 360RPM

    {40, 9, 2, 2000, 100000, 2, false},  // 5.25" 360kB, 300RPM

    {77, 26, 1, 2000, 80000, 0, true}, // 8" 256kB, 360RPM
};

const floppy_format_info_t *cur_format = &format_info[0];

typedef struct  
	{
		/*0x000*/	uint8_t		HEADERSIGNATURE[8];		// "HXCPICFE" for HFEv1 and HFEv2, "HXCHFEV3" for HFEv3
		/*0x008*/	uint8_t		formatrevision;			// 0 for the HFEv1, 1 for the HFEv2. Reset to 0 for HFEv3.
		/*0x009*/	uint8_t		number_of_track;		// Number of track(s) in the file
		/*0x00A*/	uint8_t		number_of_side;			// Number of valid side(s) (Not used by the emulator)
		/*0x00B*/	uint8_t		track_encoding;			// Track Encoding mode (Used for the write support - Please see the list above)
		/*0x00C*/	uint16_t	bitRate;			// Bitrate in Kbit/s. Ex : 250=250000bits/s, Max value : 1000
		/*0x00E*/	uint16_t	floppyRPM;			// Rotation per minute  (Not used by the emulator)
		/*0x010*/	uint8_t		floppyinterfacemode;		// Floppy interface mode. (Please see the list above.)
		/*0x011*/	uint8_t		dnu;				// Reserved
		/*0x012*/	uint16_t	track_list_offset;		// Offset of the track list LUT in block of 512 bytes (Ex: '1' means offset 0x200, '2' means 0x400 ...)
		/*0x014*/	uint8_t		write_allowed;			// 0x00 : Write protected, 0xFF: Unprotected

		// v1.1 addition - Set them to 0xFF if unused.
		/*0x015*/	uint8_t		single_step;			// 0xFF : Single Step - 0x00 Double Step mode
		/*0x016*/	uint8_t		track0s0_altencoding;		// 0x00 : Use an alternate track_encoding for track 0 Side 0
		/*0x017*/	uint8_t		track0s0_encoding;		// alternate track_encoding for track 0 Side 0
		/*0x018*/	uint8_t		track0s1_altencoding;		// 0x00 : Use an alternate track_encoding for track 0 Side 1
		/*0x019*/	uint8_t		track0s1_encoding;		// alternate track_encoding for track 0 Side 1
	} hfe_header;

  typedef struct  
	{
		uint16_t	offset;		// Track data offset in block of 512 bytes (Ex: 2 = 0x400)
		uint16_t	track_len;	// Length of the track data in byte.
	} pictrack;

  pictrack Pic_track0[82];
  pictrack Pic_track1[82];


////////////////////////////////
// Code & data for core 1
// Generate index pulses & flux
////////////////////////////////
#define FLUX_OUT_PIN (READ_PIN) // "read pin" is named from the controller's POV
#define FLUX_IN_PIN (WRDATA_PIN) // "read pin" is named from the controller's POV

PIO pio = pio0;
uint sm_fluxout, offset_fluxout;
uint sm_fluxin, offset_fluxin;
uint sm_index_pulse, offset_index_pulse;

volatile bool early_setup_done;

void setup1() {
  while (!early_setup_done) {
  }
}

void __not_in_flash_func(loop1)() {
  static bool once;
/*
  if (fluxin0 >= 0) {
    Serial.print("leggo");
     pio_sm_put_blocking(pio, sm_index_pulse,
                        2000); //  put index high for 2ms (out of 200ms)  
    for (size_t i = 0; i < flux_count_long; i++) {
      int f = fluxin0;
      if (f < 0)
        break;
      uint32_t wd = pio_sm_get_blocking(pio, sm_fluxin);
      flux_datain0[fluxin0][i]=wd;
      }
    // terminate index pulse if ongoing
    pio_sm_exec(pio, sm_index_pulse,
                0 | offset_index_pulse); // JMP to the first instruction
  }
*/
  //if ((fluxout0 >= 0)&&(!digitalRead(SELECT_PINA))) {
  if ((fluxout0 >= 0)) {
 
    pio_sm_put_blocking(pio, sm_index_pulse,
                        2000); // put index high for 2ms (out of 200ms)  
                        for (size_t i = 0; i < flux_count_long; i++) {
      int f = fluxout0;
      if (f < 0)
        break;
      auto d = flux_data0[fluxout0][i];
      pio_sm_put_blocking(pio, sm_fluxout, __builtin_bswap32(d));
      }
    // terminate index pulse if ongoing
    pio_sm_exec(pio, sm_index_pulse,
                0 | offset_index_pulse); // JMP to the first instruction
  }
  
    
  //if ((fluxout1 >= 0)&& (!digitalRead(SELECT_PINB))) {
  if ((fluxout1 >= 0)) {
    pio_sm_put_blocking(pio, sm_index_pulse,
                        2000); //  put index high for 2ms (out of 200ms) 
    for (size_t i = 0; i < flux_count_long; i++) {
      int f = fluxout1;
      if (f < 0)
        break;
      auto d = flux_data1[fluxout1][i];
      pio_sm_put_blocking(pio, sm_fluxout, __builtin_bswap32(d));
    }
    // terminate index pulse if ongoing
    pio_sm_exec(pio, sm_index_pulse,
                0 | offset_index_pulse); // JMP to the first instruction
  }
}

////////////////////////////////////////////////
// Code & data for core 0
// "UI", control signal handling & MFM encoding
////////////////////////////////////////////////

// Set via IRQ so must be volatile
volatile int trackno0;
volatile int trackno1;

enum {
  max_sector_count = 8,
  mfm_io_block_size = 1024,
  track_max_bytes = max_sector_count * mfm_io_block_size
};

uint8_t track_data0[track_max_bytes];
uint8_t track_data1[track_max_bytes];

void onStep() {
  
  if ((!digitalRead(SELECT_PINB))) {
    auto enabled1 = !digitalRead(SELECT_PINB); // motor need not be enabled to seek tracks
    auto direction = digitalRead(DIR_PIN);
    int new_track1 = trackno1;
    if (direction) {
      if (new_track1 > 0)
        new_track1--;
        tone(BUZZER_PIN, NOTE_G4, 250);
        noTone(BUZZER_PIN);
    } else {
    if (new_track1 < 77)
      new_track1++;
      tone(BUZZER_PIN, NOTE_F7, 250);
      noTone(BUZZER_PIN);
    }
    if (enabled1) {
    trackno1 = new_track1;
    digitalWrite(TRK0_PIN, trackno1 != 0); // active LOW
    }
  }
  
  
  if ((!digitalRead(SELECT_PINA))) {
    auto enabled0 =  !digitalRead(SELECT_PINA); // motor need not be enabled to seek tracks
    auto direction = digitalRead(DIR_PIN);
    int new_track0 = trackno0;
    if (direction) {
      if (new_track0 > 0)
        new_track0--;
        tone(BUZZER_PIN, NOTE_G4, 250);
        noTone(BUZZER_PIN);
    } else {
      if (new_track0 < 77)
      new_track0++;
      tone(BUZZER_PIN, NOTE_F7, 250);
      noTone(BUZZER_PIN);
    }
    if (enabled0) {
      trackno0 = new_track0;
      digitalWrite(TRK0_PIN, trackno0 != 0); // active LOW
    }
  }
  
}

void pio_sm_set_clk_ns(PIO pio, uint sm, uint time_ns) {
  //Serial.printf("set_clk_ns %u\n", time_ns);
  float f = clock_get_hz(clk_sys) * 1e-9 * time_ns;
  int scaled_clkdiv = (int)roundf(f * 256);
  pio_sm_set_clkdiv_int_frac(pio, sm, scaled_clkdiv / 256, scaled_clkdiv % 256);
}

bool setFormat(size_t size) {
  cur_format = NULL;
  for (const auto &i : format_info) {
    auto img_size = (size_t)i.sectors * i.cylinders * i.sides * (128 << i.n);
   // Serial.println("  ");
   // Serial.printf("sec/cyl/side/n: %d %d %d %d \n",(size_t)i.sectors,i.cylinders,i.sides,i.n);
     if (size != img_size)
      continue;
    cur_format = &i;
    if (cur_format->is_fm) {
      pio_sm_set_wrap(pio, sm_fluxout, offset_fluxout, offset_fluxout + 1);
      pio_sm_set_clk_ns(pio, sm_fluxout, i.bit_time_ns / 4);
      gpio_set_outover(FLUX_OUT_PIN, GPIO_OVERRIDE_INVERT);
    } else {
      pio_sm_set_wrap(pio, sm_fluxout, offset_fluxout, offset_fluxout + 0);
      pio_sm_set_clk_ns(pio, sm_fluxout, i.bit_time_ns);
      gpio_set_outover(FLUX_OUT_PIN, GPIO_OVERRIDE_NORMAL);
     // Serial.println("set to mfm");
    }
    flux_count_long = (i.flux_count_bit + 31) / 32;
    //Serial.println("*******************************");
    //Serial.print("flux count long:");Serial.println(flux_count_long);
    return true;
  }
  return false;
}
bool checkExt(const char filename[],const char extension[]) {

  // Ottieni la lunghezza del filename e dell'estensione
    int filename_len = strlen(filename);
    int extension_len = strlen(extension);

    // Se il filename è più corto dell'estensione, non può essere valido
    if (filename_len < extension_len) {
        return false;
    }

    // Confronta gli ultimi caratteri del filename con l'estensione
    return strcasecmp(filename + filename_len - extension_len, extension) == 0;
    
}

void get_HFE_track0() {
  hfe_header HFE_header;
  file0.seek(0);
  char buffer[512];
  file0.readBytes(buffer,512);
  Serial.println("Read buffer");
  memcpy(&HFE_header,buffer,sizeof(HFE_header));
/*
  Serial.print("Header:");
  for(int i=0;i<8;i++) {
    char a=HFE_header.HEADERSIGNATURE[i];
    Serial.print(a);
  }
  Serial.print("Ver.:");Serial.println(HFE_header.formatrevision);
  Serial.print("Tracks:");Serial.println(HFE_header.number_of_track);
  Serial.print("Side:");Serial.println(HFE_header.number_of_side);
  Serial.print("Encoding:");Serial.println(HFE_header.track_encoding);
  Serial.print("Bit rate:");Serial.println(HFE_header.bitRate); // MFM=250 (==250KHz)
  Serial.print("floppyRPM:");Serial.println(HFE_header.floppyRPM);
  Serial.print("interface:");Serial.println(HFE_header.floppyinterfacemode);
  Serial.print("Offset:");Serial.println(HFE_header.track_list_offset); // 1 = 0x200 (multiplied by 0x200)
  Serial.print("Step:");Serial.println(HFE_header.single_step); 	// 0xff: single step, 0x00: double step
*/  

  for (int i=0;i<HFE_header.number_of_track;i++) {
    char buffer[4];
    file0.readBytes(buffer,4);
    memcpy(&Pic_track0[i],buffer,4);
  //  Serial.println(Pic_track0[i].offset);
  }
}
void get_HFE_track1() {
  hfe_header HFE_header;
  char buffer[512];
  file1.seek(0);
  file1.readBytes(buffer,512);
  memcpy(&HFE_header,buffer,sizeof(HFE_header));

  for (int i=0;i<HFE_header.number_of_track;i++) {
    char buffer[4];
    file1.readBytes(buffer,4);
    memcpy(&Pic_track1[i],buffer,4);
  }
}


void writeTRK() {
      digitalWrite(READY_PIN,LOW);
       digitalWrite(LVC245_EN,HIGH);
    // Serial.println("Start Write>>>>>>>");
      fluxout0=-1;fluxout1=-1;
     pio_sm_put_blocking(pio, sm_index_pulse,
                        2000); //  put index high for 2ms (out of 200ms)  
    //for (size_t i = 0; i < flux_count_long; i++) {
    int nflux=0;
    while(!digitalRead(WRGATE_PIN)) { 
     // int f = fluxin0;
     // if (f < 0)
     //   break;
      uint32_t wd = pio_sm_get_blocking(pio, sm_fluxin);
      if (nflux>=40)flux_datain0[0][nflux-40]=__builtin_bswap32(wd);
      nflux++;
    }
    
    // terminate index pulse if ongoing
    pio_sm_exec(pio, sm_index_pulse,
                0 | offset_index_pulse); // JMP to the first instruction
    
    //digitalWrite(LVC245_EN,LOW);
    Serial.printf("\n flux len %d\n",nflux);
    for (size_t i = 0; i < nflux; i++) {
      Serial.print(flux_datain0[0][i],HEX);
    }
      int sector_count = cur_format->sectors;
      int side_count = cur_format->sides;
      int sector_size = 128 << cur_format->n;
      size_t offset = sector_size * sector_count * side_count * trackno0;
      size_t count = sector_size * sector_count;
      int dummy_byte = trackno0 * side_count;
      decode_track0(0, trackno0);
        
// terminate index pulse if ongoing
    Serial.println("<<<<<<<<<<<<End Write");
    fluxin0=-1;
    writeon0=0;
    //digitalWrite(READY_PIN,HIGH);
    
}

void openFile0() {
  bool rewound = false;
    file0.close();
    fluxout0 = -1; 
    fluxin0 = -1;
    char path[512];
    strcpy(path,curDir);
    strcat(path,filename0);
    auto res0 = file0.open(path);  
    if (!res0) {
      Serial.print("*********** open error in file:");
      Serial.println(path);
    } else {
      if (checkExt(filename0,"HFE")) {
        HFE0_On=true;
        get_HFE_track0();
        Serial.println("HFE0 On");
      } 
      Serial.print("opened 0: ");Serial.println(path);
    }
}
void openFile1() {
  bool rewound = false;
    file1.close();
    fluxout1 = -1;     
    char path[512];
    strcpy(path,curDir);
    strcat(path,filename1);
     auto res1 = file1.open(path); 
     if (!res1) {
      Serial.print("*********** open error in file:");
      Serial.println(path);
    } else {
      if (checkExt(filename1,"HFE")) {
        HFE1_On=true;
        get_HFE_track1();
        Serial.println("HFE1 On");
      } 
      Serial.print("opened 1: ");Serial.println(path);
    }
}



void handleRoot() {
  static int cnt = 0;
  digitalWrite(led, 1);

  StreamString temp;
  StreamString tmpfile;
  StreamString tmpdir;
  temp.reserve(12000); // Preallocate a large chunk to avoid memory fragmentation
  tmpdir.reserve(8000); 
  if (refreshOn) {
    temp.printf("<html><head><meta http-equiv='refresh' content='5'/>");
  } else {
    temp.printf("<html><head>");
  }
  temp.printf("<title>X68KFDPi2</title>\
    <style>body { background-color: #cccccc; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; }</style></head>\
    <body><h1>X68KFDPi2 by Andrea Ottaviani - v.1.0 (02/2025)</h1>\
    <div style='display: inline-block;'><form action='./eject0'style='display: inline;'>Drive %d: %s   ->trk: %d <input type='submit' value='Eject' ></form>\
    <form action='./default0' style='display: inline;'><input type='submit' value='Make default' ></form> \
    <br><div style='display: inline-block;'><form action='./eject1'style='display: inline;'>Drive %d: %s   ->trk: %d <input type='submit' value='Eject' ></form> \
   <form action='./default1' style='display: inline;'><input type='submit' value='Make default' ></form> </div>\
    <p>Current directory: ==== %s ====</p></form>",numdrv0,filename0,curtrack0,numdrv1,filename1,curtrack1,curDir);
  temp.printf("<form action='./refresh'><label for='refresh'>Auto refresh page (5s):</label><select id='refresh' name='refresh'>");
  if (refreshOn) {
    temp.printf("<option value='ON'>ON</option><option value='OFF'>OFF</option></select><input type='submit'></form>");
  } else {
    temp.printf("<option value='ON'>ON</option><option value='OFF'selected>OFF</option></select><input type='submit'></form>");
  }
  file2.close();
 
  dir.close();
  dir.open(curDir);
  int dircounter=0,filecounter=0;
  if (subdirOn) {
    tmpdir.printf("<form action='./radiodirs'> <input type='radio' id='UP' name='dirlist' value='UP'><label for='UP'>[DIR UP] ..</label><br>");
    dircounter++;
  }
  while (true) {
    file2 = dir.openNextFile();
    if (!file2) break;

    char filename[256];
    memset(filename,0,sizeof(filename));
    file2.getName(filename, sizeof(filename));
    
    if (!file2.isHidden()) {
      if (checkExt(filename,"HDM")||checkExt(filename,"HFE")) {
        if (filecounter==0) tmpfile=tmpfile+"<form action='./radiofiles'>";
        tmpfile.printf(" <input type='radio' id='%s' name='filelist' value='%s'><label for='%s'>%s</label><br>",filename,filename,filename,filename);
        filecounter++;
      } else {
        if (file2.isDirectory()) {
          if (dircounter==0) tmpdir.printf("<form action='./radiodirs'>");
          tmpdir.printf(" <input type='radio' id='%s' name='dirlist' value='%s'><label for='%s'>[DIR] %s</label><br>",filename,filename,filename,filename);
        dircounter++;
        }
      }
    } 
  }  
      
  if (dircounter>0) { 
    temp.concat(tmpdir);
    temp.concat(" <input type='submit' value='Enter'></form>"); 
  }
  if (filecounter>0) {
    temp.concat(tmpfile);
    temp.concat("<label for='drivelist'>Insert in drive:</label><select id='drivelist' name='drivelist'><option value='0'>Drive 0</option><option value='1'>Drive 1</option></select>");
    temp.concat("<input type='submit' value='Insert'></form>");
  }
  temp.concat("</body></html>");
  //Serial.println(temp);
  server.send(200, "text/html", temp);
  digitalWrite(led, 0);
}


String ejback="<html><head><meta http-equiv='refresh' content='1; url=/' /><title>Ejected</title></head><body><!-- Back button link --><a href='javascript:history.back()'>Done!</a></body></html>";


// Funzione per leggere una riga da un file
int readRiga(File32 &file, char *buffer, size_t bufferSize) {
  int index = 0;
  int byteRead;
  memset(buffer,0,sizeof(buffer));

  while ((byteRead = file.read()) != -1) {
    char c=byteRead;
    //Serial.print(c);Serial.print("-");Serial.println(byteRead,HEX);
    // Se incontri un carattere di nuova riga o ritorno a capo, esci dal ciclo
    if ((byteRead == 0x0A || byteRead == 0x0D)) {
        break;
    }

    // Aggiungi il byte letto al buffer
     buffer[index++] = (char)byteRead;

    // Se il buffer è pieno, esci dal ciclo
    if (index >= (bufferSize - 1)) {
      break;
    }
  }
  
  // Aggiungi il terminatore di stringa al buffer
  buffer[index] = 0xd;
  //Serial.print("readriga:");Serial.print(buffer);Serial.println("#");
  if ((index>=(bufferSize-1))||(byteRead==-1)) {
    return 0;
  } else {
  return index;
  }
}

void handleDefault0() {
    file2.close();
    auto res = file2.open("wifi.cfg");  
    SD.remove("tmp.cfg");
    tempfile.open("tmp.cfg",O_CREAT | O_WRITE);
    
    if (!res) {
      Serial.println("wifi.cfg missing!");
      while(1);
    }
    
    char buf[255];
    memset(buf,0,sizeof(buf));
    if (readRiga(file2,buf,sizeof(buf))) { //SSID
      tempfile.write(buf);
    } 
    memset(buf,0,sizeof(buf));
    if (readRiga(file2,buf,sizeof(buf))) { //PASSWORD
      tempfile.write(buf);
    }
    char fullpath[255];
    memset(fullpath,0,sizeof(fullpath));
    memset(buf,0,sizeof(buf));
    readRiga(file2,buf,sizeof(buf)); //default0
    //Serial.print("Fullpath0 letto:");Serial.print(buf);Serial.println("*");
    if (filename0[0]!=0) strcat(fullpath,curDir);
    strcat(fullpath,filename0);
    tempfile.write(fullpath);
    tempfile.write(0x0d);
    //Serial.print("Fullpath0 scritto:");Serial.print(fullpath);Serial.println("*");
   
    memset(buf,0,sizeof(buf));
    int pos=readRiga(file2,buf,sizeof(buf)); //default1
    if (!pos) buf[0]=0xd;
    tempfile.write(buf);
    //Serial.print("Fullpath1:");Serial.print(buf);Serial.println("*");
    
    file2.close();
    tempfile.close();
    SD.remove("wifi.cfg");
    file2.open("wifi.cfg",O_CREAT | O_WRITE); 
    res=tempfile.open("tmp.cfg",O_READ);
    if (!res) {
      Serial.println("Error open tmp.cfg");
    } 
    
    for (int i=0;i<4;i++) {
      memset(buf,0,sizeof(buf));
      int pos=readRiga(tempfile,buf,sizeof(buf));
      //buf[pos]=0;
      file2.write(buf);
    } 
    file2.close();
    tempfile.close();
   
    
    server.send(200,"text/html",ejback);
}

void handleDefault1() {
     file2.close();
    auto res = file2.open("wifi.cfg");  
    SD.remove("tmp.cfg");
    tempfile.open("tmp.cfg",O_CREAT | O_WRITE);
    
    if (!res) {
      Serial.println("wifi.cfg missing!");
      while(1);
    }
    
    char buf[255];
    memset(buf,0,sizeof(buf));
    if (readRiga(file2,buf,sizeof(buf))) { //SSID
      tempfile.write(buf);
    } 
    memset(buf,0,sizeof(buf));
    if (readRiga(file2,buf,sizeof(buf))) { //PASSWORD
      tempfile.write(buf);
    
    }
   
    memset(buf,0,sizeof(buf));
    int pos=readRiga(file2,buf,sizeof(buf)); //default1
    if (!pos) buf[0]=0xd;
    tempfile.write(buf);
    //Serial.print("Fullpath0:");Serial.print(buf);Serial.println("*");
    
    char fullpath[255];
    memset(fullpath,0,sizeof(fullpath));
    memset(buf,0,sizeof(buf));
    readRiga(file2,buf,sizeof(buf)); //default0
    //Serial.print("Fullpath1 letto:");Serial.print(buf);Serial.println("*");
    if (filename0[1]!=0) strcat(fullpath,curDir);
    strcat(fullpath,filename1);
    tempfile.write(fullpath);
    tempfile.write(0x0d);
    //Serial.print("Fullpath1 scritto:");Serial.print(fullpath);Serial.println("*");
     
    file2.close();
    tempfile.close();
    SD.remove("wifi.cfg");
    file2.open("wifi.cfg",O_CREAT | O_WRITE);  
    res=tempfile.open("tmp.cfg",O_READ);
    if (!res) {
      Serial.println("Error open tmp.cfg");
    } 
    
    for (int i=0;i<4;i++) {
      memset(buf,0,sizeof(buf));
      int pos=readRiga(tempfile,buf,sizeof(buf));
      //buf[pos]=0;
      file2.write(buf);
    } 
    file2.close();
    tempfile.close();
   
    
    server.send(200,"text/html",ejback);
}

void handleEject0() {
     digitalWrite(led, 1);
    memset(filename0,0,sizeof(filename0));
    file0.close(); 
    fluxout0=-1;
    fluxin0=-1;
    server.send(200,"text/html",ejback);
   digitalWrite(led, 0);
}

void handleEject1() {
     digitalWrite(led, 1);
     memset(filename1,0,sizeof(filename1));
     file1.close();
     fluxout1=-1;
     server.send(200,"text/html",ejback);
    digitalWrite(led, 0);
}
void handleNotFound() {
  digitalWrite(led, 1);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }

  server.send(404, "text/plain", message);
  digitalWrite(led, 0);
}


void setup() {
#if defined(FLOPPY_DIRECTION_PIN)
  pinMode(FLOPPY_DIRECTION_PIN, OUTPUT);
  digitalWrite(FLOPPY_DIRECTION_PIN, LOW); // we are emulating a floppy
#endif
#if defined(FLOPPY_ENABLE_PIN)
  pinMode(FLOPPY_ENABLE_PIN, OUTPUT);
  digitalWrite(FLOPPY_ENABLE_PIN, LOW); // do second after setting direction
#endif

  offset_fluxin = pio_add_program(pio, &fluxin_compact_program);
  sm_fluxin = pio_claim_unused_sm(pio, true);
  fluxin_compact_program_init(pio, sm_fluxin, offset_fluxin, FLUX_IN_PIN, 1000);
  
  offset_fluxout = pio_add_program(pio, &fluxout_compact_program);
  sm_fluxout = pio_claim_unused_sm(pio, true);
  fluxout_compact_program_init(pio, sm_fluxout, offset_fluxout, FLUX_OUT_PIN,1000);

  offset_index_pulse = pio_add_program(pio, &index_pulse_program);
  sm_index_pulse = pio_claim_unused_sm(pio, true);
  index_pulse_program_init(pio, sm_index_pulse, offset_index_pulse, INDEX_PIN,
                           1000);
  
  pinMode(DIR_PIN, INPUT_PULLUP);
  pinMode(STEP_PIN, INPUT_PULLUP);
  pinMode(SIDE_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PIN, INPUT_PULLUP);
  pinMode(SELECT_PIN0, INPUT_PULLUP);
  pinMode(SELECT_PIN1, INPUT_PULLUP);
  pinMode(SELECT_PIN2, INPUT_PULLUP);
  pinMode(SELECT_PIN3, INPUT_PULLUP);
  pinMode(WRGATE_PIN, INPUT_PULLUP);
  //pinMode(WRDATA_PIN, INPUT_PULLUP);
  pinMode(TRK0_PIN, OUTPUT);
  pinMode(READY_PIN, OUTPUT);
  pinMode(PROT_PIN, OUTPUT);  
  digitalWrite(READY_PIN, HIGH); // active low
  digitalWrite(PROT_PIN, HIGH); // active low
  pinMode(DF0_EN, INPUT);
  pinMode(DF1_EN, INPUT);
  pinMode(INTEXT,INPUT_PULLUP);
  pinMode(LVC245_EN,OUTPUT);
  digitalWrite(LVC245_EN, HIGH); // Output when HIGH
 
  
  memset(filename0,0,sizeof(filename0));
  memset(filename1,0,sizeof(filename1)); 
 
   early_setup_done = true;

Serial.begin(115200);
 
#if defined(WAIT_SERIAL)
  while (!Serial) {
  }
  Serial.println("Serial connected");
#endif
delay(200);
if (digitalRead(INTEXT)) { //poivia
//if (0) { //poivia debug: 1 to force drive 0/1, 0 for drive 2/3
     SELECT_PINA=SELECT_PIN0;
     SELECT_PINB=SELECT_PIN1;
     numdrv0=0;numdrv1=1;
     Serial.println("Drive 0/1");
  } else {
     SELECT_PINA=SELECT_PIN2;
     SELECT_PINB=SELECT_PIN3;
     numdrv0=2;numdrv1=3;
     Serial.println("Drive 2/3");
  }  
attachInterrupt(digitalPinToInterrupt(STEP_PIN), onStep, FALLING);
//attachInterrupt(digitalPinToInterrupt(WRGATE_PIN), writeTRK, FALLING);

if (!SD.begin(SD_CONFIG)) {
    Serial.println("SD card initialization failed");
    while(1) {
      digitalWrite(led, 1);delay(800);
      digitalWrite(led, 0);delay(300);
    }
  } else if (!dir.open("/")) {
    Serial.println("SD card directory could not be read");
    delay(2000);
  } else {
    Serial.println("SD Card ok");
  }
  setFormat(1261568);
  
    auto res = file2.open("wifi.cfg");  
    
    if (!res) {
        Serial.println("No wifi.cfg found, using default..");
        return;
      }
    char buf[255];
    int pos;
    pos=readRiga(file2,buf,sizeof(buf));
    if (!pos) {
      Serial.println("No valid SSID found");
    } else {
      //Serial.print("SSID len:");Serial.println(pos);
      memcpy(ssid,buf,pos);
    }
    pos=readRiga(file2,buf,sizeof(buf));
    if (!pos) {
      Serial.println("No valid PASSWORD found");
    } else {
      //Serial.print("PSWD len:");Serial.println(pos);
      memcpy(password,buf,pos);
    }
    pos=readRiga(file2,buf,sizeof(buf));
    if (!pos) {
      Serial.println("No default image for Drive0");
      memset(filename0,0,sizeof(filename0));
    } else {
      Serial.print("Def.file0 len:");Serial.println(pos);
      memcpy(filename0,buf,pos);
      Serial.printf("Default file0: %s\n",filename0);
      openFile0();
    }
    memset(buf,0,sizeof(buf));
    pos=readRiga(file2,buf,sizeof(buf));
    if (!pos) {
      Serial.println("No default image for Drive1");
      memset(filename1,0,sizeof(filename1));
    } else {
      Serial.print("Def.file1 len:");Serial.println(pos);
      memcpy(filename1,buf,pos);
      openFile1();
      Serial.printf("Default file1: %s\n",filename1);
    }

  WiFi.mode(WIFI_STA);
  //Serial.print(ssid);Serial.println("*"); 
  //Serial.print(password);Serial.println("*");  
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  #ifdef WAIT_WIFI
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  #endif

  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  if (MDNS.begin("picow")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);
  server.on("/inline", []() {
    server.send(200, "text/plain", "this works as well");
  });
  server.on("/eject0",handleEject0);
  server.on("/eject1",handleEject1);
  server.on("/default0",handleDefault0);
  server.on("/default1",handleDefault1);
  server.on("/radiofiles",handleFilelist);
  server.on("/radiodirs",handleDirlist);
  server.on("/refresh",handleRefresh);  
  
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");

}

void handleFilelist() {
    digitalWrite(led, 1);
    /*
   Serial.println(server.uri());
   for (uint8_t i = 0; i < server.args(); i++) {
    Serial.print("i:");Serial.print(i);Serial.print("name:");Serial.print(server.argName(i));
    Serial.print("value:");Serial.print(server.arg(i));
    Serial.println(server.argName(i) + ": " + server.arg(i));
  }
  */
  String sfilename= server.arg(0);
  //Serial.println(sfilename);
  String sdrive=server.arg(1);
  //Serial.println(sdrive);
  char drive[2];
  sdrive.toCharArray(drive,2);
  if (!strcmp(drive,"0")) {
    sfilename.toCharArray(filename0,sizeof(filename0)); 
    openFile0();
    Serial.print(filename0);Serial.println(" opened..");
  }
   if (!strcmp(drive,"1")) {
    sfilename.toCharArray(filename1,sizeof(filename1)); 
    openFile1();
     Serial.print(filename1);Serial.println(" opened..");
  }
  //Serial.print("Drive:");Serial.println(drive);
  //Serial.print("file0:");Serial.println(filename0);
  //Serial.print("file1:");Serial.println(filename1);
  server.send(200, "text/html", ejback);
  digitalWrite(led, 0);
}

void handleDirlist() {

  digitalWrite(led, 1);

  String sdirname= server.arg(0);
  char dirname[256];
  sdirname.toCharArray(dirname,256);
  Serial.println(dirname);
  if (!(strcmp(dirname,"UP"))) {
    Serial.println("dir up!");
    int len = strlen(curDir);
	  curDir[len]=0;
    len--;
 	  if (len>0) {
		  while (len && curDir[--len] != '/');
		  curDir[len] = '/';
      curDir[len+1]=0;
	  }
    if (len==0) {
      curDir[0]='/';
      curDir[1]=0;
      subdirOn=false;
    } 
  } else {
    strcat(curDir,dirname);
    strcat(curDir,"/");
     subdirOn=true;
  }
  
  Serial.print("New path:");Serial.println(curDir);

  server.send(200, "text/html", ejback);
  digitalWrite(led, 0);
}


void handleRefresh() {
   digitalWrite(led, 1);
/*   Serial.println(server.uri());
   for (uint8_t i = 0; i < server.args(); i++) {
    Serial.print("i:");Serial.print(i);Serial.print("name:");Serial.print(server.argName(i));
    Serial.print("value:");Serial.print(server.arg(i));
    Serial.println(server.argName(i) + ": " + server.arg(i));
  }
  */
  String srefresh= server.arg(0);
  char refresh[3];
  Serial.println(refresh);
  srefresh.toCharArray(refresh,3);
  if (!strcmp(refresh,"ON")) {
    refreshOn=true;
    //Serial.print("Refresh: on");
  } else {
    refreshOn=false;
    //Serial.print("Refresh: off");
  }
  
  server.send(200, "text/html", ejback);
}

static void decode_track0(uint8_t head, uint8_t cylinder) {

  mfm_io_t io = {
      .encode_compact = true,
      .pulses = (uint8_t *)flux_datain0[head],
      .n_pulses = flux_count_long * sizeof(long),
      .sectors = track_data0,
      .n_sectors = cur_format->sectors,
      .head = head,
      .cylinder = cylinder,
      .n = cur_format->n,
      .settings = cur_format->is_fm ? &standard_fm : &standard_mfm,
  };
  size_t pos = decode_track_mfm(&io);
  Serial.printf("Decoded0 to %zu flux\n", pos);
}

static void encode_track0(uint8_t head, uint8_t cylinder) {

  mfm_io_t io = {
      .encode_compact = true,
      .pulses = (uint8_t *)flux_data0[head],
      .n_pulses = flux_count_long * sizeof(long),
      .sectors = track_data0,
      .n_sectors = cur_format->sectors,
      .head = head,
      .cylinder = cylinder,
      .n = cur_format->n,
      .settings = cur_format->is_fm ? &standard_fm : &standard_mfm,
  };
  size_t pos = encode_track_mfm(&io);
  //Serial.printf("Encode head %d track %d \n",head,cylinder);
   
}

static void encode_track1(uint8_t head, uint8_t cylinder) {

  mfm_io_t io = {
      .encode_compact = true,
      .pulses = (uint8_t *)flux_data1[head],
      .n_pulses = flux_count_long * sizeof(long),
      .sectors = track_data1,
      .n_sectors = cur_format->sectors,
      .head = head,
      .cylinder = cylinder,
      .n = cur_format->n,
      .settings = cur_format->is_fm ? &standard_fm : &standard_mfm,
  };
  size_t pos = encode_track_mfm(&io);
 // Serial.printf("Encoded1 to %zu flux\n", pos);
}

uint32_t swap32(uint32_t value) {
      uint32_t result = 0;
    for (int i = 0; i < 32; i++) {
        result <<= 1;           // Sposta i bit a sinistra
        result |= (value & 1);  // Prende il bit meno significativo e lo aggiunge
        value >>= 1;            // Sposta il valore a destra per il prossimo bit
    }
    return result;
}

uint8_t swap8(uint8_t value) {
      uint8_t result = 0;
    for (int i = 0; i < 8; i++) {
        result <<= 1;           // Sposta i bit a sinistra
        result |= (value & 1);  // Prende il bit meno significativo e lo aggiunge
        value >>= 1;            // Sposta il valore a destra per il prossimo bit
    }
    return result;
}

static void encode_track_HFE0(uint8_t head, uint8_t cylinder) {
// flux_data[head] must be filled from hfe track file

char x[256];
int offset=(Pic_track0[cylinder].offset)*0x200;
if (head==1) offset=offset+0x100;
int npulse=0;
int len=Pic_track0[cylinder].track_len;
//Serial.printf("Reading track: %d head: %d len: %d offset: %d\n",cylinder,head,len,offset);

while (npulse<=(max_flux_count_long)) {
  file0.seek(0);
  file0.seek(offset); 
  file0.readBytes(x,0x100);
  for (int i=0;i<64;i++) {
     uint32_t flu=((x[i*4+0])<<24);
     flu=flu|((x[i*4+1])<<16);
     flu=flu|((x[i*4+2])<<8);
     flu=flu|((x[i*4+3])<<0);
     uint32_t flu1=swap32(flu);
     if (npulse<max_flux_count_long) {
      flux_data0[head][npulse]=flu1;
     } 
        npulse++;
     }
      offset=offset+0x200;
  }  
}
static void encode_track_HFE1(uint8_t head, uint8_t cylinder) {
// flux_data[head] must be filled from hfe track file

  char x[256];
  int offset=(Pic_track1[cylinder].offset)*0x200;
  if (head==1) offset=offset+0x100;
  int npulse=0;
  int len=Pic_track1[cylinder].track_len;
  //Serial.printf("Reading track: %d head: %d len: %d offset: %d\n",cylinder,head,len,offset);

  while (npulse<=(max_flux_count_long)) {
    file1.seek(0);
    file1.seek(offset); 
    file1.readBytes(x,0x100);
    for (int i=0;i<64;i++) {
      uint32_t flu=((x[i*4+0])<<24);
      flu=flu|((x[i*4+1])<<16);
      flu=flu|((x[i*4+2])<<8);
      flu=flu|((x[i*4+3])<<0);
      uint32_t flu1=swap32(flu);
      if (npulse<max_flux_count_long) {
        flux_data1[head][npulse]=flu1;
      } 
      npulse++;
    }
    offset=offset+0x200;
  }  
}
void loop() {
  static int cached_trackno0 = -1;
  static int cached_trackno1 = -1;
  auto new_trackno0 = trackno0;
  auto new_trackno1 = trackno1;
  int motor_pin = !digitalRead(MOTOR_PIN);
  int select_pin0 = !digitalRead(SELECT_PINA);
  int select_pin1 = !digitalRead(SELECT_PINB);
  int inserted0 = digitalRead(DF0_EN);
  int inserted1 = digitalRead(DF1_EN);
  int side = !digitalRead(SIDE_PIN);

  auto enabled0 = (motor_pin && select_pin0);  
  auto enabled1 = (motor_pin && select_pin1);
  if (!filename0[0]) enabled0=0;
  if (!filename1[0]) enabled1=0;
  
  
  writeon0 = (enabled0 && !digitalRead(WRGATE_PIN));
  if (writeon0) {
   // writeTRK();
   // Serial.println("Write ON ++++");
  }
    
   //auto enabled = true;
  curtrack0=trackno0;curtrack1=trackno1;  
       
  if (!enabled0) { 
    fluxout0 = -1; cached_trackno0 = -1; new_trackno0=0;trackno0=0;
    fluxin0 = -1;
    //Serial.println("flushed 0 .......");
    } // poivia
  if (!enabled1) { 
    fluxout1 = -1; cached_trackno1 = -1; new_trackno1=0;trackno1=0;
    //Serial.println("flushed 1 .......");
    } 
   //poivia
  static bool old_enabled0 = false, old_enabled1 = false, old_select_pin0 = false,
              old_select_pin1 = false, old_motor_pin = false;
  static bool old_inserted0 = false, old_inserted1 = false;
 
  if (motor_pin != old_motor_pin) {
    Serial.printf("motor_pin -> %s\n", motor_pin ? "true" : "false");
    old_motor_pin = motor_pin;
  }
  
  if (select_pin0 != old_select_pin0) {
    Serial.printf("select_pin0 -> %s\n", select_pin0 ? "true" : "false");
    old_select_pin0 = select_pin0;
  }
  
  if (select_pin1 != old_select_pin1) {
    Serial.printf("select_pin1 -> %s\n", select_pin1 ? "true" : "false");
    old_select_pin1 = select_pin1;
  }
  
  if (enabled0 != old_enabled0) {
    Serial.printf("enabled0 -> %s\n", enabled0 ? "true" : "false");
    old_enabled0 = enabled0;
  }
  if (enabled1 != old_enabled1) {
    Serial.printf("enabled1 -> %s\n", enabled0 ? "true" : "false");
    old_enabled1 = enabled1;
  }
  
  if (inserted0 != old_inserted0) {
    Serial.printf("inserted0 -> %s\n", inserted0 ? "true" : "false");
    old_inserted0 = inserted0;
  }
  if (inserted1 != old_inserted1) {
    Serial.printf("inserted1 -> %s\n", inserted1 ? "true" : "false");
    old_inserted1 = inserted1;
  }

  if ((cur_format && new_trackno0 != cached_trackno0) &&(enabled0)) {
    fluxout0 = -1;
    fluxin0 = -1;
    //Serial.printf("Preparing flux0 data for track %d\n", new_trackno0);
    int sector_count = cur_format->sectors;
    int side_count = cur_format->sides;
    int sector_size = 128 << cur_format->n;
    size_t offset = sector_size * sector_count * side_count * new_trackno0;
    size_t count = sector_size * sector_count;
    int dummy_byte = new_trackno0 * side_count;
    file0.seek(offset);
    for (auto side = 0; side < side_count; side++) {
      int n = file0.read(track_data0, count);
      if (n != count) {
        Serial.println("Read file0 failed -- using dummy data");
        //make_dummy_data(side, new_trackno0, count);
      } else {
        if (!HFE0_On) {
          encode_track0(side, new_trackno0);
        } else {
          encode_track_HFE0(side, new_trackno0);
        }
      }
    }
   // Serial.println("flux0 data prepared");
    cached_trackno0 = new_trackno0;
  } 
  fluxout0 =
      (cur_format != NULL && enabled0 && cached_trackno0 == trackno0) ? side : -1;
  //fluxin0=fluxout0;
  
  if ((cur_format && new_trackno1 != cached_trackno1)&&(enabled1)) {
    fluxout1 = -1;
    //Serial.printf("------------->Preparing flux1 data for track %d\n", new_trackno1); 
    int sector_count = cur_format->sectors;
    int side_count = cur_format->sides;
    int sector_size = 128 << cur_format->n;
    size_t offset = sector_size * sector_count * side_count * new_trackno1;
    size_t count = sector_size * sector_count;
    int dummy_byte = new_trackno1 * side_count;
    file1.seek(offset);
    for (auto side = 0; side < side_count; side++) {
      int n = file1.read(track_data1, count);
      if (n != count) {
        Serial.println("Read file1 failed -- using dummy data");
      //make_dummy_data(side, new_trackno0, count);
      } else {
        if (!HFE1_On) {
          encode_track1(side, new_trackno1);
        } else {
          encode_track_HFE1(side, new_trackno1);
        }
      }
    }
    //Serial.println("flux1 data prepared");
    cached_trackno1 = new_trackno1;  
  }
  fluxout1 =
   (cur_format != NULL && enabled1 && cached_trackno1 == trackno1) ? side : -1;
  fluxin1=fluxout1;
  // this is not correct handling of the ready/disk change flag. on my test
  // computer, just leaving the pin HIGH works, while immediately reporting LOW
  // on the "ready / disk change:
  //digitalWrite(READY_PIN, !motor_pin); 
   
  //if ((enabled0)||(enabled1)) {
  if ((inserted0)||(inserted1)) {
      //pinMode(READY_PIN,OUTPUT);
      digitalWrite(READY_PIN, LOW);
      digitalWrite(LVC245_EN, HIGH); // Output when HIGH
    } else {
     // pinMode(READY_PIN,INPUT);
     digitalWrite(LVC245_EN, LOW); // Output when HIGH
    }
       
  server.handleClient();
  server.client();
  MDNS.update();

}
