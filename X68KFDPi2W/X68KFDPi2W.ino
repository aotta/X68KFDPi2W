/////////////////////////////////////////////////////////////////////////////////////////
// X68KFDPI2W a floppy emulator AIO for Sharp X68000 by Andrea Ottaviani (c) 02/2025   //
// thanks to Adafruit for its great floppy library !!                                  
//
/////////////////////////////////////////////////////////////////////////////////////////

//#define WAIT_SERIAL
//#define WAIT_WIFI

// 1 = print every change of MOTOR / DRIVE SELECT on the serial port (very
// verbose: the X68000 selects and deselects the drives continuously)
#define DEBUG_BUS 0

// 1 = when a drive is deselected or the motor stops, the head goes back to
// track 0 and the track is read again from the SD at the next access.
// Needed by games that ask to swap disks (e.g. A-Jax): with 0 they report a
// disk change a few seconds after the request, even with no new disk.
// 0 = like a real drive: head position and track data are kept, and the reset
// to track 0 happens only when the disk is changed from the web page.
#define RESET_ON_DESELECT 1 // reset is needed by A-Jax and others games for change disk detection

// Experiments of 01/2026 ("poivia"), off = behaviour of the 03/2025 release.
// With both at 1 Dracula hangs after loading Human68k.
#define READY_FOLLOWS_MOTOR 0 // 1 = READY = !motor at every loop
#define DF_EN_PULLDOWN 0      // 1 = pull-down on the DF0_EN/DF1_EN inputs
#define BUS_ON_INSERTED 0     // 1 = READY and bus outputs (74LVC245) on while a
                              // disk is inserted (DF0_EN/DF1_EN); 0 = only while
                              // an emulated drive is enabled (motor + select)
#define TRK0_FOLLOWS_SELECT 0 // 1 = TRACK 0 output updated for the selected drive
                              // at every loop; 0 = updated at every step only
bool HFE0_On=false,HFE1_On=false;   

#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/pio.h>
#include "drive.pio.h"

#include "SdFat.h"
#include "pitches.h"
#include "logo.h"
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
char default0[128] = "";   // image loaded at boot (wifi.cfg), "" = empty drive
char default1[128] = "";
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

// Adafruit library debug output ("mark @ ..."): change to Serial.printf(__VA_ARGS__) to enable
#define DEBUG_PRINTF(...) ((void)0)
#define DEBUG_ASSERT(x)                                                        \
  do {                                                                         \
    if (!(x)) {                                                                \
      Serial.printf(__FILE__ ":%d: Assert fail: " #x "\n", __LINE__);          \
    }                                                                          \
  } while (0)
#include "mfm_impl.h"
#define IMG_LOG(...) Serial.printf(__VA_ARGS__)
#include "disk_image.h"

enum {
  max_flux_bits = 166667 // 300RPM (200ms rotational time), 1us bit times era 200000  
};
enum { max_flux_count_long = (max_flux_bits + 31) / 32 };

// Data shared between the two CPU cores
volatile int fluxout0; // side number 0/1 or -1 if no flux should be generated
volatile int fluxin0; // side number 0/1 or -1 if no flux should be generated
volatile int fluxout1; // side number 0/1 or -1 if no flux should be generated
volatile int fluxin1; // side number 0/1 or -1 if no flux should be generated
// Set via IRQ so must be volatile
volatile int trackno0;
volatile int trackno1;
volatile size_t flux_count_long =
    max_flux_count_long; // in units of uint32_ts (longs)
volatile uint32_t flux_data0[2]
                           [max_flux_count_long]; // one track of flux data for
                                                  // both sides of the disk
volatile uint32_t flux_data1[2]
                           [max_flux_count_long]; // one track of flux data for
                                                  // both sides of the disk

/////////////////////////////////////////////////////////////////////////////
// Write support
//
// WRDATA is captured by the PIO program "fluxin_capture", which measures the
// time between falling edges, and by a DMA channel that copies one byte per
// flux transition into wr_pulses[]. The DMA is always armed, so capture does
// not depend on how quickly core 0 reacts (WiFi, web server...).
//
// Core 1, which generates the read flux, polls WRGATE once per 32-bit word
// (every 32us at 500 kbit/s). Each time WRGATE is asserted and released it
// queues a "segment" with: drive, side, track, rotational position at the
// start of the write and the [start, end) range inside wr_pulses[].
//
// Core 0 (loop) decodes the segments with the mfm_impl.h routines, copies the
// valid sectors into the track buffer, re-encodes the track flux (so a
// verify read returns the new data straight away) and saves the track to the
// SD card when the write session ends (step, side/drive change, motor off,
// or WR_IDLE_MS without writes).
/////////////////////////////////////////////////////////////////////////////

// Same RAM previously used by flux_datain0[2][max_flux_count_long]
// (41672 bytes), minus the 1KB of the temporary sector buffer.
enum { WR_BUF_SIZE = 2 * max_flux_count_long * 4 - 1024 };
uint8_t wr_pulses[WR_BUF_SIZE] __attribute__((aligned(4)));
uint8_t wr_sector_tmp[1024];

// Capture clock: one count every 1/24us (PIO at 48MHz, 2 cycles per count)
#define FLUXIN_COUNT_HZ 24000000.0f
#define FLUXIN_OVERHEAD 2 // fixed cycles of the PIO loop, in counts
// Words still in the PIO TX FIFO/OSR when core 1 sees WRGATE (5 x 32us)
#define WR_POS_LATENCY_WORDS 5
#define WR_IDLE_MS 100
#define WR_QUEUE_LEN 16

struct wr_segment_t {
  int8_t drive;   // 0/1, -1 = write meant for a drive not emulated here
  int8_t side;
  uint8_t track;
  bool overflow;  // the capture buffer filled up before WRGATE was released
  int32_t pos;    // word index of the read flux when WRGATE fell, -1 = unknown
  uint32_t start, end; // range inside wr_pulses[]
  uint32_t limit;       // end of the buffer area available to this write
};

wr_segment_t wr_queue[WR_QUEUE_LEN];
volatile uint32_t wr_head = 0;       // written by core 1 only
volatile uint32_t wr_tail = 0;       // written by core 0 only, after a segment
                                     // has been fully processed
volatile bool wr_capture_ready = false;
volatile uint32_t wr_dropped = 0;
int wr_dma_chan = -1;

// Write session in progress (core 0 only)
struct {
  bool active;
  int drive, track, side;
  uint32_t dirty; // bitmap of written sectors
  uint32_t last_ms;
  bool need_encode; // read flux not yet regenerated
} wr_sess;

bool wr_allowed0 = false, wr_allowed1 = false;
// highest head position of each drive: 77 as in the 03/2025 release, more
// only for images with more than 78 cylinders
volatile int max_track[2] = {77, 77};
img_t img[2]; // format and sector layout of the image in each drive
bool img_err0 = false, img_err1 = false; // image could not be opened
uint32_t wr_last_ms[2] = {0, 0}; // last write per drive (web page LED)
// Track whose flux is currently in flux_data0/1 (-1 = none)
volatile int cached_trackno0 = -1;
volatile int cached_trackno1 = -1;

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

static inline uint32_t wr_dma_index() {
  return (uint32_t)((uint8_t *)dma_hw->ch[wr_dma_chan].write_addr - wr_pulses);
}

// Restart the capture at wr_pulses[pos], with room up to wr_pulses[limit]
// (core 1 only)
static uint32_t wr_limit = WR_BUF_SIZE;
static void __not_in_flash_func(wr_rewind)(uint32_t pos, uint32_t limit) {
  dma_channel_abort(wr_dma_chan);
  pio_sm_set_enabled(pio, sm_fluxin, false);
  pio_sm_clear_fifos(pio, sm_fluxin);
  pio_sm_restart(pio, sm_fluxin);
  pio_sm_exec(pio, sm_fluxin, pio_encode_jmp(offset_fluxin));
  dma_channel_set_write_addr(wr_dma_chan, wr_pulses + pos, false);
  dma_channel_set_trans_count(wr_dma_chan, limit - pos, true);
  wr_limit = limit;
  pio_sm_set_enabled(pio, sm_fluxin, true);
}

// Called by core 1 for every flux word sent (and continuously when idle).
// out_drive/out_side/out_pos describe what is being sent right now.
static void __not_in_flash_func(wr_poll)(int out_drive, int out_side, int out_pos) {
  static bool open = false;
  static wr_segment_t *seg;
  if (!wr_capture_ready)
    return;
  bool gate = !gpio_get(WRGATE_PIN); // active LOW
  if (gate && !open) {
    if (wr_head - wr_tail >= WR_QUEUE_LEN) { // queue full: lose this write
      wr_dropped++;
      open = true;
      seg = NULL;
      return;
    }
    // Choose where to capture this write: right after the writes still to
    // be decoded, or back at the beginning of the buffer if there is no room
    // left at the end (ring). With nothing left to decode the whole buffer is
    // available (track format). Only a few us of the 00 preamble are lost by
    // the restart, the sync mark is intact.
    {
      const uint32_t room = 9000; // largest sector write (1024 bytes, all 00)
      uint32_t lo = WR_BUF_SIZE, hi = 0;
      for (uint32_t q = wr_tail; q != wr_head; q++) {
        const wr_segment_t &u = wr_queue[q % WR_QUEUE_LEN];
        if (u.start < lo) lo = u.start;
        if (u.end > hi) hi = u.end;
      }
      if (wr_tail == wr_head)
        wr_rewind(0, WR_BUF_SIZE);
      else if (WR_BUF_SIZE - hi >= room)
        wr_rewind(hi, WR_BUF_SIZE);
      else if (lo >= room)
        wr_rewind(0, lo);
      // else: no free area big enough, keep appending (may overflow)
    }
    seg = &wr_queue[wr_head % WR_QUEUE_LEN];
    int drive = -1;
    if (!gpio_get(SELECT_PINA))
      drive = 0;
    else if (!gpio_get(SELECT_PINB))
      drive = 1;
    seg->drive = drive;
    seg->side = !gpio_get(SIDE_PIN);
    seg->track = (drive == 1) ? trackno1 : trackno0;
    seg->pos = (drive >= 0 && drive == out_drive && seg->side == out_side)
                   ? out_pos
                   : -1;
    seg->start = wr_dma_index();
    seg->limit = wr_limit;
    open = true;
  } else if (!gate && open) {
    open = false;
    if (seg) {
      uint32_t end = wr_dma_index();
      seg->end = end;
      seg->overflow = end >= seg->limit;
      __dmb();
      wr_head = wr_head + 1; // publish to core 0
    }
  }
}

static void __not_in_flash_func(send_revolution)(int drive) {
  volatile int &fluxout = drive ? fluxout1 : fluxout0;
  volatile uint32_t(*flux)[max_flux_count_long] = drive ? flux_data1 : flux_data0;
  pio_sm_put_blocking(pio, sm_index_pulse,
                      2000); // put index high for 2ms (out of 200ms)
  for (size_t i = 0; i < flux_count_long; i++) {
    int f = fluxout;
    if (f < 0)
      break;
    auto d = flux[f][i];
    pio_sm_put_blocking(pio, sm_fluxout, __builtin_bswap32(d));
    wr_poll(drive, f, i);
  }
  // terminate index pulse if ongoing
  pio_sm_exec(pio, sm_index_pulse,
              0 | offset_index_pulse); // JMP to the first instruction
}

void __not_in_flash_func(loop1)() {
  bool sent = false;
  // start a revolution only while the drive is still selected (03/2025)
  if (fluxout0 >= 0 && !gpio_get(SELECT_PINA)) {
    send_revolution(0);
    sent = true;
  }
  if (fluxout1 >= 0 && !gpio_get(SELECT_PINB)) {
    send_revolution(1);
    sent = true;
  }
  if (!sent)
    wr_poll(-1, -1, -1);
}

////////////////////////////////////////////////
// Code & data for core 0
// "UI", control signal handling & MFM encoding
////////////////////////////////////////////////

// trackno0/trackno1 are declared at the top (also used by core 1)

enum {
  max_sector_count = 9, // 9 x 1024: DIM 2HS/2HDE
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
    if (new_track1 < max_track[1])
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
      if (new_track0 < max_track[0])
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


void wr_flush(); // defined below, near loop()
static void wr_update_flux();

// Open an image read/write when possible. HFE images stay read-only (write
// support covers sector images only); the drive then reports write-protect.
static bool openImage(File32 &f, const char *path, bool is_hfe) {
  if (!is_hfe && f.open(path, O_RDWR))
    return true;
  return f.open(path, O_RDONLY);
}

// Open the image of a drive: RAW sector image (HDM, XDF...), D88 or HFE
static const char *imageTypeName(int d) {
  switch (img[d].type) {
  case IMG_RAW: return img[d].fmt_name;
  case IMG_D88: return "D88";
  case IMG_HFE: return "HFE";
  default: return "?";
  }
}

static void openDrive(int d) {
  char *fname = d ? filename1 : filename0;
  File32 &f = d ? file1 : file0;
  if (wr_sess.active && wr_sess.drive == d) wr_flush();
  f.close();
  // disk change: new flux, head back to track 0 (needed by games that ask
  // to swap disks, e.g. A-Jax)
  if (d) { fluxout1 = -1; cached_trackno1 = -1; trackno1 = 0; }
  else { fluxout0 = -1; fluxin0 = -1; cached_trackno0 = -1; trackno0 = 0; }
  bool &allowed = d ? wr_allowed1 : wr_allowed0;
  allowed = false;
  char path[512];
  if (fname[0] == '/') strcpy(path, fname); // full path (normal case)
  else { strcpy(path,curDir); strcat(path,fname); } // old wifi.cfg entries
  bool hfe = checkExt(fname, ".HFE");
  if (d) HFE1_On = hfe; else HFE0_On = hfe;
  bool res = openImage(f, path, hfe);
  if (res && f.isDirectory()) { f.close(); res = false; } // not an image
  if (res && !img_open(img[d], f, fname)) { f.close(); res = false; } // unsupported
  (d ? img_err1 : img_err0) = !res;
  if (!res) {
    Serial.print("*********** open error in file:");
    Serial.println(path);
    return;
  }
  if (hfe) {
    if (d) get_HFE_track1(); else get_HFE_track0();
    Serial.printf("HFE%d On\n", d);
  }
  allowed = !hfe && f.isWritable() && !img[d].wp;
  int cyls = img[d].type == IMG_RAW ? img[d].geom.cyls
           : img[d].type == IMG_D88 ? img[d].d88_cyls : 0;
  max_track[d] = cyls > 78 ? (cyls > 82 ? 81 : cyls - 1) : 77;
  Serial.printf("opened %d: %s [%s] %s\n", d, path, imageTypeName(d),
                allowed ? "(R/W)" : "(read only)");
}

void openFile0() { openDrive(0); }
void openFile1() { openDrive(1); }

/////////////////////////////////////////////////////////////////////////////
// Web interface
/////////////////////////////////////////////////////////////////////////////

// Disk images shown in the file list
static bool isImageFile(const char *name) {
  return checkExt(name, ".HDM") || checkExt(name, ".XDF") || checkExt(name, ".HFE") ||
         checkExt(name, ".D88") || checkExt(name, ".D68") || checkExt(name, ".88D") ||
         checkExt(name, ".DIM");
}

// Part of a path after the last '/'
static const char *baseName(const char *path) {
  const char *s = strrchr(path, '/');
  return s ? s + 1 : path;
}

// Page sent in chunks: no big String in RAM, whatever the number of files
static StreamString web_buf;
static void webFlush() {
  if (web_buf.length()) {
    server.sendContent(web_buf);
    web_buf = "";
  }
}
static void webOut(const char *s) {
  web_buf.concat(s);
  if (web_buf.length() > 1200)
    webFlush();
}
static void webHtml(const char *s) { // HTML escaped text
  char c[2] = {0, 0};
  for (; *s; s++) {
    switch (*s) {
    case '&': webOut("&amp;"); break;
    case '<': webOut("&lt;"); break;
    case '>': webOut("&gt;"); break;
    case '"': webOut("&quot;"); break;
    case '\'': webOut("&#39;"); break;
    default: c[0] = *s; webOut(c);
    }
  }
}
static void webUrl(const char *s) { // URL encoded text (for query strings)
  char c[4];
  for (; *s; s++) {
    unsigned char ch = *s;
    if (isalnum(ch) || ch == '-' || ch == '_' || ch == '.' || ch == '~') {
      c[0] = ch; c[1] = 0;
    } else {
      snprintf(c, sizeof(c), "%%%02X", ch);
    }
    webOut(c);
  }
}
static void webJson(const char *s) { // JSON string content
  char c[8];
  for (; *s; s++) {
    unsigned char ch = *s;
    if (ch == '"' || ch == '\\') { c[0] = '\\'; c[1] = ch; c[2] = 0; }
    else if (ch < 0x20) snprintf(c, sizeof(c), "\\u%04x", ch);
    else { c[0] = ch; c[1] = 0; }
    webOut(c);
  }
}

// After an action go back to the main page
static void sendBack() {
  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

// Message shown once at the top of the next page (result of an action)
static char ui_msg[96] = "";
static bool ui_msg_err = false;
static void uiMessage(bool err, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(ui_msg, sizeof(ui_msg), fmt, ap);
  va_end(ap);
  ui_msg_err = err;
  Serial.printf("WEB: %s\n", ui_msg);
}

// Drive status, set by loop() and read by the web page
volatile bool ui_active0 = false, ui_active1 = false;

static const char web_style[] =
  "<style>"
  ":root{--bg:#1d2026;--card:#2a2e36;--line:#3a3f4a;--txt:#e6e6e6;--dim:#9aa0aa;"
  "--acc:#e8a33d;--ok:#4caf50;--ro:#d9534f;--wr:#ff5c5c}"
  "*{box-sizing:border-box}"
  "body{margin:0;background:var(--bg);color:var(--txt);font:15px/1.4 system-ui,Segoe UI,Arial,sans-serif}"
  "header{padding:14px 16px;border-bottom:2px solid var(--acc);display:flex;justify-content:space-between;flex-wrap:wrap;gap:8px}"
  "header{align-items:flex-end}header span{color:var(--dim)}"
  ".logo{max-width:100%;height:auto;display:block}"
  "main{max-width:960px;margin:0 auto;padding:16px}"
  ".drives{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:12px}"
  ".drv{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:12px}"
  ".drv h2{margin:0 0 8px;font-size:16px;display:flex;align-items:center;gap:8px}"
  ".led{width:10px;height:10px;border-radius:50%;background:#444;display:inline-block}"
  ".led.on{background:var(--ok);box-shadow:0 0 6px var(--ok)}.led.wr{background:var(--wr);box-shadow:0 0 6px var(--wr)}"
  ".img{font-family:Consolas,monospace;word-break:break-all;min-height:1.4em}"
  ".meta{color:var(--dim);font-size:13px;margin:4px 0 10px}"
  ".tag{display:inline-block;padding:1px 6px;border-radius:4px;font-size:12px;margin-left:4px;border:1px solid}"
  ".tag.rw{color:var(--ok)}.tag.ro{color:var(--ro)}.tag.def{color:var(--acc)}.tag.fmt{color:var(--dim)}"
  "a.btn{display:inline-block;padding:5px 10px;border-radius:5px;background:#3a3f4a;color:var(--txt);"
  "text-decoration:none;font-size:13px;margin:2px 4px 2px 0}a.btn:hover{background:var(--acc);color:#000}"
  "a.btn.off{opacity:.4;pointer-events:none}"
  ".path{margin:18px 0 8px;font-family:Consolas,monospace;color:var(--acc)}"
  ".msg{margin:12px 0 0;padding:8px 12px;border-radius:6px;border:1px solid var(--ok);color:var(--ok)}"
  ".msg.err{border-color:var(--ro);color:var(--ro)}"
  ".new{display:flex;flex-wrap:wrap;align-items:center;gap:6px;margin:16px 0 0}"
  ".new span{color:var(--dim);font-size:13px}.new .hint{flex-basis:100%}"
  ".new input,.new select,.new button{background:#3a3f4a;color:var(--txt);border:1px solid var(--line);"
  "border-radius:5px;padding:5px 8px;font:inherit;font-size:13px}"
  ".new input{flex:1;min-width:140px}.new button{cursor:pointer}.new button:hover{background:var(--acc);color:#000}"
  "table{width:100%;border-collapse:collapse;background:var(--card);border-radius:8px;overflow:hidden}"
  "td{padding:7px 10px;border-top:1px solid var(--line)}tr:first-child td{border-top:0}"
  "td.n{word-break:break-all}td.s{color:var(--dim);white-space:nowrap;text-align:right;font-size:13px}"
  "td.a{white-space:nowrap;text-align:right}a.dir{color:var(--acc);text-decoration:none}"
  ".cur td{background:#33302a}.empty{color:var(--dim);padding:12px}"
  "</style>";

static void webDriveCard(int idx) {
  const char *fname = idx ? filename1 : filename0;
  int num = idx ? numdrv1 : numdrv0;
  bool loaded = fname[0] != 0;
  bool rw = idx ? wr_allowed1 : wr_allowed0;
  char tmp[160];
  snprintf(tmp, sizeof(tmp), "<div class='drv'><h2><span class='led' id='l%d'></span>Drive %d</h2>", idx, num);
  webOut(tmp);
  snprintf(tmp, sizeof(tmp), "<div class='img' id='i%d' title='", idx);
  webOut(tmp);
  webHtml(fname);
  webOut("'>");
  if (loaded) webHtml(*baseName(fname) ? baseName(fname) : fname);
  else webOut("<span class='meta'>&lt;empty&gt;</span>");
  webOut("</div><div class='meta'>");
  snprintf(tmp, sizeof(tmp), "Track <span id='t%d'>%d</span>", idx, idx ? curtrack1 : curtrack0);
  webOut(tmp);
  if (loaded && (idx ? img_err1 : img_err0))
    webOut("<span class='tag ro'>OPEN ERROR</span>");
  else if (loaded) {
    webOut(rw ? "<span class='tag rw'>R/W</span>" : "<span class='tag ro'>READ ONLY</span>");
    webOut("<span class='tag fmt'>");
    webOut(imageTypeName(idx));
    webOut("</span>");
  }
  if (!strcmp(fname, idx ? default1 : default0))
    webOut(loaded ? "<span class='tag def' title='Loaded at power on'>&#9733; DEFAULT</span>"
                  : "<span class='tag def' title='Empty at power on'>&#9733; DEFAULT (empty)</span>");
  webOut("</div>");
  snprintf(tmp, sizeof(tmp), "<a class='btn%s' href='/eject%d'>Eject</a>", loaded ? "" : " off", idx);
  webOut(tmp);
  snprintf(tmp, sizeof(tmp), "<a class='btn' href='/default%d' title='%s'>Make default</a></div>", idx,
           loaded ? "Load this disk at power on" : "Leave this drive empty at power on");
  webOut(tmp);
}

void handleRoot() {
  digitalWrite(led, 1);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  web_buf = "";
  web_buf.reserve(1400);

  webOut("<!DOCTYPE html><html><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width,initial-scale=1'>"
         "<title>X68KFDPi2W</title>");
  webOut(web_style);
  webOut("</head><body><header><img class='logo' src='/logo.png' alt='X68KFDPi2W' "
         "width='214' height='62'><span>Andrea Ottaviani &middot; v1.1</span></header>"
         "<main><div class='drives'>");
  webDriveCard(0);
  webDriveCard(1);
  webOut("</div>");
  if (ui_msg[0]) { // result of the last action, shown once
    webOut(ui_msg_err ? "<div class='msg err'>" : "<div class='msg'>");
    webHtml(ui_msg);
    webOut("</div>");
    ui_msg[0] = 0;
  }
  webOut("<form class='new' action='/create'><span>New disk</span>"
         "<input name='name' maxlength='40' placeholder='name' required>"
         "<select name='type'><option value='hdm'>.HDM</option>"
         "<option value='d88'>.D88</option></select>"
         "<button type='submit'>Create</button>"
         "<span class='hint'>blank 2HD 1232KB, to be formatted on the X68000</span></form>");
  webOut("<div class='path'>");
  webHtml(curDir);
  webOut("</div><table>");

  file2.close();
  dir.close();
  dir.open(curDir);
  int rows = 0;
  if (subdirOn) {
    webOut("<tr><td class='n' colspan='3'><a class='dir' href='/radiodirs?dirlist=UP'>&#8617; ..</a></td></tr>");
    rows++;
  }
  // first pass: directories, second pass: disk images
  for (int pass = 0; pass < 2; pass++) {
    dir.rewind();
    while (true) {
      file2 = dir.openNextFile();
      if (!file2) break;
      char filename[256];
      memset(filename, 0, sizeof(filename));
      file2.getName(filename, sizeof(filename));
      if (file2.isHidden()) { file2.close(); continue; }
      bool isdir = file2.isDirectory();
      if (pass == 0 && isdir) {
        webOut("<tr><td class='n' colspan='3'><a class='dir' href='/radiodirs?dirlist=");
        webUrl(filename);
        webOut("'>&#128193; ");
        webHtml(filename);
        webOut("</a></td></tr>");
        rows++;
      } else if (pass == 1 && !isdir && isImageFile(filename)) {
        char tmp[128];
        bool cur = !strcmp(baseName(filename0), filename) || !strcmp(baseName(filename1), filename);
        webOut(cur ? "<tr class='cur'><td class='n'>" : "<tr><td class='n'>");
        webHtml(filename);
        snprintf(tmp, sizeof(tmp), "</td><td class='s'>%lu KB</td><td class='a'>",
                 (unsigned long)(file2.fileSize() / 1024));
        webOut(tmp);
        for (int d = 0; d < 2; d++) {
          snprintf(tmp, sizeof(tmp), "<a class='btn' href='/radiofiles?drivelist=%d&amp;filelist=", d);
          webOut(tmp);
          webUrl(filename);
          snprintf(tmp, sizeof(tmp), "'>&#9654; FD%d</a>", d ? numdrv1 : numdrv0);
          webOut(tmp);
        }
        webOut("</td></tr>");
        rows++;
      }
      file2.close();
    }
  }
  if (!rows)
    webOut("<tr><td class='empty'>No disk images (.HDM .XDF .D88 .DIM .HFE) in this folder</td></tr>");
  webOut("</table></main><script>"
         "function u(){fetch('/status').then(r=>r.json()).then(s=>{s.d.forEach((d,i)=>{"
         "document.getElementById('t'+i).textContent=d.t;"
         "document.getElementById('l'+i).className='led'+(d.w?' wr':d.a?' on':'');"
         "var e=document.getElementById('i'+i);if(e.title!=d.f)location.reload();});})"
         ".catch(()=>{}).finally(()=>setTimeout(u,1000));}u();"
         "</script></body></html>");
  webFlush();
  server.sendContent(""); // end of the chunked reply
  web_buf = "";
  digitalWrite(led, 0);
}

// Create a blank (unformatted) 2HD 1232KB disk in the current folder:
// .HDM = raw sector image, .D88 = D88 with 77x2x8 sectors of 1024 bytes.
// All the data bytes are E5, like a disk not yet formatted by Human68k.
void handleCreate() {
  digitalWrite(led, 1);
  // keep only safe characters, max 40, no extension typed by the user
  char name[48];
  size_t n = 0;
  String req = server.arg("name");
  for (size_t i = 0; i < req.length() && n < 40; i++) {
    char c = req[i];
    if (isalnum((unsigned char)c) || c == ' ' || c == '-' || c == '_' || c == '(' ||
        c == ')' || c == '.')
      name[n++] = c;
  }
  name[n] = 0;
  while (n && (name[n - 1] == ' ' || name[n - 1] == '.')) name[--n] = 0; // trailing
  size_t lead = strspn(name, " ."); // leading spaces/dots (hidden files, "..")
  memmove(name, name + lead, n - lead + 1);
  char *dot = strrchr(name, '.');
  if (dot && (checkExt(name, ".HDM") || checkExt(name, ".D88") || checkExt(name, ".XDF")))
    *dot = 0;
  bool d88 = server.arg("type") == "d88";
  char path[600];
  snprintf(path, sizeof(path), "%s%s%s", curDir, name, d88 ? ".d88" : ".hdm");

  if (!name[0]) {
    uiMessage(true, "Invalid name");
  } else if (wr_sess.active) {
    uiMessage(true, "Busy writing a disk, try again");
  } else if (SD.exists(path)) {
    uiMessage(true, "%s already exists", baseName(path));
  } else {
    File32 nf;
    const uint32_t trk_bytes = 8 * (16 + 1024);              // D88 track
    const uint32_t size = d88 ? 0x2b0 + 154 * trk_bytes : 1261568;
    bool ok = nf.open(path, O_RDWR | O_CREAT | O_EXCL);
    if (ok) {
      nf.preAllocate(size); // contiguous if possible (faster, not required)
      uint8_t *blk = wr_sector_tmp; // 1KB, free when no write is in progress
      if (d88) {
        // header: name, write protect 0, media 2HD, size, track table
        uint8_t h[0x2b0];
        memset(h, 0, sizeof(h));
        strncpy((char *)h, name, 16);
        h[0x1b] = 0x20;
        for (int i = 0; i < 4; i++) h[0x1c + i] = size >> (8 * i);
        for (int t = 0; t < 164; t++) {
          uint32_t off = t < 154 ? 0x2b0 + t * trk_bytes : 0;
          for (int i = 0; i < 4; i++) h[0x20 + 4 * t + i] = off >> (8 * i);
        }
        ok = nf.write(h, sizeof(h)) == sizeof(h);
        memset(blk, 0xe5, 1024);
        for (int t = 0; t < 154 && ok; t++) {
          for (int r = 1; r <= 8 && ok; r++) {
            uint8_t sh[16] = {(uint8_t)(t / 2), (uint8_t)(t % 2), (uint8_t)r, 3, 8, 0,
                              0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x04}; // size 1024
            ok = nf.write(sh, 16) == 16 && nf.write(blk, 1024) == 1024;
          }
        }
      } else {
        memset(blk, 0xe5, 1024);
        for (uint32_t i = 0; i < size / 1024 && ok; i++)
          ok = nf.write(blk, 1024) == 1024;
      }
      ok = nf.close() && ok;
      if (!ok) SD.remove(path); // do not leave a broken image
    }
    if (ok)
      uiMessage(false, "%s created: insert it and format it on the X68000", baseName(path));
    else
      uiMessage(true, "Cannot create %s (SD full or write error)", baseName(path));
  }
  sendBack();
  digitalWrite(led, 0);
}

// Header logo, cached by the browser
void handleLogo() {
  server.sendHeader("Cache-Control", "max-age=86400");
  server.send(200, "image/png", (const char *)logo_png, logo_png_len);
}

// Live drive status for the page (JSON)
void handleStatus() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  web_buf = "";
  webOut("{\"d\":[");
  for (int i = 0; i < 2; i++) {
    char tmp[64];
    bool writing = wr_last_ms[i] && millis() - wr_last_ms[i] < 1500;
    snprintf(tmp, sizeof(tmp), "%s{\"t\":%d,\"a\":%d,\"w\":%d,\"f\":\"", i ? "," : "",
             i ? curtrack1 : curtrack0, (int)(i ? ui_active1 : ui_active0), (int)writing);
    webOut(tmp);
    webJson(i ? filename1 : filename0);
    webOut("\"}");
  }
  webOut("]}");
  webFlush();
  server.sendContent("");
}


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
      Serial.println("wifi.cfg missing: default not saved");
      tempfile.close();
      sendBack();
      return;
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
    strcat(fullpath,filename0); // already a full path (empty = no disk at boot)
    strcpy(default0, filename0);
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
   
    
    sendBack();
}

void handleDefault1() {
     file2.close();
    auto res = file2.open("wifi.cfg");  
    SD.remove("tmp.cfg");
    tempfile.open("tmp.cfg",O_CREAT | O_WRITE);
    
    if (!res) {
      Serial.println("wifi.cfg missing: default not saved");
      tempfile.close();
      sendBack();
      return;
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
    strcat(fullpath,filename1); // already a full path (empty = no disk at boot)
    strcpy(default1, filename1);
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
   
    
    sendBack();
}

void handleEject0() {
     digitalWrite(led, 1);
    if (wr_sess.active && wr_sess.drive == 0) wr_flush();
    memset(filename0,0,sizeof(filename0));
    file0.close(); 
    fluxout0=-1;
    fluxin0=-1;
    cached_trackno0=-1; trackno0=0; // disk removed
    wr_allowed0=false;
    img_err0=false;
    sendBack();
   digitalWrite(led, 0);
}

void handleEject1() {
     digitalWrite(led, 1);
     if (wr_sess.active && wr_sess.drive == 1) wr_flush();
     memset(filename1,0,sizeof(filename1));
     file1.close();
     fluxout1=-1;
     cached_trackno1=-1; trackno1=0; // disk removed
     wr_allowed1=false;
     img_err1=false;
     sendBack();
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

  // WRDATA capture: PIO measures the flux intervals, DMA stores them
  pinMode(FLUX_IN_PIN, INPUT_PULLUP);
  offset_fluxin = pio_add_program(pio, &fluxin_capture_program);
  sm_fluxin = pio_claim_unused_sm(pio, true);
  fluxin_capture_program_init(pio, sm_fluxin, offset_fluxin, FLUX_IN_PIN,
                              2 * FLUXIN_COUNT_HZ);
  wr_dma_chan = dma_claim_unused_channel(true);
  {
    dma_channel_config dc = dma_channel_get_default_config(wr_dma_chan);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_8);
    channel_config_set_read_increment(&dc, false);
    channel_config_set_write_increment(&dc, true);
    channel_config_set_dreq(&dc, pio_get_dreq(pio, sm_fluxin, false));
    dma_channel_configure(wr_dma_chan, &dc, wr_pulses, &pio->rxf[sm_fluxin],
                          WR_BUF_SIZE, true);
  }
  
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
#if DF_EN_PULLDOWN
  pinMode(DF0_EN, INPUT_PULLDOWN);
  pinMode(DF1_EN, INPUT_PULLDOWN);
#else
  pinMode(DF0_EN, INPUT);
  pinMode(DF1_EN, INPUT);
#endif
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
if (digitalRead(INTEXT)) { // jumper: HIGH = drives 0/1, LOW = drives 2/3
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
wr_capture_ready = true; // core 1 may now watch WRGATE (SELECT_PINA/B are set)

if (!SD.begin(SD_CONFIG)) {
    Serial.println("SD card initialization failed");
    int j=0;
    while(j<10) {
      digitalWrite(led, 1);delay(800);
      digitalWrite(led, 0);delay(300);
      j++;
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
        //poivia return;
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
      if (pos >= (int)sizeof(filename0)) pos = sizeof(filename0) - 1;
      memcpy(filename0,buf,pos);
      strcpy(default0, filename0);
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
      if (pos >= (int)sizeof(filename1)) pos = sizeof(filename1) - 1;
      memcpy(filename1,buf,pos);
      strcpy(default1, filename1);
      openFile1();
      Serial.printf("Default file1: %s\n",filename1);
    }
  Serial.println("Starting WIFI:");
  WiFi.mode(WIFI_STA);
  Serial.print(ssid);Serial.println("*"); 
  Serial.print(password);Serial.println("*");  
  WiFi.begin(ssid, password);
  Serial.println("-");

  // Wait for connection
  #ifdef WAIT_WIFI
  Serial.println("wait for wifi connection");
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
  server.on("/status",handleStatus);
  server.on("/logo.png",handleLogo);
  server.on("/create",handleCreate);
  
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
  // full path of the image: the default drive setting stays valid even
  // after moving to another folder in the browser
  String sfilename = String(curDir) + server.arg("filelist");
  String sdrive = server.arg("drivelist");
  Serial.printf("WEB insert: uri=%s args=%d drivelist='%s' filelist='%s'\n",
                server.uri().c_str(), server.args(), sdrive.c_str(),
                server.arg("filelist").c_str());
  if (server.arg("filelist").length() == 0) { // nothing selected
    sendBack();
    digitalWrite(led, 0);
    return;
  }
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
  sendBack();
  digitalWrite(led, 0);
}

void handleDirlist() {

  digitalWrite(led, 1);

  String sdirname= server.arg("dirlist");
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

  sendBack();
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
  char refresh[4];
  srefresh.toCharArray(refresh,sizeof(refresh));
  if (!strcmp(refresh,"ON")) {
    refreshOn=true;
    //Serial.print("Refresh: on");
  } else {
    refreshOn=false;
    //Serial.print("Refresh: off");
  }
  
  sendBack();
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

/////////////////////////////////////////////////////////////////////////////
// Write support - core 0 side
/////////////////////////////////////////////////////////////////////////////

static File32 &wr_file(int drive) { return drive ? file1 : file0; }
static uint8_t *wr_track_buf(int drive) { return drive ? track_data1 : track_data0; }
static uint8_t *drive_flux(int drive, int side) {
  return (uint8_t *)(drive ? flux_data1[side] : flux_data0[side]);
}

// Load one side of a track from the image and generate its read flux
static void load_and_encode_side(int drive, int track, int side) {
  uint8_t *buf = wr_track_buf(drive);
  if (img[drive].type == IMG_HFE) {
    if (drive) encode_track_HFE1(side, track); else encode_track_HFE0(side, track);
    return;
  }
  img_load_side(img[drive], wr_file(drive), track, side, buf, track_max_bytes);
  img_encode_side(img[drive].side[side], buf, drive_flux(drive, side),
                  flux_count_long * 4);
}

// Decode one write segment into the track buffer; returns the sectors updated
static uint32_t wr_decode_segment(const wr_segment_t &sg, uint8_t *track_buf) {
  uint32_t end = sg.end > WR_BUF_SIZE ? WR_BUF_SIZE : sg.end;
  if (end <= sg.start)
    return 0;
  uint8_t *pulses = wr_pulses + sg.start;
  size_t n_pulses = end - sg.start;
  // raw PIO values -> interval length in counts (what mfm_impl.h expects)
  for (size_t i = 0; i < n_pulses; i++) {
    int v = 255 - pulses[i] + FLUXIN_OVERHEAD;
    pulses[i] = v > 255 ? 255 : v;
  }
  // byte offset in the read flux where the write started (one word = 4 bytes)
  int raw_pos = sg.pos >= 0 ? 4 * (sg.pos - WR_POS_LATENCY_WORDS) : -1;
  const float t1 = FLUXIN_COUNT_HZ * cur_format->bit_time_ns * 1e-9f;
  return img_decode_write(pulses, n_pulses, sg.overflow, raw_pos,
                          img[sg.drive].side[sg.side], track_buf, wr_sector_tmp,
                          sizeof(wr_sector_tmp), t1);
}

// Save the track being written to the SD card
void wr_flush() {
  if (!wr_sess.active)
    return;
  wr_update_flux();
  if (wr_sess.dirty) {
    uint32_t t0 = millis();
    bool ok = img_write_sectors(img[wr_sess.drive], wr_file(wr_sess.drive),
                                wr_sess.side, wr_track_buf(wr_sess.drive), wr_sess.dirty);
    Serial.printf("WR: drive %d track %d side %d saved (sectors %02lx) %s, %lu ms\n",
                  wr_sess.drive, wr_sess.track, wr_sess.side,
                  (unsigned long)wr_sess.dirty, ok ? "OK" : "*** SD WRITE ERROR ***",
                  (unsigned long)(millis() - t0));
  }
  wr_sess.active = false;
  wr_sess.dirty = 0;
}

// Load the whole side from the image, so that written sectors are merged
// with the existing ones and the read flux can be regenerated
static bool wr_begin(int drive, int track, int side) {
  if (wr_sess.active && (wr_sess.drive != drive || wr_sess.track != track ||
                         wr_sess.side != side))
    wr_flush();
  if (wr_sess.active)
    return true;
  // data of the side being written (the sector list is the one in use for
  // the read flux, so the positions match)
  if (img[drive].side[side].track != track ||
      !img_reload_data(img[drive], wr_file(drive), side, wr_track_buf(drive))) {
    Serial.printf("WR: cannot read track %d side %d\n", track, side);
    return false;
  }
  wr_sess.active = true;
  wr_sess.drive = drive;
  wr_sess.track = track;
  wr_sess.side = side;
  wr_sess.dirty = 0;
  wr_sess.last_ms = millis();
  return true;
}

static void wr_handle_segment(const wr_segment_t &sg) {
  if (sg.drive < 0)
    return; // write to a drive emulated by someone else on the bus
  if (sg.end <= sg.start) {
    Serial.println("WR: write lost (capture buffer full)");
    return;
  }
  if (!(sg.drive ? wr_allowed1 : wr_allowed0)) {
    Serial.printf("WR: drive %d is read only, write ignored\n", sg.drive);
    return;
  }
  int cached = sg.drive ? cached_trackno1 : cached_trackno0;
  if (sg.track != cached) {
    Serial.printf("WR: track %d not loaded (%d), write ignored\n", sg.track, cached);
    return;
  }
  if (!wr_begin(sg.drive, sg.track, sg.side))
    return;
  uint32_t mask = wr_decode_segment(sg, wr_track_buf(sg.drive));
  wr_sess.last_ms = millis();
  wr_last_ms[sg.drive] = wr_sess.last_ms;
  Serial.printf("WR: drive %d track %d side %d pos %ld: %u pulses, sectors %02lx%s\n",
                sg.drive, sg.track, sg.side, (long)sg.pos,
                (unsigned)(sg.end - sg.start), (unsigned long)mask,
                sg.overflow ? " (overflow)" : "");
  if (!mask)
    return;
  wr_sess.dirty |= mask;
  wr_sess.need_encode = true; // done by wr_process() when the queue is empty
}

// Regenerate the read flux in place, so that a verify read returns the new
// data: the bytes outside the written sectors are identical, so the flux
// being sent by core 1 is not disturbed.
static void wr_update_flux() {
  if (!wr_sess.active || !wr_sess.need_encode)
    return;
  wr_sess.need_encode = false;
  img_encode_side(img[wr_sess.drive].side[wr_sess.side], wr_track_buf(wr_sess.drive),
                  drive_flux(wr_sess.drive, wr_sess.side), flux_count_long * 4);
}

// Consume the segments queued by core 1
static void wr_process() {
  while (wr_tail != wr_head) {
    __dmb();
    wr_segment_t sg = wr_queue[wr_tail % WR_QUEUE_LEN];
    wr_handle_segment(sg);
    __dmb();
    wr_tail = wr_tail + 1; // its pulses in wr_pulses[] are no longer needed
  }
  wr_update_flux();
  if (wr_dropped) {
    Serial.printf("WR: %lu writes lost (queue full)\n", (unsigned long)wr_dropped);
    wr_dropped = 0;
  }
}

void loop() {
  auto new_trackno0 = trackno0;
  auto new_trackno1 = trackno1;
  int motor_pin = !digitalRead(MOTOR_PIN);
  int select_pin0 = !digitalRead(SELECT_PINA);
  int select_pin1 = !digitalRead(SELECT_PINB);
  int inserted0 = digitalRead(DF0_EN);
  int inserted1 = digitalRead(DF1_EN);
  int side = !digitalRead(SIDE_PIN);
  
  //Serial.println(inserted0);
  
  auto enabled0 = (motor_pin && select_pin0);  
  auto enabled1 = (motor_pin && select_pin1);
  if (!filename0[0] || img_err0) enabled0=0;
  if (!filename1[0] || img_err1) enabled1=0;
  
  
  // Write support: decode what core 1 captured, then save the track when
  // the write session is over (before a new track can be loaded)
  wr_process();
  if (wr_sess.active) {
    int d = wr_sess.drive;
    bool en = d ? enabled1 : enabled0;
    int trk = d ? trackno1 : trackno0;
    // (with the drive deselected the data is kept: it is saved after
    // WR_IDLE_MS like after any write)
    if (trk != wr_sess.track || millis() - wr_sess.last_ms > WR_IDLE_MS)
      wr_flush();
  }

  // Write protect of the selected drive (active LOW)
  bool wprot = false;
  if (select_pin0 && filename0[0]) wprot = !wr_allowed0;
  else if (select_pin1 && filename1[0]) wprot = !wr_allowed1;
  digitalWrite(PROT_PIN, wprot ? LOW : HIGH);

#if TRK0_FOLLOWS_SELECT
  // TRACK 0 of the selected drive (active LOW): each drive has its own head
  if (select_pin0) digitalWrite(TRK0_PIN, trackno0 != 0);
  else if (select_pin1) digitalWrite(TRK0_PIN, trackno1 != 0);
#endif
    
   //auto enabled = true;
  curtrack0=trackno0;curtrack1=trackno1;
  ui_active0 = enabled0; ui_active1 = enabled1;  
       
  if (!enabled0) { // no flux while not selected / motor off
    fluxout0 = -1;
    fluxin0 = -1;
#if RESET_ON_DESELECT
    cached_trackno0 = -1; new_trackno0=0;trackno0=0; // poivia
#endif
  }
  if (!enabled1) {
    fluxout1 = -1;
#if RESET_ON_DESELECT
    cached_trackno1 = -1; new_trackno1=0;trackno1=0; // poivia
#endif
  }
  static bool old_inserted0 = false, old_inserted1 = false;
#if DEBUG_BUS
  static int old_bus = -1;
  int bus = motor_pin | (select_pin0 << 1) | (select_pin1 << 2);
  if (bus != old_bus) {
    Serial.printf("BUS: motor=%d sel0=%d sel1=%d trk=%d/%d\n", motor_pin, select_pin0,
                  select_pin1, trackno0, trackno1);
    old_bus = bus;
  }
#endif
  
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
    for (int sd = 0; sd < 2; sd++)
      load_and_encode_side(0, new_trackno0, sd);
    cached_trackno0 = new_trackno0;
  }
  fluxout0 =
      (cur_format != NULL && enabled0 && cached_trackno0 == trackno0) ? side : -1;
  //fluxin0=fluxout0;
  
  if ((cur_format && new_trackno1 != cached_trackno1)&&(enabled1)) {
    fluxout1 = -1;
    for (int sd = 0; sd < 2; sd++)
      load_and_encode_side(1, new_trackno1, sd);
    cached_trackno1 = new_trackno1;
  }
  fluxout1 =
   (cur_format != NULL && enabled1 && cached_trackno1 == trackno1) ? side : -1;
  fluxin1=fluxout1;
  // this is not correct handling of the ready/disk change flag. on my test
  // computer, just leaving the pin HIGH works, while immediately reporting LOW
  // on the "ready / disk change:
  
#if READY_FOLLOWS_MOTOR
  digitalWrite(READY_PIN, !motor_pin); //poivia
#endif
   
#if BUS_ON_INSERTED
  if ((inserted0)||(inserted1)) {
#else
  if ((enabled0)||(enabled1)) { // 03/2025: bus released when no drive is in use
#endif
      //pinMode(READY_PIN,OUTPUT);
      digitalWrite(READY_PIN, LOW);
      digitalWrite(LVC245_EN, HIGH); // Output when HIGH
   //   Serial.println("ready");
    } else {
     // pinMode(READY_PIN,INPUT);
      digitalWrite(LVC245_EN, LOW); // Output when HIGH
   //  Serial.println("not ready");
    }
       
  // While the X68000 is writing, keep core 0 free for the write handling
  if (!wr_sess.active) {
    server.handleClient();
    server.client();
    MDNS.update();
  }

}
