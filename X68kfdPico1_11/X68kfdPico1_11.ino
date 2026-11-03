/*
 * X68KFDINO - External FD adapter for Sharp X68000
 * by Andrea Ottaviani june 2023
 *
 * v1.10: the OPTION_SEL protocol runs on core 1 in a tight loop, so the
 * drive answers a ~2.5us strobe in well under 0.1us. Core 0 handles the
 * buttons, the LEDs and the serial log.
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

/DISK_IN_DRIVE is shared by all four drives: it reports the drive addressed
by the last OPTION_SEL and is held until the next one. When a drive that is
not ours is addressed it must go HIGH, otherwise the X68000 sees disks in
drives 2/3 (or 0/1) that do not exist.
 */

// 1 = always drives 0/1 (ignore the INTEXT jumper), 0 = read the jumper
// Like a real drive: after the eject command the disk takes a while to leave
// the drive, then /INT is asserted until the next access to that drive.
#define EJECT_DELAY_MS 0   // 0 = "no disk" right after the eject command (A-Jax checks it at once)
#define INT_MIN_MS 0      // 0 = cleared by the first access, like a real drive
#define INT_REPEAT_MS 100  // /INT not served: new falling edge every INT_REPEAT_MS
#define INT_GAP_US 200     // ... after releasing the line for INT_GAP_US
#define BUTTON_MS 30       // a button press counts only if held this long

// 1 = every second, print how many accesses the X68000 made on OPT0..OPT3
#define DIAG 0

#define FORCE_DRIVE01 0

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
#define DF0_EN             21  // "disk in drive 0" to the Pico W
#define DF1_EN             22  // "disk in drive 1" to the Pico W
#define INTLED             25
#define INTEXT             26  // jumper: HIGH = drives 0/1, LOW = drives 2/3

// ---- state shared between the cores ----
volatile bool disk[2]   = {false, false};  // disk inserted (owned by core 1)
volatile bool locked[2] = {false, false};
volatile bool blink[2]  = {false, false};
volatile uint32_t btn_req[2] = {0, 0};     // button presses posted by core 0
volatile uint32_t ev_eject[2] = {0, 0};    // counters for the log (core 0 prints)
volatile uint32_t strobes[2] = {0, 0};
volatile uint32_t opt_cnt[4] = {0, 0, 0, 0};   // accesses seen on OPT0..OPT3 (diagnostics)
volatile uint32_t int_rep = 0;
volatile uint32_t int_cnt = 0, ins_low_cnt = 0;  // /INT assertions, /DISK_IN_DRIVE low answers
volatile int32_t dbg_who = -1;
volatile bool core1_ready = false;
uint optPin[2];

void setup() {
  pinMode(OPT0, INPUT_PULLUP);
  pinMode(OPT1, INPUT_PULLUP);
  pinMode(OPT2, INPUT_PULLUP);
  pinMode(OPT3, INPUT_PULLUP);
  pinMode(Eject, INPUT_PULLUP);
  pinMode(BT_FD0, INPUT);
  pinMode(BT_FD1, INPUT);
  pinMode(LED_BLINK, INPUT_PULLUP); // Blink
  pinMode(EjectMSK, INPUT_PULLUP);  // LOCK
  pinMode(Motor, INPUT_PULLUP);
  digitalWrite(Inserted, HIGH); pinMode(Inserted, OUTPUT);
  digitalWrite(Error, HIGH);    pinMode(Error, OUTPUT);
  digitalWrite(FDDINT, HIGH);   pinMode(FDDINT, OUTPUT);
  pinMode(LEDFD0, OUTPUT);
  pinMode(LEDFD1, OUTPUT);
  pinMode(LED_FD0_RED, OUTPUT);
  pinMode(LED_FD1_RED, OUTPUT);
  pinMode(BT_FD0_INT, OUTPUT);
  pinMode(BT_FD1_INT, OUTPUT);
  digitalWrite(DF0_EN, LOW); pinMode(DF0_EN, OUTPUT);
  digitalWrite(DF1_EN, LOW); pinMode(DF1_EN, OUTPUT);
  pinMode(INTLED, OUTPUT);
  pinMode(INTEXT, INPUT);

  Serial.begin(115200);
#if FORCE_DRIVE01
  bool d01 = true;
#else
  bool d01 = digitalRead(INTEXT);
#endif
  optPin[0] = d01 ? OPT0 : OPT2;
  optPin[1] = d01 ? OPT1 : OPT3;
  core1_ready = true;
}

// ---------------- core 1: OPTION_SEL protocol ----------------
void setup1() {
  while (!core1_ready) tight_loop_contents();
}

void __attribute__((optimize("O2"))) __not_in_flash_func(loop1)() {
  const uint32_t mA = 1u << optPin[0], mB = 1u << optPin[1];
  const uint32_t mOpt = (1u << OPT0) | (1u << OPT1) | (1u << OPT2) | (1u << OPT3);
  const uint32_t mOther = mOpt & ~(mA | mB);
  const uint32_t mEj = 1u << Eject, mLk = 1u << EjectMSK, mBl = 1u << LED_BLINK;
  const uint32_t mIns = 1u << Inserted, mInt = 1u << FDDINT;
  int who = -1;                       // drive addressed by the last strobe, -1 = not ours
  bool d[2] = {disk[0], disk[1]};
  bool intp[2] = {false, false};      // interrupt pending per drive
  uint32_t int_min[2] = {0, 0};       // the interrupt is not cleared before this time (us)
  bool ejecting[2] = {false, false};  // eject accepted, disk still "in the drive"
  uint32_t ej_at[2] = {0, 0};         // time the disk leaves the drive (us)
  uint32_t acc[2] = {~0u, ~0u};       // AND of the inputs during the strobe (a line seen low -> bit 0)
  uint32_t req_done[2] = {btn_req[0], btn_req[1]};
  uint32_t prev = mOpt;               // OPT lines at the previous iteration (all inactive)
  bool timers = false;                // an eject or an interrupt is in progress

  for (;;) {
    uint32_t in = sio_hw->gpio_in;
    uint32_t opt = in & mOpt;

    if (opt == prev) {
      // fast path: nothing changed on the OPT lines
      if (!(opt & mA)) acc[0] &= in;
      if (!(opt & mB)) acc[1] &= in;
      if (!timers && btn_req[0] == req_done[0] && btn_req[1] == req_done[1]) continue;
    } else {
      // /DISK_IN_DRIVE first: status of the drive addressed by the LAST strobe.
      // It is held after the strobe ends (the X68000 may sample it late) and
      // goes HIGH (no disk) as soon as a drive that is not ours is addressed.
      if (!(opt & mA)) who = 0;
      else if (!(opt & mB)) who = 1;
      else if ((opt & mOther) != mOther) who = -1;
      if (who >= 0 && d[who]) sio_hw->gpio_clr = mIns; else sio_hw->gpio_set = mIns;

      const uint32_t m[2] = {mA, mB};
      for (int k = 0; k < 2; k++) {
        bool was = !(prev & m[k]), now = !(opt & m[k]);
        if (now && !was) acc[k] = in;               // start of the access
        else if (now) acc[k] &= in;
        else if (was) {                             // rising edge of OPTION_SEL: end of the access
          strobes[k]++;
          if (intp[k] && (int32_t)(time_us_32() - int_min[k]) >= 0) intp[k] = false; // cleared by the access
          locked[k] = !(acc[k] & mLk);
          blink[k] = !(acc[k] & mBl);
          if (!(acc[k] & mEj) && (acc[k] & mLk) && d[k] && !ejecting[k]) {  // eject only if not locked in the same access  // eject: the disk leaves after a while
            ejecting[k] = true; ej_at[k] = time_us_32() + EJECT_DELAY_MS * 1000u;
            ev_eject[k]++;
          }
        }
      }
      { uint32_t rise = opt & ~prev;                // OPT lines released now (end of an access)
        if (rise & (1u << OPT0)) opt_cnt[0]++;
        if (rise & (1u << OPT1)) opt_cnt[1]++;
        if (rise & (1u << OPT2)) opt_cnt[2]++;
        if (rise & (1u << OPT3)) opt_cnt[3]++;
        if (who >= 0 && d[who] && (opt & ~prev & mOpt) == 0) ins_low_cnt++;
        dbg_who = who; }
      prev = opt;
    }

    // insert/eject button posted by core 0
    for (int k = 0; k < 2; k++) {
      if (btn_req[k] != req_done[k]) {
        req_done[k] = btn_req[k];
        ejecting[k] = false;
        d[k] = !d[k]; disk[k] = d[k];
        if (d[k]) blink[k] = false;
        intp[k] = true; int_min[k] = time_us_32() + INT_MIN_MS * 1000u;
        if (who == k) { if (d[k]) sio_hw->gpio_clr = mIns; else sio_hw->gpio_set = mIns; }
      }
    }

    // end of the mechanical eject: status change -> interrupt
    uint32_t t = time_us_32();
    for (int k = 0; k < 2; k++) {
      if (ejecting[k] && (int32_t)(t - ej_at[k]) >= 0) {
        ejecting[k] = false;
        d[k] = false; disk[k] = false;
        intp[k] = true; int_min[k] = t + INT_MIN_MS * 1000u;
        if (who == k) sio_hw->gpio_set = mIns;
      }
    }

    // /INT output. The X68000 reacts to the falling edge: if the interrupt is
    // not served (no access to our drives) within INT_REPEAT_MS, release the
    // line for INT_GAP_US and assert it again to produce a new edge.
    { static bool int_was = false, gap = false; static uint32_t edge_t = 0, gap_t = 0;
      bool i = intp[0] || intp[1];
      uint32_t tn = time_us_32();
      if (!i) { gap = false; sio_hw->gpio_set = mInt; }
      else if (!int_was) { edge_t = tn; gap = false; int_cnt++; sio_hw->gpio_clr = mInt; }
      else if (gap) { if ((int32_t)(tn - gap_t) >= 0) { gap = false; edge_t = tn; int_rep++; sio_hw->gpio_clr = mInt; } }
      else if (tn - edge_t >= INT_REPEAT_MS * 1000u) { gap = true; gap_t = tn + INT_GAP_US; sio_hw->gpio_set = mInt; }
      int_was = i; }
    timers = ejecting[0] || ejecting[1] || intp[0] || intp[1];
  }
}

// ---------------- core 0: buttons, LEDs, log ----------------
void loop() {
  static uint32_t last_btn[2] = {0, 0};
  static bool prev_disk[2] = {false, false}, prev_lock[2] = {false, false}, prev_blink[2] = {false, false};
  static uint32_t prev_ej[2] = {0, 0};
  static bool started = false;
  const int btn[2] = {BT_FD0, BT_FD1};
  const int green[2] = {LEDFD0, LEDFD1}, red[2] = {LED_FD0_RED, LED_FD1_RED};
  const int intout[2] = {BT_FD0_INT, BT_FD1_INT}, en[2] = {DF0_EN, DF1_EN};
  uint32_t now = millis();

  if (!started && now > 2000) {
    started = true;
    Serial.printf("X68kfdPico1 v1.10 - drives %s\n", optPin[0] == OPT0 ? "0/1" : "2/3");
  }

#if DIAG
  { static uint32_t last = 0, pc[4] = {0, 0, 0, 0}, pi = 0, pl = 0;
    if (now - last >= 1000) {
      last = now;
      uint32_t c[4] = {opt_cnt[0], opt_cnt[1], opt_cnt[2], opt_cnt[3]}, i = int_cnt, l = ins_low_cnt, r = int_rep;
      static uint32_t pr = 0;
      if (c[0] != pc[0] || c[1] != pc[1] || c[2] != pc[2] || c[3] != pc[3] || i != pi || r - pr >= 10) {
        Serial.printf("[%lu] OPT0..3 +%lu +%lu +%lu +%lu  INT+%lu rep+%lu  diskLOW+%lu  disk=%d%d  INT=%d INS=%d who=%ld\n",
                      (unsigned long)(now / 1000), (unsigned long)(c[0] - pc[0]), (unsigned long)(c[1] - pc[1]),
                      (unsigned long)(c[2] - pc[2]), (unsigned long)(c[3] - pc[3]), (unsigned long)(i - pi), (unsigned long)(r - pr),
                      (unsigned long)(l - pl), disk[0], disk[1], digitalRead(FDDINT), digitalRead(Inserted), (long)dbg_who);
        for (int k = 0; k < 4; k++) pc[k] = c[k];
        pi = i; pl = l; pr = r;
      }
    }
  }
#endif
  bool motor_on = !digitalRead(Motor);
  bool blink_phase = (now / 250) & 1;

  for (int k = 0; k < 2; k++) {
    int n = (optPin[0] == OPT0 ? 0 : 2) + k;
    // debounce: the press counts once, after BUTTON_MS held, and the button
    // must be released before the next one (filters glitches, e.g. plugging USB)
    static uint32_t down_since[2] = {0, 0};
    static bool down[2] = {false, false}, done[2] = {false, false};
    if (digitalRead(btn[k])) {
      if (!down[k]) { down[k] = true; done[k] = false; down_since[k] = now; }
      else if (!done[k] && now - down_since[k] >= BUTTON_MS && now - last_btn[k] > 300) {
        done[k] = true; last_btn[k] = now;
        btn_req[k]++;
      }
    } else down[k] = false;
    bool dk = disk[k];
    digitalWrite(en[k], dk ? HIGH : LOW);
    if (dk != prev_disk[k]) {
      prev_disk[k] = dk;
      digitalWrite(INTLED, dk ? LOW : HIGH);
      if (ev_eject[k] != prev_ej[k]) { prev_ej[k] = ev_eject[k]; Serial.printf("DF%d: ejected by the X68000\n", n); }
      else Serial.printf("DF%d: %s (button)\n", n, dk ? "inserted" : "ejected");
    }
    bool lk = locked[k], bl = blink[k];
    if (lk != prev_lock[k]) { prev_lock[k] = lk; Serial.printf("DF%d: %s\n", n, lk ? "locked" : "unlocked"); }
    if (bl != prev_blink[k]) { prev_blink[k] = bl; Serial.printf("DF%d: blink %s\n", n, bl ? "on" : "off"); }

    bool g = dk || (bl && blink_phase);
    if (bl && !dk) g = blink_phase;
    digitalWrite(green[k], g ? HIGH : LOW);
    digitalWrite(intout[k], g ? HIGH : LOW);
    digitalWrite(red[k], (motor_on && (dk || lk)) ? HIGH : LOW);
  }
}
