/* ------------------------------------------------------------------
 *  dualParaLudPower  -  LGT8F328P (Nano 3 compatible)
 *  Power management + Mute master out + SoftSerial pre-off link
 *  + Debug su UART hardware (Serial)
 * ------------------------------------------------------------------
 *  Pin:
 *    PB0  LED_PIN       -> LED di stato
 *    PB1  BTN_PIN       -> pulsante (HIGH = premuto, pull-down est.)
 *    PB2  PWR_HOLD_PIN  -> latch alimentazione
 *    PB4  MUTE_PIN      -> mute master out
 *    PD2  SOFT_RX       -> SoftSerial RX  (link verso slave audio)
 *    PD3  SOFT_TX       -> SoftSerial TX  (link verso slave audio)
 *    PD0  Serial RX     -> debug (USB/UART)
 *    PD1  Serial TX     -> debug (USB/UART)
 * ------------------------------------------------------------------ */

#include <SoftwareSerial.h>

// ==================================================================
//  DEBUG
// ==================================================================
#define DEBUG_ENABLED   1          // 0 = nessun output di debug
#define DEBUG_BAUD      115200UL
#define DEBUG_DUMP_LINK 1          // 1 = dump byte per byte del link
#define DEBUG_CMDS      1          // 1 = comandi da seriale ('o','f','s','m')

#if DEBUG_ENABLED
  #define DBG(x)              do { Serial.print(x); } while (0)
  #define DBGLN(x)            do { Serial.println(x); } while (0)
  #define DBGF(...)           do { Serial.print(F(__VA_ARGS__)); } while (0)
  #define DBGF2(fmt, ...)     do { char _b[96]; snprintf_P(_b, sizeof(_b), PSTR(fmt), ##__VA_ARGS__); Serial.println(_b); } while (0)
#else
  #define DBG(x)
  #define DBGLN(x)
  #define DBGF(...)
  #define DBGF2(...)
#endif

// -------- Pinout --------------------------------------------------
#define LED_PIN        PB0
#define BTN_PIN        PB1
#define PWR_HOLD_PIN   PB2
#define MUTE_PIN       PB4

#define SOFT_RX        PD2
#define SOFT_TX        PD3

#define MUTE_ACTIVE    HIGH
#define MUTE_INACTIVE  LOW

// -------- Link SoftSerial ----------------------------------------
#define LINK_BAUD        9600
#define MSG_LEN          4

static const uint8_t MSG_PREREOFF[MSG_LEN] = { 'W', 'R', '&', '!' };

SoftwareSerial link(SOFT_RX, SOFT_TX);

// -------- Tempi (ms) ---------------------------------------------
#define LONG_PRESS_MS     3000UL
#define DEBOUNCE_MS         30UL
#define MUTE_SETTLE_MS     150UL
#define AMP_SETTLE_MS     1000UL
#define PREOFF_ACK_MS     3000UL
#define PREOFF_HOLD_MS     500UL
#define BOOT_DELAY_MS      300UL

// -------- Stato pulsante -----------------------------------------
static bool          btnStable    = false;
static bool          btnRawPrev   = false;
static unsigned long btnRawChange = 0;
static unsigned long btnPressTime = 0;
static bool          longFired    = false;

// -------- Stato sistema ------------------------------------------
static bool powerOn = false;
static bool busy    = false;

// ==================================================================
//  LED
// ==================================================================
static void blinkLed(uint8_t times, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < times; i++) {
    digitalWrite(LED_PIN, HIGH); delay(onMs);
    digitalWrite(LED_PIN, LOW);  delay(offMs);
  }
}
static void startupChime() { blinkLed(9, 200, 200); blinkLed(5, 400, 400); }
static void errorBlink()   { blinkLed(6, 80,  120); }

// ==================================================================
//  Debug helpers
// ==================================================================
static void dbgByteHex(uint8_t b, char c) {
#if DEBUG_ENABLED
  char buf[8];
  snprintf(buf, sizeof(buf), "0x%02X", b);
  Serial.print(buf);
  Serial.print('(');
  Serial.write((c >= 32 && c < 127) ? c : '.');
  Serial.print(") ");
#endif
}

static void dbgState() {
#if DEBUG_ENABLED
  DBGF("  [state] powerOn=");  DBG(powerOn ? 1 : 0);
  DBGF(" busy=");              DBG(busy ? 1 : 0);
  DBGF(" mute=");              DBG(digitalRead(MUTE_PIN) ? 1 : 0);
  DBGF(" pwr_hold=");          DBG(digitalRead(PWR_HOLD_PIN) ? 1 : 0);
  DBGLN();
#endif
}

// ==================================================================
//  Pulsante con debounce non bloccante
// ==================================================================
static bool readButton() {
  bool raw = (digitalRead(BTN_PIN) == HIGH);
  if (raw != btnRawPrev) { btnRawPrev = raw; btnRawChange = millis(); }
  if ((millis() - btnRawChange) >= DEBOUNCE_MS && raw != btnStable) {
    btnStable = raw;
    if (btnStable) {
      btnPressTime = millis();
      longFired    = false;
      DBGF("[btn] press @ "); DBGLN(btnPressTime);
    } else {
      DBGF("[btn] release @ "); DBGLN(millis());
    }
  }
  return btnStable;
}

// ==================================================================
//  SoftSerial: invia richiesta pre-off e attendi echo di conferma
// ==================================================================
static bool sendPreOffRequestAndWait() {
  while (link.available()) link.read();   // flush RX residuo

  DBGF("[link] TX ");
  for (uint8_t i = 0; i < MSG_LEN; i++) {
    link.write(MSG_PREREOFF[i]);
#if DEBUG_DUMP_LINK
    dbgByteHex(MSG_PREREOFF[i], (char)MSG_PREREOFF[i]);
#endif
  }
  DBGLN();

  link.flush();

  uint8_t idx = 0;
  unsigned long t0 = millis();
  while ((millis() - t0) < PREOFF_ACK_MS) {
    if (link.available()) {
      uint8_t b = link.read();
#if DEBUG_DUMP_LINK
      DBGF("[link] RX ");
      dbgByteHex(b, (char)b);
      Serial.print("idx="); Serial.println(idx);
#endif
      if (b == MSG_PREREOFF[idx]) {
        idx++;
        if (idx >= MSG_LEN) {
          DBGF("[link] ACK ok in "); DBG(millis() - t0); DBGLN(" ms");
          return true;
        }
      } else if (b == MSG_PREREOFF[0]) {
        idx = 1;
      } else {
        idx = 0;
      }
    }
  }
  DBGF("[link] TIMEOUT dopo "); DBG(PREOFF_ACK_MS); DBGLN(" ms (nessun ACK)");
  return false;
}

// ==================================================================
//  Sequenze
// ==================================================================
static void powerOnSequence() {
  busy = true;
  DBGLN("[seq] POWER-ON: start");

  digitalWrite(MUTE_PIN, MUTE_ACTIVE);
  DBGF("  mute ON (+"); DBG(MUTE_SETTLE_MS); DBGLN(" ms)");
  delay(MUTE_SETTLE_MS);

  digitalWrite(PWR_HOLD_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);
  DBGLN("  PWR_HOLD = HIGH (latch)");

  DBGF("  amp settle +"); DBG(AMP_SETTLE_MS); DBGLN(" ms");
  delay(AMP_SETTLE_MS);

  digitalWrite(MUTE_PIN, MUTE_INACTIVE);
  DBGLN("  mute OFF (audio attivo)");

  powerOn = true;
  busy    = false;
  DBGLN("[seq] POWER-ON: done");
  dbgState();
}

static void powerOffSequence() {
  busy = true;
  DBGLN("[seq] POWER-OFF: start");

  digitalWrite(MUTE_PIN, MUTE_ACTIVE);
  DBGF("  mute ON (+"); DBG(MUTE_SETTLE_MS); DBGLN(" ms)");
  delay(MUTE_SETTLE_MS);

  DBGLN("  invio richiesta pre-off allo slave...");
  bool ack = sendPreOffRequestAndWait();

  if (!ack) {
    DBGLN("  !! ACK mancante: POWER-OFF ANNULLATO");
    digitalWrite(MUTE_PIN, MUTE_INACTIVE);
    errorBlink();
    busy = false;
    dbgState();
    return;
  }

  DBGLN("  ACK ricevuto, procedo con lo spegnimento");
  blinkLed(5, 400, 400);
  delay(PREOFF_HOLD_MS);

  digitalWrite(PWR_HOLD_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
  DBGLN("  PWR_HOLD = LOW -> alimentazione rilasciata");

  powerOn = false;
  busy    = false;
  DBGLN("[seq] POWER-OFF: done");
  dbgState();
}

// ==================================================================
//  Comandi di debug da Serial
// ==================================================================
static void handleSerialCmds() {
#if DEBUG_ENABLED && DEBUG_CMDS
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'o': case 'O':
        DBGLN("[cmd] power-on manuale");
        if (!powerOn && !busy) powerOnSequence();
        break;
      case 'f': case 'F':
        DBGLN("[cmd] power-off manuale");
        if (powerOn && !busy) powerOffSequence();
        break;
      case 's': case 'S':
        DBGLN("[cmd] stato:");
        dbgState();
        break;
      case 'm': case 'M': {
        bool m = digitalRead(MUTE_PIN);
        digitalWrite(MUTE_PIN, m ? MUTE_INACTIVE : MUTE_ACTIVE);
        DBGF("[cmd] mute "); DBGLN(m ? "OFF" : "ON");
        break;
      }
      case '\r': case '\n': case ' ':
        break;
      default:
        DBGF("[cmd] sconosciuto: '"); Serial.write(c); DBGLN("' (usa o/f/s/m)");
        break;
    }
  }
#endif
}

// ==================================================================
//  Setup
// ==================================================================
void setup() {
#if DEBUG_ENABLED
  Serial.begin(DEBUG_BAUD);
  while (!Serial && millis() < 1500) { /* attesa breve se USB-serial */ }
  DBGLN();
  DBGLN(F("=== dualParaLudPower (LGT8F328P) ==="));
  DBGF("DEBUG_BAUD="); DBGLN(DEBUG_BAUD);
  DBGF("LINK_BAUD=");  DBGLN(LINK_BAUD);
  DBGLN(F("comandi: o=on, f=off, s=stato, m=toggle mute"));
#endif

  pinMode(LED_PIN,      OUTPUT);
  pinMode(BTN_PIN,      INPUT);          // INPUT_PULLUP se va a GND
  pinMode(PWR_HOLD_PIN, OUTPUT);
  pinMode(MUTE_PIN,     OUTPUT);

  digitalWrite(LED_PIN,      LOW);
  digitalWrite(PWR_HOLD_PIN, LOW);
  digitalWrite(MUTE_PIN,     MUTE_ACTIVE);

  btnStable = false; btnRawPrev = false; btnRawChange = millis();
  btnPressTime = 0;  longFired = false;
  powerOn = false;   busy = false;

  link.begin(LINK_BAUD);
  DBGLN("[init] SoftSerial link pronto");

  delay(BOOT_DELAY_MS);
  DBGLN("[init] startup chime");
  startupChime();

  DBGLN("[init] pronto");
  dbgState();
}

// ==================================================================
//  Loop
// ==================================================================
void loop() {
  handleSerialCmds();

  bool pressed = readButton();

  if (pressed && !longFired && !busy) {
    if ((millis() - btnPressTime) >= LONG_PRESS_MS) {
      longFired = true;
      DBGLN("[btn] long-press rilevato");
      if (!powerOn) powerOnSequence();
      else          powerOffSequence();
    }
  }
}