/*
 * Traffic Tube Counter - RAK3172 (RUI3) firmware, board "Izzy V10" (Oct 2026)
 *
 * Signal chain : piezo tube -> clamp/RC -> TLV365 comparator -> PB5 (rising edge = axle)
 * Storage      : FM25V02A 32 KB SPI FRAM on SPI1 (CS = PA4), used as a ring buffer
 * Radio        : RAK3172 LoRaWAN (AU915), confirmed uplinks, time sync by downlink
 * Power        : event-driven. MCU sleeps (api.system.sleep.all) until PB5 edge or timer.
 *
 * Pin map taken from the KiCad PCB netlist (traffictubecounterIzzyV10_18_10_1518):
 *   PB5  <- TLV365 OUT (also fed back through R_HYST1 for hysteresis)
 *   PA4  -> FRAM /CS      PA5 -> CLK     PA6 <- MISO(DO)     PA7 -> MOSI(DI)
 *   PA8  <- USB VBUS / 2  (R_VB1 / R_Vb2 divider) = "USB present" input, NOT a battery sense
 *   PB6  -> CP2102 RXD    (UART1 TX, log out only)
 *   PB7  == GND on this PCB revision (UART1 has no input, see HARDWARE NOTES)
 *   PA2/PA3 -> header J12 (UART2): plug a USB-UART dongle here for two way commands, no PCB change
 *
 * HARDWARE NOTES (fix on next board spin, firmware works around them):
 *   1. PB7/UART1_RX is tied to GND and CP2102 TXD only goes to GND via R_TX1, so the USB-C
 *      port cannot send anything to the MCU. Workarounds with no PCB change: use J12 (UART2)
 *      with an external USB-UART dongle for SETTIME / AT commands, seed the clock from the
 *      build time at flash, and take real time from a LoRaWAN downlink.
 *      Proper fix next spin: remove R_TX1 pulldown, wire U5-TXD -> 33k -> U1 PB7.
 *   2. No battery ADC. api.system.bat.get() reads the module VDD, which is a fixed 3.0 V
 *      LDO rail, so it only drops once the LDO falls out of regulation. Optional fix:
 *      divider to PB3 (free, pad 32) and set BAT_ADC_ENABLED 1.
 *   3. FM25V02A idles at tens of uA unless put in sleep mode. This firmware sleeps it
 *      (opcode 0xB9) after every access and wakes it with a CS pulse.
 *
 * Everything RUI3-specific is in the "PLATFORM" section so it is easy to adjust if a call
 * signature differs in your RUI3 version. Items marked VERIFY need one bench check.
 */

#include <SPI.h>
#include <string.h>

// ============================================================
// BUILD OPTIONS
// ============================================================
#define FW_VERSION          4
#define PKT_VERSION         3
#define DEBUG_NO_SLEEP      1      // 1 = never sleep (bench debugging)
#define SERIAL_BAUD         115200
#define CONSOLE             Serial   // RUI3 on RAK3172: Serial = UART2 (header J12, bidirectional),
                                     // Serial1 = UART1 (CP2102, TX only on this PCB). VERIFY, swap if needed.
#define BUILD_UTC_OFFSET_S  (11L * 3600L)  // PC timezone when compiling (AEDT = +11), seeds the clock

#define PROVISION_LORAWAN   0      // 1 = write region/keys below on first boot
#define LOG_AXLES           0      // 0 = log vehicles (axle groups), 1 = log raw axle count

#define BAT_ADC_ENABLED     0      // 1 = read battery through divider on BAT_ADC_PIN
#define BAT_ADC_PIN         PB3
#define BAT_DIV_RATIO       2.0f   // Vbat = Vpin * ratio

// ============================================================
// PINS (from PCB netlist)
// ============================================================
#define SENSOR_PIN          PB5
#define FRAM_CS_PIN         PA4
#define USB_SENSE_PIN       PA8

// ============================================================
// DETECTION TUNING
// ============================================================
#define ISR_GUARD_MS        2        // ignore edges closer than this inside the ISR (ringing)
#define MIN_GAP_MS          30       // axle-to-axle debounce (150 km/h, 2 m axles ~ 48 ms)
#define MIN_AXLES           2        // groups with fewer axles are noise (leaf, foot, one wheel)
#define MAX_AXLES           10       // groups with more are an anomaly and are discarded
#define SUSTAINED_MS        2000     // comparator high this long = object on tube, discard
#define GAP_RATIO           2.5f     // silence > maxGap * ratio ends the vehicle
#define IDLE_MIN_MS         600      // lower clamp of the vehicle-end timeout
#define IDLE_MAX_MS         3500     // upper clamp, also used while only one axle is seen
#define PULSE_Q             32

// ============================================================
// FRAM LAYOUT (FM25V02A: 32768 bytes, 2 byte address)
// ============================================================
#define FAKE_FRAM           1        // 1 = RAM-backed fake FRAM for bench, 0 = real FM25V02A
#if FAKE_FRAM
#define FRAM_SIZE           4096UL   // small, RAK3172 RAM is tight. Ring ~4 KB, 80% flag at ~3.2 KB
#else
#define FRAM_SIZE           32768UL
#endif
#define META_A_ADDR         0x0000   // two 32 byte metadata slots, written alternately
#define META_B_ADDR         0x0020
#define SCRATCH_ADDR        0x0040   // 8 bytes for self test
#define DATA_START          0x0048
#define RING_BYTES          (FRAM_SIZE - DATA_START)
#define META_MAGIC          0x54544331UL   // "TTC1"
#define FRAM_OP_WREN        0x06
#define FRAM_OP_WRITE       0x02
#define FRAM_OP_READ        0x03
#define FRAM_OP_RDID        0x9F
#define FRAM_OP_SLEEP       0xB9
#define STORAGE_WARN_PCT    80

// 16 bit record words (big endian on the wire)
//   data  : 0 mmmmmmmmmmm cccc   minuteOfDay (0..1439), count 1..15
//   marker: 1 ddddddddddddddd    day number (unix days) low 15 bits, applies to following data words
#define REC_MARKER_BIT      0x8000
#define REC_MAX_COUNT       15

// ============================================================
// LORAWAN / UPLINK
// ============================================================
#define PORT_DATA           1
#define PORT_TIME_DL        2        // downlink: 4 byte big endian unix time
#define PORT_DIAG           3
#define PACKET_HDR          5        // ver, seq, status, startDay(2); record count = payload length
#define PACKET_SIZE         51       // AU915 DR0-DR2 max payload; raise to 115 only if ADR never drops below DR3
#define TOK_ABS             0xC0     // 110 mmm.. + byte : absolute minute of day (11 bits)
#define TOK_DAY             0xFF     // + 2 bytes : day number, starts a new day
#define TOK_MAX_DELTA       31       // data token 0 cc ddddd : count-1 (2 bits), minutes since previous (5 bits)

#define FIRST_UPLINK_DELAY_S   120UL
#define UPLINK_INTERVAL_S      86400UL    // daily
#define FULL_UPLINK_INTERVAL_S 21600UL    // 6 h when storage >= warn level
#define UNSYNCED_RETRY_S       3600UL
#define RETRY_BASE_S           900UL
#define MORE_DATA_RETRY_S      600UL
#define MAX_PKTS_PER_SESSION   40
#define INTER_PKT_MS           8000UL
#define JOIN_TIMEOUT_MS        90000UL
#define TX_TIMEOUT_MS          90000UL
#define TX_RETRIES             2

#define HOUSEKEEP_S            600UL      // max sleep, also meta/time checkpoint period
#define MAX_SYNC_AGE_S         (48UL * 3600UL * 7UL)   // time considered stale after 14 days
#define MAX_PENDING_RAW        256

#if PROVISION_LORAWAN
// Fill in from the network server. AU915 sub-band 2 (channels 8-15 + 65) assumed.
static uint8_t DEV_EUI[8]  = {0,0,0,0,0,0,0,0};
static uint8_t APP_EUI[8]  = {0,0,0,0,0,0,0,0};
static uint8_t APP_KEY[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
#define LORA_BAND_AU915     6
#define LORA_MASK           0x0002
#endif

// ============================================================
// STATE
// ============================================================
// ISR -> loop pulse queue (single producer, single consumer)
volatile uint32_t pulseQ[PULSE_Q];
volatile uint8_t  qHead = 0, qTail = 0;
volatile uint16_t qDrops = 0;
volatile uint32_t lastIsrMs = 0;

// detection group, 64 bit ms timeline
struct Group {
  bool     active;
  uint8_t  axles;
  uint64_t firstMs;
  uint64_t lastMs;
  uint32_t maxGapMs;
};
static Group g = {false, 0, 0, 0, 0};
static bool     sustainedActive = false;
static uint16_t cntSingles = 0, cntAnomalies = 0, cntSustained = 0, cntDebounced = 0;
static uint32_t totalVehicles = 0;     // RAM only, survives FRAM faults, resets on reboot

// FRAM ring + metadata
static uint16_t wPtr = DATA_START, aPtr = DATA_START;
static uint16_t writeDay = 0xFFFF, ackDay = 0xFFFF;    // day context at wPtr / aPtr
static uint32_t lastUnixMeta = 0;
static uint8_t  uplinkSeq = 0;
static uint8_t  metaFlags = 0;
static uint16_t lostWords = 0;
static uint16_t resetCount = 0;
static uint32_t metaGen = 0;
static bool     framFault = false;
static bool     framAsleep = false;

// open tail word, may be incremented in place until sealed
static bool     tailValid = false;
static uint16_t tailAddr = 0, tailDay = 0, tailMinute = 0;
static uint8_t  tailCount = 0;

// packet in flight
static bool     ifValid = false;
static uint8_t  ifWords = 0;
static uint16_t ifEndPtr = 0, ifEndDay = 0xFFFF;

// time
static bool     timeValid = false;      // some usable estimate exists
static bool     timeSynced = false;     // true network/serial time received since boot
static uint32_t baseUnix = 0;
static uint64_t baseMonoSec = 0;
static uint64_t lastSyncMonoSec = 0;
static int32_t  lastSyncErr = 0;
static uint32_t pendingRaw[MAX_PENDING_RAW];   // seconds since boot, before any time exists
static uint16_t pendingRawCount = 0;
static bool     pendingOverflow = false;

// uplink state machine
enum TxKind { TX_NONE, TX_DATA, TX_DIAG };
static TxKind   txKind = TX_NONE;
static bool     sessionActive = false, joinWait = false, diagPending = false;
static uint8_t  sessionPkts = 0, failCount = 0;
static uint32_t txStartMs = 0, joinStartMs = 0, nextSendMs = 0;
static uint64_t nextUplinkSec = 0;
static volatile bool txDone = false, txOk = false;
static volatile bool joinDone = false, joinOk = false;
static volatile bool dlTimePending = false;
static volatile uint32_t dlTimeValue = 0;

static bool     usbPresent = false;
static uint64_t lastHouseSec = 0;
static char     cmdBuf[48];
static uint8_t  cmdLen = 0;

// ============================================================
// PLATFORM (RUI3 specific calls live here)
// ============================================================
static inline bool dbgOn() { return usbPresent || DEBUG_NO_SLEEP; }
#define LOG(...) do { if (dbgOn()) CONSOLE.printf(__VA_ARGS__); } while (0)

static void platformSleep(uint32_t ms) {
  if (ms < 1) ms = 1;
  api.system.sleep.all(ms);          // wakes on timer or PB5 rising edge
}

static float readVddVolts() {
  float v = api.system.bat.get();    // VERIFY units: V vs mV
  if (v > 100.0f) v /= 1000.0f;
  return v;
}

static uint16_t readBatteryMv() {
#if BAT_ADC_ENABLED
  uint32_t raw = analogRead(BAT_ADC_PIN);
  return (uint16_t)((raw * 3000UL / 4095UL) * BAT_DIV_RATIO);
#else
  return (uint16_t)(readVddVolts() * 1000.0f);
#endif
}

// 3 = full/ok, 2 = half, 1 = low, 0 = critical
static uint8_t batteryBucket() {
  uint16_t mv = readBatteryMv();
#if BAT_ADC_ENABLED
  if (mv >= 3900) return 3;          // Li-ion 18650 thresholds, calibrate on bench
  if (mv >= 3700) return 2;
  if (mv >= 3400) return 1;
  return 0;
#else
  // VDD is a 3.0 V LDO output: it only moves when the battery is nearly flat.
  if (mv >= 2950) return 3;
  if (mv >= 2850) return 2;
  if (mv >= 2700) return 1;
  return 0;
#endif
}

static bool loraJoined() { return api.lorawan.njs.get() == 1; }

static void joinCallback(int32_t status) { joinOk = (status == 0); joinDone = true; }
static void sendCallback(int32_t status) { txOk = (status == RAK_LORAMAC_STATUS_OK); txDone = true; }

static void recvCallback(SERVICE_LORA_RECEIVE_T *d) {      // VERIFY field names in your RUI3 headers
  if (d->Port == PORT_TIME_DL && d->BufferSize == 4) {
    dlTimeValue = ((uint32_t)d->Buffer[0] << 24) | ((uint32_t)d->Buffer[1] << 16) |
                  ((uint32_t)d->Buffer[2] << 8) | d->Buffer[3];
    dlTimePending = true;                                   // applied in loop(), not here
  }
}

// ============================================================
// SMALL HELPERS
// ============================================================
static uint16_t crc16(const uint8_t *p, uint8_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= (uint16_t)(*p++) << 8;
    for (uint8_t i = 0; i < 8; i++) c = (c & 0x8000) ? (c << 1) ^ 0x1021 : (c << 1);
  }
  return c;
}

static uint64_t extendMs(uint32_t m) {          // 32 bit millis -> monotonic 64 bit
  static uint32_t last = 0, hi = 0;
  if (m < last) hi++;
  last = m;
  return ((uint64_t)hi << 32) | m;
}

static inline uint16_t nextPtr(uint16_t p) { return (p + 2 >= FRAM_SIZE) ? DATA_START : p + 2; }
static inline uint16_t usedBytes() { return (wPtr >= aPtr) ? (wPtr - aPtr) : (RING_BYTES - (aPtr - wPtr)); }
static inline bool ringFull() { return nextPtr(wPtr) == aPtr; }
static inline uint8_t usedPct() { return (uint8_t)((uint32_t)usedBytes() * 100UL / (RING_BYTES - 2)); }
static inline bool storageWarn() { return usedPct() >= STORAGE_WARN_PCT; }

// ============================================================
// FRAM DRIVER
// ============================================================
static inline void csLow()  { digitalWrite(FRAM_CS_PIN, LOW); }
static inline void csHigh() { digitalWrite(FRAM_CS_PIN, HIGH); }

static void framBegin() { SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); }
static void framEnd()   { SPI.endTransaction(); }

#if FAKE_FRAM
static uint8_t fakeFram[FRAM_SIZE];

static void framWake()  {}
static void framSleep() {}

static void framRawRead(uint16_t addr, uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) buf[i] = (addr + i < FRAM_SIZE) ? fakeFram[addr + i] : 0xFF;
}

static void framRawWrite(uint16_t addr, const uint8_t *buf, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) if (addr + i < FRAM_SIZE) fakeFram[addr + i] = buf[i];
}

static void framReadId(uint8_t *id9) {          // fake FM25V02A-style ID, passes self test
  static const uint8_t id[9] = {0x7F,0x7F,0x7F,0x7F,0x7F,0x7F,0xC2,0x22,0x00};
  memcpy(id9, id, 9);
}
#else
// ---- original real-FRAM code, unchanged ----
static void framWake() {
  if (!framAsleep) return;
  csLow(); delay(1); csHigh(); delay(1);
  framAsleep = false;
}
static void framSleep() {
  framBegin(); csLow(); SPI.transfer(FRAM_OP_SLEEP); csHigh(); framEnd();
  framAsleep = true;
}
static void framRawRead(uint16_t addr, uint8_t *buf, uint8_t len) {
  framBegin(); csLow();
  SPI.transfer(FRAM_OP_READ); SPI.transfer(addr >> 8); SPI.transfer(addr & 0xFF);
  for (uint8_t i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
  csHigh(); framEnd();
}
static void framRawWrite(uint16_t addr, const uint8_t *buf, uint8_t len) {
  framBegin();
  csLow(); SPI.transfer(FRAM_OP_WREN); csHigh();
  csLow();
  SPI.transfer(FRAM_OP_WRITE); SPI.transfer(addr >> 8); SPI.transfer(addr & 0xFF);
  for (uint8_t i = 0; i < len; i++) SPI.transfer(buf[i]);
  csHigh(); framEnd();
}
static void framReadId(uint8_t *id9) {
  framWake();
  framBegin(); csLow();
  SPI.transfer(FRAM_OP_RDID);
  for (uint8_t i = 0; i < 9; i++) id9[i] = SPI.transfer(0x00);
  csHigh(); framEnd();
}
#endif

static bool framRead(uint16_t addr, uint8_t *buf, uint8_t len) {
  framWake(); framRawRead(addr, buf, len); return true;
}

// write with read-back verify and 2 retries; sets framFault on failure
static bool framWriteVerified(uint16_t addr, const uint8_t *buf, uint8_t len) {
  uint8_t chk[32];
  framWake();
  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    framRawWrite(addr, buf, len);
    framRawRead(addr, chk, len);
    if (memcmp(chk, buf, len) == 0) return true;
  }
  framFault = true;
  return false;
}

static bool framSelfTest() {
  static const uint8_t pat[8] = {0xA5, 0x5A, 0xFF, 0x00, 0x12, 0x34, 0xC3, 0x3C};
  uint8_t id[9]; framReadId(id);
  bool idDead = true, idFF = true;
  for (uint8_t i = 0; i < 9; i++) { if (id[i] != 0x00) idDead = false; if (id[i] != 0xFF) idFF = false; }
  if (idDead || idFF) return false;                    // bus floating or part absent
  framFault = false;
  bool ok = framWriteVerified(SCRATCH_ADDR, pat, 8);
  framFault = !ok;
  return ok;
}

// ============================================================
// METADATA (two slots, generation counter, CRC) - survives power loss mid write
// ============================================================
static void putU16(uint8_t *b, uint16_t v) { b[0] = v >> 8; b[1] = v & 0xFF; }
static void putU32(uint8_t *b, uint32_t v) { b[0] = v >> 24; b[1] = v >> 16; b[2] = v >> 8; b[3] = v; }
static uint16_t getU16(const uint8_t *b) { return ((uint16_t)b[0] << 8) | b[1]; }
static uint32_t getU32(const uint8_t *b) {
  return ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | b[3];
}

static void saveMeta() {
  uint8_t m[32]; memset(m, 0, sizeof(m));
  metaGen++;
  putU32(&m[0], META_MAGIC);
  putU32(&m[4], metaGen);
  putU16(&m[8], wPtr);   putU16(&m[10], aPtr);
  putU16(&m[12], writeDay); putU16(&m[14], ackDay);
  putU32(&m[16], lastUnixMeta);
  m[20] = uplinkSeq; m[21] = metaFlags;
  putU16(&m[22], lostWords); putU16(&m[24], resetCount);
  putU16(&m[30], crc16(m, 30));
  framWriteVerified((metaGen & 1) ? META_A_ADDR : META_B_ADDR, m, 32);
}

static bool parseMeta(const uint8_t *m, uint32_t *gen) {
  if (getU32(&m[0]) != META_MAGIC) return false;
  if (getU16(&m[30]) != crc16(m, 30)) return false;
  uint16_t w = getU16(&m[8]), a = getU16(&m[10]);
  if (w < DATA_START || w >= FRAM_SIZE || a < DATA_START || a >= FRAM_SIZE || (w & 1) || (a & 1)) return false;
  *gen = getU32(&m[4]);
  return true;
}

static bool loadMeta() {
  uint8_t a[32], b[32]; uint32_t ga = 0, gb = 0;
  framRead(META_A_ADDR, a, 32); framRead(META_B_ADDR, b, 32);
  bool va = parseMeta(a, &ga), vb = parseMeta(b, &gb);
  const uint8_t *m = NULL;
  if (va && vb) m = ((int32_t)(ga - gb) >= 0) ? a : b;
  else if (va) m = a; else if (vb) m = b;
  if (!m) return false;
  metaGen = getU32(&m[4]);
  wPtr = getU16(&m[8]); aPtr = getU16(&m[10]);
  writeDay = getU16(&m[12]); ackDay = getU16(&m[14]);
  lastUnixMeta = getU32(&m[16]);
  uplinkSeq = m[20]; metaFlags = m[21];
  lostWords = getU16(&m[22]); resetCount = getU16(&m[24]);
  return true;
}

static void formatStorage() {
  wPtr = aPtr = DATA_START; writeDay = ackDay = 0xFFFF;
  tailValid = false; ifValid = false; lostWords = 0;
  saveMeta();
}

// ============================================================
// RING BUFFER OF RECORD WORDS
// ============================================================
static bool readWord(uint16_t addr, uint16_t *w) {
  uint8_t b[2]; framRead(addr, b, 2); *w = getU16(b); return true;
}

static void dropOldest() {               // storage full: overwrite oldest, keep day context right
  uint16_t w; readWord(aPtr, &w);
  if (w & REC_MARKER_BIT) ackDay = w & 0x7FFF;
  else if (lostWords < 0xFFFF) lostWords++;
  aPtr = nextPtr(aPtr);
  ifValid = false;                       // in flight packet no longer maps onto the ring
}

static bool pushWord(uint16_t w, uint16_t *addrOut) {
  if (framFault) return false;
  if (ringFull()) dropOldest();
  uint8_t b[2]; putU16(b, w);
  uint16_t addr = wPtr;
  if (!framWriteVerified(addr, b, 2)) return false;
  if (addrOut) *addrOut = addr;
  wPtr = nextPtr(wPtr);
  return true;
}

// add `n` counts at unix time `ts` (seconds)
static void appendCount(uint32_t ts, uint8_t n) {
  uint16_t day = (uint16_t)((ts / 86400UL) & 0x7FFF);
  uint16_t minute = (uint16_t)((ts % 86400UL) / 60UL);
  while (n > 0) {
    if (tailValid && tailDay == day && tailMinute == minute && tailCount < REC_MAX_COUNT) {
      uint8_t add = min((uint8_t)(REC_MAX_COUNT - tailCount), n);
      uint16_t w = (uint16_t)((minute << 4) | (tailCount + add));
      uint8_t b[2]; putU16(b, w);
      if (!framWriteVerified(tailAddr, b, 2)) return;
      tailCount += add; n -= add;
      continue;
    }
    if (writeDay != day) {
      if (!pushWord(REC_MARKER_BIT | day, NULL)) return;
      writeDay = day;
    }
    uint8_t put = min((uint8_t)REC_MAX_COUNT, n);
    uint16_t addr;
    if (!pushWord((uint16_t)((minute << 4) | put), &addr)) return;
    tailValid = true; tailAddr = addr; tailDay = day; tailMinute = minute; tailCount = put;
    n -= put;
  }
  lastUnixMeta = ts;
  saveMeta();
}

// ============================================================
// TIME
// ============================================================
static uint64_t monoSec() { return extendMs(millis()) / 1000ULL; }

static uint32_t unixFromMonoSec(uint64_t ms) {
  return (uint32_t)((int64_t)baseUnix + ((int64_t)ms - (int64_t)baseMonoSec));
}

static uint32_t unixNow() { return timeValid ? unixFromMonoSec(monoSec()) : 0; }

static bool timeIsTrusted() {
  return timeSynced && (monoSec() - lastSyncMonoSec) < MAX_SYNC_AGE_S;
}

static uint32_t buildUnixTime() {            // compile time of this sketch, treated as an untrusted lower bound
  static const char *mon = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char m[4] = {__DATE__[0], __DATE__[1], __DATE__[2], 0};
  int mi = (int)((strstr(mon, m) - mon) / 3);
  int d = atoi(__DATE__ + 4), y = atoi(__DATE__ + 7);
  int hh = atoi(__TIME__), mm = atoi(__TIME__ + 3), ss = atoi(__TIME__ + 6);
  static const uint16_t cum[12] = {0,31,59,90,120,151,181,212,243,273,304,334};
  uint32_t days = (uint32_t)(y - 1970) * 365 + (y - 1969) / 4 + cum[mi] + (d - 1);
  if (mi >= 2 && (y % 4) == 0) days++;
  return (uint32_t)((int64_t)days * 86400 + hh * 3600L + mm * 60L + ss - BUILD_UTC_OFFSET_S);
}

static void applyTime(uint32_t unix) {
  if (unix < 1700000000UL || unix > 4102444800UL) { LOG("Time rejected: %lu\n", (unsigned long)unix); return; }
  bool first = !timeValid;
  lastSyncErr = timeValid ? (int32_t)((int64_t)unix - (int64_t)unixNow()) : 0;
  baseUnix = unix; baseMonoSec = monoSec();
  timeValid = true; timeSynced = true; lastSyncMonoSec = baseMonoSec;
  lastUnixMeta = unix;
  if (first || pendingRawCount) {                 // replay events recorded before any time existed
    for (uint16_t i = 0; i < pendingRawCount; i++) appendCount(unixFromMonoSec(pendingRaw[i]), 1);
    if (pendingOverflow) LOG("WARNING: early events lost before first sync\n");
    pendingRawCount = 0; pendingOverflow = false;
  }
  saveMeta();
  LOG("Time set: %lu\n", (unsigned long)unix);
}

// ============================================================
// EVENT SINK (a finished vehicle)
// ============================================================
static void recordUnits(uint64_t firstMs, uint8_t units) {
  totalVehicles += units;
  if (timeValid) {
    appendCount(unixFromMonoSec(firstMs / 1000ULL), units);
  } else {
    for (uint8_t i = 0; i < units; i++) {
      if (pendingRawCount < MAX_PENDING_RAW) pendingRaw[pendingRawCount++] = (uint32_t)(firstMs / 1000ULL);
      else pendingOverflow = true;
    }
  }
}

// ============================================================
// VEHICLE DETECTION
// ============================================================
static void sensorISR() {                         // keep tiny: timestamp and queue only
  uint32_t t = millis();
  if ((t - lastIsrMs) < ISR_GUARD_MS) return;
  lastIsrMs = t;
  uint8_t n = (qHead + 1) % PULSE_Q;
  if (n == qTail) { qDrops++; return; }
  pulseQ[qHead] = t; qHead = n;
}

static uint32_t groupTimeoutMs() {
  if (g.axles < 2) return IDLE_MAX_MS;
  uint32_t t = (uint32_t)(g.maxGapMs * GAP_RATIO);
  if (t < IDLE_MIN_MS) t = IDLE_MIN_MS;
  if (t > IDLE_MAX_MS) t = IDLE_MAX_MS;
  return t;
}

static void finalizeGroup() {
  if (!g.active) return;
  if (g.axles < MIN_AXLES) {
    cntSingles++; LOG("Discard: %u axle(s)\n", g.axles);
  } else if (g.axles > MAX_AXLES) {
    cntAnomalies++; LOG("Discard: %u axles (anomaly)\n", g.axles);
  } else {
#if LOG_AXLES
    recordUnits(g.firstMs, g.axles);
#else
    recordUnits(g.firstMs, 1);
#endif
    LOG("Vehicle: %u axles, total %lu\n", g.axles, (unsigned long)totalVehicles);
  }
  g.active = false; g.axles = 0; g.maxGapMs = 0;
}

static void startGroup(uint64_t t) { g.active = true; g.axles = 1; g.firstMs = g.lastMs = t; g.maxGapMs = 0; }

static void onAxle(uint64_t t) {
  if (g.active) {
    uint32_t gap = (uint32_t)(t - g.lastMs);
    if (gap < MIN_GAP_MS) { cntDebounced++; return; }
    if (gap > groupTimeoutMs()) { finalizeGroup(); startGroup(t); return; }
    if (g.axles < 255) g.axles++;
    if (gap > g.maxGapMs) g.maxGapMs = gap;
    g.lastMs = t;
  } else {
    startGroup(t);
  }
}

static void processPulses() {
  if (sustainedActive) {                          // wait for the comparator to drop, drop everything meanwhile
    if (digitalRead(SENSOR_PIN) == LOW) sustainedActive = false;
    noInterrupts(); qTail = qHead; interrupts();
    return;
  }
  while (qTail != qHead) {
    uint32_t t32 = pulseQ[qTail];
    qTail = (qTail + 1) % PULSE_Q;
    uint32_t nowMs = millis();
    uint64_t now64 = extendMs(nowMs);
    uint64_t t64 = now64 - (uint32_t)(nowMs - t32);

    // sustained check: pin still high well after the edge means an object, not an axle
    if (digitalRead(SENSOR_PIN) == HIGH) {
      uint32_t waited = 0;
      while (digitalRead(SENSOR_PIN) == HIGH && waited < SUSTAINED_MS) { delay(1); waited++; }
      if (waited >= SUSTAINED_MS) {
        cntSustained++; sustainedActive = true; LOG("Discard: sustained pulse\n");
        noInterrupts(); qTail = qHead; interrupts();
        return;
      }
    }
    onAxle(t64);
  }
  if (g.active && (extendMs(millis()) - g.lastMs) > groupTimeoutMs()) finalizeGroup();
}

// ============================================================
// UPLINK PACKETS
// ============================================================
static uint8_t statusByte() {
  uint8_t s = (batteryBucket() & 0x03) << 6;
  if (storageWarn())        s |= 1 << 5;     // storage >= 80 %
  if (!timeIsTrusted())     s |= 1 << 4;     // clock estimated or never synced
  /* bit3 tamper: no sensor fitted */
  if (framFault)            s |= 1 << 2;     // FRAM write/verify failed
  if (lostWords)            s |= 1 << 1;     // old data was overwritten
  if (usbPresent)           s |= 1 << 0;
  return s;
}

// Builds the next data packet from the ring, does NOT advance the ack pointer.
// Storage stays 2 bytes/record; the radio stream is delta coded, ~1 byte per vehicle-minute:
//   0 cc ddddd         record: count = cc+1 (1..4), minute = previous minute + ddddd (0..31)
//   110 mmm  mmmmmmmm  set absolute minute of day (first record of packet/day, or gap > 31 min)
//   0xFF dd dd         new day number (applies to everything after it)
// Decoder: day = header startDay, minute unset; walk tokens. A data token after ABS uses delta 0.
static uint8_t buildDataPacket(uint8_t *buf) {
  memset(buf, 0, PACKET_SIZE);
  buf[0] = PKT_VERSION; buf[1] = uplinkSeq; buf[2] = statusByte();
  putU16(&buf[3], ackDay);
  uint8_t len = PACKET_HDR, words = 0;
  uint16_t p = aPtr, ctx = ackDay, cur = 0; bool haveMin = false;
  while (p != wPtr) {
    uint16_t w; readWord(p, &w);
    uint8_t tmp[8]; uint8_t t = 0;
    uint16_t nCtx = ctx, nCur = cur; bool nHave = haveMin;
    if (w & REC_MARKER_BIT) {
      tmp[t++] = TOK_DAY; tmp[t++] = (w >> 8) & 0x7F; tmp[t++] = w & 0xFF;
      nCtx = w & 0x7FFF; nHave = false;
    } else {
      uint16_t m = w >> 4; uint8_t left = w & 0x0F; uint8_t delta;
      if (left == 0) { p = nextPtr(p); words++; continue; }       // empty word, skip safely
      if (!haveMin || m < cur || (m - cur) > TOK_MAX_DELTA) {
        tmp[t++] = TOK_ABS | (m >> 8); tmp[t++] = m & 0xFF; delta = 0;
      } else delta = (uint8_t)(m - cur);
      while (left) {
        uint8_t k = left > 4 ? 4 : left;
        tmp[t++] = ((k - 1) << 5) | delta; delta = 0; left -= k;
      }
      nCur = m; nHave = true;
    }
    if (len + t > PACKET_SIZE) break;
    memcpy(&buf[len], tmp, t); len += t;
    ctx = nCtx; cur = nCur; haveMin = nHave;
    p = nextPtr(p); words++;
  }
  ifWords = words; ifEndPtr = p; ifEndDay = ctx; ifValid = true;
  tailValid = false;                         // seal: never edit a word that is being sent
  return len;
}

static void ackInFlight() {
  if (!ifValid) return;                      // ring moved underneath us, data resends next time
  aPtr = ifEndPtr; ackDay = ifEndDay; ifValid = false;
  if (lostWords && aPtr == wPtr) lostWords = 0;
  uplinkSeq++;
  saveMeta();
}

static uint8_t buildDiagPacket(uint8_t *b) {
  memset(b, 0, 28);
  b[0] = PKT_VERSION; b[1] = FW_VERSION;
  putU32(&b[2], totalVehicles);
  putU16(&b[6], cntSingles); putU16(&b[8], cntAnomalies); putU16(&b[10], cntSustained);
  putU16(&b[12], qDrops); putU16(&b[14], resetCount);
  putU16(&b[16], readBatteryMv());
  putU32(&b[18], (uint32_t)monoSec());
  b[22] = usedPct(); b[23] = framFault ? 1 : 0;
  putU32(&b[24], (uint32_t)lastSyncErr);   // seconds the clock was off at last sync; server shifts records written since the previous trusted sync
  return 28;
}

// ============================================================
// UPLINK STATE MACHINE (flags set by callbacks, all work done here in loop())
// ============================================================
static void scheduleNext(bool ok, bool moreData) {
  uint64_t now = monoSec(); uint32_t delayS;
  if (ok) {
    failCount = 0;
    delayS = UPLINK_INTERVAL_S;
    if (storageWarn() && delayS > FULL_UPLINK_INTERVAL_S) delayS = FULL_UPLINK_INTERVAL_S;
    if (!timeIsTrusted()) delayS = UNSYNCED_RETRY_S;
    if (moreData) delayS = MORE_DATA_RETRY_S;
  } else {
    if (failCount < 8) failCount++;
    delayS = RETRY_BASE_S << (failCount > 4 ? 4 : failCount - 1);
    if (delayS > UPLINK_INTERVAL_S) delayS = UPLINK_INTERVAL_S;
  }
  nextUplinkSec = now + delayS;
}

static void endSession(bool ok) {
  sessionActive = false; joinWait = false; txKind = TX_NONE;
  scheduleNext(ok, usedBytes() > 0);
  LOG("Uplink session %s, next in %lu s\n", ok ? "ok" : "failed", (unsigned long)(nextUplinkSec - monoSec()));
}

static void sendPacket(TxKind kind) {
  uint8_t buf[PACKET_SIZE]; uint8_t len; uint8_t port;
  if (kind == TX_DATA) { len = buildDataPacket(buf); port = PORT_DATA; }
  else                 { len = buildDiagPacket(buf); port = PORT_DIAG; }
  txDone = false; txKind = kind; txStartMs = millis();
  bool confirmed = (kind == TX_DATA);
  if (!api.lorawan.send(len, buf, port, confirmed, confirmed ? TX_RETRIES : 0)) {
    txKind = TX_NONE; endSession(false);
  }
}

static void uplinkTask() {
  uint32_t nowMs = millis();

  if (joinWait) {
    if (joinDone) {
      joinDone = false; joinWait = false;
      if (joinOk) nextSendMs = nowMs; else endSession(false);
    } else if (nowMs - joinStartMs > JOIN_TIMEOUT_MS) endSession(false);
    return;
  }
  if (txKind != TX_NONE) {
    if (txDone) {
      txDone = false; TxKind k = txKind; txKind = TX_NONE;
      if (k == TX_DATA) {
        if (txOk) { ackInFlight(); sessionPkts++; nextSendMs = nowMs + INTER_PKT_MS; }
        else { ifValid = false; endSession(false); }
      } else { diagPending = false; endSession(true); }
    } else if (nowMs - txStartMs > TX_TIMEOUT_MS) { ifValid = false; endSession(false); }
    return;
  }
  if (!sessionActive) {
    if (monoSec() < nextUplinkSec) return;
    sessionActive = true; sessionPkts = 0; diagPending = true; nextSendMs = nowMs;
    LOG("Uplink session start, %u bytes stored\n", usedBytes());
  }
  if ((int32_t)(nowMs - nextSendMs) < 0) return;

  if (!loraJoined()) {
    joinDone = false; joinWait = true; joinStartMs = nowMs;
    api.lorawan.join(1, 0, 8, 0);
    return;
  }
  bool haveData = usedBytes() > 0;
  if (haveData && sessionPkts < MAX_PKTS_PER_SESSION) sendPacket(TX_DATA);
  else if (sessionPkts == 0 && !haveData)             sendPacket(TX_DATA);   // heartbeat, also asks for time
  else if (diagPending)                                sendPacket(TX_DIAG);
  else                                                 endSession(true);
}

// ============================================================
// HOUSEKEEPING
// ============================================================
static void housekeeping() {
  uint64_t now = monoSec();
  if (dlTimePending) { dlTimePending = false; applyTime(dlTimeValue); }
  if (now - lastHouseSec >= HOUSEKEEP_S) {
    lastHouseSec = now;
    if (timeValid) { lastUnixMeta = unixNow(); saveMeta(); }     // checkpoint so a reset keeps a time estimate
    if (framFault && framSelfTest()) LOG("FRAM recovered\n");
  }
}

// ============================================================
// SERIAL SERVICE COMMANDS (needs PB7 fix on the PCB to receive anything)
// ============================================================
static void dumpRecords() {
  uint16_t p = aPtr, ctx = ackDay;
  while (p != wPtr) {
    uint16_t w; readWord(p, &w);
    if (w & REC_MARKER_BIT) { ctx = w & 0x7FFF; CONSOLE.printf("day %u\n", ctx); }
    else CONSOLE.printf("  day %u %02u:%02u x%u\n", ctx, (w >> 4) / 60 % 24, (w >> 4) % 60, w & 0x0F);
    p = nextPtr(p);
  }
}

static void simulateVehicle(uint8_t axles) {
  uint32_t now = millis();
  for (uint8_t i = 0; i < axles; i++) {
    noInterrupts();
    uint8_t n = (qHead + 1) % PULSE_Q;
    if (n != qTail) { pulseQ[qHead] = now - (uint32_t)(axles - 1 - i) * 150U; qHead = n; }
    interrupts();
  }
}

static void handleCommand(char *c) {
  if (!strncmp(c, "SETTIME ", 8)) { applyTime((uint32_t)strtoul(c + 8, NULL, 10)); }
  else if (!strcmp(c, "STATUS")) {
    uint8_t id[9]; framReadId(id);
    CONSOLE.printf("fw %d  usb %d  vdd %u mV  batt bucket %u\n", FW_VERSION, usbPresent, readBatteryMv(), batteryBucket());
    CONSOLE.printf("time valid %d synced %d trusted %d unix %lu\n", timeValid, timeSynced, timeIsTrusted(), (unsigned long)unixNow());
    CONSOLE.printf("fram fault %d  id %02X %02X %02X  w %u a %u used %u B (%u%%)  lost %u\n",
                  framFault, id[6], id[7], id[8], wPtr, aPtr, usedBytes(), usedPct(), lostWords);
    CONSOLE.printf("vehicles %lu  singles %u  anomalies %u  sustained %u  debounced %u  qdrops %u  pending %u  resets %u\n",
                  (unsigned long)totalVehicles, cntSingles, cntAnomalies, cntSustained, cntDebounced, qDrops, pendingRawCount, resetCount);
    CONSOLE.printf("joined %d  session %d  seq %u  next uplink in %ld s\n", loraJoined(), sessionActive, uplinkSeq,
                  (long)((int64_t)nextUplinkSec - (int64_t)monoSec()));
  }
  else if (!strcmp(c, "DUMP")) dumpRecords();
  else if (!strcmp(c, "UPLINK")) { nextUplinkSec = 0; CONSOLE.println("uplink forced"); }
  else if (!strncmp(c, "SIM", 3)) { simulateVehicle((uint8_t)max(1L, strtol(c + 3, NULL, 10))); }
  else if (!strcmp(c, "TESTPKT")) {
    uint8_t b[PACKET_SIZE]; uint8_t n = buildDataPacket(b); ifValid = false;
    for (uint8_t i = 0; i < n; i++) CONSOLE.printf("%02X ", b[i]);
    CONSOLE.println();
  }
  else if (!strcmp(c, "ERASE YES")) { formatStorage(); CONSOLE.println("storage erased"); }
  else if (!strcmp(c, "RESETCNT")) {
  totalVehicles = 0;
  cntSingles = cntAnomalies = cntSustained = cntDebounced = 0;
  qDrops = 0;
  CONSOLE.println("counters cleared");
  }
  else CONSOLE.println("Commands: STATUS, SETTIME <unix>, DUMP, UPLINK, SIM <axles>, TESTPKT, ERASE YES, RESETCNT");
}

static void serviceSerial() {
  while (CONSOLE.available()) {
    char ch = (char)CONSOLE.read();
    if (ch == '\r') continue;
    if (ch == '\n') { cmdBuf[cmdLen] = 0; if (cmdLen) handleCommand(cmdBuf); cmdLen = 0; }
    else if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = ch;
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  CONSOLE.begin(SERIAL_BAUD, RAK_CUSTOM_MODE);
  pinMode(USB_SENSE_PIN, INPUT);
  usbPresent = (digitalRead(USB_SENSE_PIN) == HIGH);

  pinMode(SENSOR_PIN, INPUT);
  pinMode(FRAM_CS_PIN, OUTPUT); csHigh();
  SPI.begin();
  analogReadResolution(12);

  // storage first: nothing else matters if counts cannot be kept
  framAsleep = true;                                   // unknown after reset, force a wake pulse
  bool framOk = framSelfTest();
  if (!loadMeta()) { LOG("No valid metadata, formatting ring\n"); formatStorage(); }
  resetCount++;
  if (framOk) saveMeta();
  uint32_t seed = buildUnixTime();                     // never earlier than the firmware build
  if (lastUnixMeta > seed) seed = lastUnixMeta;        // or the last checkpoint after a reset
  if (seed > 1700000000UL) {                           // usable but untrusted until a real sync arrives
    baseUnix = seed; baseMonoSec = monoSec(); timeValid = true; timeSynced = false;
  }

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensorISR, RISING);
  api.system.sleep.setup(RUI_WAKEUP_RISING_EDGE, SENSOR_PIN);   // VERIFY PB5 usable as wake pin
  api.system.lpm.set(1);

  // LoRaWAN
#if PROVISION_LORAWAN
  if (!(metaFlags & 1)) {
    api.lorawan.nwm.set(1); api.lorawan.band.set(LORA_BAND_AU915); api.lorawan.mask.set(LORA_MASK);
    api.lorawan.njm.set(1);
    api.lorawan.deui.set(DEV_EUI, 8); api.lorawan.appeui.set(APP_EUI, 8); api.lorawan.appkey.set(APP_KEY, 16);
    api.lorawan.deviceClass.set(0); api.lorawan.adr.set(true);
    metaFlags |= 1; saveMeta();
  }
#endif
  api.lorawan.registerJoinCallback(joinCallback);
  api.lorawan.registerSendCallback(sendCallback);
  api.lorawan.registerRecvCallback(recvCallback);

  nextUplinkSec = monoSec() + FIRST_UPLINK_DELAY_S;
  lastHouseSec = monoSec();
  framSleep();
  LOG("Traffic tube counter fw %d ready, %u B stored, resets %u\n", FW_VERSION, usedBytes(), resetCount);
}

void loop() {
  usbPresent = (digitalRead(USB_SENSE_PIN) == HIGH);
  serviceSerial();
  processPulses();
  housekeeping();
  uplinkTask();
  framSleep();                                         // FRAM back to its low power state after any access

  // ---- decide how long we may sleep ----
  bool radioBusy = sessionActive || joinWait || (txKind != TX_NONE);
  bool pending = (qTail != qHead) || dlTimePending || (digitalRead(SENSOR_PIN) == HIGH);
  if (pending) return;
  if (usbPresent || radioBusy || DEBUG_NO_SLEEP) { delay(radioBusy ? 10 : 5); return; }

  uint64_t nowMs = extendMs(millis());
  uint64_t nowS  = nowMs / 1000ULL;
  uint64_t sleepMs = HOUSEKEEP_S * 1000ULL;
  if (g.active) {
    uint64_t age = nowMs - g.lastMs, thr = groupTimeoutMs();
    uint64_t rem = (age >= thr) ? 1 : (thr - age + 2);
    if (rem < sleepMs) sleepMs = rem;
  }
  if (nextUplinkSec > nowS) {
    uint64_t u = (nextUplinkSec - nowS) * 1000ULL;
    if (u < sleepMs) sleepMs = u;
  } else sleepMs = 1;
  platformSleep((uint32_t)sleepMs);
}
