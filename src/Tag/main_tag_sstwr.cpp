// ============================================================
//  UWB Spatial Tracking - TAG Firmware (SS-TWR, Multi-Tag TDMA)
//
//  Upgrade from DS-TWR to Single-Sided TWR:
//    DS-TWR: POLL -> RESP -> FINAL -> REPORT  (4 frames, ~12ms/anchor)
//    SS-TWR: POLL -> RESP w/ timestamps       (2 frames, ~4ms/anchor)
//
//  SS-TWR trade-off: no clock-offset correction, so systematic error
//  is ~10-30cm higher than DS-TWR. Acceptable for spatial audio.
//  Speed gain: 60ms slots -> 30ms slots -> 8Hz vs 4Hz per tag.
//
//  Other fixes vs previous DS-TWR firmware:
//    - Outlier rejection in filter (prevents EMA phantom lock)
//    - Dropped median filter (EMA only, faster response)
//    - Tighter RX timeout (5ms vs 8ms)
//    - No inter-range delay
//    - setFrameLength corrected (4 + NUM_ANCHORS*4, not 3+)
//
//  FLASH: Same file for all tags. Set TAG_ID via build_flags:
//    build_flags = -DTAG_ID=7   (or 8, 9, 10)
// ============================================================

#include <Arduino.h>
#include <SPI.h>

// ==================== CONFIGURATION ====================

#ifndef TAG_ID
#define TAG_ID              7
#endif

#define NUM_ANCHORS         6
#define FIRST_ANCHOR_ID     1
#define LISTENER_ANCHOR_ID  1
#define NUM_TAGS            4
#define FIRST_TAG_ID        7

#define RST_PIN             27
#define CHIP_SELECT_PIN     4

#define RX_TIMEOUT_MS       5       // Tighter than DS-TWR since only 2 frames
#define SLOT_DURATION_MS    30      // 30ms vs old 60ms -> 8Hz vs 4Hz
#define MAX_RANGE_RETRIES   0       // No retries; move on and let EMA handle it

// Filter: EMA only (no median), with outlier rejection to prevent phantom lock
#define EMA_ALPHA           0.5f    // More responsive than 0.35; good at 8Hz
#define OUTLIER_THRESH_CM   120.0f  // Reject if >120cm from current EMA
#define MIN_DISTANCE        0.0f
#define MAX_DISTANCE        2000.0f

#define SYNC_TIMEOUT_MS     5000

// ==================== UWB CONSTANTS ====================

#define FCS_LEN 2
#define STDRD_SYS_CONFIG 0x188
#define SYS_STATUS_FRAME_RX_SUCC 0x2000
#define SYS_STATUS_RX_ERR 0x4279000
#define SYS_STATUS_FRAME_TX_SUCC 0x80
#define PREAMBLE_128 5
#define CHANNEL_5 0x0
#define CHANNEL_9 0x1
#define PAC8 0x00
#define DATARATE_6_8MB 0x1
#define PHR_MODE_STANDARD 0x0
#define PHR_RATE_850KB 0x0
#define SPIRDY_MASK 0x80
#define RCINIT_MASK 0x100
#define BIAS_CTRL_BIAS_MASK 0x1F
#define PMSC_STATE_IDLE 0x3
#define PS_UNIT 15.6500400641025641
#define SPEED_OF_LIGHT 0.029979245800
#define NO_OFFSET 0x0

#define GEN_CFG_AES_LOW_REG  0x00
#define GEN_CFG_AES_HIGH_REG 0x01
#define STS_CFG_REG  0x2
#define RX_TUNE_REG  0x3
#define EXT_SYNC_REG 0x4
#define DRX_REG      0x6
#define RF_CONF_REG  0x7
#define FS_CTRL_REG  0x9
#define AON_REG      0xA
#define OTP_IF_REG   0xB
#define CIA_REG1     0xC
#define PMSC_REG     0x11
#define RX_BUFFER_0_REG 0x12
#define TX_BUFFER_REG   0x14

// SS-TWR uses only 3 stages (no FINAL/REPORT)
#define STAGE_POLL   1
#define STAGE_RESP   2   // Anchor encodes t_reply in this frame
#define STAGE_BCAST  5
#define STAGE_SYNC   6

// ==================== GLOBALS ====================

static int ANTENNA_DELAY = 16350;
int destination = 0x0;
int sender = 0x0;

int config[] = {
    CHANNEL_5, PREAMBLE_128, 9, PAC8,
    DATARATE_6_8MB, PHR_MODE_STANDARD, PHR_RATE_850KB
};

static int      my_slot        = 0;
static unsigned long slot_start_ms  = 0;
static unsigned long cycle_epoch_ms = 0;
static bool     synced         = false;
static unsigned long cycle_count    = 0;

static unsigned long stat_attempts = 0;
static unsigned long stat_ok       = 0;
static unsigned long stat_timeout  = 0;
static unsigned long stat_err      = 0;
static unsigned long stat_bcast    = 0;

// ==================== ANCHOR DATA ====================

struct AnchorData {
    int   anchor_id;
    float distance      = 0.0f;
    float ema_distance  = -1.0f;   // -1 = uninitialized
    float signal_strength = 0.0f;
};

AnchorData anchors[NUM_ANCHORS];

void initAnchors() {
    for (int i = 0; i < NUM_ANCHORS; i++)
        anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
}

// ==================== FILTERING ====================
//
// Outlier rejection: if the new reading deviates from the current EMA
// by more than OUTLIER_THRESH_CM, discard it entirely.
// This prevents a single bad boot-time measurement from locking the EMA
// onto a phantom distance (the "98.5cm stuck forever" bug).

void updateFilter(AnchorData &a) {
    if (a.distance < MIN_DISTANCE || a.distance > MAX_DISTANCE) return;

    if (a.ema_distance < 0.0f) {
        // First valid reading: seed EMA unconditionally
        a.ema_distance = a.distance;
        return;
    }

    // Outlier rejection
    if (fabsf(a.distance - a.ema_distance) > OUTLIER_THRESH_CM) {
        // Don't update — log if useful for debugging
        Serial.print("# [OUTLIER] A"); Serial.print(a.anchor_id);
        Serial.print(" raw="); Serial.print(a.distance, 1);
        Serial.print(" ema="); Serial.println(a.ema_distance, 1);
        return;
    }

    a.ema_distance = EMA_ALPHA * a.distance + (1.0f - EMA_ALPHA) * a.ema_distance;
}

// ==================== TDMA ====================

void waitForSlot() {
    unsigned long cycle = (unsigned long)NUM_TAGS * SLOT_DURATION_MS;
    unsigned long my_start = (unsigned long)my_slot * SLOT_DURATION_MS;

    if (!synced) {
        unsigned long pos = millis() % cycle;
        if (pos < my_start) delay(my_start - pos);
        else if (pos >= my_start + SLOT_DURATION_MS)
            delay(cycle - pos + my_start);
        slot_start_ms = millis();
        return;
    }

    unsigned long elapsed = millis() - cycle_epoch_ms;
    unsigned long pos     = elapsed % cycle;
    unsigned long wait    = 0;
    if (pos <= my_start)
        wait = my_start - pos;
    else if (pos >= my_start + SLOT_DURATION_MS)
        wait = cycle - pos + my_start;

    if (wait > 0) delay(wait);
    slot_start_ms = millis();
}

bool slotExpired() {
    return (millis() - slot_start_ms) >= SLOT_DURATION_MS;
}

// ==================== DWM3000 DRIVER ====================

class DWM3000Class {
public:
    static void begin();
    static void init();
    static void writeSysConfig();
    static void configureAsTX();
    static void setupGPIO();
    static void ss_sendPoll(int dest);
    static int  ss_getStage();
    static bool ss_isErrorFrame();
    static void setMode(int mode);
    static void setFrameLength(int len);
    static void setTXAntennaDelay(int d);
    static void setSenderID(int s);
    static void setDestinationID(int d);
    static int  receivedFrameSucc();
    static int  sentFrameSucc();
    static int  getSenderID();
    static int  getDestinationID();
    static bool checkForIDLE();
    static bool checkSPI();
    static double getSignalStrength();
    static unsigned long long readRXTimestamp();
    static unsigned long long readTXTimestamp();
    static uint32_t write(int base, int sub, uint32_t data, int len);
    static uint32_t write(int base, int sub, uint32_t data);
    static uint32_t read(int base, int sub);
    static uint8_t  read8bit(int base, int sub);
    static uint32_t readOTP(uint8_t addr);
    static void forceIdle();
    static void standardTX();
    static void standardRX();
    static void TXInstantRX();
    static void softReset();
    static void hardReset();
    static void clearSystemStatus();
    static double convertToCM(long long u);
private:
    static void spiSelect(uint8_t cs);
    static void setBit(int r, int s, int sh, bool b);
    static void setBitHigh(int r, int s, int sh);
    static void writeFastCommand(int cmd);
    static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub,
                                           uint32_t data, uint32_t len, uint32_t rw);
    static uint32_t sendBytes(int b[], int lenB, int recLen);
    static void clearAONConfig();
    static unsigned int countBits(unsigned int n);
    static int checkForDevID();
};

DWM3000Class DWM3000;

// ==================== FORWARD DECLARATIONS ====================

bool rangeWithAnchor(int idx);
void broadcastDistances();

// ==================== SYNC ====================

bool waitForSyncBeacon(unsigned long timeout_ms) {
    DWM3000.forceIdle();
    DWM3000.clearSystemStatus();
    DWM3000.standardRX();
    unsigned long t0 = millis();
    while ((millis() - t0) < timeout_ms) {
        int rx = DWM3000.receivedFrameSucc();
        if (rx == 1) {
            int stage = DWM3000.ss_getStage();
            int dest  = DWM3000.getDestinationID();
            DWM3000.clearSystemStatus();
            if (stage == STAGE_SYNC && dest == 0xFF) {
                cycle_epoch_ms = millis();
                synced = true;
                Serial.println("# [SYNC] Beacon received");
                DWM3000.forceIdle();
                return true;
            }
            DWM3000.standardRX();
        } else if (rx == 2) {
            DWM3000.clearSystemStatus();
            DWM3000.standardRX();
        }
    }
    return false;
}

// ==================== BROADCAST ====================

void broadcastDistances() {
    DWM3000.forceIdle();
    delay(1);
    DWM3000.clearSystemStatus();

    DWM3000.setMode(1);
    DWM3000.write(TX_BUFFER_REG, 0x01, TAG_ID & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x02, LISTENER_ANCHOR_ID & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x03, STAGE_BCAST & 0x7);

    for (int i = 0; i < NUM_ANCHORS; i++) {
        uint16_t d = (uint16_t)(anchors[i].ema_distance * 10.0f);
        DWM3000.write(TX_BUFFER_REG, 0x04 + (i * 2), d, 2);
    }
    for (int i = 0; i < NUM_ANCHORS; i++) {
        int16_t r = (int16_t)(anchors[i].signal_strength * 100);
        DWM3000.write(TX_BUFFER_REG, 0x04 + (NUM_ANCHORS * 2) + (i * 2), (uint16_t)r, 2);
    }

    // FIX: 4 header bytes (mode + sender + dest + stage) + distances + RSSI
    DWM3000.setFrameLength(4 + NUM_ANCHORS * 4);
    DWM3000.clearSystemStatus();
    DWM3000.standardTX();

    bool ok = false;
    unsigned long t0 = millis();
    while ((millis() - t0) < 15) {
        if (DWM3000.sentFrameSucc()) { ok = true; break; }
        delayMicroseconds(100);
    }

    stat_bcast += ok ? 1 : 0;
    if (!ok) Serial.println("# [BCAST] FAIL");

    DWM3000.forceIdle();
    DWM3000.clearSystemStatus();
}

// ==================== SS-TWR RANGING ====================
//
// SS-TWR protocol (2 frames only):
//
//   Tag                         Anchor N
//    |------- POLL ------------>|   tag TX timestamp: t_poll_tx
//    |                          |   anchor RX timestamp: t_poll_rx
//    |                          |   anchor processes, prepares RESP
//    |                          |   anchor TX timestamp: t_resp_tx
//    |<------- RESP ------------|   (RESP frame contains t_reply = t_resp_tx - t_poll_rx)
//    | tag RX timestamp: t_resp_rx
//
//   TOF = (t_resp_rx - t_poll_tx - t_reply) / 2
//
// No FINAL or REPORT frames needed. Clock offset error is small
// for short reply times (~1-2ms) and negligible for spatial audio.

bool rangeWithAnchor(int idx) {
    AnchorData *a = &anchors[idx];
    int aid = a->anchor_id;

    stat_attempts++;

    // --- Step 1: Send POLL ---
    DWM3000.clearSystemStatus();
    DWM3000.forceIdle();
    delayMicroseconds(50);

    DWM3000.setMode(1);
    DWM3000.write(TX_BUFFER_REG, 0x01, TAG_ID & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x02, aid & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x03, STAGE_POLL & 0x7);
    DWM3000.setFrameLength(4);
    DWM3000.clearSystemStatus();
    DWM3000.TXInstantRX();   // TX then immediately listen for RESP

    // Wait for TX complete
    unsigned long t0 = millis();
    while ((millis() - t0) < 5) {
        if (DWM3000.sentFrameSucc()) break;
        delayMicroseconds(50);
    }
    unsigned long long t_poll_tx = DWM3000.readTXTimestamp();

    // --- Step 2: Wait for RESP ---
    bool got_resp = false;
    t0 = millis();
    while ((millis() - t0) < RX_TIMEOUT_MS) {
        int rx = DWM3000.receivedFrameSucc();
        if (rx == 1) {
            DWM3000.clearSystemStatus();
            if (!DWM3000.ss_isErrorFrame() &&
                DWM3000.ss_getStage() == STAGE_RESP &&
                DWM3000.getSenderID() == aid) {
                got_resp = true;
                a->signal_strength = DWM3000.getSignalStrength();
            } else {
                stat_err++;
            }
            break;
        } else if (rx == 2) {
            DWM3000.clearSystemStatus();
            stat_err++;
            break;
        }
    }

    if (!got_resp) {
        stat_timeout++;
        DWM3000.forceIdle();
        DWM3000.clearSystemStatus();
        return false;
    }

    // --- Step 3: Compute distance from SS-TWR ---
    unsigned long long t_resp_rx = DWM3000.readRXTimestamp();

    // t_reply is packed by anchor into RESP frame bytes 0x04..0x07 (uint32, little-endian)
    uint32_t t_reply_raw = DWM3000.read(RX_BUFFER_0_REG, 0x04);
    long long t_reply = (long long)t_reply_raw;

    // SS-TWR: TOF = (round_trip - t_reply) / 2
    long long t_round = (long long)t_resp_rx - (long long)t_poll_tx;
    long long tof = (t_round - t_reply) / 2;

    // Sanity check: TOF should be positive and < ~300ns (100m at c)
    if (tof < 0 || tof > 20000) {
        DWM3000.forceIdle();
        DWM3000.clearSystemStatus();
        stat_err++;
        return false;
    }

    a->distance = (float)DWM3000.convertToCM(tof);

    Serial.print("# [SS] A"); Serial.print(aid);
    Serial.print(" raw="); Serial.println(a->distance, 1);

    updateFilter(*a);
    stat_ok++;

    DWM3000.forceIdle();
    DWM3000.clearSystemStatus();
    return true;
}

// ==================== DWM3000 IMPLEMENTATIONS ====================

void DWM3000Class::spiSelect(uint8_t cs) {
    pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); delay(5);
}
void DWM3000Class::begin() {
    delay(5); pinMode(CHIP_SELECT_PIN, OUTPUT);
    SPI.begin(); delay(5); spiSelect(CHIP_SELECT_PIN);
}

void DWM3000Class::init() {
    if (!checkForDevID()) { Serial.println("[ERROR] Dev ID wrong!"); return; }
    setBitHigh(GEN_CFG_AES_LOW_REG, 0x10, 4);
    while (!checkForIDLE()) { Serial.println("[WARN] IDLE failed (1)"); delay(100); }
    softReset(); delay(200);
    while (!checkForIDLE()) { Serial.println("[WARN] IDLE failed (2)"); delay(100); }
    uint32_t ldo_low  = readOTP(0x04);
    uint32_t ldo_high = readOTP(0x05);
    uint32_t bias_tune = (readOTP(0xA) >> 16) & BIAS_CTRL_BIAS_MASK;
    if (ldo_low && ldo_high && bias_tune) {
        write(0x11, 0x1F, bias_tune); write(0x0B, 0x08, 0x0100);
    }
    int xtrim = readOTP(0x1E);
    xtrim = (xtrim == 0) ? 0x2E : xtrim;
    write(FS_CTRL_REG, 0x14, xtrim);
    writeSysConfig();
    write(0x00, 0x3C, 0xFFFFFFFF); write(0x00, 0x40, 0xFFFF);
    write(0x0A, 0x00, 0x000900, 3);
    write(0x3, 0x1C, 0x10000240); write(0x3, 0x20, 0x1B6DA489);
    write(0x3, 0x38, 0x0001C0FD); write(0x3, 0x3C, 0x0001C43E);
    write(0x3, 0x40, 0x0001C6BE); write(0x3, 0x44, 0x0001C77E);
    write(0x3, 0x48, 0x0001CF36); write(0x3, 0x4C, 0x0001CFB5);
    write(0x3, 0x50, 0x0001CFF5); write(0x3, 0x18, 0xE5E5);
    read(0x4, 0x20);
    write(0x6, 0x0, 0x81101C); write(0x07, 0x34, 0x4); write(0x07, 0x48, 0x14);
    write(0x07, 0x1A, 0x0E); write(0x07, 0x1C, 0x1C071134);
    write(0x09, 0x00, 0x1F3C); write(0x09, 0x80, 0x81);
    write(0x11, 0x04, 0xB40200); write(0x11, 0x08, 0x80030738);
    Serial.println("[OK] DWM3000 init complete");
}

void DWM3000Class::writeSysConfig() {
    int usr_cfg = (STDRD_SYS_CONFIG & 0xFFF) | (config[5] << 3) | (config[6] << 4);
    write(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg);
    int otp_write = 0x1400;
    if (config[1] >= 256) otp_write |= 0x04;
    write(OTP_IF_REG, 0x08, otp_write);
    write(DRX_REG, 0x00, 0x00, 1); write(DRX_REG, 0x0, config[3]);
    write(STS_CFG_REG, 0x0, 64 / 8 - 1);
    write(GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1);
    write(DRX_REG, 0x0C, 0xAF5F584C);
    int chan_ctrl = read(GEN_CFG_AES_HIGH_REG, 0x14);
    chan_ctrl &= (~0x1FFF);
    chan_ctrl |= config[0];
    chan_ctrl |= 0x1F00 & (config[2] << 8);
    chan_ctrl |= 0xF8   & (config[2] << 3);
    chan_ctrl |= 0x06   & (0x01 << 1);
    write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl);
    int tx_fctrl = read(GEN_CFG_AES_LOW_REG, 0x24);
    tx_fctrl |= (config[1] << 12);
    tx_fctrl |= (config[4] << 10);
    write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl);
    write(DRX_REG, 0x02, 0x81);
    int rf_tx = 0x1C071134;
    int pll   = 0x0F3C;
    if (config[0]) { rf_tx &= ~0x00FFFF; rf_tx |= 0x000001; pll &= 0x00FF; pll |= 0x001F; }
    write(RF_CONF_REG, 0x1C, rf_tx); write(FS_CTRL_REG, 0x00, pll);
    write(RF_CONF_REG, 0x51, 0x14); write(RF_CONF_REG, 0x1A, 0x0E);
    write(FS_CTRL_REG, 0x08, 0x81); write(GEN_CFG_AES_LOW_REG, 0x44, 0x02);
    write(PMSC_REG, 0x04, 0x300200); write(PMSC_REG, 0x08, 0x0138);
    int ok = 0;
    for (int i = 0; i < 100; i++) {
        if (read(GEN_CFG_AES_LOW_REG, 0x0) & 0x2) { ok = 1; break; }
    }
    if (!ok) Serial.println("[ERROR] PLL lock failed!");
    int otp_val = read(OTP_IF_REG, 0x08);
    otp_val |= 0x40;
    if (config[0]) otp_val |= 0x2000;
    write(OTP_IF_REG, 0x08, otp_val);
    write(RX_TUNE_REG, 0x19, 0xF0);
    int ldo_ctrl = read(RF_CONF_REG, 0x48);
    write(RF_CONF_REG, 0x48, 0x105 | 0x100 | 0x4 | 0x1);
    write(EXT_SYNC_REG, 0x0C, 0x020000); read(0x04, 0x0C); delay(20);
    write(EXT_SYNC_REG, 0x0C, 0x11);
    int succ = 0;
    for (int i = 0; i < 100; i++) {
        if (read(EXT_SYNC_REG, 0x20)) { succ = 1; break; }
        delay(10);
    }
    if (!succ) Serial.println("[ERROR] PGF cal failed!");
    write(EXT_SYNC_REG, 0x0C, 0x00); write(EXT_SYNC_REG, 0x20, 0x01);
    if (read(EXT_SYNC_REG, 0x14) == 0x1fffffff) Serial.println("[ERROR] PGF I fail!");
    if (read(EXT_SYNC_REG, 0x1C) == 0x1fffffff) Serial.println("[ERROR] PGF Q fail!");
    write(RF_CONF_REG, 0x48, ldo_ctrl);
    write(0x0E, 0x02, 0x01);
    setTXAntennaDelay(ANTENNA_DELAY);
}

void DWM3000Class::configureAsTX() {
    write(RF_CONF_REG, 0x1C, 0x34);
    write(GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD);
}
void DWM3000Class::setupGPIO() { write(0x05, 0x08, 0xF0); }

// SS-TWR: tag only sends POLL; anchor sends RESP with embedded t_reply.
// This replaces ds_sendFrame/ds_sendRTInfo/ds_processRTInfo entirely.
void DWM3000Class::ss_sendPoll(int dest) {
    setMode(1);
    write(TX_BUFFER_REG, 0x01, sender & 0xFF);
    write(TX_BUFFER_REG, 0x02, dest  & 0xFF);
    write(TX_BUFFER_REG, 0x03, STAGE_POLL & 0x7);
    setFrameLength(4);
    TXInstantRX();
    for (int i = 0; i < 50; i++) { if (sentFrameSucc()) return; }
    Serial.println("[ERROR] POLL TX failed");
}

int  DWM3000Class::ss_getStage()     { return read(RX_BUFFER_0_REG, 0x03) & 0b111; }
bool DWM3000Class::ss_isErrorFrame() { return ((read(RX_BUFFER_0_REG, 0x00) & 0x7) == 7); }
void DWM3000Class::setMode(int mode) { write(TX_BUFFER_REG, 0x00, mode & 0x7); }
void DWM3000Class::setFrameLength(int len) {
    len += FCS_LEN;
    int cfg = read(0x00, 0x24);
    write(GEN_CFG_AES_LOW_REG, 0x24, (cfg & 0xFFFFFC00) | len);
}
void DWM3000Class::setTXAntennaDelay(int d) { ANTENNA_DELAY = d; write(0x01, 0x04, d); }
void DWM3000Class::setSenderID(int s)   { sender = s; }
void DWM3000Class::setDestinationID(int d) { destination = d; }
int  DWM3000Class::receivedFrameSucc() {
    int s = read(GEN_CFG_AES_LOW_REG, 0x44);
    if (s & SYS_STATUS_FRAME_RX_SUCC) return 1;
    if (s & SYS_STATUS_RX_ERR)        return 2;
    return 0;
}
int  DWM3000Class::sentFrameSucc()    { return (read(GEN_CFG_AES_LOW_REG, 0x44) & SYS_STATUS_FRAME_TX_SUCC) ? 1 : 0; }
int  DWM3000Class::getSenderID()      { return read(RX_BUFFER_0_REG, 0x01) & 0xFF; }
int  DWM3000Class::getDestinationID() { return read(RX_BUFFER_0_REG, 0x02) & 0xFF; }
bool DWM3000Class::checkForIDLE() {
    return ((read(0x0F, 0x30) >> 16) & PMSC_STATE_IDLE) == PMSC_STATE_IDLE ||
           ((read(0x00, 0x44) >> 16) & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK);
}
bool DWM3000Class::checkSPI() { return checkForDevID(); }
double DWM3000Class::getSignalStrength() {
    int cir = read(CIA_REG1, 0x2C) & 0x1FF;
    int pac = read(CIA_REG1, 0x58) & 0xFFF;
    if (pac == 0) return 0.0;
    unsigned int dgc = (read(RX_TUNE_REG, 0x60) >> 28) & 0x7;
    return 10 * log10((cir * (1 << 21)) / pow(pac, 2)) + (6 * dgc) - 121.7;
}
unsigned long long DWM3000Class::readRXTimestamp() {
    uint32_t lo = read(CIA_REG1, 0x00);
    unsigned long long hi = read(CIA_REG1, 0x04) & 0xFF;
    return (hi << 32) | lo;
}
unsigned long long DWM3000Class::readTXTimestamp() {
    unsigned long long lo = read(0x00, 0x74);
    unsigned long long hi = read(0x00, 0x78) & 0xFF;
    return (hi << 32) + lo;
}
uint32_t DWM3000Class::write(int base, int sub, uint32_t data, int len) {
    return readOrWriteFullAddress(base, sub, data, len, 1);
}
uint32_t DWM3000Class::write(int base, int sub, uint32_t data) {
    return readOrWriteFullAddress(base, sub, data, 0, 1);
}
uint32_t DWM3000Class::read(int base, int sub)  { return readOrWriteFullAddress(base, sub, 0, 0, 0); }
uint8_t  DWM3000Class::read8bit(int base, int sub) { return (uint8_t)(read(base, sub) >> 24); }
uint32_t DWM3000Class::readOTP(uint8_t addr) {
    write(OTP_IF_REG, 0x04, addr); write(OTP_IF_REG, 0x08, 0x02);
    return read(OTP_IF_REG, 0x10);
}
void DWM3000Class::forceIdle()         { writeFastCommand(0x00); }
void DWM3000Class::standardTX()        { writeFastCommand(0x01); }
void DWM3000Class::standardRX()        { writeFastCommand(0x02); }
void DWM3000Class::TXInstantRX()       { writeFastCommand(0x0C); }
void DWM3000Class::softReset() {
    clearAONConfig();
    write(PMSC_REG, 0x04, 0x1); write(PMSC_REG, 0x00, 0x00, 2);
    delay(100);
    write(PMSC_REG, 0x00, 0xFFFF); write(PMSC_REG, 0x04, 0x00, 1);
}
void DWM3000Class::hardReset() {
    pinMode(RST_PIN, OUTPUT); digitalWrite(RST_PIN, LOW);
    delay(10); pinMode(RST_PIN, INPUT);
}
void DWM3000Class::clearSystemStatus() { write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF); }
double DWM3000Class::convertToCM(long long u) { return (double)u * PS_UNIT * SPEED_OF_LIGHT; }
void DWM3000Class::setBit(int r, int s, int sh, bool b) {
    uint8_t t = read8bit(r, s);
    if (b) bitSet(t, sh); else bitClear(t, sh);
    write(r, s, t);
}
void DWM3000Class::setBitHigh(int r, int s, int sh) { setBit(r, s, sh, 1); }
void DWM3000Class::writeFastCommand(int cmd) {
    int h = 0x1 | ((cmd & 0x1F) << 1) | 0x80;
    int arr[] = {h};
    sendBytes(arr, 1, 0);
}

uint32_t DWM3000Class::readOrWriteFullAddress(uint32_t base, uint32_t sub,
                                               uint32_t data, uint32_t dataLen, uint32_t rw) {
    uint32_t header = 0x00;
    if (rw) header |= 0x80;
    header |= ((base & 0x1F) << 1);
    if (sub > 0) { header |= 0x40; header <<= 8; header |= ((sub & 0x7F) << 2); }
    uint32_t hs = header > 0xFF ? 2 : 1;
    if (!rw) {
        int ha[hs];
        if (hs == 1) ha[0] = header;
        else { ha[0] = (header >> 8); ha[1] = header & 0xFF; }
        return (uint32_t)sendBytes(ha, hs, 4);
    } else {
        uint32_t pb = 0;
        if (dataLen == 0) {
            if (data > 0) { uint32_t bits = countBits(data); pb = bits / 8; if (bits % 8) pb++; }
            else pb = 1;
        } else pb = dataLen;
        int payload[hs + pb];
        if (hs == 1) payload[0] = header;
        else { payload[0] = (header >> 8); payload[1] = header & 0xFF; }
        for (uint32_t i = 0; i < pb; i++) payload[hs + i] = (data >> i * 8) & 0xFF;
        return (uint32_t)sendBytes(payload, 2 + pb, 0);
    }
}

uint32_t DWM3000Class::sendBytes(int b[], int lenB, int recLen) {
    digitalWrite(CHIP_SELECT_PIN, LOW);
    for (int i = 0; i < lenB; i++) SPI.transfer(b[i]);
    uint32_t val = 0;
    if (recLen > 0) {
        for (int i = 0; i < recLen; i++) {
            uint32_t tmp = SPI.transfer(0x00);
            if (i == 0) val = tmp; else val |= tmp << (8 * i);
        }
    }
    digitalWrite(CHIP_SELECT_PIN, HIGH);
    return val;
}

void DWM3000Class::clearAONConfig() {
    write(AON_REG, NO_OFFSET, 0x00, 2); write(AON_REG, 0x14, 0x00, 1);
    write(AON_REG, 0x04, 0x00, 1); write(AON_REG, 0x04, 0x02); delay(1);
}
unsigned int DWM3000Class::countBits(unsigned int n) { return (int)log2(n) + 1; }
int DWM3000Class::checkForDevID() {
    int res = read(GEN_CFG_AES_LOW_REG, NO_OFFSET);
    if (res != 0xDECA0302 && res != 0xDECA0312) { Serial.println("[ERROR] DEV_ID wrong!"); return 0; }
    return 1;
}

// ==================== SETUP ====================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== UWB Tag (SS-TWR TDMA) ===");
    Serial.print("Tag ID: "); Serial.println(TAG_ID);
    my_slot = TAG_ID - FIRST_TAG_ID;
    Serial.print("Slot: "); Serial.print(my_slot);
    Serial.print("/"); Serial.println(NUM_TAGS);
    Serial.print("Slot duration: "); Serial.print(SLOT_DURATION_MS); Serial.println("ms");
    Serial.print("Cycle: "); Serial.print(NUM_TAGS * SLOT_DURATION_MS); Serial.println("ms");

    initAnchors();
    DWM3000.begin(); DWM3000.hardReset(); delay(200);
    if (!DWM3000.checkSPI()) { Serial.println("[FATAL] SPI failed"); while (1); }
    while (!DWM3000.checkForIDLE()) { Serial.println("[ERROR] IDLE"); delay(1000); }
    DWM3000.softReset(); delay(200);
    if (!DWM3000.checkForIDLE()) { Serial.println("[FATAL] IDLE2"); while (1); }
    DWM3000.init(); DWM3000.setupGPIO();
    DWM3000.setTXAntennaDelay(ANTENNA_DELAY);
    DWM3000.setSenderID(TAG_ID);
    DWM3000.configureAsTX();
    DWM3000.clearSystemStatus();

    Serial.println("# Waiting for sync beacon...");
    if (waitForSyncBeacon(SYNC_TIMEOUT_MS)) {
        Serial.println("# [SYNC] Aligned");
    } else {
        Serial.println("# [SYNC] No beacon, free-running");
        cycle_epoch_ms = millis();
        synced = true;
    }

    Serial.println("[OK] Tag ready\n");
}

// ==================== MAIN LOOP ====================

#define RESYNC_EVERY_N_CYCLES 100

void loop() {
    cycle_count++;
    if (cycle_count % RESYNC_EVERY_N_CYCLES == 0) {
        waitForSyncBeacon((unsigned long)NUM_TAGS * SLOT_DURATION_MS);
    }

    waitForSlot();

    for (int a = 0; a < NUM_ANCHORS; a++) {
        if (slotExpired()) {
            Serial.println("# [SLOT] Expired mid-ranging");
            break;
        }
        rangeWithAnchor(a);
    }

    if (!slotExpired()) {
        broadcastDistances();
    } else {
        Serial.println("# [BCAST] Skipped - slot expired");
    }
}
