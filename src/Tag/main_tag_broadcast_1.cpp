// ============================================================
//  UWB Spatial Tracking - TAG Firmware (Multi-Tag TDMA)
//
//  Tags are powered externally (USB/battery). They:
//    1. Range with each anchor via DS-TWR
//    2. Broadcast all distances in one UWB frame to anchor 1
//    3. Anchor 1 (listener) outputs CSV over serial to computer
//
//  FLASH: Same file for all tags. Set TAG_ID via build_flags.
//  SYSTEM CONFIG must match across all devices.
// ============================================================

#include <Arduino.h>
#include <SPI.h>

// ==================== CONFIGURATION ====================

#ifndef TAG_ID
#define TAG_ID              8
#endif

#define NUM_ANCHORS         4
#define FIRST_ANCHOR_ID     1
#define LISTENER_ANCHOR_ID  1
#define NUM_TAGS            3
#define FIRST_TAG_ID        8

#define RST_PIN             27
#define CHIP_SELECT_PIN     4

#define RX_TIMEOUT_MS       10
#define INTER_RANGE_DELAY   2
#define SLOT_DURATION_MS    100
#define MAX_RANGE_RETRIES   1

#define FILTER_SIZE         5
#define MIN_DISTANCE        0.0
#define MAX_DISTANCE        2000.0

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
#define CLOCK_OFFSET_CHAN_5_CONSTANT -0.5731e-3f
#define CLOCK_OFFSET_CHAN_9_CONSTANT -0.1252e-3f
#define PS_UNIT 15.6500400641025641
#define SPEED_OF_LIGHT 0.029979245800
#define NO_OFFSET 0x0

#define GEN_CFG_AES_LOW_REG 0x00
#define GEN_CFG_AES_HIGH_REG 0x01
#define STS_CFG_REG 0x2
#define RX_TUNE_REG 0x3
#define EXT_SYNC_REG 0x4
#define DRX_REG 0x6
#define RF_CONF_REG 0x7
#define FS_CTRL_REG 0x9
#define AON_REG 0xA
#define OTP_IF_REG 0xB
#define CIA_REG1 0xC
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define TX_BUFFER_REG 0x14

#define STAGE_POLL     1
#define STAGE_RESP     2
#define STAGE_FINAL    3
#define STAGE_REPORT   4
#define STAGE_BCAST    5

// ==================== GLOBALS ====================

static int ANTENNA_DELAY = 16350;
int led_status = 0;
int destination = 0x0;
int sender = 0x0;

int config[] = {
    CHANNEL_5, PREAMBLE_128, 9, PAC8,
    DATARATE_6_8MB, PHR_MODE_STANDARD, PHR_RATE_850KB
};

static int my_slot = 0;
static unsigned long slot_start_ms = 0;

static unsigned long stat_attempts = 0;
static unsigned long stat_ok = 0;
static unsigned long stat_timeout = 0;
static unsigned long stat_err = 0;
static unsigned long stat_bcast = 0;

// ==================== ANCHOR DATA ====================

struct AnchorData {
    int anchor_id;
    int t_roundA = 0;
    int t_replyA = 0;
    long long rx_ts = 0;
    long long tx_ts = 0;
    int clock_offset = 0;
    float distance = 0;
    float distance_history[FILTER_SIZE] = {0};
    int history_index = 0;
    float filtered_distance = 0;
    float signal_strength = 0;
};

AnchorData anchors[NUM_ANCHORS];

void initAnchors() {
    for (int i = 0; i < NUM_ANCHORS; i++)
        anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
}

// ==================== FILTERING ====================

float calcMedian(float arr[], int size) {
    float tmp[size];
    for (int i = 0; i < size; i++) tmp[i] = arr[i];
    for (int i = 0; i < size - 1; i++)
        for (int j = i + 1; j < size; j++)
            if (tmp[j] < tmp[i]) { float t = tmp[i]; tmp[i] = tmp[j]; tmp[j] = t; }
    return (size % 2 == 0) ? (tmp[size/2 - 1] + tmp[size/2]) / 2.0 : tmp[size/2];
}

void updateFilter(AnchorData &a) {
    a.distance_history[a.history_index] = a.distance;
    a.history_index = (a.history_index + 1) % FILTER_SIZE;
    float valid[FILTER_SIZE];
    int cnt = 0;
    for (int i = 0; i < FILTER_SIZE; i++)
        if (a.distance_history[i] >= MIN_DISTANCE && a.distance_history[i] <= MAX_DISTANCE)
            valid[cnt++] = a.distance_history[i];
    a.filtered_distance = (cnt > 0) ? calcMedian(valid, cnt) : 0;
}

// ==================== TDMA ====================

void waitForSlot() {
    unsigned long cycle = (unsigned long)NUM_TAGS * SLOT_DURATION_MS;
    unsigned long now = millis();
    unsigned long pos = now % cycle;
    unsigned long my_start = (unsigned long)my_slot * SLOT_DURATION_MS;
    unsigned long wait = 0;
    if (pos <= my_start) wait = my_start - pos;
    else if (pos < my_start + SLOT_DURATION_MS) wait = 0;
    else wait = cycle - pos + my_start;
    if (wait > 0) delay(wait);
    slot_start_ms = millis();
}

bool slotExpired() { return (millis() - slot_start_ms) >= SLOT_DURATION_MS; }

// ==================== DWM3000 DRIVER ====================

class DWM3000Class {
public:
    static void begin();
    static void init();
    static void writeSysConfig();
    static void configureAsTX();
    static void setupGPIO();
    static void ds_sendFrame(int stage);
    static void ds_sendRTInfo(int trB, int trpB);
    static int  ds_processRTInfo(int trA, int trpA, int trB, int trpB, int clk);
    static int  ds_getStage();
    static bool ds_isErrorFrame();
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
    static int    getRawClockOffset();
    static long double getClockOffset(int32_t o);
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
    static double convertToCM(int u);
private:
    static void spiSelect(uint8_t cs);
    static void setBit(int r, int s, int sh, bool b);
    static void setBitHigh(int r, int s, int sh);
    static void writeFastCommand(int cmd);
    static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t len, uint32_t rw);
    static uint32_t sendBytes(int b[], int lenB, int recLen);
    static void clearAONConfig();
    static unsigned int countBits(unsigned int n);
    static int checkForDevID();
};

DWM3000Class DWM3000;

// ==================== FORWARD DECLARATIONS ====================

bool rangeWithAnchor(int idx);
void broadcastDistances();

// ==================== BROADCAST ====================
// KEY FIX: Use standardTX() for broadcast (fire-and-forget).
// The old code used TXInstantRX() which auto-transitions to RX after TX.
// But after the last failed ranging attempt, the radio was left in a
// bad RX state that prevented TXInstantRX from working. standardTX()
// just transmits and stops, which is what we want for a broadcast.
// The listener is already in RX mode waiting.

void broadcastDistances() {
    // Full reset to clean state
    DWM3000.forceIdle();
    delay(2);
    DWM3000.clearSystemStatus();
    delay(1);

    // Build the frame
    DWM3000.setMode(1);
    DWM3000.write(TX_BUFFER_REG, 0x01, TAG_ID & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x02, LISTENER_ANCHOR_ID & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x03, STAGE_BCAST & 0x7);

    for (int i = 0; i < NUM_ANCHORS; i++) {
        uint16_t d = (uint16_t)(anchors[i].filtered_distance);
        DWM3000.write(TX_BUFFER_REG, 0x04 + (i * 2), d, 2);
    }

    for (int i = 0; i < NUM_ANCHORS; i++) {
        int16_t r = (int16_t)(anchors[i].signal_strength * 100);
        DWM3000.write(TX_BUFFER_REG, 0x04 + (NUM_ANCHORS * 2) + (i * 2), (uint16_t)r, 2);
    }

    DWM3000.setFrameLength(3 + NUM_ANCHORS * 4);

    // Clear status bits RIGHT before TX
    DWM3000.clearSystemStatus();

    // Use standardTX (not TXInstantRX) - just send and stop
    DWM3000.standardTX();

    // Poll for TX complete
    bool ok = false;
    unsigned long t0 = millis();
    while ((millis() - t0) < 15) {
        if (DWM3000.sentFrameSucc()) { ok = true; break; }
        delayMicroseconds(100);
    }

    if (ok) {
        stat_bcast++;
        Serial.println("[BCAST] OK");
    } else {
        Serial.println("[BCAST] FAIL");
    }

    DWM3000.forceIdle();
    DWM3000.clearSystemStatus();
}

// ==================== DS-TWR RANGING ====================

bool rangeWithAnchor(int idx) {
    AnchorData *a = &anchors[idx];
    int aid = a->anchor_id;

    for (int attempt = 0; attempt <= MAX_RANGE_RETRIES; attempt++) {
        stat_attempts++;
        unsigned long t0;

        // Step 1: Send poll
        DWM3000.clearSystemStatus();
        DWM3000.forceIdle();
        delayMicroseconds(50);
        a->t_roundA = 0;
        a->t_replyA = 0;
        DWM3000.setDestinationID(aid);
        DWM3000.ds_sendFrame(STAGE_POLL);
        a->tx_ts = DWM3000.readTXTimestamp();

        // Step 2: Wait for response
        bool got_resp = false;
        t0 = millis();
        while ((millis() - t0) < RX_TIMEOUT_MS) {
            int rx = DWM3000.receivedFrameSucc();
            if (rx == 1) {
                DWM3000.clearSystemStatus();
                if (!DWM3000.ds_isErrorFrame() &&
                    DWM3000.ds_getStage() == STAGE_RESP &&
                    DWM3000.getSenderID() == aid) {
                    got_resp = true;
                } else { stat_err++; }
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
            continue;
        }

        // Step 3: Send final
        a->rx_ts = DWM3000.readRXTimestamp();
        DWM3000.ds_sendFrame(STAGE_FINAL);
        a->t_roundA = a->rx_ts - a->tx_ts;
        a->tx_ts = DWM3000.readTXTimestamp();
        a->t_replyA = a->tx_ts - a->rx_ts;

        // Step 4: Wait for report
        bool got_report = false;
        t0 = millis();
        while ((millis() - t0) < RX_TIMEOUT_MS) {
            int rx = DWM3000.receivedFrameSucc();
            if (rx == 1) {
                DWM3000.clearSystemStatus();
                if (!DWM3000.ds_isErrorFrame() &&
                    DWM3000.ds_getStage() == STAGE_REPORT &&
                    DWM3000.getSenderID() == aid) {
                    a->clock_offset = DWM3000.getRawClockOffset();
                    got_report = true;
                } else { stat_err++; }
                break;
            } else if (rx == 2) {
                DWM3000.clearSystemStatus();
                stat_err++;
                break;
            }
        }
        if (!got_report) {
            stat_timeout++;
            DWM3000.forceIdle();
            DWM3000.clearSystemStatus();
            continue;
        }

        // Step 5: Compute distance
        int tof = DWM3000.ds_processRTInfo(
            a->t_roundA, a->t_replyA,
            DWM3000.read(RX_BUFFER_0_REG, 0x04),
            DWM3000.read(RX_BUFFER_0_REG, 0x08),
            a->clock_offset);

        a->distance = DWM3000.convertToCM(tof);
        a->signal_strength = DWM3000.getSignalStrength();
        updateFilter(*a);
        stat_ok++;

        Serial.print("[DIST] A"); Serial.print(aid);
        Serial.print(": "); Serial.print(a->filtered_distance, 1);
        Serial.println("cm");

        DWM3000.forceIdle();
        DWM3000.clearSystemStatus();
        return true;
    }

    DWM3000.forceIdle();
    DWM3000.clearSystemStatus();
    return false;
}

// ==================== DWM3000 IMPLEMENTATIONS ====================

void DWM3000Class::spiSelect(uint8_t cs) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); delay(5); }
void DWM3000Class::begin() { delay(5); pinMode(CHIP_SELECT_PIN, OUTPUT); SPI.begin(); delay(5); spiSelect(CHIP_SELECT_PIN); }

void DWM3000Class::init() {
    if (!checkForDevID()) { Serial.println("[ERROR] Dev ID wrong!"); return; }
    setBitHigh(GEN_CFG_AES_LOW_REG, 0x10, 4);
    while (!checkForIDLE()) { Serial.println("[WARN] IDLE failed (1)"); delay(100); }
    softReset(); delay(200);
    while (!checkForIDLE()) { Serial.println("[WARN] IDLE failed (2)"); delay(100); }
    uint32_t ldo_low = readOTP(0x04); uint32_t ldo_high = readOTP(0x05);
    uint32_t bias_tune = (readOTP(0xA) >> 16) & BIAS_CTRL_BIAS_MASK;
    if (ldo_low && ldo_high && bias_tune) { write(0x11, 0x1F, bias_tune); write(0x0B, 0x08, 0x0100); }
    int xtrim = readOTP(0x1E); xtrim = xtrim == 0 ? 0x2E : xtrim;
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
    int otp_write = 0x1400; if (config[1] >= 256) otp_write |= 0x04;
    write(OTP_IF_REG, 0x08, otp_write);
    write(DRX_REG, 0x00, 0x00, 1); write(DRX_REG, 0x0, config[3]);
    write(STS_CFG_REG, 0x0, 64 / 8 - 1);
    write(GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1); write(DRX_REG, 0x0C, 0xAF5F584C);
    int chan_ctrl = read(GEN_CFG_AES_HIGH_REG, 0x14);
    chan_ctrl &= (~0x1FFF); chan_ctrl |= config[0];
    chan_ctrl |= 0x1F00 & (config[2] << 8); chan_ctrl |= 0xF8 & (config[2] << 3);
    chan_ctrl |= 0x06 & (0x01 << 1);
    write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl);
    int tx_fctrl = read(GEN_CFG_AES_LOW_REG, 0x24);
    tx_fctrl |= (config[1] << 12); tx_fctrl |= (config[4] << 10);
    write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl); write(DRX_REG, 0x02, 0x81);
    int rf_tx = 0x1C071134; int pll = 0x0F3C;
    if (config[0]) { rf_tx &= ~0x00FFFF; rf_tx |= 0x000001; pll &= 0x00FF; pll |= 0x001F; }
    write(RF_CONF_REG, 0x1C, rf_tx); write(FS_CTRL_REG, 0x00, pll);
    write(RF_CONF_REG, 0x51, 0x14); write(RF_CONF_REG, 0x1A, 0x0E);
    write(FS_CTRL_REG, 0x08, 0x81); write(GEN_CFG_AES_LOW_REG, 0x44, 0x02);
    write(PMSC_REG, 0x04, 0x300200); write(PMSC_REG, 0x08, 0x0138);
    int ok = 0;
    for (int i = 0; i < 100; i++) { if (read(GEN_CFG_AES_LOW_REG, 0x0) & 0x2) { ok = 1; break; } }
    if (!ok) Serial.println("[ERROR] PLL lock failed!");
    int otp_val = read(OTP_IF_REG, 0x08); otp_val |= 0x40;
    if (config[0]) otp_val |= 0x2000;
    write(OTP_IF_REG, 0x08, otp_val); write(RX_TUNE_REG, 0x19, 0xF0);
    int ldo_ctrl = read(RF_CONF_REG, 0x48);
    write(RF_CONF_REG, 0x48, 0x105 | 0x100 | 0x4 | 0x1);
    write(EXT_SYNC_REG, 0x0C, 0x020000); read(0x04, 0x0C); delay(20);
    write(EXT_SYNC_REG, 0x0C, 0x11);
    int succ = 0;
    for (int i = 0; i < 100; i++) { if (read(EXT_SYNC_REG, 0x20)) { succ = 1; break; } delay(10); }
    if (!succ) Serial.println("[ERROR] PGF cal failed!");
    write(EXT_SYNC_REG, 0x0C, 0x00); write(EXT_SYNC_REG, 0x20, 0x01);
    if (read(EXT_SYNC_REG, 0x14) == 0x1fffffff) Serial.println("[ERROR] PGF I fail!");
    if (read(EXT_SYNC_REG, 0x1C) == 0x1fffffff) Serial.println("[ERROR] PGF Q fail!");
    write(RF_CONF_REG, 0x48, ldo_ctrl); write(0x0E, 0x02, 0x01);
    setTXAntennaDelay(ANTENNA_DELAY);
}

void DWM3000Class::configureAsTX() { write(RF_CONF_REG, 0x1C, 0x34); write(GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD); }
void DWM3000Class::setupGPIO() { write(0x05, 0x08, 0xF0); }

void DWM3000Class::ds_sendFrame(int stage) {
    setMode(1);
    write(TX_BUFFER_REG, 0x01, sender & 0xFF);
    write(TX_BUFFER_REG, 0x02, destination & 0xFF);
    write(TX_BUFFER_REG, 0x03, stage & 0x7);
    setFrameLength(4);
    TXInstantRX();
    for (int i = 0; i < 50; i++) { if (sentFrameSucc()) return; }
    Serial.println("[ERROR] TX failed");
}

void DWM3000Class::ds_sendRTInfo(int trB, int trpB) {
    setMode(1);
    write(TX_BUFFER_REG, 0x01, destination & 0xFF);
    write(TX_BUFFER_REG, 0x02, sender & 0xFF);
    write(TX_BUFFER_REG, 0x03, STAGE_REPORT);
    write(TX_BUFFER_REG, 0x04, trB);
    write(TX_BUFFER_REG, 0x08, trpB);
    setFrameLength(12);
    TXInstantRX();
}

int DWM3000Class::ds_processRTInfo(int trA, int trpA, int trB, int trpB, int clk) {
    int reply_diff = trpA - trpB;
    long double co = trpA > trpB ? 1.0 + getClockOffset(clk) : 1.0 - getClockOffset(clk);
    return (trA - trpB + trB - trpA - (reply_diff - (reply_diff * co))) / 4;
}

int  DWM3000Class::ds_getStage()     { return read(RX_BUFFER_0_REG, 0x03) & 0b111; }
bool DWM3000Class::ds_isErrorFrame() { return ((read(RX_BUFFER_0_REG, 0x00) & 0x7) == 7); }
void DWM3000Class::setMode(int mode) { write(TX_BUFFER_REG, 0x00, mode & 0x7); }
void DWM3000Class::setFrameLength(int len) { len += FCS_LEN; int cfg = read(0x00, 0x24); write(GEN_CFG_AES_LOW_REG, 0x24, (cfg & 0xFFFFFC00) | len); }
void DWM3000Class::setTXAntennaDelay(int d) { ANTENNA_DELAY = d; write(0x01, 0x04, d); }
void DWM3000Class::setSenderID(int s) { sender = s; }
void DWM3000Class::setDestinationID(int d) { destination = d; }
int DWM3000Class::receivedFrameSucc() { int s = read(GEN_CFG_AES_LOW_REG, 0x44); if (s & SYS_STATUS_FRAME_RX_SUCC) return 1; if (s & SYS_STATUS_RX_ERR) return 2; return 0; }
int  DWM3000Class::sentFrameSucc()    { return (read(GEN_CFG_AES_LOW_REG, 0x44) & SYS_STATUS_FRAME_TX_SUCC) ? 1 : 0; }
int  DWM3000Class::getSenderID()      { return read(RX_BUFFER_0_REG, 0x01) & 0xFF; }
int  DWM3000Class::getDestinationID() { return read(RX_BUFFER_0_REG, 0x02) & 0xFF; }
bool DWM3000Class::checkForIDLE() { return ((read(0x0F, 0x30) >> 16) & PMSC_STATE_IDLE) == PMSC_STATE_IDLE || ((read(0x00, 0x44) >> 16) & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK); }
bool DWM3000Class::checkSPI() { return checkForDevID(); }
double DWM3000Class::getSignalStrength() { int cir = read(CIA_REG1, 0x2C) & 0x1FF; int pac = read(CIA_REG1, 0x58) & 0xFFF; unsigned int dgc = (read(RX_TUNE_REG, 0x60) >> 28) & 0x7; return 10 * log10((cir * (1 << 21)) / pow(pac, 2)) + (6 * dgc) - 121.7; }
int DWM3000Class::getRawClockOffset() { int raw = read(DRX_REG, 0x29) & 0x1FFFFF; if (raw & (1 << 20)) raw |= ~((1 << 21) - 1); return raw; }
long double DWM3000Class::getClockOffset(int32_t o) { return o * (config[0] == CHANNEL_5 ? CLOCK_OFFSET_CHAN_5_CONSTANT : CLOCK_OFFSET_CHAN_9_CONSTANT) / 1000000; }
unsigned long long DWM3000Class::readRXTimestamp() { uint32_t lo = read(CIA_REG1, 0x00); unsigned long long hi = read(CIA_REG1, 0x04) & 0xFF; return (hi << 32) | lo; }
unsigned long long DWM3000Class::readTXTimestamp() { unsigned long long lo = read(0x00, 0x74); unsigned long long hi = read(0x00, 0x78) & 0xFF; return (hi << 32) + lo; }
uint32_t DWM3000Class::write(int base, int sub, uint32_t data, int len) { return readOrWriteFullAddress(base, sub, data, len, 1); }
uint32_t DWM3000Class::write(int base, int sub, uint32_t data) { return readOrWriteFullAddress(base, sub, data, 0, 1); }
uint32_t DWM3000Class::read(int base, int sub) { return readOrWriteFullAddress(base, sub, 0, 0, 0); }
uint8_t  DWM3000Class::read8bit(int base, int sub) { return (uint8_t)(read(base, sub) >> 24); }
uint32_t DWM3000Class::readOTP(uint8_t addr) { write(OTP_IF_REG, 0x04, addr); write(OTP_IF_REG, 0x08, 0x02); return read(OTP_IF_REG, 0x10); }
void DWM3000Class::forceIdle()        { writeFastCommand(0x00); }
void DWM3000Class::standardTX()       { writeFastCommand(0x01); }
void DWM3000Class::standardRX()       { writeFastCommand(0x02); }
void DWM3000Class::TXInstantRX()      { writeFastCommand(0x0C); }
void DWM3000Class::softReset()        { clearAONConfig(); write(PMSC_REG, 0x04, 0x1); write(PMSC_REG, 0x00, 0x00, 2); delay(100); write(PMSC_REG, 0x00, 0xFFFF); write(PMSC_REG, 0x04, 0x00, 1); }
void DWM3000Class::hardReset()        { pinMode(RST_PIN, OUTPUT); digitalWrite(RST_PIN, LOW); delay(10); pinMode(RST_PIN, INPUT); }
void DWM3000Class::clearSystemStatus(){ write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF); }
double DWM3000Class::convertToCM(int u) { return (double)u * PS_UNIT * SPEED_OF_LIGHT; }
void DWM3000Class::setBit(int r, int s, int sh, bool b) { uint8_t t = read8bit(r, s); if (b) bitSet(t, sh); else bitClear(t, sh); write(r, s, t); }
void DWM3000Class::setBitHigh(int r, int s, int sh) { setBit(r, s, sh, 1); }
void DWM3000Class::writeFastCommand(int cmd) { int h = 0x1 | ((cmd & 0x1F) << 1) | 0x80; int arr[] = {h}; sendBytes(arr, 1, 0); }

uint32_t DWM3000Class::readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t dataLen, uint32_t rw) {
    uint32_t header = 0x00;
    if (rw) header |= 0x80;
    header |= ((base & 0x1F) << 1);
    if (sub > 0) { header |= 0x40; header <<= 8; header |= ((sub & 0x7F) << 2); }
    uint32_t hs = header > 0xFF ? 2 : 1;
    if (!rw) {
        int ha[hs]; if (hs == 1) ha[0] = header; else { ha[0] = (header >> 8); ha[1] = header & 0xFF; }
        return (uint32_t)sendBytes(ha, hs, 4);
    } else {
        uint32_t pb = 0;
        if (dataLen == 0) { if (data > 0) { uint32_t bits = countBits(data); pb = bits / 8; if (bits % 8) pb++; } else pb = 1; }
        else pb = dataLen;
        int payload[hs + pb];
        if (hs == 1) payload[0] = header; else { payload[0] = (header >> 8); payload[1] = header & 0xFF; }
        for (uint32_t i = 0; i < pb; i++) payload[hs + i] = (data >> i * 8) & 0xFF;
        return (uint32_t)sendBytes(payload, 2 + pb, 0);
    }
}

uint32_t DWM3000Class::sendBytes(int b[], int lenB, int recLen) {
    digitalWrite(CHIP_SELECT_PIN, LOW);
    for (int i = 0; i < lenB; i++) SPI.transfer(b[i]);
    uint32_t val = 0;
    if (recLen > 0) { for (int i = 0; i < recLen; i++) { uint32_t tmp = SPI.transfer(0x00); if (i == 0) val = tmp; else val |= tmp << (8 * i); } }
    digitalWrite(CHIP_SELECT_PIN, HIGH);
    return val;
}

void DWM3000Class::clearAONConfig() { write(AON_REG, NO_OFFSET, 0x00, 2); write(AON_REG, 0x14, 0x00, 1); write(AON_REG, 0x04, 0x00, 1); write(AON_REG, 0x04, 0x02); delay(1); }
unsigned int DWM3000Class::countBits(unsigned int n) { return (int)log2(n) + 1; }
int DWM3000Class::checkForDevID() { int res = read(GEN_CFG_AES_LOW_REG, NO_OFFSET); if (res != 0xDECA0302 && res != 0xDECA0312) { Serial.println("[ERROR] DEV_ID wrong!"); return 0; } return 1; }

// ==================== SETUP ====================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== UWB Tag (TDMA) ===");
    Serial.print("Tag ID: "); Serial.println(TAG_ID);
    my_slot = TAG_ID - FIRST_TAG_ID;
    Serial.print("Slot: "); Serial.print(my_slot); Serial.print("/"); Serial.println(NUM_TAGS);
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
    Serial.println("[OK] Tag ready\n");
}

// ==================== MAIN LOOP ====================

void loop() {
    waitForSlot();
    for (int a = 0; a < NUM_ANCHORS; a++) {
        if (slotExpired()) break;
        rangeWithAnchor(a);
        delay(INTER_RANGE_DELAY);
    }
    if (!slotExpired()) broadcastDistances();
    else Serial.println("[BCAST] Slot expired");
}