// ============================================================
//  UWB Spatial Tracking - TAG Firmware
//  Architecture: Tag-initiated DS-TWR with broadcast to listener
//
//  The tag sequentially ranges with each anchor, computes
//  distances, then broadcasts all distances in a single UWB
//  frame to the listener anchor. The listener outputs CSV
//  over serial for trilateration on a computer.
//
//  To scale: just change NUM_ANCHORS. Anchor IDs are
//  sequential starting from FIRST_ANCHOR_ID.
// ============================================================

#include <Arduino.h>
#include <SPI.h>

// ==================== CONFIGURATION ====================
// Change these to match your setup

#define TAG_ID              10
#define NUM_ANCHORS         1       // Number of anchors to range with (change to 3, 4, etc.)
#define FIRST_ANCHOR_ID     1       // Anchors are numbered 1, 2, 3, ...
#define LISTENER_ANCHOR_ID  1       // Which anchor receives the broadcast (typically anchor 1)

// Hardware pins
#define RST_PIN             27
#define CHIP_SELECT_PIN     4

// Ranging tuning
#define FILTER_SIZE         5       // Median filter window
#define MIN_DISTANCE        0.0     // cm
#define MAX_DISTANCE        2000.0  // cm
#define RX_TIMEOUT          5000    // Loop iterations before retry
#define INTER_RANGE_DELAY   5       // ms between ranging cycles

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
#define TRANSMIT_DELAY 0x3B9ACA00
#define TRANSMIT_DIFF 0x1FF
#define NO_OFFSET 0x0

// Register addresses
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
#define DIG_DIAG_REG 0xF
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define TX_BUFFER_REG 0x14

// Frame stages
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

static int current_anchor_index = 0;
static int curr_stage = 0;
static unsigned long rx_wait_count = 0;

// Stats
static unsigned long range_attempts = 0;
static unsigned long range_successes = 0;
static unsigned long range_timeouts = 0;
static unsigned long range_errors = 0;
static unsigned long bcast_successes = 0;

// ==================== ANCHOR DATA ====================

struct AnchorData {
    int anchor_id;
    int t_roundA = 0;
    int t_replyA = 0;
    long long rx = 0;
    long long tx = 0;
    int clock_offset = 0;
    float distance = 0;
    float distance_history[FILTER_SIZE] = {0};
    int history_index = 0;
    float filtered_distance = 0;
    float signal_strength = 0;
    float fp_signal_strength = 0;
};

AnchorData anchors[NUM_ANCHORS];

void initializeAnchors() {
    for (int i = 0; i < NUM_ANCHORS; i++)
        anchors[i].anchor_id = FIRST_ANCHOR_ID + i;
}

AnchorData *getCurrentAnchor()  { return &anchors[current_anchor_index]; }
int getCurrentAnchorId()        { return anchors[current_anchor_index].anchor_id; }
void advanceToNextAnchor()      { current_anchor_index = (current_anchor_index + 1) % NUM_ANCHORS; }

// ==================== FILTERING ====================

bool isValidDistance(float d) {
    return (d >= MIN_DISTANCE && d <= MAX_DISTANCE);
}

float calculateMedian(float arr[], int size) {
    float temp[size];
    for (int i = 0; i < size; i++) temp[i] = arr[i];
    for (int i = 0; i < size - 1; i++)
        for (int j = i + 1; j < size; j++)
            if (temp[j] < temp[i]) { float t = temp[i]; temp[i] = temp[j]; temp[j] = t; }
    return (size % 2 == 0) ? (temp[size/2 - 1] + temp[size/2]) / 2.0 : temp[size/2];
}

void updateFilteredDistance(AnchorData &data) {
    data.distance_history[data.history_index] = data.distance;
    data.history_index = (data.history_index + 1) % FILTER_SIZE;
    float valid[FILTER_SIZE];
    int count = 0;
    for (int i = 0; i < FILTER_SIZE; i++)
        if (isValidDistance(data.distance_history[i]))
            valid[count++] = data.distance_history[i];
    data.filtered_distance = (count > 0) ? calculateMedian(valid, count) : 0;
}

// ==================== DWM3000 DRIVER ====================

class DWM3000Class {
public:
    static void begin();
    static void init();
    static void writeSysConfig();
    static void configureAsTX();
    static void setupGPIO();

    // DS-TWR frame helpers
    static void ds_sendFrame(int stage);
    static void ds_sendRTInfo(int t_roundB, int t_replyB);
    static int  ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clock_offset);
    static int  ds_getStage();
    static bool ds_isErrorFrame();
    static void ds_sendErrorFrame();

    // Radio control
    static void setMode(int mode);
    static void setFrameLength(int frame_len);
    static void setTXAntennaDelay(int delay);
    static void setSenderID(int senderID);
    static void setDestinationID(int destID);
    static int  receivedFrameSucc();
    static int  sentFrameSucc();
    static int  getSenderID();
    static int  getDestinationID();
    static bool checkForIDLE();
    static bool checkSPI();

    // Diagnostics
    static double getSignalStrength();
    static double getFirstPathSignalStrength();
    static int    getTXAntennaDelay();
    static int    getRawClockOffset();
    static long double getClockOffset();
    static long double getClockOffset(int32_t ext);

    // Timestamps
    static unsigned long long readRXTimestamp();
    static unsigned long long readTXTimestamp();

    // SPI
    static uint32_t write(int base, int sub, uint32_t data, int data_len);
    static uint32_t write(int base, int sub, uint32_t data);
    static uint32_t read(int base, int sub);
    static uint8_t  read8bit(int base, int sub);
    static uint32_t readOTP(uint8_t addr);

    // Radio commands
    static void forceIdle();
    static void standardTX();
    static void standardRX();
    static void TXInstantRX();
    static void softReset();
    static void hardReset();
    static void clearSystemStatus();

    static double convertToCM(int ps_units);

private:
    static void spiSelect(uint8_t cs);
    static void setBit(int r, int s, int sh, bool b);
    static void setBitLow(int r, int s, int sh);
    static void setBitHigh(int r, int s, int sh);
    static void writeFastCommand(int cmd);
    static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t rw);
    static uint32_t sendBytes(int b[], int lenB, int recLen);
    static void clearAONConfig();
    static unsigned int countBits(unsigned int number);
    static int checkForDevID();
};

DWM3000Class DWM3000;

// ==================== BROADCAST ====================
// Sends all distances to the listener anchor in a single UWB frame.
// Frame layout (TX buffer offsets):
//   0x00: frame type
//   0x01: sender (tag ID)
//   0x02: destination (listener anchor ID)
//   0x03: stage = STAGE_BCAST (5)
//   0x04 + i*2: distance[i] as uint16 (cm), for i = 0..NUM_ANCHORS-1
//   0x04 + NUM_ANCHORS*2 + i*2: RSSI[i] as int16 (dBm*100)

void broadcastDistances() {
    // Force clean state
    DWM3000.clearSystemStatus();
    DWM3000.forceIdle();
    delayMicroseconds(50);

    DWM3000.setMode(1);
    DWM3000.write(TX_BUFFER_REG, 0x01, TAG_ID & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x02, LISTENER_ANCHOR_ID & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x03, STAGE_BCAST & 0x7);

    for (int i = 0; i < NUM_ANCHORS; i++) {
        uint16_t dist_cm = (uint16_t)(anchors[i].filtered_distance);
        // MUST use explicit 2-byte write. The default write() uses countBits()
        // which only writes 1 byte for values < 256, leaving stale data in
        // the second byte that the anchor reads as a huge number.
        DWM3000.write(TX_BUFFER_REG, 0x04 + (i * 2), dist_cm, 2);
    }

    for (int i = 0; i < NUM_ANCHORS; i++) {
        int16_t rssi_x100 = (int16_t)(anchors[i].signal_strength * 100);
        DWM3000.write(TX_BUFFER_REG, 0x04 + (NUM_ANCHORS * 2) + (i * 2), (uint16_t)rssi_x100, 2);
    }

    // Payload = 3 header bytes + NUM_ANCHORS * 4 bytes (2 dist + 2 rssi each)
    DWM3000.setFrameLength(3 + NUM_ANCHORS * 4);

    // Use TXInstantRX (not standardTX!) - standardTX leaves radio in a
    // dead state. TXInstantRX auto-transitions to RX after TX, same as
    // all the DS-TWR frames that work reliably.
    DWM3000.TXInstantRX();

    bool tx_ok = false;
    for (int i = 0; i < 200; i++) {
        if (DWM3000.sentFrameSucc()) { tx_ok = true; break; }
        delayMicroseconds(50);
    }

    if (tx_ok) bcast_successes++;

    // Force IDLE and clear for next ranging cycle
    DWM3000.forceIdle();
    delayMicroseconds(50);
    DWM3000.clearSystemStatus();
}

// ==================== DWM3000 IMPLEMENTATIONS ====================

void DWM3000Class::spiSelect(uint8_t cs) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); delay(5); }

void DWM3000Class::begin() {
    delay(5); pinMode(CHIP_SELECT_PIN, OUTPUT); SPI.begin(); delay(5);
    spiSelect(CHIP_SELECT_PIN);
}

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
    write(0x00, 0x3C, 0xFFFFFFFF); write(0x00, 0x40, 0xFFFF); write(0x0A, 0x00, 0x000900, 3);
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

int DWM3000Class::ds_processRTInfo(int t_rA, int t_rpA, int t_rB, int t_rpB, int clk) {
    int reply_diff = t_rpA - t_rpB;
    long double co = t_rpA > t_rpB ? 1.0 + getClockOffset(clk) : 1.0 - getClockOffset(clk);
    int first_rt = t_rA - t_rpB;
    int second_rt = t_rB - t_rpA;
    return (first_rt + second_rt - (reply_diff - (reply_diff * co))) / 4;
}

int  DWM3000Class::ds_getStage()     { return read(RX_BUFFER_0_REG, 0x03) & 0b111; }
bool DWM3000Class::ds_isErrorFrame() { return ((read(RX_BUFFER_0_REG, 0x00) & 0x7) == 7); }
void DWM3000Class::ds_sendErrorFrame() { setMode(7); setFrameLength(3); standardTX(); }

void DWM3000Class::setMode(int mode) { write(TX_BUFFER_REG, 0x00, mode & 0x7); }
void DWM3000Class::setFrameLength(int len) {
    len += FCS_LEN;
    int cfg = read(0x00, 0x24);
    write(GEN_CFG_AES_LOW_REG, 0x24, (cfg & 0xFFFFFC00) | len);
}
void DWM3000Class::setTXAntennaDelay(int d) { ANTENNA_DELAY = d; write(0x01, 0x04, d); }
void DWM3000Class::setSenderID(int s)       { sender = s; }
void DWM3000Class::setDestinationID(int d)  { destination = d; }

int DWM3000Class::receivedFrameSucc() {
    int s = read(GEN_CFG_AES_LOW_REG, 0x44);
    if (s & SYS_STATUS_FRAME_RX_SUCC) return 1;
    if (s & SYS_STATUS_RX_ERR) return 2;
    return 0;
}

int  DWM3000Class::sentFrameSucc()   { return (read(GEN_CFG_AES_LOW_REG, 0x44) & SYS_STATUS_FRAME_TX_SUCC) ? 1 : 0; }
int  DWM3000Class::getSenderID()     { return read(RX_BUFFER_0_REG, 0x01) & 0xFF; }
int  DWM3000Class::getDestinationID(){ return read(RX_BUFFER_0_REG, 0x02) & 0xFF; }
bool DWM3000Class::checkForIDLE() {
    return ((read(0x0F, 0x30) >> 16) & PMSC_STATE_IDLE) == PMSC_STATE_IDLE ||
           ((read(0x00, 0x44) >> 16) & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK);
}
bool DWM3000Class::checkSPI() { return checkForDevID(); }

double DWM3000Class::getSignalStrength() {
    int cir = read(CIA_REG1, 0x2C) & 0x1FF;
    int pac = read(CIA_REG1, 0x58) & 0xFFF;
    unsigned int dgc = (read(RX_TUNE_REG, 0x60) >> 28) & 0x7;
    return 10 * log10((cir * (1 << 21)) / pow(pac, 2)) + (6 * dgc) - 121.7;
}

double DWM3000Class::getFirstPathSignalStrength() {
    float f1 = (read(CIA_REG1, 0x30) & 0x3FFFFF) >> 2;
    float f2 = (read(CIA_REG1, 0x34) & 0x3FFFFF) >> 2;
    float f3 = (read(CIA_REG1, 0x38) & 0x3FFFFF) >> 2;
    int pac = read(CIA_REG1, 0x58) & 0xFFF;
    unsigned int dgc = (read(RX_TUNE_REG, 0x60) >> 28) & 0x7;
    return 10 * log10((f1*f1 + f2*f2 + f3*f3) / pow(pac, 2)) + (6 * dgc) - 121.7;
}

int DWM3000Class::getTXAntennaDelay() { return read(0x01, 0x04) & 0xFFFF; }
long double DWM3000Class::getClockOffset() { return getRawClockOffset() * (config[0] == CHANNEL_5 ? CLOCK_OFFSET_CHAN_5_CONSTANT : CLOCK_OFFSET_CHAN_9_CONSTANT) / 1000000; }
long double DWM3000Class::getClockOffset(int32_t o) { return o * (config[0] == CHANNEL_5 ? CLOCK_OFFSET_CHAN_5_CONSTANT : CLOCK_OFFSET_CHAN_9_CONSTANT) / 1000000; }
int DWM3000Class::getRawClockOffset() {
    int raw = read(DRX_REG, 0x29) & 0x1FFFFF;
    if (raw & (1 << 20)) raw |= ~((1 << 21) - 1);
    return raw;
}

unsigned long long DWM3000Class::readRXTimestamp() { uint32_t lo = read(CIA_REG1, 0x00); unsigned long long hi = read(CIA_REG1, 0x04) & 0xFF; return (hi << 32) | lo; }
unsigned long long DWM3000Class::readTXTimestamp() { unsigned long long lo = read(0x00, 0x74); unsigned long long hi = read(0x00, 0x78) & 0xFF; return (hi << 32) + lo; }
uint32_t DWM3000Class::write(int base, int sub, uint32_t data, int len) { return readOrWriteFullAddress(base, sub, data, len, 1); }
uint32_t DWM3000Class::write(int base, int sub, uint32_t data) { return readOrWriteFullAddress(base, sub, data, 0, 1); }
uint32_t DWM3000Class::read(int base, int sub) { return readOrWriteFullAddress(base, sub, 0, 0, 0); }
uint8_t  DWM3000Class::read8bit(int base, int sub) { return (uint8_t)(read(base, sub) >> 24); }
uint32_t DWM3000Class::readOTP(uint8_t addr) { write(OTP_IF_REG, 0x04, addr); write(OTP_IF_REG, 0x08, 0x02); return read(OTP_IF_REG, 0x10); }

void DWM3000Class::forceIdle()        { writeFastCommand(0x00); }  // TXRXOFF
void DWM3000Class::standardTX()       { writeFastCommand(0x01); }
void DWM3000Class::standardRX()       { writeFastCommand(0x02); }
void DWM3000Class::TXInstantRX()      { writeFastCommand(0x0C); }
void DWM3000Class::softReset()        { clearAONConfig(); write(PMSC_REG, 0x04, 0x1); write(PMSC_REG, 0x00, 0x00, 2); delay(100); write(PMSC_REG, 0x00, 0xFFFF); write(PMSC_REG, 0x04, 0x00, 1); }
void DWM3000Class::hardReset()        { pinMode(RST_PIN, OUTPUT); digitalWrite(RST_PIN, LOW); delay(10); pinMode(RST_PIN, INPUT); }
void DWM3000Class::clearSystemStatus(){ write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF); }
double DWM3000Class::convertToCM(int u) { return (double)u * PS_UNIT * SPEED_OF_LIGHT; }

void DWM3000Class::setBit(int r, int s, int sh, bool b) { uint8_t t = read8bit(r, s); if (b) bitSet(t, sh); else bitClear(t, sh); write(r, s, t); }
void DWM3000Class::setBitLow(int r, int s, int sh) { setBit(r, s, sh, 0); }
void DWM3000Class::setBitHigh(int r, int s, int sh) { setBit(r, s, sh, 1); }

void DWM3000Class::writeFastCommand(int cmd) {
    int h = 0x1 | ((cmd & 0x1F) << 1) | 0x80;
    int arr[] = {h}; sendBytes(arr, 1, 0);
}

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
int DWM3000Class::checkForDevID() {
    int res = read(GEN_CFG_AES_LOW_REG, NO_OFFSET);
    if (res != 0xDECA0302 && res != 0xDECA0312) { Serial.println("[ERROR] DEV_ID wrong!"); return 0; }
    return 1;
}

// ==================== SETUP ====================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== UWB Tag ===");
    Serial.print("Tag ID: "); Serial.println(TAG_ID);
    Serial.print("Anchors: "); Serial.println(NUM_ANCHORS);
    Serial.print("Listener: A"); Serial.println(LISTENER_ANCHOR_ID);

    initializeAnchors();
    DWM3000.begin(); DWM3000.hardReset(); delay(200);
    if (!DWM3000.checkSPI()) { Serial.println("[FATAL] SPI failed"); while (1); }
    while (!DWM3000.checkForIDLE()) { Serial.println("[ERROR] IDLE failed"); delay(1000); }
    DWM3000.softReset(); delay(200);
    if (!DWM3000.checkForIDLE()) { Serial.println("[FATAL] IDLE2 failed"); while (1); }

    DWM3000.init(); DWM3000.setupGPIO();
    DWM3000.setTXAntennaDelay(ANTENNA_DELAY);
    DWM3000.setSenderID(TAG_ID);
    DWM3000.configureAsTX();
    DWM3000.clearSystemStatus();

    Serial.println("[OK] Tag ready, starting ranging\n");
}

// ==================== MAIN LOOP ====================

void loop() {
    AnchorData *anchor = getCurrentAnchor();
    int anchorId = getCurrentAnchorId();

    switch (curr_stage) {

    // ---- Send poll (stage 1) ----
    case 0:
        DWM3000.clearSystemStatus();
        DWM3000.forceIdle();
        delayMicroseconds(50);

        range_attempts++;
        anchor->t_roundA = 0;
        anchor->t_replyA = 0;
        DWM3000.setDestinationID(anchorId);
        DWM3000.ds_sendFrame(STAGE_POLL);
        anchor->tx = DWM3000.readTXTimestamp();
        rx_wait_count = 0;
        curr_stage = 1;
        break;

    // ---- Wait for response (stage 2) ----
    case 1: {
        int rx = DWM3000.receivedFrameSucc();
        if (rx == 1) {
            DWM3000.clearSystemStatus();
            if (DWM3000.ds_isErrorFrame() || DWM3000.ds_getStage() != STAGE_RESP) {
                range_errors++;
                curr_stage = 0;
            } else {
                curr_stage = 2;
            }
        } else if (rx == 2) {
            DWM3000.clearSystemStatus();
            range_errors++;
            curr_stage = 0;
        } else {
            rx_wait_count++;
            if (rx_wait_count >= RX_TIMEOUT) {
                range_timeouts++;
                DWM3000.clearSystemStatus();
                DWM3000.forceIdle();
                curr_stage = 0;
            }
        }
        break;
    }

    // ---- Send final (stage 3) ----
    case 2:
        anchor->rx = DWM3000.readRXTimestamp();
        DWM3000.ds_sendFrame(STAGE_FINAL);
        anchor->t_roundA = anchor->rx - anchor->tx;
        anchor->tx = DWM3000.readTXTimestamp();
        anchor->t_replyA = anchor->tx - anchor->rx;
        rx_wait_count = 0;
        curr_stage = 3;
        break;

    // ---- Wait for report (stage 4) ----
    case 3: {
        int rx = DWM3000.receivedFrameSucc();
        if (rx == 1) {
            DWM3000.clearSystemStatus();
            if (DWM3000.ds_isErrorFrame()) {
                range_errors++;
                curr_stage = 0;
            } else {
                anchor->clock_offset = DWM3000.getRawClockOffset();
                curr_stage = 4;
            }
        } else if (rx == 2) {
            DWM3000.clearSystemStatus();
            range_errors++;
            curr_stage = 0;
        } else {
            rx_wait_count++;
            if (rx_wait_count >= RX_TIMEOUT) {
                range_timeouts++;
                DWM3000.clearSystemStatus();
                DWM3000.forceIdle();
                curr_stage = 0;
            }
        }
        break;
    }

    // ---- Compute distance, broadcast after last anchor ----
    case 4: {
        int tof = DWM3000.ds_processRTInfo(
            anchor->t_roundA, anchor->t_replyA,
            DWM3000.read(RX_BUFFER_0_REG, 0x04),
            DWM3000.read(RX_BUFFER_0_REG, 0x08),
            anchor->clock_offset);

        anchor->distance = DWM3000.convertToCM(tof);
        anchor->signal_strength = DWM3000.getSignalStrength();
        anchor->fp_signal_strength = DWM3000.getFirstPathSignalStrength();
        updateFilteredDistance(*anchor);
        range_successes++;

        // Print distance on tag serial (for debugging)
        Serial.print("[DIST] A");
        Serial.print(anchorId);
        Serial.print(": ");
        Serial.print(anchor->filtered_distance, 1);
        Serial.println("cm");

        // After ranging all anchors, broadcast distances to listener
        if (current_anchor_index == NUM_ANCHORS - 1) {
            broadcastDistances();
        }

        // Stats every 200 ranges
        if (range_successes % 200 == 0) {
            Serial.print("[STATS] ");
            Serial.print(range_successes); Serial.print("/");
            Serial.print(range_attempts); Serial.print(" ok, ");
            Serial.print(range_timeouts); Serial.print(" timeout, ");
            Serial.print(range_errors); Serial.print(" err, ");
            Serial.print(bcast_successes); Serial.println(" bcast");
        }

        advanceToNextAnchor();
        curr_stage = 0;
        delay(INTER_RANGE_DELAY);
        break;
    }

    default:
        curr_stage = 0;
        break;
    }
}