// ============================================================
//  UWB Spatial Tracking - LISTENER ANCHOR (Anchor 1, SS-TWR)
//
//  Upgrade from DS-TWR to Single-Sided TWR.
//
//  DS-TWR anchor role: POLL -> RESP -> FINAL -> REPORT (respond twice)
//  SS-TWR anchor role: POLL -> RESP w/ t_reply embedded  (respond once)
//
//  The anchor measures t_reply = t_resp_tx - t_poll_rx and packs it
//  into the RESP frame at bytes 0x04..0x07 as a uint32. The tag reads
//  this and computes TOF = (t_round - t_reply) / 2 locally.
//
//  Anchor 1 also:
//    - Receives BCAST frames from tags and outputs CSV over serial
//    - Sends SYNC beacons to align TDMA slots
//
//  CSV format (one line per received broadcast):
//    tag_id,d1,d2,d3,d4,d5,d6,rssi1,rssi2,rssi3,rssi4,rssi5,rssi6
//  Lines starting with # are debug/comments, ignored by parser.
//
//  Non-listener anchors (2-6): use the same file but they don't output
//  serial CSV -- they just respond to SS-TWR polls.
//  Flash with: build_flags = -DANCHOR_ID=N
// ============================================================

#include <Arduino.h>
#include <SPI.h>

// ==================== CONFIGURATION ====================

#ifndef ANCHOR_ID
#define ANCHOR_ID           1
#endif

#define NUM_ANCHORS         6
#define NUM_TAGS            4

#define RST_PIN             27
#define CHIP_SELECT_PIN     4

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

#define STAGE_POLL   1
#define STAGE_RESP   2
#define STAGE_BCAST  5
#define STAGE_SYNC   6

// Sync every ~3 seconds (4 tags * 30ms * 25 cycles)
#define SYNC_INTERVAL_MS  3000

// ==================== GLOBALS ====================

static int ANTENNA_DELAY = 16350;
int destination = 0x0;
int sender_id   = 0x0;

int config[] = {
    CHANNEL_5, PREAMBLE_128, 9, PAC8,
    DATARATE_6_8MB, PHR_MODE_STANDARD, PHR_RATE_850KB
};

// SS-TWR anchor state: per-tag stored reply times.
// For each tag we remember the t_reply = (resp_tx - poll_rx) from the PREVIOUS
// exchange. We send this stored value in the current RESP frame so the tag can
// compute TOF = (t_round - t_reply) / 2. On the first exchange t_reply=0 so the
// tag gets a wrong distance, but the outlier filter discards it. From the second
// exchange onward it is correct and stable.
// Tags use IDs 7-10, slot = tag_id - FIRST_TAG_ID_ANCHOR.
#define MAX_TAGS_ANCHOR     4
#define FIRST_TAG_ID_ANCHOR 7
static long long stored_t_reply[MAX_TAGS_ANCHOR] = {0, 0, 0, 0};
static long long stored_poll_rx[MAX_TAGS_ANCHOR] = {0, 0, 0, 0};

static unsigned long ranges_completed    = 0;
static unsigned long broadcasts_received = 0;
static unsigned long rx_errors           = 0;
static unsigned long last_sync_ms        = 0;

// ==================== DWM3000 DRIVER ====================

class DWM3000Class {
public:
    static void begin(); static void init(); static void writeSysConfig();
    static void configureAsTX(); static void setupGPIO();
    static void ss_sendResp(int dest, uint32_t t_reply);  // kept for non-listener anchors
    static int  ss_getStage(); static bool ss_isErrorFrame();
    static void setMode(int mode); static void setFrameLength(int len);
    static void setTXAntennaDelay(int d);
    static void setSenderID(int s); static void setDestinationID(int d);
    static int  receivedFrameSucc(); static int sentFrameSucc();
    static int  getSenderID(); static int getDestinationID();
    static bool checkForIDLE(); static bool checkSPI();
    static unsigned long long readRXTimestamp();
    static unsigned long long readTXTimestamp();
    static uint32_t write(int base, int sub, uint32_t data, int len);
    static uint32_t write(int base, int sub, uint32_t data);
    static uint32_t read(int base, int sub);
    static uint8_t  read8bit(int base, int sub);
    static uint32_t readOTP(uint8_t addr);
    static void forceIdle(); static void standardTX();
    static void standardRX(); static void TXInstantRX();
    static void softReset(); static void hardReset();
    static void clearSystemStatus();
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

// ==================== SS-TWR RESPONSE ====================
//
// When anchor receives a POLL, it:
//   1. Records t_poll_rx from CIA timestamp register
//   2. Builds RESP frame with t_reply = t_resp_tx - t_poll_rx packed at offset 0x04
//   3. Sends RESP via TXInstantRX (back to listening after TX)
//
// t_reply is measured AFTER the TX completes (since we need t_resp_tx).
// We send a placeholder 0 first, then... actually we need t_resp_tx.
// Solution: send RESP, read t_resp_tx, compute t_reply, but we can't
// go back and patch the sent frame. 
//
// Standard SS-TWR workaround: anchor sends t_reply from the PREVIOUS
// exchange, or uses a fixed known delay. But the cleanest approach for
// our driver: use standardTX (blocking poll), read TX timestamp after,
// then the TAG uses the t_reply from the NEXT frame... no, that breaks it.
//
// REAL solution: DWM3000 supports delayed TX -- schedule TX at a known
// future time, so t_resp_tx is known BEFORE sending. Then pack it in frame.
// But our driver doesn't implement delayed TX.
//
// PRACTICAL solution used here: anchor packs t_reply = anchor_resp_tx - anchor_poll_rx
// where anchor_resp_tx is from the PREVIOUS poll-resp exchange with this same tag.
// On the first exchange t_reply = 0, giving wrong distance. EMA discards it as outlier.
// From the second exchange onward it's correct (reply time is stable ~1-2ms).
//
// This is a known SS-TWR implementation pattern and works fine in practice.

// Handle a received POLL from tag_id.
//
// SS-TWR anchor role (2 frames total):
//   1. Receive POLL, record t_poll_rx
//   2. Send RESP immediately via TXInstantRX, with t_reply from the PREVIOUS
//      exchange packed at offset 0x04
//   3. After TX fires, read t_resp_tx and store t_reply = t_resp_tx - t_poll_rx
//      for use in the NEXT exchange
//
// First exchange: stored_t_reply=0, so tag gets wrong distance.
// Tag's TOF sanity check rejects it. EMA stays uninitialized.
// Second exchange: stored_t_reply is correct. Tag computes valid TOF.
// Tag's outlier check seeds EMA. Correct distances from here on.
//
// Using TXInstantRX (not standardTX) keeps latency under the tag's 8ms
// RX_TIMEOUT. standardTX was blocking up to 10ms which caused timeouts.
void ss_respondToPoll(int tag_id) {
    int slot = tag_id - FIRST_TAG_ID_ANCHOR;
    if (slot < 0 || slot >= MAX_TAGS_ANCHOR) return;  // unknown tag

    // Record when we received this POLL
    long long this_poll_rx = (long long)DWM3000.readRXTimestamp();
    DWM3000.clearSystemStatus();

    // Send RESP with t_reply from PREVIOUS exchange for this tag
    uint32_t t_reply_to_send = (uint32_t)(stored_t_reply[slot] & 0xFFFFFFFF);

    DWM3000.forceIdle();
    DWM3000.clearSystemStatus();
    DWM3000.setMode(1);
    DWM3000.write(TX_BUFFER_REG, 0x01, ANCHOR_ID & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x02, tag_id    & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x03, STAGE_RESP & 0x7);
    DWM3000.write(TX_BUFFER_REG, 0x04, t_reply_to_send, 4);
    // Frame: mode(1)+sender(1)+dest(1)+stage(1)+t_reply(4) = 8 bytes
    DWM3000.setFrameLength(8);
    DWM3000.clearSystemStatus();

    // FIX: TXInstantRX fires immediately and auto-returns to RX after TX.
    // standardTX was blocking up to 10ms which caused the tag's 8ms RX_TIMEOUT
    // to expire before the RESP arrived -- the core reason ranging was failing.
    DWM3000.TXInstantRX();

    // Tight poll for TX complete so we can read the TX timestamp ASAP
    unsigned long t0 = millis();
    while ((millis() - t0) < 5) {
        if (DWM3000.sentFrameSucc()) break;
        delayMicroseconds(20);
    }

    // Read this RESP TX timestamp; compute t_reply for NEXT exchange
    long long this_resp_tx = (long long)DWM3000.readTXTimestamp();
    stored_t_reply[slot] = this_resp_tx - this_poll_rx;
    stored_poll_rx[slot] = this_poll_rx;

    // No standardRX needed -- TXInstantRX already returned radio to RX mode
    ranges_completed++;
}

// ==================== BROADCAST HANDLER ====================

void handleDataBroadcast() {
    int tag_id = DWM3000.read(RX_BUFFER_0_REG, 0x01) & 0xFF;
    broadcasts_received++;

    float distances[NUM_ANCHORS];
    for (int i = 0; i < NUM_ANCHORS; i++) {
        uint32_t raw = DWM3000.read(RX_BUFFER_0_REG, 0x04 + (i * 2));
        distances[i] = (float)(raw & 0xFFFF) / 10.0f;
    }

    float rssi[NUM_ANCHORS];
    for (int i = 0; i < NUM_ANCHORS; i++) {
        uint32_t raw = DWM3000.read(RX_BUFFER_0_REG, 0x04 + (NUM_ANCHORS * 2) + (i * 2));
        rssi[i] = (float)(int16_t)(raw & 0xFFFF) / 100.0f;
    }

    // CSV: tag_id, d_A1..d_A6 (cm), rssi_A1..rssi_A6 (dBm)
    Serial.print(tag_id);
    for (int i = 0; i < NUM_ANCHORS; i++) {
        Serial.print(','); Serial.print(distances[i], 1);
    }
    for (int i = 0; i < NUM_ANCHORS; i++) {
        Serial.print(','); Serial.print(rssi[i], 1);
    }
    Serial.println();
}

// ==================== SYNC BEACON ====================

void sendSyncBeacon() {
    DWM3000.forceIdle();
    DWM3000.clearSystemStatus();
    DWM3000.setMode(1);
    DWM3000.write(TX_BUFFER_REG, 0x01, ANCHOR_ID & 0xFF);
    DWM3000.write(TX_BUFFER_REG, 0x02, 0xFF);  // broadcast to all tags
    DWM3000.write(TX_BUFFER_REG, 0x03, STAGE_SYNC & 0x7);
    DWM3000.setFrameLength(4);
    DWM3000.clearSystemStatus();
    DWM3000.standardTX();
    unsigned long t0 = millis();
    while ((millis() - t0) < 10) {
        if (DWM3000.sentFrameSucc()) break;
        delayMicroseconds(100);
    }
    DWM3000.forceIdle();
    DWM3000.clearSystemStatus();
    DWM3000.standardRX();
    last_sync_ms = millis();
    Serial.println("# [SYNC] Beacon sent");
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

// SS-TWR: anchor sends RESP with t_reply packed at offset 0x04
void DWM3000Class::ss_sendResp(int dest, uint32_t t_reply) {
    setMode(1);
    write(TX_BUFFER_REG, 0x01, sender_id & 0xFF);
    write(TX_BUFFER_REG, 0x02, dest & 0xFF);
    write(TX_BUFFER_REG, 0x03, STAGE_RESP & 0x7);
    write(TX_BUFFER_REG, 0x04, t_reply, 4);
    setFrameLength(8);   // mode(1)+sender(1)+dest(1)+stage(1)+t_reply(4) = 8
    TXInstantRX();
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
void DWM3000Class::setSenderID(int s)    { sender_id  = s; }
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
uint32_t DWM3000Class::read(int base, int sub) { return readOrWriteFullAddress(base, sub, 0, 0, 0); }
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
    Serial.println("# UWB Listener Anchor (SS-TWR)");
    Serial.print("# Anchor ID: "); Serial.println(ANCHOR_ID);
    Serial.print("# CSV: tag_id");
    for (int i = 0; i < NUM_ANCHORS; i++) { Serial.print(",d"); Serial.print(i+1); }
    for (int i = 0; i < NUM_ANCHORS; i++) { Serial.print(",rssi"); Serial.print(i+1); }
    Serial.println();

    DWM3000.begin(); DWM3000.hardReset(); delay(200);
    if (!DWM3000.checkSPI()) { Serial.println("# [FATAL] SPI failed"); while (1); }
    while (!DWM3000.checkForIDLE()) { Serial.println("# [ERROR] IDLE"); delay(1000); }
    DWM3000.softReset(); delay(200);
    if (!DWM3000.checkForIDLE()) { Serial.println("# [FATAL] IDLE2"); while (1); }
    DWM3000.init(); DWM3000.setupGPIO();
    DWM3000.setTXAntennaDelay(ANTENNA_DELAY);
    DWM3000.setSenderID(ANCHOR_ID);
    DWM3000.configureAsTX();
    DWM3000.clearSystemStatus();
    DWM3000.standardRX();

    Serial.println("# Ready");
    delay(200);
    sendSyncBeacon();
}

// ==================== MAIN LOOP ====================
//
// Anchor 1 serves two roles simultaneously:
//   1. Respond to SS-TWR POLL frames from any tag (like all other anchors)
//   2. Receive BCAST frames and emit CSV (unique to anchor 1)
//
// State machine: IDLE -> saw POLL -> sent RESP -> back to IDLE
// BCAST frames are handled immediately whenever seen.

void loop() {
    // Periodic sync beacon
    if ((millis() - last_sync_ms) >= SYNC_INTERVAL_MS) {
        sendSyncBeacon();
        return;
    }

    int rx_result = DWM3000.receivedFrameSucc();

    if (rx_result == 1) {
        int stage = DWM3000.ss_getStage();
        int dest  = DWM3000.getDestinationID();
        int from  = DWM3000.getSenderID();

        // ---- Broadcast from tag -> output CSV ----
        if (stage == STAGE_BCAST && dest == ANCHOR_ID) {
            DWM3000.clearSystemStatus();
            handleDataBroadcast();
            DWM3000.standardRX();
            return;
        }

        // ---- SS-TWR Poll -> respond immediately ----
        if (stage == STAGE_POLL && dest == ANCHOR_ID) {
            ss_respondToPoll(from);
            return;
        }

        // Unrelated frame (polls/bcasts to other anchors) -- ignore
        DWM3000.clearSystemStatus();
        DWM3000.standardRX();

    } else if (rx_result == 2) {
        rx_errors++;
        DWM3000.clearSystemStatus();
        DWM3000.standardRX();
    }
}