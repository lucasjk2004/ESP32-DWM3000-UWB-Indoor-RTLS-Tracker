#include <Arduino.h>
#include <SPI.h>

// SPI Setup
#define RST_PIN 27
#define CHIP_SELECT_PIN 4

// This anchor's ID - must match what the tag expects
#define ANCHOR_ID 1
#define RESPONSE_TIMEOUT_MS 10
#define MAX_RETRIES 3

// Number of anchors in the system (for parsing broadcast data)
#define NUM_ANCHORS 3
#define FIRST_ANCHOR_ID 1

// Data broadcast stage marker (must match tag firmware)
#define STAGE_DATA_BROADCAST 5

static int rx_status;
static int tx_status;
static int curr_stage = 0;
static int t_roundB = 0;
static int t_replyB = 0;
static long long rx = 0;
static long long tx = 0;
int retry_count = 0;
unsigned long last_ranging_time = 0;

// UWB Configuration (must match tag)
#define LEN_RX_CAL_CONF 4
#define LEN_TX_FCTRL_CONF 6
#define LEN_AON_DIG_CFG_CONF 3
#define PMSC_STATE_IDLE 0x3
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
#define DATARATE_850KB 0x0
#define PHR_MODE_STANDARD 0x0
#define PHR_RATE_850KB 0x0
#define SPIRDY_MASK 0x80
#define RCINIT_MASK 0x100
#define BIAS_CTRL_BIAS_MASK 0x1F
#define GEN_CFG_AES_LOW_REG 0x00
#define GEN_CFG_AES_HIGH_REG 0x01
#define STS_CFG_REG 0x2
#define RX_TUNE_REG 0x3
#define EXT_SYNC_REG 0x4
#define GPIO_CTRL_REG 0x5
#define DRX_REG 0x6
#define RF_CONF_REG 0x7
#define RF_CAL_REG 0x8
#define FS_CTRL_REG 0x9
#define AON_REG 0xA
#define OTP_IF_REG 0xB
#define CIA_REG1 0xC
#define CIA_REG2 0xD
#define CIA_REG3 0xE
#define DIG_DIAG_REG 0xF
#define PMSC_REG 0x11
#define RX_BUFFER_0_REG 0x12
#define RX_BUFFER_1_REG 0x13
#define TX_BUFFER_REG 0x14
#define ACC_MEM_REG 0x15
#define SCRATCH_RAM_REG 0x16
#define AES_RAM_REG 0x17
#define SET_1_2_REG 0x18
#define INDIRECT_PTR_A_REG 0x1D
#define INDIRECT_PTR_B_REG 0x1E
#define IN_PTR_CFG_REG 0x1F
#define TRANSMIT_DELAY 0x3B9ACA00
#define TRANSMIT_DIFF 0x1FF
#define NS_UNIT 4.0064102564102564
#define PS_UNIT 15.6500400641025641
#define SPEED_OF_LIGHT 0.029979245800
#define CLOCK_OFFSET_CHAN_5_CONSTANT -0.5731e-3f
#define CLOCK_OFFSET_CHAN_9_CONSTANT -0.1252e-3f
#define NO_OFFSET 0x0
#define DEBUG_OUTPUT 0

static int ANTENNA_DELAY = 16350;
int led_status = 0;
int destination = 0x0;
int sender_id = 0x0;

int config[] = {
    CHANNEL_5, PREAMBLE_128, 9, PAC8,
    DATARATE_6_8MB, PHR_MODE_STANDARD, PHR_RATE_850KB
};

// ============== DWM3000 Driver Class ==============

class DWM3000Class
{
public:
    static void spiSelect(uint8_t cs);
    static void begin();
    static void init();
    static void writeSysConfig();
    static void configureAsTX();
    static void setupGPIO();
    static void ds_sendFrame(int stage);
    static void ds_sendRTInfo(int t_roundB, int t_replyB);
    static int ds_processRTInfo(int t_roundA, int t_replyA, int t_roundB, int t_replyB, int clock_offset);
    static int ds_getStage();
    static bool ds_isErrorFrame();
    static void ds_sendErrorFrame();
    static void setMode(int mode);
    static void setFrameLength(int frame_len);
    static void setTXAntennaDelay(int delay);
    static void setSenderID(int senderID);
    static void setDestinationID(int destID);
    static int receivedFrameSucc();
    static int sentFrameSucc();
    static int getSenderID();
    static int getDestinationID();
    static bool checkForIDLE();
    static bool checkSPI();
    static int getTXAntennaDelay();
    static long double getClockOffset();
    static long double getClockOffset(int32_t ext_clock_offset);
    static int getRawClockOffset();
    static unsigned long long readRXTimestamp();
    static unsigned long long readTXTimestamp();
    static uint32_t write(int base, int sub, uint32_t data, int data_len);
    static uint32_t write(int base, int sub, uint32_t data);
    static uint32_t read(int base, int sub);
    static uint8_t read8bit(int base, int sub);
    static uint32_t readOTP(uint8_t addr);
    static void standardTX();
    static void standardRX();
    static void TXInstantRX();
    static void softReset();
    static void hardReset();
    static void clearSystemStatus();
    static double convertToCM(int DWM3000_ps_units);
    static void printDouble(double val, unsigned int precision, bool linebreak);
private:
    static void setBit(int reg_addr, int sub_addr, int shift, bool b);
    static void setBitLow(int reg_addr, int sub_addr, int shift);
    static void setBitHigh(int reg_addr, int sub_addr, int shift);
    static void writeFastCommand(int cmd);
    static uint32_t readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t data_len, uint32_t readWriteBit);
    static uint32_t sendBytes(int b[], int lenB, int recLen);
    static void clearAONConfig();
    static unsigned int countBits(unsigned int number);
    static int checkForDevID();
};

DWM3000Class DWM3000;

// ============== Handle distance broadcast from tag ==============

void handleDataBroadcast()
{
    // Read the packed distance and RSSI data from the RX buffer
    // Format matches what the tag wrote:
    //   Bytes 4-5:  A1 distance (uint16, cm)
    //   Bytes 6-7:  A2 distance (uint16, cm)
    //   Bytes 8-9:  A3 distance (uint16, cm)
    //   Bytes 10-11: A1 RSSI (int16, dBm*100)
    //   Bytes 12-13: A2 RSSI (int16, dBm*100)
    //   Bytes 14-15: A3 RSSI (int16, dBm*100)

    int tag_id = DWM3000.read(0x12, 0x01) & 0xFF;

    // Build JSON and print to serial
    Serial.print("{\"tag_id\":");
    Serial.print(tag_id);
    Serial.print(",\"anchors\":{");

    for (int i = 0; i < NUM_ANCHORS; i++)
    {
        if (i > 0) Serial.print(",");

        // Read distance (uint16 at offset 0x04 + i*2)
        uint32_t raw_dist = DWM3000.read(0x12, 0x04 + (i * 2));
        uint16_t dist_cm = raw_dist & 0xFFFF;

        // Read RSSI (int16 at offset 0x04 + NUM_ANCHORS*2 + i*2)
        uint32_t raw_rssi = DWM3000.read(0x12, 0x04 + (NUM_ANCHORS * 2) + (i * 2));
        int16_t rssi_x100 = (int16_t)(raw_rssi & 0xFFFF);
        float rssi_dbm = rssi_x100 / 100.0;

        Serial.print("\"A");
        Serial.print(FIRST_ANCHOR_ID + i);
        Serial.print("\":{\"distance\":");
        Serial.print(dist_cm);
        Serial.print(",\"rssi\":");
        Serial.print(rssi_dbm, 2);
        Serial.print("}");
    }

    Serial.println("}}");
}

// ============== DWM3000 implementations (anchor-side) ==============

void DWM3000Class::spiSelect(uint8_t cs) { pinMode(cs, OUTPUT); digitalWrite(cs, HIGH); delay(5); }

void DWM3000Class::begin()
{
    delay(5); pinMode(CHIP_SELECT_PIN, OUTPUT); SPI.begin(); delay(5);
    spiSelect(CHIP_SELECT_PIN); Serial.println("[INFO] SPI ready");
}

void DWM3000Class::init()
{
    if (!checkForDevID()) { Serial.println("[ERROR] Dev ID wrong!"); return; }
    setBitHigh(GEN_CFG_AES_LOW_REG, 0x10, 4);
    while (!checkForIDLE()) { Serial.println("[WARNING] IDLE FAILED (1)"); delay(100); }
    softReset(); delay(200);
    while (!checkForIDLE()) { Serial.println("[WARNING] IDLE FAILED (2)"); delay(100); }
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
    Serial.println("[INFO] Init finished.\n");
}

void DWM3000Class::writeSysConfig()
{
    int usr_cfg = (STDRD_SYS_CONFIG & 0xFFF) | (config[5] << 3) | (config[6] << 4);
    write(GEN_CFG_AES_LOW_REG, 0x10, usr_cfg);
    int otp_write = 0x1400; if (config[1] >= 256) otp_write |= 0x04;
    write(OTP_IF_REG, 0x08, otp_write);
    write(DRX_REG, 0x00, 0x00, 1); write(DRX_REG, 0x0, config[3]);
    write(STS_CFG_REG, 0x0, 64 / 8 - 1);
    write(GEN_CFG_AES_LOW_REG, 0x29, 0x00, 1); write(DRX_REG, 0x0C, 0xAF5F584C);
    int chan_ctrl_val = read(GEN_CFG_AES_HIGH_REG, 0x14);
    chan_ctrl_val &= (~0x1FFF); chan_ctrl_val |= config[0];
    chan_ctrl_val |= 0x1F00 & (config[2] << 8); chan_ctrl_val |= 0xF8 & (config[2] << 3);
    chan_ctrl_val |= 0x06 & (0x01 << 1);
    write(GEN_CFG_AES_HIGH_REG, 0x14, chan_ctrl_val);
    int tx_fctrl_val = read(GEN_CFG_AES_LOW_REG, 0x24);
    tx_fctrl_val |= (config[1] << 12); tx_fctrl_val |= (config[4] << 10);
    write(GEN_CFG_AES_LOW_REG, 0x24, tx_fctrl_val); write(DRX_REG, 0x02, 0x81);
    int rf_tx_ctrl_2 = 0x1C071134; int pll_conf = 0x0F3C;
    if (config[0]) { rf_tx_ctrl_2 &= ~0x00FFFF; rf_tx_ctrl_2 |= 0x000001; pll_conf &= 0x00FF; pll_conf |= 0x001F; }
    write(RF_CONF_REG, 0x1C, rf_tx_ctrl_2); write(FS_CTRL_REG, 0x00, pll_conf);
    write(RF_CONF_REG, 0x51, 0x14); write(RF_CONF_REG, 0x1A, 0x0E);
    write(FS_CTRL_REG, 0x08, 0x81); write(GEN_CFG_AES_LOW_REG, 0x44, 0x02);
    write(PMSC_REG, 0x04, 0x300200); write(PMSC_REG, 0x08, 0x0138);
    int success = 0;
    for (int i = 0; i < 100; i++) { if (read(GEN_CFG_AES_LOW_REG, 0x0) & 0x2) { success = 1; break; } }
    if (!success) Serial.println("[ERROR] PLL lock failed!");
    else Serial.println("[INFO] PLL locked.");
    int otp_val = read(OTP_IF_REG, 0x08); otp_val |= 0x40;
    if (config[0]) otp_val |= 0x2000;
    write(OTP_IF_REG, 0x08, otp_val); write(RX_TUNE_REG, 0x19, 0xF0);
    int ldo_ctrl_val = read(RF_CONF_REG, 0x48);
    write(RF_CONF_REG, 0x48, 0x105 | 0x100 | 0x4 | 0x1);
    write(EXT_SYNC_REG, 0x0C, 0x020000); read(0x04, 0x0C); delay(20);
    write(EXT_SYNC_REG, 0x0C, 0x11);
    int succ = 0;
    for (int i = 0; i < 100; i++) { if (read(EXT_SYNC_REG, 0x20)) { succ = 1; break; } delay(10); }
    if (succ) Serial.println("[INFO] PGF cal complete.");
    else Serial.println("[ERROR] PGF cal failed!");
    write(EXT_SYNC_REG, 0x0C, 0x00); write(EXT_SYNC_REG, 0x20, 0x01);
    if (read(EXT_SYNC_REG, 0x14) == 0x1fffffff) Serial.println("[ERROR] PGF_CAL stage I fail!");
    if (read(EXT_SYNC_REG, 0x1C) == 0x1fffffff) Serial.println("[ERROR] PGF_CAL stage Q fail!");
    write(RF_CONF_REG, 0x48, ldo_ctrl_val); write(0x0E, 0x02, 0x01);
    setTXAntennaDelay(ANTENNA_DELAY);
}

void DWM3000Class::configureAsTX() { write(RF_CONF_REG, 0x1C, 0x34); write(GEN_CFG_AES_HIGH_REG, 0x0C, 0xFDFDFDFD); }
void DWM3000Class::setupGPIO() { write(0x05, 0x08, 0xF0); }

void DWM3000Class::ds_sendFrame(int stage)
{
    setMode(1); write(0x14, 0x01, sender_id & 0xFF); write(0x14, 0x02, destination & 0xFF);
    write(0x14, 0x03, stage & 0x7); setFrameLength(4); TXInstantRX();
    for (int i = 0; i < 50; i++) { if (sentFrameSucc()) return; }
    Serial.println("[ERROR] TX failed!");
}

void DWM3000Class::ds_sendRTInfo(int t_roundB, int t_replyB)
{
    setMode(1); write(0x14, 0x01, destination & 0xFF); write(0x14, 0x02, sender_id & 0xFF);
    write(0x14, 0x03, 4); write(0x14, 0x04, t_roundB); write(0x14, 0x08, t_replyB);
    setFrameLength(12); TXInstantRX();
}

int DWM3000Class::ds_getStage() { return read(0x12, 0x03) & 0b111; }
bool DWM3000Class::ds_isErrorFrame() { return ((read(0x12, 0x00) & 0x7) == 7); }
void DWM3000Class::ds_sendErrorFrame() { setMode(7); setFrameLength(3); standardTX(); }
void DWM3000Class::setMode(int mode) { write(0x14, 0x00, mode & 0x7); }

void DWM3000Class::setFrameLength(int frameLen)
{
    frameLen += FCS_LEN; int curr_cfg = read(0x00, 0x24);
    write(GEN_CFG_AES_LOW_REG, 0x24, (curr_cfg & 0xFFFFFC00) | frameLen);
}

void DWM3000Class::setTXAntennaDelay(int delay) { ANTENNA_DELAY = delay; write(0x01, 0x04, delay); }
void DWM3000Class::setSenderID(int s) { sender_id = s; }
void DWM3000Class::setDestinationID(int d) { destination = d; }

int DWM3000Class::receivedFrameSucc()
{
    int s = read(GEN_CFG_AES_LOW_REG, 0x44);
    if ((s & SYS_STATUS_FRAME_RX_SUCC) > 0) return 1;
    else if ((s & SYS_STATUS_RX_ERR) > 0) return 2;
    return 0;
}

int DWM3000Class::sentFrameSucc() { return ((read(GEN_CFG_AES_LOW_REG, 0x44) & SYS_STATUS_FRAME_TX_SUCC) == SYS_STATUS_FRAME_TX_SUCC) ? 1 : 0; }
int DWM3000Class::getSenderID() { return read(0x12, 0x01) & 0xFF; }
int DWM3000Class::getDestinationID() { return read(0x12, 0x02) & 0xFF; }

bool DWM3000Class::checkForIDLE()
{
    return (read(0x0F, 0x30) >> 16 & PMSC_STATE_IDLE) == PMSC_STATE_IDLE ||
           (read(0x00, 0x44) >> 16 & (SPIRDY_MASK | RCINIT_MASK)) == (SPIRDY_MASK | RCINIT_MASK) ? 1 : 0;
}

bool DWM3000Class::checkSPI() { return checkForDevID(); }
int DWM3000Class::getTXAntennaDelay() { return read(0x01, 0x04) & 0xFFFF; }
long double DWM3000Class::getClockOffset() { return getRawClockOffset() * (config[0] == CHANNEL_5 ? CLOCK_OFFSET_CHAN_5_CONSTANT : CLOCK_OFFSET_CHAN_9_CONSTANT) / 1000000; }
long double DWM3000Class::getClockOffset(int32_t o) { return o * (config[0] == CHANNEL_5 ? CLOCK_OFFSET_CHAN_5_CONSTANT : CLOCK_OFFSET_CHAN_9_CONSTANT) / 1000000; }

int DWM3000Class::getRawClockOffset()
{
    int raw = read(0x06, 0x29) & 0x1FFFFF;
    if (raw & (1 << 20)) raw |= ~((1 << 21) - 1);
    return raw;
}

unsigned long long DWM3000Class::readRXTimestamp() { uint32_t lo = read(0x0C, 0x00); unsigned long long hi = read(0x0C, 0x04) & 0xFF; return (hi << 32) | lo; }
unsigned long long DWM3000Class::readTXTimestamp() { unsigned long long lo = read(0x00, 0x74); unsigned long long hi = read(0x00, 0x78) & 0xFF; return (hi << 32) + lo; }
uint32_t DWM3000Class::write(int base, int sub, uint32_t data, int dataLen) { return readOrWriteFullAddress(base, sub, data, dataLen, 1); }
uint32_t DWM3000Class::write(int base, int sub, uint32_t data) { return readOrWriteFullAddress(base, sub, data, 0, 1); }
uint32_t DWM3000Class::read(int base, int sub) { return readOrWriteFullAddress(base, sub, 0, 0, 0); }
uint8_t DWM3000Class::read8bit(int base, int sub) { return (uint8_t)(read(base, sub) >> 24); }
uint32_t DWM3000Class::readOTP(uint8_t addr) { write(OTP_IF_REG, 0x04, addr); write(OTP_IF_REG, 0x08, 0x02); return read(OTP_IF_REG, 0x10); }
void DWM3000Class::standardTX() { writeFastCommand(0x01); }
void DWM3000Class::standardRX() { writeFastCommand(0x02); }
void DWM3000Class::TXInstantRX() { writeFastCommand(0x0C); }
void DWM3000Class::softReset() { clearAONConfig(); write(PMSC_REG, 0x04, 0x1); write(PMSC_REG, 0x00, 0x00, 2); delay(100); write(PMSC_REG, 0x00, 0xFFFF); write(PMSC_REG, 0x04, 0x00, 1); }
void DWM3000Class::hardReset() { pinMode(RST_PIN, OUTPUT); digitalWrite(RST_PIN, LOW); delay(10); pinMode(RST_PIN, INPUT); }
void DWM3000Class::clearSystemStatus() { write(GEN_CFG_AES_LOW_REG, 0x44, 0x3F7FFFFF); }
double DWM3000Class::convertToCM(int u) { return (double)u * PS_UNIT * SPEED_OF_LIGHT; }

void DWM3000Class::printDouble(double val, unsigned int precision, bool linebreak)
{
    Serial.print(int(val)); Serial.print(".");
    unsigned int frac = val >= 0 ? (val - int(val)) * precision : (int(val) - val) * precision;
    if (linebreak) Serial.println(frac, DEC); else Serial.print(frac, DEC);
}

void DWM3000Class::setBit(int r, int s, int sh, bool b) { uint8_t t = read8bit(r, s); if (b) bitSet(t, sh); else bitClear(t, sh); write(r, s, t); }
void DWM3000Class::setBitLow(int r, int s, int sh) { setBit(r, s, sh, 0); }
void DWM3000Class::setBitHigh(int r, int s, int sh) { setBit(r, s, sh, 1); }
void DWM3000Class::writeFastCommand(int cmd) { int h = 0x1 | ((cmd & 0x1F) << 1) | 0x80; int a[] = {h}; sendBytes(a, 1, 0); }

uint32_t DWM3000Class::readOrWriteFullAddress(uint32_t base, uint32_t sub, uint32_t data, uint32_t dataLen, uint32_t rw)
{
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

uint32_t DWM3000Class::sendBytes(int b[], int lenB, int recLen)
{
    digitalWrite(CHIP_SELECT_PIN, LOW);
    for (int i = 0; i < lenB; i++) SPI.transfer(b[i]);
    uint32_t val = 0;
    if (recLen > 0) { for (int i = 0; i < recLen; i++) { uint32_t tmp = SPI.transfer(0x00); if (i == 0) val = tmp; else val |= tmp << (8 * i); } }
    digitalWrite(CHIP_SELECT_PIN, HIGH);
    return val;
}

void DWM3000Class::clearAONConfig() { write(AON_REG, NO_OFFSET, 0x00, 2); write(AON_REG, 0x14, 0x00, 1); write(AON_REG, 0x04, 0x00, 1); write(AON_REG, 0x04, 0x02); delay(1); }
unsigned int DWM3000Class::countBits(unsigned int n) { return (int)log2(n) + 1; }

int DWM3000Class::checkForDevID()
{
    int res = read(GEN_CFG_AES_LOW_REG, NO_OFFSET);
    if (res != 0xDECA0302 && res != 0xDECA0312) { Serial.println("[ERROR] DEV_ID WRONG!"); return 0; }
    return 1;
}

// ============== setup() ==============

void setup()
{
    Serial.begin(115200);
    Serial.println("[INFO] Anchor 1 - Listener Mode");

    DWM3000.begin(); DWM3000.hardReset(); delay(200);
    if (!DWM3000.checkSPI()) { Serial.println("[ERROR] SPI failed!"); while (1); }
    while (!DWM3000.checkForIDLE()) { Serial.println("[ERROR] IDLE1"); delay(1000); }
    DWM3000.softReset(); delay(200);
    if (!DWM3000.checkForIDLE()) { Serial.println("[ERROR] IDLE2"); while (1); }

    DWM3000.init(); DWM3000.setupGPIO();
    DWM3000.setTXAntennaDelay(16350);
    DWM3000.setSenderID(ANCHOR_ID);

    Serial.println("> ANCHOR 1 - Ranging Responder + Data Listener <");
    Serial.print("Antenna delay: "); Serial.println(DWM3000.getTXAntennaDelay());

    DWM3000.configureAsTX();
    DWM3000.clearSystemStatus();
    DWM3000.standardRX();  // Start listening
}

// ============== loop() ==============

void loop()
{
    // Check for any received frame
    if (DWM3000.receivedFrameSucc() == 1)
    {
        int recv_stage = DWM3000.ds_getStage();
        int recv_dest = DWM3000.getDestinationID();

        // === Distance data broadcast from tag (stage 5) ===
        if (recv_stage == STAGE_DATA_BROADCAST && recv_dest == ANCHOR_ID)
        {
            DWM3000.clearSystemStatus();
            handleDataBroadcast();
            DWM3000.standardRX();  // Back to listening
            curr_stage = 0;
            return;
        }

        // === Normal ranging: stage 1 poll from tag addressed to us ===
        if (recv_stage == 1 && recv_dest == ANCHOR_ID)
        {
            DWM3000.clearSystemStatus();

            if (curr_stage != 0)
            {
                // New ranging request resets session
                curr_stage = 0;
                t_roundB = 0;
                t_replyB = 0;
            }

            // Stage 0 -> 1: received poll, send response
            rx = DWM3000.readRXTimestamp();
            DWM3000.ds_sendFrame(2);  // Send response (stage 2)
            tx = DWM3000.readTXTimestamp();
            t_replyB = tx - rx;
            curr_stage = 2;  // Wait for final
            return;
        }

        // === Stage 3: final from tag ===
        if (recv_stage == 3 && recv_dest == ANCHOR_ID && curr_stage == 2)
        {
            DWM3000.clearSystemStatus();
            rx = DWM3000.readRXTimestamp();
            t_roundB = rx - tx;
            DWM3000.ds_sendRTInfo(t_roundB, t_replyB);  // Send report (stage 4)
            curr_stage = 0;  // Done, back to idle
            DWM3000.standardRX();  // Resume listening
            return;
        }

        // Unrelated frame, ignore
        DWM3000.clearSystemStatus();
        DWM3000.standardRX();
    }
    else if (DWM3000.receivedFrameSucc() == 2)
    {
        // RX error
        DWM3000.clearSystemStatus();
        DWM3000.standardRX();
    }
}