/*
 * ============================================================
 * QUAD ESP32-WROOM RF ANTENNA TESTING SYSTEM
 *
 * TEST NODE MASTER FIRMWARE
 * ============================================================
 *
 * Hardware Role:
 *   - Test antenna radio under evaluation
 *   - Controller for Test Node Slave (Test Reference Antenna) via Serial1
 *   - ESP-NOW slave/responder to Reference Node Master
 *
 * Flow:
 *   1. Stay on CONTROL_CHANNEL (Ch 1)
 *   2. Respond to ESP-NOW CMD_SYNC_REQ with high precision timestamps (t1, t2, t3)
 *   3. On CMD_PREPARE:
 *      - Pass PREPARE over Serial1 to local Test Node Slave
 *      - Send CMD_READY back to Reference Node Master over ESP-NOW
 *   4. On CMD_START_AT:
 *      - Convert startAtUs for local Slave using Serial1 clock offset and send LOCAL_START_AT
 *      - Switch to target RF channel
 *      - Execute RF window:
 *        * If phase == PHASE_TEST_A_TX: transmit raw 802.11 action frames
 *        * Else: receive promiscuously & collect CSI / packet stats
 *      - NO ESP-NOW OR SERIAL1 MESSAGES DURING RF WINDOW
 *      - Return to CONTROL_CHANNEL (Ch 1) after guard interval
 *      - Poll Serial1 for Test Node Slave LOCAL_RESULT
 *      - Combine local master summary (radio0) + slave summary (radio1) into ResultPacket
 *      - Send CMD_RESULT over ESP-NOW to Reference Node Master
 *
 * ============================================================
 */

#include <Arduino.h>
#include <WiFi.h>

#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "esp_err.h"

#include "../include/rf_test_protocol.h"

// ============================================================
// CONFIGURATION & PINS
// ============================================================

static constexpr uint8_t CONTROL_CHANNEL = CONTROL_CHANNEL_DEFAULT;

static constexpr int LOCAL_SLAVE_RX_PIN = 16;
static constexpr int LOCAL_SLAVE_TX_PIN = 17;
static constexpr uint32_t LOCAL_SLAVE_BAUD = 460800;

// Reference Node Master MAC address - CHANGE THIS TO MATCH YOUR REF MASTER BOARD MAC
static uint8_t REF_MASTER_MAC[6] = { 0x24, 0x6F, 0x28, 0x11, 0x22, 0x33 };

// PMK for ESP-NOW
static const uint8_t ESPNOW_PMK[16] = {
    'R','F','T','E','S','T','-','P','M','K','-','2','0','2','6','!'
};

// ============================================================
// GLOBAL STATE
// ============================================================

static uint32_t gRunId = 0;
static uint16_t gTestId = 0;

static uint8_t gCurrentRFChannel = CONTROL_CHANNEL;
static Phase gCurrentPhase = PHASE_REF_V_TX;

static uint16_t gWindowMs = 400;
static uint32_t gGuardUs = 100000;
static uint32_t gTxIntervalUs = 2000;

static int64_t gLocalSlaveClockOffset = 0; // offset = t_slave - t_local
static bool gLocalSlaveSynced = false;

static bool gPrepared = false;
static bool gRunning = false;

// Queue for ESP-NOW incoming messages
struct EspNowRxItem {
    uint8_t mac[6];
    uint16_t len;
    uint8_t data[250];
};

static QueueHandle_t gEspNowQueue = nullptr;

// CSI & RX Packet Accumulators
static CsiAccumulator gCsiAcc;
static TestPacketStats gPktStats;
static portMUX_TYPE gMux = portMUX_INITIALIZER_UNLOCKED;

// Parser for Local Serial1 from Slave
class SerialParser {
public:
    enum State : uint8_t {
        MAGIC1, MAGIC2, TYPE, LENGTH, PAYLOAD, CRC_LO, CRC_HI
    } state = MAGIC1;

    uint8_t type = 0;
    uint8_t length = 0;
    uint8_t payload[SERIAL_MAX_PAYLOAD];
    uint8_t position = 0;
    uint16_t receivedCRC = 0;

    void reset() {
        state = MAGIC1;
        type = 0;
        length = 0;
        position = 0;
        receivedCRC = 0;
    }

    bool feed(uint8_t b) {
        switch (state) {
            case MAGIC1:
                if (b == SERIAL_MAGIC1) state = MAGIC2;
                break;
            case MAGIC2:
                if (b == SERIAL_MAGIC2) state = TYPE;
                else reset();
                break;
            case TYPE:
                type = b;
                state = LENGTH;
                break;
            case LENGTH:
                length = b;
                if (length > SERIAL_MAX_PAYLOAD) reset();
                else if (length == 0) state = CRC_LO;
                else { position = 0; state = PAYLOAD; }
                break;
            case PAYLOAD:
                payload[position++] = b;
                if (position >= length) state = CRC_LO;
                break;
            case CRC_LO:
                receivedCRC = b;
                state = CRC_HI;
                break;
            case CRC_HI:
                receivedCRC |= ((uint16_t)b << 8);
                {
                    SerialHeader hdr{ SERIAL_MAGIC1, SERIAL_MAGIC2, type, length };
                    uint16_t calc = calculate_crc16((const uint8_t*)&hdr, sizeof(hdr));
                    for (uint8_t i = 0; i < length; ++i) {
                        uint8_t byteVal = payload[i];
                        calc ^= byteVal;
                        for (uint8_t j = 0; j < 8; ++j) {
                            if (calc & 1) calc = (calc >> 1) ^ 0xA001;
                            else calc >>= 1;
                        }
                    }
                    if (calc == receivedCRC) {
                        state = MAGIC1;
                        return true;
                    }
                }
                reset();
                break;
        }
        return false;
    }
};

static SerialParser gLocalSerialParser;
static RFResultSummary gLocalSlaveResult;
static bool gLocalSlaveResultValid = false;

// ============================================================
// CSI & PROMISCUOUS CALLBACKS
// ============================================================

static void IRAM_ATTR csiCallback(void *ctx, wifi_csi_info_t *data) {
    (void)ctx;
    if (!data || !data->buf || data->len < 2) return;

    const int pairs = min(data->len / 2, (int)MAX_CSI_PAIRS);
    const int8_t rssi = data->rx_ctrl.rssi;

    portENTER_CRITICAL_ISR(&gMux);
    gCsiAcc.frames++;
    gCsiAcc.rssiSum += rssi;
    gCsiAcc.rssiSumSq += (uint32_t)(rssi * rssi);
    if (rssi < gCsiAcc.rssiMin) gCsiAcc.rssiMin = rssi;
    if (rssi > gCsiAcc.rssiMax) gCsiAcc.rssiMax = rssi;
    gCsiAcc.pairCountTotal += pairs;

    for (int i = 0; i < pairs; ++i) {
        int8_t imag = data->buf[2 * i];
        int8_t real = data->buf[2 * i + 1];
        gCsiAcc.sumI[i] += real;
        gCsiAcc.sumQ[i] += imag;
        gCsiAcc.count[i]++;
    }
    portEXIT_CRITICAL_ISR(&gMux);
}

static void promiscuousCallback(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (!buf) return;
    if (type != WIFI_PKT_MGMT && type != WIFI_PKT_DATA) return;

    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    uint16_t len = pkt->rx_ctrl.sig_len;
    if (len < 40) return;

    const uint8_t *frame = pkt->payload;
    uint16_t fc = frame[0] | ((uint16_t)frame[1] << 8);
    uint8_t frameType = (fc >> 2) & 0x03;
    uint8_t subtype = (fc >> 4) & 0x0F;

    if (frameType != 0 || subtype != 13) return;

    if (frame[24] != 127) return;
    if (memcmp(frame + 25, TEST_OUI, 3) != 0) return;
    if (frame[28] != PROTOCOL_VERSION) return;
    if (memcmp(frame + 29, TEST_FRAME_MAGIC, 4) != 0) return;

    uint32_t runId; memcpy(&runId, frame + 33, 4);
    uint16_t testId; memcpy(&testId, frame + 37, 2);

    if (runId != gRunId || testId != gTestId) return;

    uint32_t seq; memcpy(&seq, frame + 40, 4);

    portENTER_CRITICAL_ISR(&gMux);
    gPktStats.packets++;
    gPktStats.matchingPackets++;
    if (gPktStats.haveSequence && seq <= gPktStats.lastSequence) {
        gPktStats.duplicates++;
    }
    gPktStats.lastSequence = seq;
    gPktStats.haveSequence = true;
    portEXIT_CRITICAL_ISR(&gMux);
}

static void resetMeasurements() {
    portENTER_CRITICAL(&gMux);
    memset((void*)&gCsiAcc, 0, sizeof(gCsiAcc));
    gCsiAcc.rssiMin = 127;
    gCsiAcc.rssiMax = -127;
    memset((void*)&gPktStats, 0, sizeof(gPktStats));
    portEXIT_CRITICAL(&gMux);
}

// ============================================================
// COMMUNICATION FUNCTIONS
// ============================================================

static void onEspNowReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (!info || !info->src_addr || !data || len <= 0 || len > 250) return;

    EspNowRxItem item;
    memcpy(item.mac, info->src_addr, 6);
    item.len = (uint16_t)len;
    memcpy(item.data, data, len);

    if (gEspNowQueue) {
        xQueueSend(gEspNowQueue, &item, 0);
    }
}

static bool setRadioChannel(uint8_t ch) {
    esp_err_t err = esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    if (err == ESP_OK) {
        gCurrentRFChannel = ch;
        return true;
    }
    return false;
}

static void sendLocalFrame(LocalType type, const void *payload, uint8_t len) {
    SerialHeader hdr{ SERIAL_MAGIC1, SERIAL_MAGIC2, (uint8_t)type, len };
    Serial1.write((const uint8_t*)&hdr, sizeof(hdr));
    if (payload && len > 0) {
        Serial1.write((const uint8_t*)payload, len);
    }
    uint16_t crc = calculate_crc16((const uint8_t*)&hdr, sizeof(hdr));
    if (payload && len > 0) {
        const uint8_t *p = (const uint8_t*)payload;
        for (uint8_t i = 0; i < len; ++i) {
            uint8_t b = p[i];
            crc ^= b;
            for (uint8_t j = 0; j < 8; ++j) {
                if (crc & 1) crc = (crc >> 1) ^ 0xA001;
                else crc >>= 1;
            }
        }
    }
    Serial1.write((const uint8_t*)&crc, 2);
    Serial1.flush();
}

static void pollLocalSerial() {
    while (Serial1.available()) {
        int v = Serial1.read();
        if (v < 0) break;
        if (gLocalSerialParser.feed((uint8_t)v)) {
            if (gLocalSerialParser.type == LOCAL_RESULT) {
                if (gLocalSerialParser.length >= sizeof(RFResultSummary)) {
                    memcpy(&gLocalSlaveResult, gLocalSerialParser.payload, sizeof(RFResultSummary));
                    gLocalSlaveResultValid = true;
                }
            }
            gLocalSerialParser.reset();
        }
    }
}

static bool syncLocalSlaveClock() {
    int64_t bestOffset = 0;
    uint64_t bestDelay = UINT64_MAX;

    for (int i = 0; i < 5; ++i) {
        SyncRequest req{};
        req.magic = RF_MAGIC;
        req.version = PROTOCOL_VERSION;
        req.type = LOCAL_SYNC_REQ;
        req.sequence = i;
        req.t1 = esp_timer_get_time();

        sendLocalFrame(LOCAL_SYNC_REQ, &req, sizeof(req));

        uint64_t waitStart = esp_timer_get_time();
        while (esp_timer_get_time() - waitStart < 50000) {
            pollLocalSerial();
            if (gLocalSerialParser.type == LOCAL_SYNC_RESP && gLocalSerialParser.length >= sizeof(SyncResponse)) {
                SyncResponse resp;
                memcpy(&resp, gLocalSerialParser.payload, sizeof(resp));
                uint64_t t4 = esp_timer_get_time();

                int64_t offset = ((int64_t)resp.t2 - (int64_t)req.t1 + (int64_t)resp.t3 - (int64_t)t4) / 2;
                uint64_t delayUs = (t4 - req.t1) - (resp.t3 - resp.t2);

                if (delayUs < bestDelay) {
                    bestDelay = delayUs;
                    bestOffset = offset;
                }
                break;
            }
            delay(1);
        }
        delay(5);
    }

    if (bestDelay != UINT64_MAX) {
        gLocalSlaveClockOffset = bestOffset;
        gLocalSlaveSynced = true;
        return true;
    }
    return false;
}

// ============================================================
// RF TRANSMISSION & RECEIVE WINDOW
// ============================================================

static void runRFTransmission(uint64_t startAtUs, uint32_t windowUs, uint32_t intervalUs, uint8_t channel, const uint8_t *dstMac) {
    while (esp_timer_get_time() < startAtUs) {
        uint64_t rem = startAtUs - esp_timer_get_time();
        if (rem > 5000) delay(rem / 1000);
        else delayMicroseconds(rem);
    }

    uint64_t endAt = startAtUs + windowUs;
    uint64_t nextTx = startAtUs;
    uint32_t seq = 0;
    uint8_t frame[128];
    uint8_t ownMac[6];
    esp_wifi_get_mac(WIFI_IF_STA, ownMac);

    while (esp_timer_get_time() < endAt) {
        uint64_t now = esp_timer_get_time();
        if (now < nextTx) {
            uint64_t rem = nextTx - now;
            if (rem > 1000) delay(rem / 1000);
            else delayMicroseconds(rem);
            continue;
        }

        uint16_t len = build_raw_80211_action_frame(
            frame, ownMac, dstMac, gRunId, gTestId, (uint8_t)gCurrentPhase, seq, channel, 2 // Role 2 = Test Master
        );

        esp_wifi_80211_tx(WIFI_IF_STA, frame, len, true);
        seq++;
        nextTx += intervalUs;
    }
}

static void runRFReceive(uint64_t startAtUs, uint32_t windowUs) {
    while (esp_timer_get_time() < startAtUs) {
        uint64_t rem = startAtUs - esp_timer_get_time();
        if (rem > 5000) delay(rem / 1000);
        else delayMicroseconds(rem);
    }

    uint64_t endAt = startAtUs + windowUs;
    while (esp_timer_get_time() < endAt) {
        delay(1);
    }
}

static RFResultSummary buildLocalResultSummary(RadioMode mode) {
    RFResultSummary r{};
    r.mode = (uint8_t)mode;

    portENTER_CRITICAL(&gMux);
    r.csiFrames = (uint16_t)min((uint32_t)gCsiAcc.frames, (uint32_t)65535);
    r.csiValid = gCsiAcc.frames > 0 ? 1 : 0;
    r.testPackets = gPktStats.packets;
    r.matchingPackets = gPktStats.matchingPackets;
    r.crcPackets = gPktStats.crcPackets;
    r.duplicatePackets = gPktStats.duplicates;
    r.rssiSum = gCsiAcc.rssiSum;
    r.rssiSumSq = gCsiAcc.rssiSumSq;
    r.rssiMin = gCsiAcc.rssiMin;
    r.rssiMax = gCsiAcc.rssiMax;

    uint16_t pairs = gCsiAcc.frames ? (uint16_t)(gCsiAcc.pairCountTotal / gCsiAcc.frames) : 0;
    r.csiPairs = pairs;

    for (uint8_t out = 0; out < EXPORT_CSI_BINS; ++out) {
        uint16_t srcIdx = (pairs > 1) ? (uint16_t)((out * (pairs - 1)) / (EXPORT_CSI_BINS - 1)) : 0;
        uint16_t count = gCsiAcc.count[srcIdx];
        if (count > 0) {
            r.avgI[out] = (int8_t)(gCsiAcc.sumI[srcIdx] / count);
            r.avgQ[out] = (int8_t)(gCsiAcc.sumQ[srcIdx] / count);
        } else {
            r.avgI[out] = 0;
            r.avgQ[out] = 0;
        }
    }
    portEXIT_CRITICAL(&gMux);
    return r;
}

// ============================================================
// ESP-NOW COMMAND HANDLERS
// ============================================================

static void handleSyncReq(const SyncRequest *req) {
    uint64_t t2 = esp_timer_get_time();
    SyncResponse resp{};
    resp.magic = RF_MAGIC;
    resp.version = PROTOCOL_VERSION;
    resp.type = CMD_SYNC_RESP;
    resp.sequence = req->sequence;
    resp.t1 = req->t1;
    resp.t2 = t2;
    resp.t3 = esp_timer_get_time();

    esp_now_send(REF_MASTER_MAC, (const uint8_t*)&resp, sizeof(resp));
}

static void handlePrepare(const PrepareCommand *cmd) {
    gRunId = cmd->runId;
    gTestId = cmd->testId;
    gCurrentPhase = (Phase)cmd->phase;
    gCurrentRFChannel = cmd->channel;
    gWindowMs = cmd->windowMs;
    gGuardUs = cmd->guardUs;
    gTxIntervalUs = cmd->txIntervalUs;

    // Sync local slave clock and forward PREPARE to slave
    syncLocalSlaveClock();
    sendLocalFrame(LOCAL_PREPARE, cmd, sizeof(PrepareCommand));

    gLocalSlaveResultValid = false;
    resetMeasurements();
    gPrepared = true;

    ReadyMessage ready{};
    ready.magic = RF_MAGIC;
    ready.version = PROTOCOL_VERSION;
    ready.type = CMD_READY;
    ready.runId = gRunId;
    ready.testId = gTestId;
    ready.status = 0;

    esp_now_send(REF_MASTER_MAC, (const uint8_t*)&ready, sizeof(ready));
}

static void handleStartAt(const StartAtCommand *cmd) {
    if (!gPrepared) return;

    // Forward START_AT to local slave with offset adjustment
    uint64_t localSlaveStartUs = cmd->startAtUs + gLocalSlaveClockOffset;
    StartAtCommand localSlaveStartCmd = *cmd;
    localSlaveStartCmd.startAtUs = localSlaveStartUs;
    sendLocalFrame(LOCAL_START_AT, &localSlaveStartCmd, sizeof(localSlaveStartCmd));

    setRadioChannel(gCurrentRFChannel);
    resetMeasurements();
    gRunning = true;

    bool isTx = (gCurrentPhase == PHASE_TEST_A_TX);
    if (isTx) {
        runRFTransmission(cmd->startAtUs, cmd->windowUs, gTxIntervalUs, gCurrentRFChannel, REF_MASTER_MAC);
    } else {
        runRFReceive(cmd->startAtUs, cmd->windowUs);
    }

    uint64_t returnTime = cmd->startAtUs + cmd->windowUs + cmd->guardAfterUs;
    while (esp_timer_get_time() < returnTime) {
        delay(1);
    }

    setRadioChannel(CONTROL_CHANNEL);
    gRunning = false;
    gPrepared = false;

    // Poll local slave for result
    uint64_t slaveWaitStart = esp_timer_get_time();
    while (esp_timer_get_time() - slaveWaitStart < 200000) { // 200ms
        pollLocalSerial();
        if (gLocalSlaveResultValid) break;
        delay(2);
    }

    // Package local master (radio0) + slave (radio1) results
    ResultPacket pkt{};
    pkt.magic = RF_MAGIC;
    pkt.version = PROTOCOL_VERSION;
    pkt.type = CMD_RESULT;
    pkt.runId = gRunId;
    pkt.testId = gTestId;
    pkt.phase = (uint8_t)gCurrentPhase;
    pkt.status = 0;

    pkt.radio0 = buildLocalResultSummary(isTx ? RADIO_MODE_TX : RADIO_MODE_RX);
    if (gLocalSlaveResultValid) {
        pkt.radio1 = gLocalSlaveResult;
    } else {
        memset(&pkt.radio1, 0, sizeof(RFResultSummary));
        pkt.radio1.mode = RADIO_MODE_NONE;
    }

    esp_now_send(REF_MASTER_MAC, (const uint8_t*)&pkt, sizeof(pkt));
}

static void processEspNowQueue() {
    if (gRunning) return;

    EspNowRxItem item;
    while (xQueueReceive(gEspNowQueue, &item, 0) == pdTRUE) {
        if (item.len < 4) continue;
        uint8_t type = item.data[3];

        if (type == CMD_SYNC_REQ && item.len >= sizeof(SyncRequest)) {
            const SyncRequest *req = (const SyncRequest*)item.data;
            if (req->magic == RF_MAGIC) handleSyncReq(req);
        } else if (type == CMD_PREPARE && item.len >= sizeof(PrepareCommand)) {
            const PrepareCommand *cmd = (const PrepareCommand*)item.data;
            if (cmd->magic == RF_MAGIC) handlePrepare(cmd);
        } else if (type == CMD_START_AT && item.len >= sizeof(StartAtCommand)) {
            const StartAtCommand *cmd = (const StartAtCommand*)item.data;
            if (cmd->magic == RF_MAGIC) handleStartAt(cmd);
        } else if (type == CMD_ABORT) {
            gRunning = false;
            gPrepared = false;
            sendLocalFrame(LOCAL_ABORT, nullptr, 0);
            setRadioChannel(CONTROL_CHANNEL);
        }
    }
}

// ============================================================
// ARDUINO SETUP & LOOP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\nQuad ESP32-WROOM RF Antenna Tester");
    Serial.println("Test Node Master Firmware (Test Antenna)");

    Serial1.begin(LOCAL_SLAVE_BAUD, SERIAL_8N1, LOCAL_SLAVE_RX_PIN, LOCAL_SLAVE_TX_PIN);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true, true);
    delay(50);

    esp_wifi_set_ps(WIFI_PS_NONE);
    setRadioChannel(CONTROL_CHANNEL);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb((wifi_promiscuous_cb_t)promiscuousCallback);

    esp_wifi_set_csi_rx_cb(csiCallback, nullptr);
    wifi_csi_config_t csiConfig{};
    csiConfig.lltf_en = true;
    csiConfig.htltf_en = true;
    csiConfig.stbc_htltf2_en = true;
    csiConfig.ltf_merge_en = false;
    csiConfig.channel_filter_en = false;
    csiConfig.manu_scale = false;
    csiConfig.shift = 0;
    esp_wifi_set_csi_config(&csiConfig);
    esp_wifi_set_csi(true);

    esp_wifi_set_max_tx_power(78); // 19.5 dBm

    gEspNowQueue = xQueueCreate(16, sizeof(EspNowRxItem));
    if (esp_now_init() != ESP_OK) {
        Serial.println("FATAL: esp_now_init failed");
        while (1) delay(1000);
    }

    esp_now_set_pmk(ESPNOW_PMK);
    esp_now_register_recv_cb(onEspNowReceive);

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, REF_MASTER_MAC, 6);
    peer.channel = CONTROL_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (!esp_now_is_peer_exist(REF_MASTER_MAC)) {
        esp_now_add_peer(&peer);
    }

    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    Serial.printf("Test Master MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void loop() {
    processEspNowQueue();
    pollLocalSerial();
    delay(1);
}
