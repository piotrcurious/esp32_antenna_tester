/*
 * ============================================================
 * QUAD ESP32-WROOM RF ANTENNA TESTING SYSTEM
 *
 * REFERENCE NODE MASTER FIRMWARE
 * ============================================================
 *
 * Hardware Role:
 *   - Reference V-Polarized 1/4-wave antenna radio
 *   - Global Experiment Orchestrator
 *   - Serial1 Controller for Reference Node Slave (H-Pol Antenna)
 *   - ESP-NOW Controller for Remote Test Node Master
 *   - USB Serial logger to Linux PC
 *
 * Synchronization & Measurement Flow:
 *   1. Stay on CONTROL_CHANNEL (Ch 1)
 *   2. Perform clock sync with remote Test Master over ESP-NOW and local Slave over Serial1
 *   3. Send PREPARE to Test Master and local Slave (specifying target channel, phase, duration, MACs)
 *   4. Wait for READY from Test Master and local Slave
 *   5. Send START_AT with calculated synchronized start timestamp
 *   6. Switch to TARGET_CHANNEL during guard period
 *   7. Execute RF measurement window (Raw 802.11 action frames TX or promiscuous CSI RX)
 *      - NO ESP-NOW OR SERIAL1 CONTROL MESSAGES DURING RF WINDOW
 *   8. Wait guard period after window, return to CONTROL_CHANNEL
 *   9. Collect RF result summaries from local Slave and remote Test Master
 *  10. Output aggregated CSV result lines over USB Serial to Linux PC
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

// Channels to test in sweep
static const uint8_t SWEEP_CHANNELS[] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13 };
static constexpr size_t NUM_SWEEP_CHANNELS = sizeof(SWEEP_CHANNELS) / sizeof(SWEEP_CHANNELS[0]);

// Default RF window parameters
static constexpr uint16_t DEFAULT_WINDOW_MS       = 400;   // 400 ms test window
static constexpr uint32_t DEFAULT_GUARD_US         = 100000; // 100 ms guard before/after
static constexpr uint32_t DEFAULT_TX_INTERVAL_US   = 2000;   // 500 pkts/sec
static constexpr uint32_t START_AHEAD_US           = 300000; // Schedule start 300 ms in future

// Serial1 pins for local slave (Reference Node Slave - H Pol)
static constexpr int LOCAL_SLAVE_RX_PIN = 16;
static constexpr int LOCAL_SLAVE_TX_PIN = 17;
static constexpr uint32_t LOCAL_SLAVE_BAUD = 460800;

// Remote Test Master MAC address - CHANGE THIS TO MATCH YOUR TEST MASTER BOARD MAC
static uint8_t TEST_MASTER_MAC[6] = { 0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC };

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

static int64_t gTestMasterClockOffset = 0; // offset = t_remote - t_local
static int64_t gLocalSlaveClockOffset = 0; // offset = t_slave - t_local

static bool gTestMasterSynced = false;
static bool gLocalSlaveSynced = false;

// Queue for ESP-NOW incoming messages
struct EspNowRxItem {
    uint8_t mac[6];
    uint16_t len;
    uint8_t data[250];
};

static QueueHandle_t gEspNowQueue = nullptr;

// CSI & RX Packet Accumulator
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

    if (frameType != 0 || subtype != 13) return; // Action frame check

    if (frame[24] != 127) return; // Vendor specific category
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
// HARDWARE & COMMUNICATION SETUP
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

static void initWifiAndEspNow() {
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

    // Register Test Master peer
    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, TEST_MASTER_MAC, 6);
    peer.channel = CONTROL_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (!esp_now_is_peer_exist(TEST_MASTER_MAC)) {
        esp_now_add_peer(&peer);
    }
}

// ============================================================
// LOCAL SERIAL1 FUNCTIONS
// ============================================================

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

// ============================================================
// CLOCK SYNCHRONIZATION
// ============================================================

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

static bool syncTestMasterClock() {
    int64_t bestOffset = 0;
    uint64_t bestDelay = UINT64_MAX;

    for (int i = 0; i < 5; ++i) {
        SyncRequest req{};
        req.magic = RF_MAGIC;
        req.version = PROTOCOL_VERSION;
        req.type = CMD_SYNC_REQ;
        req.sequence = i;
        req.t1 = esp_timer_get_time();

        esp_now_send(TEST_MASTER_MAC, (const uint8_t*)&req, sizeof(req));

        uint64_t waitStart = esp_timer_get_time();
        while (esp_timer_get_time() - waitStart < 100000) {
            EspNowRxItem item;
            if (xQueueReceive(gEspNowQueue, &item, pdMS_TO_TICKS(2)) == pdTRUE) {
                if (item.len >= sizeof(SyncResponse)) {
                    const SyncResponse *resp = (const SyncResponse*)item.data;
                    if (resp->magic == RF_MAGIC && resp->type == CMD_SYNC_RESP && resp->sequence == (uint32_t)i) {
                        uint64_t t4 = esp_timer_get_time();
                        int64_t offset = ((int64_t)resp->t2 - (int64_t)req.t1 + (int64_t)resp->t3 - (int64_t)t4) / 2;
                        uint64_t delayUs = (t4 - req.t1) - (resp->t3 - resp->t2);

                        if (delayUs < bestDelay) {
                            bestDelay = delayUs;
                            bestOffset = offset;
                        }
                        break;
                    }
                }
            }
        }
        delay(5);
    }

    if (bestDelay != UINT64_MAX) {
        gTestMasterClockOffset = bestOffset;
        gTestMasterSynced = true;
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
            frame, ownMac, dstMac, gRunId, gTestId, (uint8_t)gCurrentPhase, seq, channel, 0
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

static RFResultSummary buildLocalMasterResult(RadioMode mode) {
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
// RESULT LOGGING TO PYTHON / LINUX PC
// ============================================================

static void logResultToPC(const char *nodeName, const char *antennaName, uint8_t ch, Phase phase, const RFResultSummary &r) {
    float meanRssi = (r.csiFrames > 0) ? ((float)r.rssiSum / r.csiFrames) : 0.0f;
    float meanSqRssi = (r.csiFrames > 0) ? ((float)r.rssiSumSq / r.csiFrames) : 0.0f;
    float stdRssi = sqrtf(max(0.0f, meanSqRssi - meanRssi * meanRssi));

    Serial.printf(
        "DATA,run=%lu,test=%u,ch=%u,phase=%u,node=%s,ant=%s,mode=%u,frames=%u,pkts=%lu,match=%lu,rssi_avg=%.2f,rssi_std=%.2f,min=%d,max=%d",
        (unsigned long)gRunId, gTestId, ch, (unsigned)phase, nodeName, antennaName, r.mode,
        r.csiFrames, (unsigned long)r.testPackets, (unsigned long)r.matchingPackets,
        meanRssi, stdRssi, r.rssiMin, r.rssiMax
    );

    Serial.print(",csi_i=[");
    for (int i = 0; i < EXPORT_CSI_BINS; ++i) {
        Serial.printf("%d%s", r.avgI[i], (i < EXPORT_CSI_BINS - 1) ? ";" : "");
    }
    Serial.print("],csi_q=[");
    for (int i = 0; i < EXPORT_CSI_BINS; ++i) {
        Serial.printf("%d%s", r.avgQ[i], (i < EXPORT_CSI_BINS - 1) ? ";" : "");
    }
    Serial.println("]");
    Serial.flush();
}

// ============================================================
// EXECUTE ONE PHASE
// ============================================================

static bool executePhase(uint8_t channel, Phase phase) {
    gCurrentPhase = phase;
    gTestId++;

    // Return to control channel
    setRadioChannel(CONTROL_CHANNEL);
    xQueueReset(gEspNowQueue);

    // Sync clocks
    syncLocalSlaveClock();
    syncTestMasterClock();

    // 1. Send PREPARE to remote Test Master
    PrepareCommand prepCmd{};
    prepCmd.magic = RF_MAGIC;
    prepCmd.version = PROTOCOL_VERSION;
    prepCmd.type = CMD_PREPARE;
    prepCmd.runId = gRunId;
    prepCmd.testId = gTestId;
    prepCmd.phase = (uint8_t)phase;
    prepCmd.channel = channel;
    prepCmd.windowMs = DEFAULT_WINDOW_MS;
    prepCmd.guardUs = DEFAULT_GUARD_US;
    prepCmd.txIntervalUs = DEFAULT_TX_INTERVAL_US;
    prepCmd.bandwidth = 20;
    prepCmd.txRate = 0;
    esp_wifi_get_mac(WIFI_IF_STA, prepCmd.targetMac);

    esp_now_send(TEST_MASTER_MAC, (const uint8_t*)&prepCmd, sizeof(prepCmd));

    // Wait for READY from Test Master
    bool testMasterReady = false;
    uint64_t waitStart = esp_timer_get_time();
    while (esp_timer_get_time() - waitStart < 200000) { // 200ms
        EspNowRxItem item;
        if (xQueueReceive(gEspNowQueue, &item, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (item.len >= sizeof(ReadyMessage)) {
                const ReadyMessage *ready = (const ReadyMessage*)item.data;
                if (ready->magic == RF_MAGIC && ready->type == CMD_READY && ready->runId == gRunId && ready->testId == gTestId) {
                    testMasterReady = true;
                    break;
                }
            }
        }
    }

    if (!testMasterReady) {
        Serial.printf("WARNING: Test Master READY timeout on ch=%u phase=%u\n", channel, (unsigned)phase);
        return false;
    }

    // 2. Send PREPARE to Local Slave
    sendLocalFrame(LOCAL_PREPARE, &prepCmd, sizeof(prepCmd));
    delay(10);

    // 3. Compute Synchronized Start Time (in local time)
    uint64_t localStartUs = esp_timer_get_time() + START_AHEAD_US;

    // Convert start time for Test Master and Local Slave using offsets
    uint64_t remoteMasterStartUs = localStartUs + gTestMasterClockOffset;
    uint64_t localSlaveStartUs  = localStartUs + gLocalSlaveClockOffset;

    StartAtCommand startRemoteCmd{};
    startRemoteCmd.magic = RF_MAGIC;
    startRemoteCmd.version = PROTOCOL_VERSION;
    startRemoteCmd.type = CMD_START_AT;
    startRemoteCmd.runId = gRunId;
    startRemoteCmd.testId = gTestId;
    startRemoteCmd.startAtUs = remoteMasterStartUs;
    startRemoteCmd.windowUs = (uint32_t)DEFAULT_WINDOW_MS * 1000UL;
    startRemoteCmd.guardAfterUs = DEFAULT_GUARD_US;

    esp_now_send(TEST_MASTER_MAC, (const uint8_t*)&startRemoteCmd, sizeof(startRemoteCmd));

    StartAtCommand startLocalCmd = startRemoteCmd;
    startLocalCmd.startAtUs = localSlaveStartUs;
    sendLocalFrame(LOCAL_START_AT, &startLocalCmd, sizeof(startLocalCmd));

    // 4. Switch Local Master to Target RF Channel
    setRadioChannel(channel);
    resetMeasurements();
    gLocalSlaveResultValid = false;

    // 5. Execute Local RF Window (No control communication)
    bool localMasterIsTx = (phase == PHASE_REF_V_TX);
    if (localMasterIsTx) {
        runRFTransmission(localStartUs, (uint32_t)DEFAULT_WINDOW_MS * 1000UL, DEFAULT_TX_INTERVAL_US, channel, TEST_MASTER_MAC);
    } else {
        runRFReceive(localStartUs, (uint32_t)DEFAULT_WINDOW_MS * 1000UL);
    }

    // 6. Guard interval after test
    delay((DEFAULT_GUARD_US / 1000) + 20);

    // 7. Return to CONTROL_CHANNEL
    setRadioChannel(CONTROL_CHANNEL);
    delay(20);

    // 8. Poll Local Slave for Result
    uint64_t slaveWaitStart = esp_timer_get_time();
    while (esp_timer_get_time() - slaveWaitStart < 200000) { // 200ms
        pollLocalSerial();
        if (gLocalSlaveResultValid) break;
        delay(2);
    }

    // 9. Receive Result from Remote Test Master
    ResultPacket remoteResult{};
    bool gotRemoteResult = false;
    uint64_t remoteWaitStart = esp_timer_get_time();
    while (esp_timer_get_time() - remoteWaitStart < 300000) { // 300ms
        EspNowRxItem item;
        if (xQueueReceive(gEspNowQueue, &item, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (item.len >= sizeof(ResultPacket)) {
                memcpy(&remoteResult, item.data, sizeof(ResultPacket));
                if (remoteResult.magic == RF_MAGIC && remoteResult.type == CMD_RESULT && remoteResult.testId == gTestId) {
                    gotRemoteResult = true;
                    break;
                }
            }
        }
    }

    // 10. Build Local Master Result
    RFResultSummary localMasterResult = buildLocalMasterResult(localMasterIsTx ? RADIO_MODE_TX : RADIO_MODE_RX);

    // Log all 4 antenna results to Linux PC
    logResultToPC("REF_MASTER", "REF_V_ANT", channel, phase, localMasterResult);
    if (gLocalSlaveResultValid) {
        logResultToPC("REF_SLAVE", "REF_H_ANT", channel, phase, gLocalSlaveResult);
    }
    if (gotRemoteResult) {
        logResultToPC("TEST_MASTER", "TEST_ANT", channel, phase, remoteResult.radio0);
        logResultToPC("TEST_SLAVE", "TEST_REF_ANT", channel, phase, remoteResult.radio1);
    }

    return true;
}

// ============================================================
// FULL SWEEP EXECUTION
// ============================================================

static void runFullSweep() {
    gRunId++;
    gTestId = 0;

    Serial.println("\n==================================================");
    Serial.printf("STARTING RF ANTENNA SWEEP [RUN_ID=%lu]\n", (unsigned long)gRunId);
    Serial.println("==================================================");

    for (size_t i = 0; i < NUM_SWEEP_CHANNELS; ++i) {
        uint8_t ch = SWEEP_CHANNELS[i];
        Serial.printf("--- TESTING CHANNEL %u ---\n", ch);

        executePhase(ch, PHASE_REF_V_TX);
        delay(50);
        executePhase(ch, PHASE_REF_H_TX);
        delay(50);
        executePhase(ch, PHASE_TEST_A_TX);
        delay(50);
        executePhase(ch, PHASE_TEST_R_TX);
        delay(100);
    }

    Serial.println("==================================================");
    Serial.printf("SWEEP COMPLETE [RUN_ID=%lu]\n", (unsigned long)gRunId);
    Serial.println("==================================================\n");
}

// ============================================================
// ARDUINO SETUP & LOOP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\nQuad ESP32-WROOM RF Antenna Tester");
    Serial.println("Reference Node Master Firmware");

    Serial1.begin(LOCAL_SLAVE_BAUD, SERIAL_8N1, LOCAL_SLAVE_RX_PIN, LOCAL_SLAVE_TX_PIN);

    initWifiAndEspNow();

    uint8_t ownMac[6];
    esp_wifi_get_mac(WIFI_IF_STA, ownMac);
    Serial.printf("Reference Master MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  ownMac[0], ownMac[1], ownMac[2], ownMac[3], ownMac[4], ownMac[5]);
}

void loop() {
    static bool sweepDone = false;
    if (!sweepDone) {
        sweepDone = true;
        delay(3000); // Initial boot delay
        runFullSweep();
    }

    // Listen for manual trigger command "SWEEP" from PC over USB
    if (Serial.available()) {
        int val = Serial.read();
        if (val == 'S' || val == 's') {
            runFullSweep();
        }
    }
    delay(100);
}
