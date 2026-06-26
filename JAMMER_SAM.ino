#include "RF24.h"
#include <SPI.h>
#include "esp_bt.h"
#include "esp_wifi.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_timer.h"

// ═══════════════════════════════════════════════════════════════════
//  AUTHOR: SAM chota larga HERITIER
//  MAXIMUM EFFICIENCY BLUETOOTH JAMMER v4.8
//  Dual E01-ML01DP5 + ESP32 Native BT Radio
//  FIX: ISR paths separados (sin branches), volatile fixes, benchmark rápido
// ═══════════════════════════════════════════════════════════════════

// ─── PIN DEFINITIONS ─────────────────────────────────────────────
#define LED_PIN         2
#define CE_HP           16
#define CSN_HP          15
#define CE_SP           22
#define CSN_SP          21
#define SPI_SPEED       16000000

// ─── CHANNEL CONFIGURATION ──────────────────────────────────────
#define NUM_CHANNELS    40

// ─── TIMER CONFIG ────────────────────────────────────────────────
#define DEFAULT_INTERVAL_US     120
#define MIN_INTERVAL_US         90
#define MARGIN_PCT              20

// ─── E01-ML01DP5 CONFIG ─────────────────────────────────────────
#define NRF24_PA_LEVEL      RF24_PA_MAX
#define NRF24_DATA_RATE     RF24_2MBPS

// ─── ESP32 BT CONFIG ────────────────────────────────────────────
#define BT_TX_POWER         ESP_PWR_LVL_P7

// ─── TABLE SIZE ──────────────────────────────────────────────────
#define TABLE_SIZE          2048
#define TABLE_MASK          (TABLE_SIZE - 1)

// ─── RADIO STRUCT ───────────────────────────────────────────────
struct JammerRadio {
    RF24 radio;
    SPIClass* spi;
    uint8_t cePin;
    uint8_t csnPin;
    uint8_t spiBus;
    const char* name;
    volatile uint8_t currentChannel;
    bool initialized;
};

// ─── GLOBALS ─────────────────────────────────────────────────────
JammerRadio radioHP = { RF24(CE_HP, CSN_HP, SPI_SPEED), nullptr, CE_HP, CSN_HP, HSPI, "HP", 0, false };
JammerRadio radioSP = { RF24(CE_SP, CSN_SP, SPI_SPEED), nullptr, CE_SP, CSN_SP, VSPI, "SP", 40, false };

hw_timer_t *jamTimer = nullptr;

volatile uint32_t jamCounter = 0;
volatile uint16_t tableIndex = 0;

uint8_t channelTableHP[TABLE_SIZE];
uint8_t channelTableSP[TABLE_SIZE];

bool hpActive = false;
bool spActive = false;
bool btActive = false;

volatile uint32_t jamIntervalUs = DEFAULT_INTERVAL_US;

// BLE GAP callback
static void ble_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    (void)event;
    (void)param;
}

// ═══════════════════════════════════════════════════════════════════
//  ISR PATHS - Una función por combinación activa, sin branches
// ═══════════════════════════════════════════════════════════════════
void IRAM_ATTR onJamTimer_HP() {
    uint16_t idx = tableIndex & TABLE_MASK;
    radioHP.radio.setChannel(channelTableHP[idx]);
    tableIndex++;
    jamCounter++;
}

void IRAM_ATTR onJamTimer_SP() {
    uint16_t idx = tableIndex & TABLE_MASK;
    radioSP.radio.setChannel(channelTableSP[idx]);
    tableIndex++;
    jamCounter++;
}

void IRAM_ATTR onJamTimer_Both() {
    uint16_t idx = tableIndex & TABLE_MASK;
    radioHP.radio.setChannel(channelTableHP[idx]);
    radioSP.radio.setChannel(channelTableSP[idx]);
    tableIndex++;
    jamCounter++;
}

// ═══════════════════════════════════════════════════════════════════
//  BENCHMARK RÁPIDO DE setChannel()
// ═══════════════════════════════════════════════════════════════════
uint32_t benchmarkSetChannel() {
    Serial.println("[CAL] Benchmarking setChannel()...");
    const int iterations = 300;  // v4.8: reducido para setup más rápido
    uint64_t tStart, tEnd;

    // Warmup
    if (hpActive) radioHP.radio.setChannel(0);
    if (spActive) radioSP.radio.setChannel(40);
    delay(1);

    if (hpActive && spActive) {
        tStart = esp_timer_get_time();
        for (int i = 0; i < iterations; i++) {
            radioHP.radio.setChannel(i % 40);
            radioSP.radio.setChannel(40 + (i % 40));
        }
        tEnd = esp_timer_get_time();
    } else if (hpActive) {
        tStart = esp_timer_get_time();
        for (int i = 0; i < iterations; i++) {
            radioHP.radio.setChannel(i % 40);
        }
        tEnd = esp_timer_get_time();
    } else if (spActive) {
        tStart = esp_timer_get_time();
        for (int i = 0; i < iterations; i++) {
            radioSP.radio.setChannel(40 + (i % 40));
        }
        tEnd = esp_timer_get_time();
    } else {
        return 0;
    }

    uint32_t avgUs = (uint32_t)(tEnd - tStart) / iterations;
    Serial.printf("[CAL] Avg setChannel(): %luμs\n", avgUs);
    return avgUs;
}

// ═══════════════════════════════════════════════════════════════════
//  ROBUST BT INITIALIZATION
// ═══════════════════════════════════════════════════════════════════
bool initNativeBTRadio() {
    Serial.println("[INIT] Starting ESP32 BT radio...");

    esp_bt_controller_status_t status = esp_bt_controller_get_status();
    Serial.printf("[INIT] BT controller status: %d (0=IDLE,1=INITED,2=ENABLED,3=OTHER)\n", status);

    esp_err_t err;

    if (status == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        Serial.println("[INIT] BT controller already enabled by Arduino framework");
    }
    else if (status == ESP_BT_CONTROLLER_STATUS_INITED) {
        Serial.println("[INIT] BT controller already initialized, enabling...");
        err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (err != ESP_OK) {
            Serial.printf("[WARN] BT enable failed: %d\n", err);
            return false;
        }
    }
    else if (status == ESP_BT_CONTROLLER_STATUS_IDLE) {
        Serial.println("[INIT] BT controller idle, doing full initialization...");

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        err = esp_bt_controller_init(&bt_cfg);
        if (err != ESP_OK) {
            if (err == ESP_ERR_INVALID_STATE) {
                Serial.println("[INIT] BT init returned INVALID_STATE (already init?), trying enable...");
            } else {
                Serial.printf("[WARN] BT controller init failed: %d\n", err);
                return false;
            }
        }

        err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (err != ESP_OK) {
            Serial.printf("[WARN] BT enable failed: %d\n", err);
            if (err != ESP_ERR_INVALID_STATE) {
                esp_bt_controller_deinit();
                return false;
            }
        }
    }
    else {
        Serial.println("[INIT] BT controller in unknown state, attempting recovery...");
        esp_bt_controller_deinit();
        delay(100);

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        err = esp_bt_controller_init(&bt_cfg);
        if (err != ESP_OK) {
            Serial.printf("[WARN] BT recovery init failed: %d\n", err);
            return false;
        }

        err = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (err != ESP_OK) {
            Serial.printf("[WARN] BT recovery enable failed: %d\n", err);
            return false;
        }
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        Serial.printf("[WARN] Bluedroid init failed: %d (may already be initialized)\n", err);
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        Serial.printf("[WARN] Bluedroid enable failed: %d (may already be enabled)\n", err);
    }

    err = esp_ble_gap_register_callback(ble_gap_cb);
    if (err != ESP_OK) {
        Serial.printf("[WARN] GAP callback register failed: %d\n", err);
    }

    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, BT_TX_POWER);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, BT_TX_POWER);
    esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, BT_TX_POWER);

    esp_ble_adv_params_t adv_params = {
        .adv_int_min = 0x0020,
        .adv_int_max = 0x0040,
        .adv_type = ADV_TYPE_NONCONN_IND,
        .own_addr_type = BLE_ADDR_TYPE_RANDOM,
        .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    uint8_t adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb,
        0x1b, 0xff, 0x4c, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00
    };

    err = esp_ble_gap_config_adv_data_raw(adv_data, sizeof(adv_data));
    if (err != ESP_OK) {
        Serial.printf("[WARN] Adv data config: %d\n", err);
    }

    err = esp_ble_gap_start_advertising(&adv_params);
    if (err != ESP_OK) {
        Serial.printf("[WARN] Advertising start: %d\n", err);
    }

    esp_ble_scan_params_t scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_RANDOM,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x0010,
        .scan_window = 0x0010,
    };
    esp_ble_gap_set_scan_params(&scan_params);
    esp_ble_gap_start_scanning(0);

    Serial.println("[OK] BT Radio | +9dBm | Adv+Scan");
    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.println("\n═══════════════════════════════════════════");
    Serial.println("  MAX EFFICIENCY BT JAMMER v4.8");
    Serial.println("  ISR Paths | Benchmarked | BT+NRF24");
    Serial.println("═══════════════════════════════════════════\n");

    // Disable WiFi (NO esp_wifi_deinit())
    esp_wifi_stop();
    esp_wifi_disconnect();
    delay(100);

    generateChannelTables();

    hpActive = initRadio(radioHP);
    spActive = initRadio(radioSP);

    if (hpActive) {
        Serial.println("[OK] HP Radio active (+20dBm)");
    } else {
        Serial.println("[WARN] HP Radio not detected");
    }

    if (spActive) {
        Serial.println("[OK] SP Radio active (+20dBm)");
    } else {
        Serial.println("[WARN] SP Radio not detected");
    }

    if (!hpActive && !spActive) {
        Serial.println("[INFO] No NRF24 modules - BT-only mode");
    }

    // Benchmark y cálculo de intervalo óptimo
    if (hpActive || spActive) {
        uint32_t avgUs = benchmarkSetChannel();
        uint32_t suggested = (avgUs * (100 + MARGIN_PCT)) / 100;
        if (suggested < MIN_INTERVAL_US) suggested = MIN_INTERVAL_US;
        jamIntervalUs = suggested;
        Serial.printf("[CAL] Optimal interval: %luμs (margin %d%%)\n", (uint32_t)jamIntervalUs, MARGIN_PCT);
    }

    // Init BT
    btActive = initNativeBTRadio();
    if (btActive) {
        Serial.println("[OK] ESP32 BT radio active (+9dBm)");
    } else {
        Serial.println("[WARN] ESP32 BT init failed");
    }

    // Safety check
    if (!hpActive && !spActive && !btActive) {
        errorHalt("No jamming sources available!");
    }

    // Selección de ISR según radios activos (sin branches en runtime)
    void (*isrFunc)() = nullptr;
    if (hpActive && spActive) {
        isrFunc = &onJamTimer_Both;
    } else if (hpActive) {
        isrFunc = &onJamTimer_HP;
    } else if (spActive) {
        isrFunc = &onJamTimer_SP;
    }

    if (isrFunc != nullptr) {
        jamTimer = timerBegin(1000000);
        if (jamTimer == nullptr) {
            errorHalt("Timer init failed");
        }
        timerAttachInterrupt(jamTimer, isrFunc);
        timerAlarm(jamTimer, jamIntervalUs, true, 0);
        timerStart(jamTimer);

        Serial.printf("[OK] Timer ISR: %luμs | Rate:%.0fHz\n", (uint32_t)jamIntervalUs, 1000000.0 / (uint32_t)jamIntervalUs);
    } else {
        Serial.println("[INFO] Timer ISR disabled (BT-only)");
    }

    Serial.printf("[OK] Active: HP=%s SP=%s BT=%s\n",
                  hpActive ? "YES" : "NO",
                  spActive ? "YES" : "NO",
                  btActive ? "YES" : "NO");
    Serial.println("[OK] JAMMING ACTIVE");
    Serial.println("═══════════════════════════════════════════\n");

    digitalWrite(LED_PIN, HIGH);
}

// ═══════════════════════════════════════════════════════════════════
//  MAIN LOOP - Stats con lecturas volatile explícitas
// ═══════════════════════════════════════════════════════════════════
void loop() {
    static unsigned long lastStats = 0;
    unsigned long now = millis();

    if (now - lastStats >= 5000) {
        lastStats = now;

        // Lecturas volatile explícitas (evita optimizaciones del compilador)
        volatile uint32_t count = jamCounter;
        volatile uint16_t idx = tableIndex;

        if (hpActive || spActive) {
            uint8_t chHP = channelTableHP[(idx - 1) & TABLE_MASK];
            uint8_t chSP = channelTableSP[(idx - 1) & TABLE_MASK];
            float rate = 1000000.0 / (uint32_t)jamIntervalUs;

            Serial.printf("[STATS] Switches:%lu | HP:%d | SP:%d | Rate:%.0fHz | Int:%luμs | BT:%s\n",
                          (uint32_t)count, chHP, chSP, rate, (uint32_t)jamIntervalUs,
                          btActive ? "ACTIVE" : "OFF");
        } else {
            Serial.printf("[STATS] BT:%s | NRF24: none\n", btActive ? "ACTIVE" : "OFF");
        }
    }

    yield();
}

// ═══════════════════════════════════════════════════════════════════
//  RADIO INIT
// ═══════════════════════════════════════════════════════════════════
bool initRadio(JammerRadio& cfg) {
    cfg.spi = new SPIClass(cfg.spiBus);
    cfg.spi->begin();

    if (!cfg.radio.begin(cfg.spi)) {
        Serial.printf("[WARN] %s radio not responding (check wiring)\n", cfg.name);
        cfg.initialized = false;
        return false;
    }

    cfg.radio.setAutoAck(false);
    cfg.radio.stopListening();
    cfg.radio.setRetries(0, 0);
    cfg.radio.setPALevel(NRF24_PA_LEVEL, true);
    cfg.radio.setDataRate(NRF24_DATA_RATE);
    cfg.radio.setCRCLength(RF24_CRC_DISABLED);
    cfg.radio.setAddressWidth(2);
    cfg.radio.setPayloadSize(0);
    cfg.radio.startConstCarrier(NRF24_PA_LEVEL, cfg.currentChannel);
    cfg.initialized = true;

    Serial.printf("[OK] %s | SPI:%d | CE:%d | CSN:%d | PWR:+20dBm\n",
                  cfg.name, cfg.spiBus, cfg.cePin, cfg.csnPin);
    return true;
}

// ═══════════════════════════════════════════════════════════════════
//  TABLES
// ═══════════════════════════════════════════════════════════════════
void generateChannelTables() {
    Serial.println("[INIT] Generating random channel tables...");

    for (int i = 0; i < TABLE_SIZE; i++) {
        uint32_t rnd = esp_random();
        channelTableHP[i] = (uint8_t)(rnd % NUM_CHANNELS);
        channelTableSP[i] = (uint8_t)(40 + ((rnd >> 8) % NUM_CHANNELS));
    }

    #ifdef DEBUG
    int freqHP[NUM_CHANNELS] = {0};
    int freqSP[NUM_CHANNELS] = {0};
    for (int i = 0; i < TABLE_SIZE; i++) {
        freqHP[channelTableHP[i]]++;
        freqSP[channelTableSP[i] - 40]++;
    }
    int minHP = TABLE_SIZE, maxHP = 0;
    int minSP = TABLE_SIZE, maxSP = 0;
    for (int i = 0; i < NUM_CHANNELS; i++) {
        if (freqHP[i] < minHP) minHP = freqHP[i];
        if (freqHP[i] > maxHP) maxHP = freqHP[i];
        if (freqSP[i] < minSP) minSP = freqSP[i];
        if (freqSP[i] > maxSP) maxSP = freqSP[i];
    }
    Serial.printf("[DEBUG] HP: Min:%d Max:%d | SP: Min:%d Max:%d\n", minHP, maxHP, minSP, maxSP);
    #endif

    Serial.println("[OK] Tables generated");
}

// ═══════════════════════════════════════════════════════════════════
//  ERROR HANDLER
// ═══════════════════════════════════════════════════════════════════
void errorHalt(const char* msg) {
    Serial.printf("\n[FATAL] %s\n", msg);
    Serial.println("[FATAL] System halted");

    unsigned long lastToggle = 0;
    bool ledState = false;
    while (true) {
        unsigned long now = millis();
        if (now - lastToggle >= 150) {
            lastToggle = now;
            ledState = !ledState;
            digitalWrite(LED_PIN, ledState);
        }
        yield();
    }
}
