#include "global.h"

static const char *TAG_GPS = "GPS";

/* GPS CONFIGURATION */
#define UBX_SYNC1       0xB5
#define UBX_SYNC2       0x62
#define UBX_MAX_PAYLOAD 256
#define UBX_HDR_LEN     4 // class, id, len_l, len_h
#define UBX_CK_LEN      2

#define UBX_ACK_TIMEOUT_MS 1500
#define UBX_ACK_RETRIES    3

#define UBX_POSLLH BIT(0)
#define UBX_VELNED BIT(1)
#define UBX_STATUS BIT(2)
#define HAVE_ALL   (UBX_STATUS | UBX_VELNED | UBX_POSLLH)

#define CAL_MIN_SAMPLES 20
#define CAL_MAX_SAMPLES 200
#define CAL_VACC_MAX_MM 5000

static bool streaming = false;

/* GPS STRUCTURES */
typedef enum {
    ACK_NONE,
    ACK_OK,
    ACK_NAK,
} ubx_ack_t;

typedef struct {
    uint8_t class;
    uint8_t id;
    uint8_t payload[UBX_MAX_PAYLOAD];
    uint8_t len;
} ubx_cmd_t;

static struct {
    uint8_t   class;
    uint8_t   id;
    ubx_ack_t response;
} ack_state;

static struct {
    uint8_t  numSV;         // Satellites used
    uint16_t pDOP;          // x0.01
    bool     valid;         // Connection established
    char     swVersion[30]; // Software Version
    char     hwVersion[10]; // Hardware Version
} gps_info;

static struct {
    uint32_t itow;   // GPS Millisecond Time of Week (ms)
    uint8_t  have;   // Messages received this epoch
    bool     fix_ok; // Valid data
} epoch;

static struct {
    float    hmsl_m;  // Height above mean sea level (m)
    uint32_t vacc_mm; // Vertical Accuracy Estimate (mm)
    float    mean;    // Calibrated altitude mean (m)
    uint32_t cnt;     // Calibration count
} alt_cal;

/* Selected configuration messages */
static const ubx_cmd_t gps_config[] = {
    // CFG-MSG: disable NMEA sentences (class 0xF0, id, rate 0)
    {0x06, 0x01, {0xF0, 0x00, 0x00}, 3}, // GGA off
    {0x06, 0x01, {0xF0, 0x01, 0x00}, 3}, // GLL off
    {0x06, 0x01, {0xF0, 0x02, 0x00}, 3}, // GSA off
    {0x06, 0x01, {0xF0, 0x03, 0x00}, 3}, // GSV off
    {0x06, 0x01, {0xF0, 0x04, 0x00}, 3}, // RMC off
    {0x06, 0x01, {0xF0, 0x05, 0x00}, 3}, // VTG off

    // CFG-MSG: enable UBX messages
    {0x06, 0x01, {0x01, 0x02, 0x01}, 3}, // NAV-POSLLH on (lat, lon, hMSL, hAcc, vAcc)
    {0x06, 0x01, {0x01, 0x12, 0x01}, 3}, // NAV-VELNED on (velD, sAcc)
    {0x06, 0x01, {0x01, 0x03, 0x01}, 3}, // NAV-STATUS on (gpsFix, gpsFixOK)
    {0x06, 0x01, {0x01, 0x06, 0x05}, 3}, // NAV-SOL on - 1/5 epoch (numSV, pDOP)

    // CFG-RATE: 200 ms, navRate 1, GPS time
    {0x06, 0x08, {GPS_SAMPLE_RATE_MS & 0xFF, GPS_SAMPLE_RATE_MS >> 8, 0x01, 0x00, 0x01, 0x00}, 6},

    // CFG-NAV5: dynModel only, Pedestrian (3)
    {0x06, 0x24, {0x01, 0x00, 0x03}, 36},

    // CFG-NAV5: dynModel only, Airborne <4g (8)
    // {0x06, 0x24, {0x01, 0x00, 0x08}, 36},
};

static inline bool altitude_calibration(void) {
    bool idle = atomic_load(&data_g.flight_state) == STATE_IDLE;

    if (idle && alt_cal.cnt < CAL_MAX_SAMPLES && alt_cal.vacc_mm < CAL_VACC_MAX_MM) {
        alt_cal.cnt++;
        alt_cal.mean += (alt_cal.hmsl_m - alt_cal.mean) / alt_cal.cnt;
    }

    return alt_cal.cnt >= CAL_MIN_SAMPLES;
}

static inline void ubx_epoch_touch(uint32_t itow) {
    if (itow != epoch.itow) {
        epoch.itow   = itow;
        epoch.have   = 0;
        epoch.fix_ok = false;
    }
}

static inline void ubx_epoch_post(uint8_t bit, gps_sample_t *gps) {
    epoch.have |= bit;
    if (epoch.have != HAVE_ALL || !streaming)
        return;

    epoch.have = 0;

    if (!epoch.fix_ok) {
        portENTER_CRITICAL(&xGPSMutex);
        gps_sample_g.fix = 0;
        portEXIT_CRITICAL(&xGPSMutex);
        return;
    }

    if (!altitude_calibration())
        return;

    gps->altitude = alt_cal.hmsl_m - alt_cal.mean;

    portENTER_CRITICAL(&xGPSMutex);
    gps_sample_g.latitude     = gps->latitude;
    gps_sample_g.longitude    = gps->longitude;
    gps_sample_g.altitude     = gps->altitude;
    gps_sample_g.vel_vertical = gps->vel_vertical;
    gps_sample_g.sAcc         = gps->sAcc;
    gps_sample_g.fix          = gps->fix;
    portEXIT_CRITICAL(&xGPSMutex);

    xTaskNotify(xTaskAcquire, GPS_BIT, eSetBits);
}

static inline void ubx_checksum(const uint8_t *frame, uint16_t len, uint8_t *ck_a, uint8_t *ck_b) {
    uint8_t a_sum = 0, b_sum = 0;

    for (int i = 0; i < len; i++) {
        a_sum += frame[i];
        b_sum += a_sum;
    }

    *ck_a = a_sum;
    *ck_b = b_sum;
}

static void ubx_handle_message(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t len, gps_sample_t *gps) {
    /* ACK / NAK */
    if (class == 0x05 && len == 2 && (id == 0x01 || id == 0x00)) {
        if (payload[0] == ack_state.class && payload[1] == ack_state.id)
            ack_state.response = (id == 0x01) ? ACK_OK : ACK_NAK;

        return;
    }

    /* NAV-POSLLH */
    if (class == 0x01 && id == 0x02 && len == 28) {
        uint32_t iTOW; // GPS Millisecond Time of Week (ms)
        int32_t  lon;  // Longitude (deg * 1e-7)
        int32_t  lat;  // Latitude (deg * 1e-7)
        int32_t  hMSL; // Height above mean sea level (mm)
        uint32_t vAcc; // Vertical Accuracy Estimate (mm)

        memcpy(&iTOW, &payload[0], sizeof(iTOW));
        memcpy(&lon, &payload[4], sizeof(lon));
        memcpy(&lat, &payload[8], sizeof(lat));
        memcpy(&hMSL, &payload[16], sizeof(hMSL));
        memcpy(&vAcc, &payload[24], sizeof(vAcc));

        alt_cal.hmsl_m  = hMSL * 0.001f;
        alt_cal.vacc_mm = vAcc;

        gps->longitude = lon; // + = East
        gps->latitude  = lat; // + = North

        ubx_epoch_touch(iTOW);
        ubx_epoch_post(UBX_POSLLH, gps);

        return;
    }

    /* NAV-VELNED */
    if (class == 0x01 && id == 0x12 && len == 36) {
        uint32_t iTOW; // GPS Millisecond Time of Week (ms)
        int32_t  velD; // NED down velocity (cm/s)
        uint32_t sAcc; // Speed Accuracy Estimate (cm/s)

        memcpy(&iTOW, &payload[0], sizeof(iTOW));
        memcpy(&velD, &payload[12], sizeof(velD));
        memcpy(&sAcc, &payload[28], sizeof(sAcc));

        if (sAcc > 2550) // Constraint for uint8_t storage (25.5 m/s * 10)
            sAcc = 2550;

        gps->vel_vertical = -(velD * 0.01f);              // m/s (up positive)
        gps->sAcc         = (uint8_t)roundf(sAcc * 0.1f); // m/s * 10

        ubx_epoch_touch(iTOW);
        ubx_epoch_post(UBX_VELNED, gps);

        return;
    }

    /* NAV-STATUS */
    if (class == 0x01 && id == 0x03 && len == 16) {
        uint32_t iTOW;                // GPS Millisecond Time of Week (ms)
        uint8_t  gpsFix = payload[4]; // GPSfix Type
        uint8_t  flags  = payload[5]; // Navigation Status Flags

        memcpy(&iTOW, &payload[0], sizeof(iTOW));

        bool fixOk = (flags & 0x01) != 0; // gpsFixOk
        gps->fix   = fixOk ? gpsFix : 0;

        ubx_epoch_touch(iTOW);
        epoch.fix_ok = fixOk && gpsFix == 3;
        ubx_epoch_post(UBX_STATUS, gps);

        return;
    }

    /* NAV-SOL */
    if (class == 0x01 && id == 0x06 && len == 52) {
        uint16_t pDOP;  // Position DOP
        uint8_t  numSV; // Number of SVs used in Nav Solution

        memcpy(&pDOP, &payload[44], sizeof(pDOP));
        numSV = payload[47];

        gps_info.numSV = numSV;
        gps_info.pDOP  = pDOP;
        gps_info.valid = true;

        return;
    }

    /* MON-VER */
    if (class == 0x0A && id == 0x04 && len >= 40) {
        memcpy(gps_info.swVersion, &payload[0], 30);
        memcpy(gps_info.hwVersion, &payload[30], 10);

        ESP_LOGI(TAG_GPS, "Software: %s, Hardware: %s", gps_info.swVersion, gps_info.hwVersion);
        return;
    }
}

static void ubx_parse_message(uint8_t c, gps_sample_t *gps) {
    static enum { SYNC, HEADER, BODY } state = SYNC;

    static uint16_t sync;
    static uint8_t  frame[UBX_HDR_LEN + UBX_MAX_PAYLOAD + UBX_CK_LEN]; // GPS message buffer
    static uint16_t len, ubx_pos;
    static uint8_t  checksum[2];

    switch (state) {
    /* Check sync char 1-2 */
    case SYNC:
        sync = (sync << 8) | c;
        if (sync == (UBX_SYNC1 << 8 | UBX_SYNC2)) {
            ubx_pos = 0;
            state   = HEADER;
        }
        break;

    /* Check message header */
    case HEADER:
        frame[ubx_pos++] = c;
        if (ubx_pos == UBX_HDR_LEN) {
            len = frame[2] | (frame[3] << 8);
            if (len > UBX_MAX_PAYLOAD) {
                state = SYNC;
                break;
            }
            state = BODY;
        }
        break;

    /* Check message payload and checksum */
    case BODY:
        frame[ubx_pos++] = c;
        if (ubx_pos == UBX_HDR_LEN + len + UBX_CK_LEN) {
            ubx_checksum(frame, UBX_HDR_LEN + len, &checksum[0], &checksum[1]);
            if (checksum[0] == frame[ubx_pos - 2] && checksum[1] == frame[ubx_pos - 1])
                ubx_handle_message(frame[0], frame[1], &frame[UBX_HDR_LEN], len, gps);
            state = SYNC;
        }
        break;
    }
}

// Generic message helper
static inline void ubx_send(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t len) {
    if (len > UBX_MAX_PAYLOAD)
        return;

    uint8_t frame[2 + UBX_HDR_LEN + UBX_MAX_PAYLOAD + UBX_CK_LEN];

    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = class;
    frame[3] = id;
    frame[4] = len & 0xFF; // len_l
    frame[5] = len >> 8;   // len_h
    memcpy(&frame[6], payload, len);
    ubx_checksum(&frame[2], UBX_HDR_LEN + len, &frame[6 + len], &frame[7 + len]);
    uart_write_bytes(GPS_UART_NUM, frame, 8 + len);
}

// Send configuration message and checks ACK messages
static bool ubx_send_wait_ack(uint8_t class, uint8_t id, const uint8_t *payload, uint16_t len) {
    gps_sample_t dummy = {0};
    uint8_t      buf[GPS_RX_CHUNK];

    for (int attempt = 0; attempt < UBX_ACK_RETRIES; attempt++) {
        ack_state.class    = class;
        ack_state.id       = id;
        ack_state.response = ACK_NONE;

        uart_flush_input(GPS_UART_NUM);    // drop stale bytes
        ubx_send(class, id, payload, len); // send configuration message

        TickType_t start = xTaskGetTickCount();
        while (ack_state.response == ACK_NONE && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(UBX_ACK_TIMEOUT_MS)) {
            int n = uart_read_bytes(GPS_UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(20));
            for (int i = 0; i < n; i++)
                ubx_parse_message(buf[i], &dummy);
        }

        if (ack_state.response == ACK_OK) {
            ESP_LOGD(TAG_GPS, "ACK 0x%02X 0x%02X after %lu ms", class, id, (uint32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS));
            return true;
        }

        if (ack_state.response == ACK_NAK) {
            ESP_LOGE(TAG_GPS, "NAK for 0x%02X 0x%02X", class, id);
            return false; // rejected
        }

        ESP_LOGW(TAG_GPS, "No ACK for 0x%02X 0x%02X (attempt %d/%d)", class, id, attempt + 1, UBX_ACK_RETRIES);
    }

    return false;
}

/* ========================================== GPS ========================================== */
// Go trough selected configuration messages in 'gps_config[]'
static bool gps_configure(void) {
    bool ok = true;
    for (size_t i = 0; i < sizeof(gps_config) / sizeof(gps_config[0]); i++) {
        const ubx_cmd_t *c = &gps_config[i];
        if (!ubx_send_wait_ack(c->class, c->id, c->payload, c->len)) {
            ESP_LOGE(TAG_GPS, "Config %lu failed (0x%02X 0x%02X)", (uint32_t)i, c->class, c->id);
            ok = false;
        }
    }
    return ok;
}

static void gps_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = GPS_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_RX, GPS_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUFF_SIZE, 0, 0, NULL, 0));

    ESP_LOGI(TAG_GPS, "GPS UART initialized (baud %d)", GPS_BAUDRATE);
    vTaskDelay(pdMS_TO_TICKS(200)); // Wait for GPS to stabilize

    if (!gps_configure())
        ESP_LOGE(TAG_GPS, "GPS configuration failed");
    else
        ESP_LOGI(TAG_GPS, "GPS configuration succeeded");

    // Reset local variables
    epoch     = (typeof(epoch)){0};
    alt_cal   = (typeof(alt_cal)){0};
    streaming = true;
}

void task_gps(void *pvParameters) {
    gps_init();

    uint8_t      rx_buffer[GPS_RX_CHUNK];
    gps_sample_t gps = {0};

    // Get GPS Software / Hardware Version
    ubx_send(0x0A, 0x04, NULL, 0);

    while (true) {
        // Read bytes from UART in chunks. The parsers handle partial data.
        int rx_len = uart_read_bytes(GPS_UART_NUM, rx_buffer, GPS_RX_CHUNK, pdMS_TO_TICKS(20));
        for (int i = 0; i < rx_len; ++i) {
            ubx_parse_message(rx_buffer[i], &gps);
        }

        static TickType_t last_log;
        if (gps_info.valid && xTaskGetTickCount() - last_log >= pdMS_TO_TICKS(5000)) {
            ESP_LOGI(TAG_GPS, "sats=%u pdop=%.2f fix=%u vAcc=%lu cal=%lu/%d", gps_info.numSV, gps_info.pDOP * 0.01f, gps_sample_g.fix,
                     (unsigned long)alt_cal.vacc_mm, (unsigned long)alt_cal.cnt, CAL_MIN_SAMPLES);
            last_log = xTaskGetTickCount();
        }
    }
}