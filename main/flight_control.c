#include "global.h"

static const char *TAG_FLIGHT = "Flight";

#define STATE_CHECK_MS 50  // Delay between state checks
#define PYRO_PULSE_MS  500 // Pulse duration for pyro ignition

#define TRANSITION_THRESHOLD_MS 150 // Time threshold for state transitions in ms (e.g. boost to coast, deploy drogue, etc.)

#define BOOST_THRESHOLD_ACC  (2 * G) // Vertical acceleration threshold to detect boost phase in m/s²
#define COAST_THRESHOLD_ACC  (2 * G) // Vertical acceleration threshold to enter coast phase in m/s²
#define DROGUE_THRESHOLD_VEL 0.5f    // Vertical velocity threshold to deploy drogue in m/s
#define MAIN_THRESHOLD_ALT   450     // Altitude above initial to deploy main in meters
#define LANDING_THRESHOLD_S  10      // Time threshold to prepare for landing in seconds

static uint32_t db_boost, db_coast, db_drogue, db_main, db_landing, db_landed = 0;

// Transition to new flight state and log
static void state_transition_to(flight_state_t new_flight_state) {
    atomic_store(&data_g.flight_state, new_flight_state);

    ESP_LOGW(TAG_FLIGHT, "New flight state: %d", new_flight_state);
}

// Check data credibility before applying state transition
static bool debounce_check(uint32_t current_time, uint32_t *debounce_time) {
    if (*debounce_time == 0)
        *debounce_time = current_time;

    return (current_time - *debounce_time > TRANSITION_THRESHOLD_MS);
}

// Sets channel to HIGH and starts timer before returning to LOW
static void fire_pyro_channel(pyro_channel_t *ch, uint32_t pulse_ms) {
    gpio_set_level(ch->gpio, HIGH);
    ESP_LOGW(TAG_FLIGHT, "%s deployed", ch->label);
    esp_timer_start_once(ch->timer, (uint64_t)pulse_ms * 1000ULL);
}

// Check RBF state and ARM/DISARM system
static void rbf_check() {
    bool is_armed    = (atomic_load(&sys_flags_g) & STATUS_ARMED) != 0;
    bool is_rbf_high = gpio_get_level(RBF_GPIO);

    // Check if the system should change ARMED status
    if (is_armed == is_rbf_high)
        return;

    if (is_rbf_high) {
        atomic_fetch_or(&sys_flags_g, STATUS_ARMED);

        ESP_LOGW(TAG_FLIGHT, "Arming system");
        for (uint8_t i = 0; i < 3; i++) {
            gpio_set_level(LED_GPIO, HIGH);
            gpio_set_level(BUZZER_GPIO, HIGH);
            vTaskDelay(pdMS_TO_TICKS(150));
            gpio_set_level(LED_GPIO, LOW);
            gpio_set_level(BUZZER_GPIO, LOW);
            vTaskDelay(pdMS_TO_TICKS(150));
        }

    } else {
        atomic_fetch_and(&sys_flags_g, ~STATUS_ARMED);

        ESP_LOGW(TAG_FLIGHT, "Disarming system");
        gpio_set_level(LED_GPIO, HIGH);
        gpio_set_level(BUZZER_GPIO, HIGH);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_GPIO, LOW);
        gpio_set_level(BUZZER_GPIO, LOW);
    }

    // ATUALIZAR PARA USAR A TASK BUZZER-LED PARA ESSAS INTERAÇÕES ============================
}

/* ========================================== BOOST ========================================== */
static void boost_check(float vertical_accel, float vertical_vel, uint32_t current_time) {
    if (vertical_accel > BOOST_THRESHOLD_ACC && vertical_vel > 0.0f) {
        if (debounce_check(current_time, &db_boost))
            state_transition_to(STATE_BOOST);
    } else
        db_boost = 0;
}

/* ========================================== COAST ========================================== */
static void coast_check(float vertical_accel, uint32_t current_time) {
    if (vertical_accel < COAST_THRESHOLD_ACC) {
        if (debounce_check(current_time, &db_coast))
            state_transition_to(STATE_COAST);
    } else
        db_coast = 0;
}

/* ========================================== DROGUE ========================================== */
static void drogue_check(float vertical_vel, float current_alt, float apogee, uint32_t current_time) {
    bool is_armed           = (atomic_load(&sys_flags_g) & STATUS_ARMED) != 0;
    bool is_drogue_deployed = (atomic_load(&sys_flags_g) & STATUS_DROGUE_DEPLOYED) != 0;

    if (!is_armed || is_drogue_deployed)
        return; // desabilitar deployment

    // Checks if absolute vertical velocity is less than Drogue activation threshold
    if (fabsf(vertical_vel) < DROGUE_THRESHOLD_VEL) {
        // If data holds, checks current altitude relative to apogee and fire pyro
        if (debounce_check(current_time, &db_drogue) && current_alt < apogee) {
            fire_pyro_channel(&pyro_drogue, PYRO_PULSE_MS);
            state_transition_to(STATE_DROGUE);
            atomic_fetch_or(&sys_flags_g, STATUS_DROGUE_DEPLOYED);
        }
    } else
        db_drogue = 0;
}

/* ========================================== MAIN ========================================== */
static void main_check(float vertical_vel, float current_alt, uint32_t current_time) {
    bool is_armed         = (atomic_load(&sys_flags_g) & STATUS_ARMED) != 0;
    bool is_main_deployed = (atomic_load(&sys_flags_g) & STATUS_MAIN_DEPLOYED) != 0;

    if (!is_armed || is_main_deployed)
        return; // desabilitar deployment

    // Checks if vertical velocity is negative (descending)
    if (vertical_vel < 0) {
        // If data holds, checks current altitude in comparison with Main activation threshold
        if (debounce_check(current_time, &db_main) && current_alt < MAIN_THRESHOLD_ALT) {
            fire_pyro_channel(&pyro_main, PYRO_PULSE_MS);
            state_transition_to(STATE_MAIN);
            atomic_fetch_or(&sys_flags_g, STATUS_MAIN_DEPLOYED);
        }
    } else
        db_main = 0;
}

/* ========================================== LANDING ========================================== */
static void landing_check(float vertical_vel, float current_alt, uint32_t current_time) {
    if (vertical_vel < -1.0f && current_alt / -vertical_vel < LANDING_THRESHOLD_S) {
        if (debounce_check(current_time, &db_landing)) {
            state_transition_to(STATE_LANDING);
            // Notify?
        }
    } else
        db_landing = 0;
}

/* ========================================== LANDED ========================================== */
static void landed_check(float vertical_vel, float acceleration, uint32_t current_time) {
    if (fabsf(vertical_vel) < 0.5f && acceleration < 2.0f) {
        if (debounce_check(current_time, &db_landed)) {
            state_transition_to(STATE_LANDED);
        }
    } else
        db_landed = 0;
}

void task_flight_control(void *pvParameters) {
    flight_state_t current_state;
    uint32_t       current_time;

    while (true) {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(STATE_CHECK_MS));

        portENTER_CRITICAL(&xDATALock);
        current_state = data_g.flight_state;
        current_time  = data_g.time;
        portEXIT_CRITICAL(&xDATALock);

        switch (current_state) {
        case STATE_IDLE: {
            rbf_check();

            portENTER_CRITICAL(&xDATALock);
            float vertical_accel = data_g.icm.az;
            float vertical_vel   = data_g.kf.x.vz;
            portEXIT_CRITICAL(&xDATALock);

            boost_check(vertical_accel, vertical_vel, current_time);
            break;
        }

        case STATE_BOOST: {
            portENTER_CRITICAL(&xDATALock);
            float vertical_accel = data_g.icm.az;
            portEXIT_CRITICAL(&xDATALock);

            coast_check(vertical_accel, current_time);
            break;
        }

        case STATE_COAST: {
            portENTER_CRITICAL(&xDATALock);
            float vertical_vel = data_g.kf.x.vz;
            float current_alt  = data_g.kf.x.h;
            float apogee       = data_g.kf.x.apogee;
            portEXIT_CRITICAL(&xDATALock);

            drogue_check(vertical_vel, current_alt, apogee, current_time);
            break;
        }

        case STATE_DROGUE: {
            portENTER_CRITICAL(&xDATALock);
            float vertical_vel = data_g.kf.x.vz;
            float current_alt  = data_g.kf.x.h;
            portEXIT_CRITICAL(&xDATALock);

            main_check(vertical_vel, current_alt, current_time);
            break;
        }

        case STATE_MAIN: {
            portENTER_CRITICAL(&xDATALock);
            float vertical_vel = data_g.kf.x.vz;
            float current_alt  = data_g.kf.x.h;
            portEXIT_CRITICAL(&xDATALock);

            landing_check(vertical_vel, current_alt, current_time);
            break;
        }

        case STATE_LANDING: {
            portENTER_CRITICAL(&xDATALock);
            float vertical_vel = data_g.kf.x.vz;
            float acceleration = data_g.icm.accel;
            portEXIT_CRITICAL(&xDATALock);

            landed_check(vertical_vel, acceleration, current_time);
            break;
        }

        case STATE_LANDED: {

            break;
        }

        case STATE_FATAL: {
            ESP_LOGE(TAG_FLIGHT, "Fatal Error detected");

            break;
        }

        default:
            ESP_LOGW(TAG_FLIGHT, "Unknown state: %d", current_state);
            break;
        }
    }
}