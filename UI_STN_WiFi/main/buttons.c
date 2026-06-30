#include "buttons.h"
#include "display.h"
#include "hardware.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

#define BUTTON_DEBOUNCE_MS 40U
#define BUTTON_REPEAT_START_MS 600U
#define BUTTON_REPEAT_INTERVAL_MS 150U

#define DEFAULT_MENU_SAVE_HOLD_MS 2000U
#define DEFAULT_POWER_MODE_HOLD_MS 2000U

static const char *TAG = "buttons";

static const char *button_name(button_id_t id)
{
    switch (id) {
    case BTN_UP: return "BTN_UP";
    case BTN_LIGHT: return "BTN_LIGHT";
    case BTN_POWER_MODE: return "BTN_POWER_MODE";
    case BTN_MENU_SAVE: return "BTN_MENU_SAVE";
    case BTN_DOWN: return "BTN_DOWN";
    default: return "BTN_UNKNOWN";
    }
}
typedef struct {
    gpio_num_t gpio;
    bool raw_pressed;
    bool stable_pressed;
    bool short_event;
    bool long_event;
    bool long_reported;
    bool step_event;
    uint32_t raw_changed_ms;
    uint32_t pressed_at_ms;
    uint32_t last_repeat_ms;
} button_state_t;

static button_state_t s_buttons[BUTTON_COUNT] = {
    [BTN_UP] = {.gpio = S1_TOP_GPIO},
    [BTN_LIGHT] = {.gpio = S2_LEFT_GPIO},
    [BTN_POWER_MODE] = {.gpio = S3_RIGHT_GPIO},
    [BTN_MENU_SAVE] = {.gpio = S4_CENTER_GPIO},
    [BTN_DOWN] = {.gpio = S5_BOTTOM_GPIO},
};

static uint32_t s_menu_save_hold_ms = DEFAULT_MENU_SAVE_HOLD_MS;
static uint32_t s_power_mode_hold_ms = DEFAULT_POWER_MODE_HOLD_MS;

static uint32_t hold_time_for(button_id_t id)
{
    if (id == BTN_MENU_SAVE) return s_menu_save_hold_ms;
    if (id == BTN_POWER_MODE) return s_power_mode_hold_ms;
    return 0;
}

esp_err_t buttons_init(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < BUTTON_COUNT; i++) {
        mask |= (1ULL << s_buttons[i].gpio);
    }

    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    uint32_t now = hardware_millis();
    for (int i = 0; i < BUTTON_COUNT; i++) {
        bool pressed = gpio_get_level(s_buttons[i].gpio) == 0;
        s_buttons[i].raw_pressed = pressed;
        s_buttons[i].stable_pressed = pressed;
        s_buttons[i].raw_changed_ms = now;
        s_buttons[i].pressed_at_ms = now;
        s_buttons[i].last_repeat_ms = now;
    }
    return ESP_OK;
}

void buttons_set_hold_times(uint32_t menu_save_hold_ms, uint32_t power_mode_hold_ms)
{
    s_menu_save_hold_ms = menu_save_hold_ms;
    s_power_mode_hold_ms = power_mode_hold_ms;
}

static void configure_shared_buttons_as_inputs(void)
{
    const uint64_t shared_mask =
        (1ULL << S1_TOP_GPIO) |
        (1ULL << S2_LEFT_GPIO) |
        (1ULL << S4_CENTER_GPIO) |
        (1ULL << S5_BOTTOM_GPIO);

    gpio_config_t cfg = {
        .pin_bit_mask = shared_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

static void read_raw_buttons(bool raw[BUTTON_COUNT])
{
    display_pause_for_button_scan();
    configure_shared_buttons_as_inputs();

    /* Let the weak pull-ups settle after releasing shared segment lines. */
    esp_rom_delay_us(20);

    raw[BTN_UP] = gpio_get_level(S1_TOP_GPIO) == 0;
    raw[BTN_LIGHT] = gpio_get_level(S2_LEFT_GPIO) == 0;
    raw[BTN_POWER_MODE] = gpio_get_level(S3_RIGHT_GPIO) == 0;
    raw[BTN_MENU_SAVE] = gpio_get_level(S4_CENTER_GPIO) == 0;
    raw[BTN_DOWN] = gpio_get_level(S5_BOTTOM_GPIO) == 0;

    display_resume_after_button_scan();
}

void buttons_update(uint32_t now_ms)
{
    bool raw_buttons[BUTTON_COUNT];
    read_raw_buttons(raw_buttons);

    for (button_id_t id = 0; id < BUTTON_COUNT; id++) {
        button_state_t *button = &s_buttons[id];
        button->short_event = false;
        button->long_event = false;
        button->step_event = false;

        bool raw = raw_buttons[id];
        if (raw != button->raw_pressed) {
            button->raw_pressed = raw;
            button->raw_changed_ms = now_ms;
        }

        if ((uint32_t)(now_ms - button->raw_changed_ms) >= BUTTON_DEBOUNCE_MS && button->stable_pressed != button->raw_pressed) {
            button->stable_pressed = button->raw_pressed;
            if (button->stable_pressed) {
                button->pressed_at_ms = now_ms;
                button->last_repeat_ms = now_ms;
                button->long_reported = false;
                ESP_LOGI(TAG, "%s", button_name(id));
                if (id == BTN_DOWN || id == BTN_UP) {
                    button->step_event = true;
                }
            } else {
                if (!button->long_reported && id != BTN_DOWN && id != BTN_UP) {
                    button->short_event = true;
                }
            }
        }

        uint32_t hold_ms = hold_time_for(id);
        if (hold_ms > 0 && button->stable_pressed && !button->long_reported && (uint32_t)(now_ms - button->pressed_at_ms) >= hold_ms) {
            button->long_reported = true;
            button->long_event = true;
        }

        if ((id == BTN_DOWN || id == BTN_UP) && button->stable_pressed) {
            if ((uint32_t)(now_ms - button->pressed_at_ms) >= BUTTON_REPEAT_START_MS &&
                (uint32_t)(now_ms - button->last_repeat_ms) >= BUTTON_REPEAT_INTERVAL_MS) {
                button->last_repeat_ms = now_ms;
                button->step_event = true;
            }
        }
    }
}

bool buttons_is_pressed(button_id_t id)
{
    return id < BUTTON_COUNT && s_buttons[id].stable_pressed;
}

bool buttons_consume_short(button_id_t id)
{
    if (id >= BUTTON_COUNT || !s_buttons[id].short_event) return false;
    s_buttons[id].short_event = false;
    return true;
}

bool buttons_consume_long(button_id_t id)
{
    if (id >= BUTTON_COUNT || !s_buttons[id].long_event) return false;
    s_buttons[id].long_event = false;
    return true;
}

bool buttons_consume_step(button_id_t id)
{
    if (id >= BUTTON_COUNT || !s_buttons[id].step_event) return false;
    s_buttons[id].step_event = false;
    return true;
}

