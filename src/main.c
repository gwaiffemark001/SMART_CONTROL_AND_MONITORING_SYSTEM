#include <ctype.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"

#define ROOM_COUNT 2
#define LOG_CAPACITY 24
#define UID_LENGTH 31
#define MAX_OCCUPANTS 64
#define LEGACY_MAX_OCCUPANTS 16
#define CLOSING_WARNING_MIN 10
#define NODE_MESSAGE_VERSION 1
#define LOGIN_MAX_FAILURES 3
#define LOGIN_LOCKOUT_MS 30000
#define MESSAGE_SHOW_MS 3000
#define MOTION_HOLD_MS 10000
#define ADMIN_PIN "2468"
#define DOOR_OPEN_MS 1200
#define EMERGENCY_EGRESS_PIN GPIO_NUM_25
#define EMERGENCY_ALARM_PIN GPIO_NUM_15
#define RTC_I2C_ADDRESS 0x68
#define RTC_SDA_PIN GPIO_NUM_21
#define RTC_SCL_PIN GPIO_NUM_22
#define SPI_MISO_PIN GPIO_NUM_19
#define SPI_MOSI_PIN GPIO_NUM_23
#define SPI_CLOCK_PIN GPIO_NUM_18
#define ENTRY_RFID_CS_PIN GPIO_NUM_32
#define EXIT_RFID_CS_PIN GPIO_NUM_33
#define SERVO_ENTRY_PIN GPIO_NUM_13
#define SERVO_EXIT_PIN GPIO_NUM_14
#define SERVO_OPEN_ANGLE 90
#define SERVO_CLOSED_ANGLE 0
#define SD_CS_PIN GPIO_NUM_5
#define SD_MOUNT_PATH "/sdcard"
#define SD_LOG_PATH "/sdcard/access.csv"
#define LCD_I2C_ADDRESS 0x27
#define KEYPAD_I2C_ADDRESS 0x20
#define LCD_BACKLIGHT_BIT 0x08
#define MCP23017_IODIRA_REG 0x00
#define MCP23017_GPPUA_REG 0x0C
#define MCP23017_GPIOA_REG 0x12

static const gpio_num_t pir_pins[ROOM_COUNT] = { GPIO_NUM_27, GPIO_NUM_26 };
static const gpio_num_t light_pins[ROOM_COUNT] = { GPIO_NUM_2, GPIO_NUM_4 };
static const gpio_num_t door_pins[2] = { GPIO_NUM_16, GPIO_NUM_17 };

typedef struct {
    uint16_t minute_of_day;
    char uid[UID_LENGTH];
    uint8_t entering;
} access_record_t;

typedef struct {
    uint32_t next;
    uint32_t count;
    access_record_t records[LOG_CAPACITY];
} access_log_t;

typedef struct {
    uint16_t open_minute;
    uint16_t close_minute;
} schedule_t;

typedef enum {
    PANEL_HOME,
    PANEL_PIN,
    PANEL_MENU,
    PANEL_OPEN_TIME,
    PANEL_CLOSE_TIME,
    PANEL_CLOCK,
    PANEL_STATUS,
} panel_screen_t;

/* Message sent by a room terminal to the entrance terminal. */
typedef struct {
    uint8_t version;
    uint8_t room;
    uint8_t occupied;
    uint8_t light_on;
    uint32_t sequence;
} room_message_t;

typedef struct {
    uint32_t count;
    char uids[MAX_OCCUPANTS][UID_LENGTH];
} occupant_snapshot_t;

typedef struct {
    uint32_t count;
    char uids[LEGACY_MAX_OCCUPANTS][UID_LENGTH];
} legacy_occupant_snapshot_t;

typedef struct {
    bool light_on[ROOM_COUNT];
    bool room_occupied[ROOM_COUNT];
    bool exit_pulse_active;
    uint16_t door_pulse_count[2];
    TickType_t last_motion[ROOM_COUNT];
    uint32_t room_sequence[ROOM_COUNT];
    TickType_t room_last_seen[ROOM_COUNT];
    char inside[MAX_OCCUPANTS][UID_LENGTH];
    size_t inside_count;
    bool admin_authenticated;
    TickType_t auth_expires;
    uint16_t clock_minute;
    TickType_t clock_tick;
    schedule_t schedule;
    access_log_t log;
} system_state_t;

static system_state_t state;
static SemaphoreHandle_t state_mutex;
static nvs_handle_t nvs_store;
static i2c_master_bus_handle_t i2c_bus;
static i2c_master_dev_handle_t rtc_device;
static i2c_master_dev_handle_t lcd_device;
static i2c_master_dev_handle_t keypad_expander_device;
static bool rtc_available;
static bool lcd_available;
static bool keypad_available;
static bool emergency_alarm_available;
static bool sd_available;
static bool shared_spi_ready;
static bool rfid_reader_ready[2];
static spi_device_handle_t rfid_device[2];
static QueueHandle_t terminal_queue;
static SemaphoreHandle_t lcd_mutex;
static uint8_t failed_logins;
static TickType_t lockout_until;
static TickType_t panel_message_until;
static bool servo_ready;
static int door_last_level[2] = { -1, -1 };
/* Shared lock: the two readers and the console "scan" command take it so their
 * multi-step SPI sequences never interleave. Lock order: this one, then state_mutex. */
static SemaphoreHandle_t rfid_bus_task_mutex;
static const char *authorized_uids[] = { "A1B2C3D4", "DEADBEEF", "CAFEBABE" };

static void door_pulse_task(void *argument);
static uint16_t current_minute_locked(void);
static uint8_t bcd_to_binary(uint8_t value);
static uint8_t binary_to_bcd(uint8_t value);
static bool rtc_read_minute(uint16_t *minute);
static bool rtc_set_minute(uint16_t minute);
static void initialize_rtc(void);
static esp_err_t lcd_write_nibble(uint8_t nibble);
static esp_err_t lcd_write_byte(uint8_t value, bool data);
static size_t lcd_encode_byte(uint8_t value, bool data, uint8_t transfer[6]);
static void lcd_show(const char *first_line, const char *second_line);
static bool initialize_lcd(void);
static bool keypad_expander_write(uint8_t reg, uint8_t value);
static bool initialize_keypad(void);
static void initialize_admin_panel(void);
static void rtc_sync_task(void *argument);
static void minute_to_text(uint16_t minute, char text[6]);
static bool is_access_time_locked(void);
static bool parse_time(const char *text, uint16_t *minute);
static bool uid_is_authorized(const char *uid);
static int inside_index_locked(const char *uid);
static void persist_log_locked(void);
static void persist_occupants_locked(void);
static void append_sd_record_locked(const access_record_t *record);
static void add_access_record_locked(const char *uid, bool entering);
static void mount_sd_card(void);
static void lcd_message(const char *first_line, const char *second_line);
static void door_output(gpio_num_t pin, bool open);
static void initialize_servos(void);
static void entrance_link_task(void *argument);
static void send_room_status(size_t room, bool occupied, bool light_on);
static bool initialize_shared_spi_bus(void);
static void initialize_rfid_readers(void);
static void room_monitor_task(void *argument);
static void print_help(void);
static void print_status(void);
static void print_logs(void);
static void handle_scan(const char *direction, char *uid);
static bool require_admin(void);
static bool admin_session_active(void);
static bool authenticate_admin_pin(const char *pin);
static bool update_schedule(uint16_t open_minute, uint16_t close_minute);
static bool update_demo_clock(uint16_t minute);
static void handle_command(char *line);
static void run_serial_console(void);
static bool read_keypad_key(char *key);
static void panel_show_home(void);
static void panel_show_menu(void);
static void panel_show_status(void);
static bool keypad_time_to_minute(const char input[5], uint16_t *minute);
static void panel_show_time_prompt(const char *title, const char input[5], size_t length);
static void panel_show_pin(const char input[5], size_t length);
static void keypad_append_digit(char input[5], size_t *length, char key);
static void admin_panel_task(void *argument);
static void configure_gpio(void);
static void emergency_egress_task(void *argument);
static void initialize_emergency_alarm(void);
static void initialize_storage(void);
void app_main(void);

/* Shared door-output pulse used by accepted entry/exit decisions. */
typedef struct {
    gpio_num_t pin;
    const char *name;
    char uid[UID_LENGTH];
} door_pulse_args_t;

/* Each pulse owns its argument block; concurrent scans cannot overwrite it. */

/* Talituleka Hope Mwelu - RTC, timekeeping, and storage */
static uint16_t current_minute_locked(void)
{
    TickType_t elapsed = xTaskGetTickCount() - state.clock_tick;
    uint32_t elapsed_minutes = (uint32_t)(elapsed / pdMS_TO_TICKS(60000));
    return (uint16_t)((state.clock_minute + elapsed_minutes) % 1440);
}

static uint8_t bcd_to_binary(uint8_t value)
{
    return (uint8_t)(((value >> 4) * 10) + (value & 0x0f));
}

static uint8_t binary_to_bcd(uint8_t value)
{
    return (uint8_t)(((value / 10) << 4) | (value % 10));
}

static bool rtc_read_minute(uint16_t *minute)
{
    uint8_t register_address = 0;
    uint8_t registers[3];
    if (!rtc_available || i2c_master_transmit_receive(
            rtc_device, &register_address, 1, registers, sizeof(registers), 100) != ESP_OK) {
        return false;
    }
    uint8_t hours = registers[2];
    if ((hours & 0x40) != 0) {
        uint8_t hour12 = bcd_to_binary(hours & 0x1f);
        uint8_t hour24 = (uint8_t)(hour12 % 12 + ((hours & 0x20) ? 12 : 0));
        *minute = (uint16_t)(hour24 * 60 + bcd_to_binary(registers[1] & 0x7f));
    } else {
        *minute = (uint16_t)(bcd_to_binary(hours & 0x3f) * 60 +
                             bcd_to_binary(registers[1] & 0x7f));
    }
    return *minute < 1440;
}

static bool rtc_set_minute(uint16_t minute)
{
    if (!rtc_available) {
        return false;
    }
    uint8_t registers[] = {
        0,
        binary_to_bcd(0),
        binary_to_bcd((uint8_t)(minute % 60)),
        binary_to_bcd((uint8_t)(minute / 60)),
    };
    return i2c_master_transmit(rtc_device, registers, sizeof(registers), 100) == ESP_OK;
}

static void initialize_rtc(void)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = RTC_SDA_PIN,
        .scl_io_num = RTC_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    if (i2c_new_master_bus(&bus_config, &i2c_bus) != ESP_OK) {
        printf("RTC unavailable: using demonstration clock\n");
        return;
    }
    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = RTC_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    if (i2c_master_bus_add_device(i2c_bus, &device_config, &rtc_device) == ESP_OK) {
        uint8_t register_address = 0;
        uint8_t seconds;
        rtc_available = i2c_master_transmit_receive(
            rtc_device, &register_address, 1, &seconds, 1, 100) == ESP_OK;
        uint16_t rtc_minute;
        if (rtc_available && rtc_read_minute(&rtc_minute)) {
            state.clock_minute = rtc_minute;
            state.clock_tick = xTaskGetTickCount();
            printf("DS1307 RTC detected\n");
            return;
        }
    }
    rtc_available = false;
    printf("DS1307 RTC not detected: using demonstration clock\n");
}

static void rtc_sync_task(void *argument)
{
    (void)argument;
    while (true) {
        uint16_t rtc_minute;
        if (rtc_read_minute(&rtc_minute)) {
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            state.clock_minute = rtc_minute;
            state.clock_tick = xTaskGetTickCount();
            xSemaphoreGive(state_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

static void minute_to_text(uint16_t minute, char text[6])
{
    unsigned hour = (minute / 60) % 24;
    unsigned minutes = minute % 60;
    snprintf(text, 6, "%02u:%02u", hour, minutes);
}

static void persist_log_locked(void)
{
    if (nvs_set_blob(nvs_store, "access_log", &state.log, sizeof(state.log)) == ESP_OK) {
        nvs_commit(nvs_store);
    }
}

static void persist_occupants_locked(void)
{
    static occupant_snapshot_t snapshot; /* static: keeps ~2 KB off the task stack */
    snapshot.count = (uint32_t)state.inside_count;
    memcpy(snapshot.uids, state.inside, sizeof(snapshot.uids));
    esp_err_t result = nvs_set_blob(nvs_store, "occupants", &snapshot, sizeof(snapshot));
    if (result == ESP_OK) {
        result = nvs_commit(nvs_store);
    }
    if (result != ESP_OK) {
        printf("Occupant snapshot save failed: %s\n", esp_err_to_name(result));
    }
}

static void append_sd_record_locked(const access_record_t *record)
{
    if (!sd_available) {
        return;
    }
    FILE *file = fopen(SD_LOG_PATH, "a");
    if (file == NULL) {
        printf("SD log write failed; NVS backup retained\n");
        return;
    }
    char time_text[6];
    minute_to_text(record->minute_of_day, time_text);
    int written = fprintf(file, "%s,%s,%s\n", time_text,
                          record->entering ? "ENTRY" : "EXIT", record->uid);
    bool saved = written > 0 && fflush(file) == 0;
    fclose(file);
    if (!saved) {
        printf("SD log flush failed; NVS backup retained\n");
    }
}

static void add_access_record_locked(const char *uid, bool entering)
{
    access_record_t *record = &state.log.records[state.log.next];
    record->minute_of_day = current_minute_locked();
    snprintf(record->uid, sizeof(record->uid), "%s", uid);
    record->entering = entering;
    state.log.next = (state.log.next + 1) % LOG_CAPACITY;
    if (state.log.count < LOG_CAPACITY) {
        state.log.count++;
    }
    append_sd_record_locked(record);
    persist_log_locked();
}

static void mount_sd_card(void)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;
    host.max_freq_khz = 10000;
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = host.slot;
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.gpio_cd = SDSPI_SLOT_NO_CD;
    slot_config.gpio_wp = SDSPI_SLOT_NO_WP;
    esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 4,
        .allocation_unit_size = 16 * 1024,
    };
    sdmmc_card_t *card = NULL;
    esp_err_t result = esp_vfs_fat_sdspi_mount(
        SD_MOUNT_PATH, &host, &slot_config, &mount_config, &card);
    if (result != ESP_OK) {
        printf("SD unavailable (%s); access log will use NVS backup\n",
               esp_err_to_name(result));
        return;
    }

    sd_available = true;
    FILE *file = fopen(SD_LOG_PATH, "r");
    if (file == NULL) {
        file = fopen(SD_LOG_PATH, "w");
        if (file != NULL) {
            fputs("time,direction,uid\n", file);
            fclose(file);
        }
    } else {
        fclose(file);
    }
    printf("SD card mounted at %s\n", SD_MOUNT_PATH);
}

static void initialize_storage(void)
{
    esp_err_t result = nvs_flash_init();
    if (result == ESP_ERR_NVS_NO_FREE_PAGES || result == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        result = nvs_flash_init();
    }
    ESP_ERROR_CHECK(result);
    ESP_ERROR_CHECK(nvs_open("cocis", NVS_READWRITE, &nvs_store));

    size_t length = sizeof(state.log);
    if (nvs_get_blob(nvs_store, "access_log", &state.log, &length) != ESP_OK ||
        length != sizeof(state.log) || state.log.next >= LOG_CAPACITY ||
        state.log.count > LOG_CAPACITY) {
        memset(&state.log, 0, sizeof(state.log));
    }
    length = sizeof(state.schedule);
    if (nvs_get_blob(nvs_store, "schedule", &state.schedule, &length) != ESP_OK ||
        length != sizeof(state.schedule) ||
        state.schedule.open_minute >= 1440 || state.schedule.close_minute >= 1440 ||
        state.schedule.open_minute == state.schedule.close_minute) {
        state.schedule.open_minute = 7 * 60;
        state.schedule.close_minute = 18 * 60;
    }
    static occupant_snapshot_t snapshot;
    length = sizeof(snapshot);
    result = nvs_get_blob(nvs_store, "occupants", &snapshot, &length);
    if (result == ESP_OK && length == sizeof(snapshot) &&
        snapshot.count <= MAX_OCCUPANTS) {
        state.inside_count = snapshot.count;
        memcpy(state.inside, snapshot.uids, sizeof(state.inside));
    } else {
        static legacy_occupant_snapshot_t legacy_snapshot;
        length = sizeof(legacy_snapshot);
        result = nvs_get_blob(
            nvs_store, "occupants", &legacy_snapshot, &length);
        if (result == ESP_OK && length == sizeof(legacy_snapshot) &&
            legacy_snapshot.count <= LEGACY_MAX_OCCUPANTS) {
            state.inside_count = legacy_snapshot.count;
            memcpy(state.inside, legacy_snapshot.uids, sizeof(legacy_snapshot.uids));

            static occupant_snapshot_t migrated_snapshot;
            migrated_snapshot.count = legacy_snapshot.count;
            memcpy(migrated_snapshot.uids, state.inside, sizeof(migrated_snapshot.uids));
            result = nvs_set_blob(
                nvs_store, "occupants", &migrated_snapshot, sizeof(migrated_snapshot));
            if (result == ESP_OK) {
                result = nvs_commit(nvs_store);
            }
            if (result == ESP_OK) {
                printf("Migrated saved occupants to the %d-person capacity format\n",
                       MAX_OCCUPANTS);
            } else {
                printf("Occupant snapshot migration save failed: %s\n",
                       esp_err_to_name(result));
            }
        } else if (result != ESP_ERR_NVS_NOT_FOUND) {
            printf("Saved occupant snapshot is invalid or unreadable; starting empty\n");
        }
    }
}

/* Ogesa Patrick Paul - room sensing and automatic lighting.
 * Each room runs as its own terminal task. It keeps its own light state and
 * reports to the entrance terminal with room_message_t through a queue. */
static void send_room_status(size_t room, bool occupied, bool light_on)
{
    static uint32_t sequence[ROOM_COUNT];
    static bool queue_warning_reported[ROOM_COUNT];
    room_message_t message = {
        .version = NODE_MESSAGE_VERSION,
        .room = (uint8_t)room,
        .occupied = occupied ? 1U : 0U,
        .light_on = light_on ? 1U : 0U,
        .sequence = ++sequence[room],
    };
    if (terminal_queue == NULL) {
        if (!queue_warning_reported[room]) {
            printf("Room %u status was not sent: terminal message queue is unavailable\n",
                   (unsigned)(room + 1));
            queue_warning_reported[room] = true;
        }
        return;
    }
    if (xQueueSend(terminal_queue, &message, 0) != pdTRUE) {
        if (!queue_warning_reported[room]) {
            printf("Room %u status queue is full; latest update will be retried\n",
                   (unsigned)(room + 1));
            queue_warning_reported[room] = true;
        }
    } else {
        queue_warning_reported[room] = false;
    }
}

static void room_monitor_task(void *argument)
{
    const size_t room = (size_t)(uintptr_t)argument;
    bool light_on = false;
    TickType_t last_motion = 0;
    TickType_t last_report = 0;
    bool reported = false;
    while (true) {
        TickType_t now = xTaskGetTickCount();
        bool motion = gpio_get_level(pir_pins[room]) != 0;
        bool changed = false;
        if (motion) {
            last_motion = now;
            if (!light_on) {
                light_on = true;
                changed = true;
            }
        } else if (light_on && (now - last_motion) >= pdMS_TO_TICKS(MOTION_HOLD_MS)) {
            light_on = false;
            changed = true;
        }
        if (changed) {
            gpio_set_level(light_pins[room], light_on);
            printf("Room %u: occupancy %s, light %s\n", (unsigned)(room + 1),
                   light_on ? "detected" : "clear", light_on ? "ON" : "OFF");
        }
        if (changed || !reported || (now - last_report) >= pdMS_TO_TICKS(1000)) {
            send_room_status(room, light_on, light_on);
            last_report = now;
            reported = true;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void entrance_link_task(void *argument)
{
    (void)argument;
    room_message_t message;
    while (true) {
        if (xQueueReceive(terminal_queue, &message, pdMS_TO_TICKS(500)) != pdTRUE) {
            continue;
        }
        if (message.version != NODE_MESSAGE_VERSION || message.room >= ROOM_COUNT ||
            message.occupied > 1 || message.light_on > 1) {
            continue;
        }
        size_t room = message.room;
        bool changed = false;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        if (state.room_last_seen[room] == 0 ||
            (int32_t)(message.sequence - state.room_sequence[room]) > 0) {
            changed = state.room_occupied[room] != (message.occupied != 0) ||
                      state.light_on[room] != (message.light_on != 0);
            state.room_sequence[room] = message.sequence;
            state.room_occupied[room] = message.occupied != 0;
            state.light_on[room] = message.light_on != 0;
        }
        state.room_last_seen[room] = xTaskGetTickCount();
        xSemaphoreGive(state_mutex);
        if (changed) {
            printf("Entrance terminal: Room %u reports %s, light %s\n",
                   (unsigned)(room + 1), message.occupied ? "occupied" : "clear",
                   message.light_on ? "ON" : "OFF");
        }
    }
}

/* Mugabe Gideon - RFID access control and serial console */
static void door_pulse_task(void *argument)
{
    door_pulse_args_t *allocated_pulse = argument;
    door_pulse_args_t pulse = *allocated_pulse;
    free(allocated_pulse);

    const size_t door_index = pulse.pin == door_pins[0] ? 0U : 1U;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.door_pulse_count[door_index]++;
    if (door_index == 1U) {
        state.exit_pulse_active = true;
    }
    door_output(pulse.pin, true);
    xSemaphoreGive(state_mutex);

    printf("Door %s released for UID %s\n", pulse.name, pulse.uid);
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(DOOR_OPEN_MS));

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (state.door_pulse_count[door_index] > 0) {
        state.door_pulse_count[door_index]--;
    }
    if (door_index == 1U) {
        state.exit_pulse_active = state.door_pulse_count[door_index] > 0;
    }
    bool keep_door_open = state.door_pulse_count[door_index] > 0;
    bool emergency_pressed = gpio_get_level(EMERGENCY_EGRESS_PIN) == 0;
    if (!keep_door_open && !(door_index == 1U && emergency_pressed)) {
        door_output(pulse.pin, false);
    }
    xSemaphoreGive(state_mutex);
    vTaskDelete(NULL);
}

static bool is_access_time_locked(void)
{
    uint16_t now = current_minute_locked();
    uint16_t opens = state.schedule.open_minute;
    uint16_t closes = state.schedule.close_minute;

    if (opens == closes) {
        return false; /* An equal start/end time is treated as a closed schedule. */
    }
    if (opens < closes) {
        return now >= opens && now < closes;
    }
    /* Overnight window, for example 20:00-06:00. */
    return now >= opens || now < closes;
}

static bool parse_time(const char *text, uint16_t *minute)
{
    int hour;
    int minutes;
    char trailing;
    if (sscanf(text, "%d:%d%c", &hour, &minutes, &trailing) != 2 ||
        hour < 0 || hour > 23 || minutes < 0 || minutes > 59) {
        return false;
    }
    *minute = (uint16_t)(hour * 60 + minutes);
    return true;
}

static bool uid_is_authorized(const char *uid)
{
    for (size_t i = 0; i < sizeof(authorized_uids) / sizeof(authorized_uids[0]); i++) {
        if (strcmp(uid, authorized_uids[i]) == 0) {
            return true;
        }
    }
    return false;
}

static int inside_index_locked(const char *uid)
{
    for (size_t i = 0; i < state.inside_count; i++) {
        if (strcmp(state.inside[i], uid) == 0) {
            return (int)i;
        }
    }
    return -1;
}

/* Bring up the shared SPI bus and hold both MFRC522 CS pins inactive before
 * initializing the SD card. The SD card must enter SPI mode before the RC522
 * readers begin communicating on the same bus. */
static bool initialize_shared_spi_bus(void)
{
    spi_bus_config_t bus_config = {
        .miso_io_num = SPI_MISO_PIN,
        .mosi_io_num = SPI_MOSI_PIN,
        .sclk_io_num = SPI_CLOCK_PIN,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 4000,
    };
    esp_err_t result = spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (result != ESP_OK && result != ESP_ERR_INVALID_STATE) {
        printf("RFID/SD SPI bus initialization failed: %s\n", esp_err_to_name(result));
        return false;
    }

    const uint64_t chip_select_mask = (1ULL << ENTRY_RFID_CS_PIN) |
                                      (1ULL << EXIT_RFID_CS_PIN) |
                                      (1ULL << SD_CS_PIN);
    gpio_config_t cs_config = {
        .pin_bit_mask = chip_select_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    result = gpio_config(&cs_config);
    if (result != ESP_OK) {
        printf("RFID chip-select GPIO setup failed: %s\n", esp_err_to_name(result));
        return false;
    }
    gpio_set_level(ENTRY_RFID_CS_PIN, 1);
    gpio_set_level(EXIT_RFID_CS_PIN, 1);
    gpio_set_level(SD_CS_PIN, 1);
    shared_spi_ready = true;
    printf("Shared SPI bus ready; SD and both RFID chip-select lines held inactive\n");
    return true;
}

/* ---- RC522 reader driver (written for this project) -------------------
 * The reader is a receptionist on the SPI bus. We ask it two questions:
 *   1. REQA          "is a card there?"           -> 2 bytes (ATQA)
 *   2. ANTICOLLISION "tell me your number"        -> 4 UID bytes + 1 check byte
 * The check byte must equal UID0 ^ UID1 ^ UID2 ^ UID3, otherwise the read is
 * discarded. The FIFO is read in ONE burst; reading it byte by byte skipped
 * every second byte in Wokwi. */
#define RC522_CLOCK_HZ 1000000
#define RC522_POLL_MS 150
#define RC522_RETRIGGER_MS 1500

enum {
    RC522_COMMAND_REG = 0x01,
    RC522_COM_IRQ_REG = 0x04,
    RC522_ERROR_REG = 0x06,
    RC522_FIFO_DATA_REG = 0x09,
    RC522_FIFO_LEVEL_REG = 0x0A,
    RC522_BIT_FRAMING_REG = 0x0D,
    RC522_COLL_REG = 0x0E,
    RC522_MODE_REG = 0x11,
    RC522_TX_CONTROL_REG = 0x14,
    RC522_TX_ASK_REG = 0x15,
    RC522_T_MODE_REG = 0x2A,
    RC522_T_PRESCALER_REG = 0x2B,
    RC522_T_RELOAD_H_REG = 0x2C,
    RC522_T_RELOAD_L_REG = 0x2D,
    RC522_VERSION_REG = 0x37,
};

static esp_err_t rc522_read(spi_device_handle_t device, uint8_t reg, uint8_t *value)
{
    spi_transaction_t transaction = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 16,
        .tx_data = { (uint8_t)(((reg << 1) & 0x7E) | 0x80), 0 },
    };
    esp_err_t result = spi_device_transmit(device, &transaction);
    if (result == ESP_OK) {
        *value = transaction.rx_data[1];
    }
    return result;
}

static esp_err_t rc522_read_fifo(
    spi_device_handle_t device, uint8_t *buffer, size_t count)
{
    if (count == 0 || count > 16) {
        return ESP_ERR_INVALID_SIZE;
    }
    uint8_t address = (uint8_t)(((RC522_FIFO_DATA_REG << 1) & 0x7E) | 0x80);
    uint8_t tx[17];
    uint8_t rx[17];
    memset(tx, address, count);
    tx[count] = 0;
    memset(rx, 0, sizeof(rx));
    spi_transaction_t transaction = {
        .length = (count + 1) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    esp_err_t result = spi_device_transmit(device, &transaction);
    if (result != ESP_OK) {
        return result;
    }
    memcpy(buffer, rx + 1, count);
    return ESP_OK;
}

static esp_err_t rc522_write(spi_device_handle_t device, uint8_t reg, uint8_t value)
{
    spi_transaction_t transaction = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 16,
        .tx_data = { (uint8_t)((reg << 1) & 0x7E), value },
    };
    return spi_device_transmit(device, &transaction);
}

static esp_err_t rc522_transceive(spi_device_handle_t device, const uint8_t *send,
                                  size_t send_length, uint8_t last_bits,
                                  uint8_t *reply, size_t *reply_length)
{
    esp_err_t result = rc522_write(device, RC522_COMMAND_REG, 0x00);
    if (result != ESP_OK) return result;
    result = rc522_write(device, RC522_COM_IRQ_REG, 0x7F);
    if (result != ESP_OK) return result;
    result = rc522_write(device, RC522_FIFO_LEVEL_REG, 0x80);
    if (result != ESP_OK) return result;
    for (size_t i = 0; i < send_length; i++) {
        result = rc522_write(device, RC522_FIFO_DATA_REG, send[i]);
        if (result != ESP_OK) return result;
    }
    result = rc522_write(device, RC522_BIT_FRAMING_REG, last_bits);
    if (result != ESP_OK) return result;
    result = rc522_write(device, RC522_COMMAND_REG, 0x0C);
    if (result != ESP_OK) return result;
    uint8_t framing;
    result = rc522_read(device, RC522_BIT_FRAMING_REG, &framing);
    if (result != ESP_OK) return result;
    result = rc522_write(device, RC522_BIT_FRAMING_REG, (uint8_t)(framing | 0x80));
    if (result != ESP_OK) return result;

    bool finished = false;
    for (int i = 0; i < 40; i++) {
        uint8_t irq;
        result = rc522_read(device, RC522_COM_IRQ_REG, &irq);
        if (result != ESP_OK) return result;
        if ((irq & 0x30) != 0) {
            finished = true;
            break;
        }
        if ((irq & 0x01) != 0) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
    if (!finished) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t error;
    result = rc522_read(device, RC522_ERROR_REG, &error);
    if (result != ESP_OK) return result;
    if ((error & 0x13) != 0) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint8_t fifo_count;
    result = rc522_read(device, RC522_FIFO_LEVEL_REG, &fifo_count);
    if (result != ESP_OK) return result;
    size_t count = fifo_count;
    if (count == 0) {
        return ESP_ERR_TIMEOUT;
    }
    if (count > *reply_length) {
        return ESP_ERR_INVALID_SIZE;
    }
    result = rc522_read_fifo(device, reply, count);
    if (result != ESP_OK) return result;
    *reply_length = count;
    return ESP_OK;
}

static esp_err_t rc522_read_uid(spi_device_handle_t device, char uid[9])
{
    uint8_t reply[8];
    size_t reply_length = sizeof(reply);
    const uint8_t request = 0x26;
    uint8_t collision;
    esp_err_t result = rc522_read(device, RC522_COLL_REG, &collision);
    if (result != ESP_OK) return result;
    result = rc522_write(device, RC522_COLL_REG, (uint8_t)(collision & ~0x80));
    if (result != ESP_OK) return result;
    result = rc522_transceive(device, &request, 1, 7, reply, &reply_length);
    if (result != ESP_OK) return result;
    if (reply_length != 2) return ESP_ERR_INVALID_RESPONSE;

    const uint8_t anticollision[2] = { 0x93, 0x20 };
    reply_length = sizeof(reply);
    result = rc522_transceive(device, anticollision, 2, 0, reply, &reply_length);
    if (result != ESP_OK) return result;
    if (reply_length != 5) return ESP_ERR_INVALID_RESPONSE;
    if ((reply[0] ^ reply[1] ^ reply[2] ^ reply[3]) != reply[4]) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    snprintf(uid, 9, "%02X%02X%02X%02X", reply[0], reply[1], reply[2], reply[3]);
    return ESP_OK;
}

static esp_err_t rc522_start_reader(spi_device_handle_t device)
{
    esp_err_t result = rc522_write(device, RC522_COMMAND_REG, 0x0F);
    if (result != ESP_OK) return result;
    vTaskDelay(pdMS_TO_TICKS(50));
    const uint8_t registers[][2] = {
        { RC522_T_MODE_REG, 0x80 },
        { RC522_T_PRESCALER_REG, 0xA9 },
        { RC522_T_RELOAD_H_REG, 0x03 },
        { RC522_T_RELOAD_L_REG, 0xE8 },
        { RC522_TX_ASK_REG, 0x40 },
        { RC522_MODE_REG, 0x3D },
    };
    for (size_t i = 0; i < sizeof(registers) / sizeof(registers[0]); i++) {
        result = rc522_write(device, registers[i][0], registers[i][1]);
        if (result != ESP_OK) return result;
    }
    uint8_t tx_control;
    result = rc522_read(device, RC522_TX_CONTROL_REG, &tx_control);
    if (result != ESP_OK) return result;
    result = rc522_write(device, RC522_TX_CONTROL_REG, (uint8_t)(tx_control | 0x03));
    if (result != ESP_OK) return result;
    uint8_t version;
    result = rc522_read(device, RC522_VERSION_REG, &version);
    if (result != ESP_OK) return result;
    return version != 0x00 && version != 0xFF ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

static void rfid_poll_task(void *argument)
{
    (void)argument;
    char last_uid[2][9] = { "", "" };
    TickType_t last_trigger[2] = { 0, 0 };
    bool present[2] = { false, false };
    bool spi_error_reported[2] = { false, false };
    uint8_t misses[2] = { 0, 0 };
    while (true) {
        for (size_t reader = 0; reader < 2; reader++) {
            if (!rfid_reader_ready[reader]) {
                continue;
            }
            xSemaphoreTake(rfid_bus_task_mutex, portMAX_DELAY);
            char uid[9];
            esp_err_t result = rc522_read_uid(rfid_device[reader], uid);
            if (result == ESP_OK) {
                spi_error_reported[reader] = false;
                misses[reader] = 0;
                TickType_t now = xTaskGetTickCount();
                bool same_card = strcmp(uid, last_uid[reader]) == 0;
                bool cooled_down = (now - last_trigger[reader]) >= pdMS_TO_TICKS(RC522_RETRIGGER_MS);
                if (!present[reader] && (!same_card || cooled_down)) {
                    present[reader] = true;
                    snprintf(last_uid[reader], sizeof(last_uid[reader]), "%s", uid);
                    last_trigger[reader] = now;
                    bool entering = reader == 0;
                    printf("RFID %s reader detected UID %s\n", entering ? "entry" : "exit", uid);
                    handle_scan(entering ? "in" : "out", uid);
                }
            } else {
                if (result != ESP_ERR_TIMEOUT && !spi_error_reported[reader]) {
                    printf("RFID %s reader communication failed: %s\n",
                           reader == 0 ? "entry" : "exit", esp_err_to_name(result));
                    spi_error_reported[reader] = true;
                }
                if (misses[reader] < 255 && ++misses[reader] >= 2) {
                    present[reader] = false;
                }
            }
            xSemaphoreGive(rfid_bus_task_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(RC522_POLL_MS));
    }
}

static void initialize_rfid_readers(void)
{
    if (!shared_spi_ready) {
        printf("RFID readers not started because the shared SPI bus is unavailable\n");
        return;
    }
    rfid_bus_task_mutex = xSemaphoreCreateMutex();
    if (rfid_bus_task_mutex == NULL) {
        printf("RFID readers not started: could not create shared SPI task mutex\n");
        return;
    }

    const gpio_num_t chip_selects[2] = { ENTRY_RFID_CS_PIN, EXIT_RFID_CS_PIN };
    const char *reader_names[2] = { "entry", "exit" };
    bool any_ready = false;
    for (size_t reader = 0; reader < 2; reader++) {
        spi_device_interface_config_t device_config = {
            .clock_speed_hz = RC522_CLOCK_HZ,
            .mode = 0,
            .spics_io_num = chip_selects[reader],
            .queue_size = 1,
        };
        esp_err_t result = spi_bus_add_device(SPI3_HOST, &device_config, &rfid_device[reader]);
        if (result != ESP_OK) {
            printf("RFID %s reader SPI device failed: %s\n",
                   reader_names[reader], esp_err_to_name(result));
            continue;
        }
        result = rc522_start_reader(rfid_device[reader]);
        rfid_reader_ready[reader] = result == ESP_OK;
        any_ready = any_ready || rfid_reader_ready[reader];
        if (rfid_reader_ready[reader]) {
            printf("RFID %s reader READY (CS GPIO%d)\n",
                   reader_names[reader], (int)chip_selects[reader]);
        } else {
            printf("RFID %s reader initialization failed (CS GPIO%d): %s\n",
                   reader_names[reader], (int)chip_selects[reader],
                   esp_err_to_name(result));
        }
    }
    if (any_ready) {
        if (xTaskCreate(rfid_poll_task, "rfid_poll", 6144, NULL, 4, NULL) != pdPASS) {
            printf("RFID polling task could not start; physical card reads are unavailable\n");
            rfid_reader_ready[0] = false;
            rfid_reader_ready[1] = false;
        }
    }
    fflush(stdout);
}

static void print_help(void)
{
    printf("\nCoCIS Smart Control console\n"
           "  help                         show commands\n"
           "  status                       show time, access, rooms, and people inside\n"
           "  scan in <UID>                simulate RFID scan at entry reader\n"
           "  scan out <UID>               simulate RFID scan at exit reader\n"
           "  logs                         show saved access events\n"
           "  login <PIN>                  authenticate as administrator (locks 30 s after 3 wrong PINs)\n"
           "  schedule HH:MM HH:MM         set access window, including overnight (administrator only)\n"
           "  time HH:MM                   set demonstration clock (administrator only)\n"
           "  logout                       end administrator session\n"
           "  Panel keypad: A opens admin login; # shows status\n"
           "  Emergency button GPIO25 always releases the exit door\n\n"
           "Authorized demo UIDs: A1B2C3D4, DEADBEEF, CAFEBABE\n");
}

static void print_status(void)
{
    char now_text[6];
    char open_text[6];
    char close_text[6];
    bool lights[ROOM_COUNT];
    bool occupied[ROOM_COUNT];
    static char occupants[MAX_OCCUPANTS][UID_LENGTH]; /* static: keeps ~2 KB off the stack */
    bool online[ROOM_COUNT];
    size_t people_inside;
    bool access_open;

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    minute_to_text(current_minute_locked(), now_text);
    minute_to_text(state.schedule.open_minute, open_text);
    minute_to_text(state.schedule.close_minute, close_text);
    memcpy(lights, state.light_on, sizeof(lights));
    memcpy(occupied, state.room_occupied, sizeof(occupied));
    for (size_t room = 0; room < ROOM_COUNT; room++) {
        online[room] = state.room_last_seen[room] != 0 &&
            (xTaskGetTickCount() - state.room_last_seen[room]) <= pdMS_TO_TICKS(3000);
    }
    people_inside = state.inside_count;
    memcpy(occupants, state.inside, sizeof(occupants));
    access_open = is_access_time_locked();
    xSemaphoreGive(state_mutex);

        printf("CoCIS controller | time %s | access %s (%s-%s) | people inside %u\n",
            now_text,
           access_open ? "OPEN" : "CLOSED", open_text, close_text,
           (unsigned)people_inside);
    printf("RFID readers: entry %s, exit %s\n",
           rfid_reader_ready[0] ? "READY" : "UNAVAILABLE",
           rfid_reader_ready[1] ? "READY" : "UNAVAILABLE");
    for (size_t room = 0; room < ROOM_COUNT; room++) {
        printf("Room %u: %s, light %s, terminal %s\n", (unsigned)(room + 1),
               occupied[room] ? "occupied" : "clear",
               lights[room] ? "ON" : "OFF",
               online[room] ? "online" : "offline");
    }
    printf("People inside:");
    for (size_t i = 0; i < people_inside; i++) {
        printf(" %s", occupants[i]);
    }
    printf("\n");
}

static void print_logs(void)
{
    if (sd_available) {
        /* Acquire the shared SPI lock before state_mutex to match the ordering
         * used by scans that write SD records while holding the state mutex. */
        if (rfid_bus_task_mutex != NULL) {
            xSemaphoreTake(rfid_bus_task_mutex, portMAX_DELAY);
        }
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        FILE *file = fopen(SD_LOG_PATH, "r");
        if (file != NULL) {
            char line[96];
            printf("SD access log (%s):\n", SD_LOG_PATH);
            while (fgets(line, sizeof(line), file) != NULL) {
                printf("  %s", line);
            }
            fclose(file);
            xSemaphoreGive(state_mutex);
            if (rfid_bus_task_mutex != NULL) {
                xSemaphoreGive(rfid_bus_task_mutex);
            }
            return;
        }
        xSemaphoreGive(state_mutex);
        if (rfid_bus_task_mutex != NULL) {
            xSemaphoreGive(rfid_bus_task_mutex);
        }
    }
    access_record_t records[LOG_CAPACITY];
    uint32_t count;
    uint32_t next;

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    count = state.log.count;
    next = state.log.next;
    memcpy(records, state.log.records, sizeof(records));
    xSemaphoreGive(state_mutex);

    printf("Saved access events (%u):\n", (unsigned)count);
    uint32_t first = (next + LOG_CAPACITY - count) % LOG_CAPACITY;
    for (uint32_t i = 0; i < count; i++) {
        access_record_t *record = &records[(first + i) % LOG_CAPACITY];
        char time_text[6];
        minute_to_text(record->minute_of_day, time_text);
        printf("  %s %s %s\n", time_text,
               record->entering ? "ENTRY" : "EXIT ", record->uid);
    }
}

static void handle_scan(const char *direction, char *uid)
{
    bool entering = strcmp(direction, "in") == 0;
    for (char *ch = uid; *ch != '\0'; ch++) {
        *ch = (char)toupper((unsigned char)*ch);
    }
    if (strlen(uid) >= UID_LENGTH || !uid_is_authorized(uid)) {
        printf("ACCESS DENIED: unknown RFID UID %s\n", uid);
        fflush(stdout);
        lcd_message("ACCESS DENIED", "Unknown card");
        return;
    }

    const gpio_num_t door_pin = door_pins[entering ? 0 : 1];
    const char *door_name = entering ? "entry" : "exit";
    const char *denial = NULL;
    const char *denial_short = "";
    door_pulse_args_t *pulse = calloc(1, sizeof(*pulse));
    if (pulse == NULL) {
        printf("ACCESS DENIED at %s: unable to allocate door-control task\n", door_name);
        fflush(stdout);
        return;
    }
    pulse->pin = door_pin;
    pulse->name = door_name;
    snprintf(pulse->uid, sizeof(pulse->uid), "%s", uid);

    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if (!is_access_time_locked()) {
        denial = "outside configured access hours";
        denial_short = "Outside hours";
    } else if (entering && state.inside_count >= MAX_OCCUPANTS) {
        denial = "occupancy capacity reached";
        denial_short = "Building full";
    } else if (entering && inside_index_locked(uid) >= 0) {
        denial = "tag is already recorded inside";
        denial_short = "Already inside";
    } else if (!entering && inside_index_locked(uid) < 0) {
        denial = "tag is not recorded inside";
        denial_short = "Not inside";
    }

    if (denial == NULL) {
        /* Create the pulse task while holding the mutex. It cannot alter the
         * door output until this critical section completes. If task creation
         * fails, occupancy and access logs remain unchanged. */
        if (xTaskCreate(door_pulse_task, entering ? "entry_pulse" : "exit_pulse",
                        2048, pulse, 5, NULL) != pdPASS) {
            denial = "door-control task could not start";
            denial_short = "Door error";
        } else {
            pulse = NULL; /* The task owns this memory from here on. */
            if (entering) {
                snprintf(state.inside[state.inside_count++], UID_LENGTH, "%s", uid);
            } else {
                size_t index = (size_t)inside_index_locked(uid);
                state.inside_count--;
                if (index != state.inside_count) {
                    memcpy(state.inside[index], state.inside[state.inside_count], UID_LENGTH);
                }
                memset(state.inside[state.inside_count], 0, UID_LENGTH);
            }
            persist_occupants_locked();
            add_access_record_locked(uid, entering);
        }
    }
    xSemaphoreGive(state_mutex);

    free(pulse);
    if (denial != NULL) {
        printf("ACCESS DENIED at %s: %s\n", door_name, denial);
        lcd_message("ACCESS DENIED", denial_short);
    } else {
        printf("ACCESS GRANTED at %s for UID %s\n", door_name, uid);
        lcd_message(entering ? "WELCOME" : "GOODBYE", uid);
    }
    fflush(stdout);
}

static void handle_command(char *line)
{
    char *command = strtok(line, " \t\r\n");
    if (command == NULL) {
        return;
    }

    if (strcmp(command, "help") == 0) {
        print_help();
    } else if (strcmp(command, "status") == 0) {
        print_status();
    } else if (strcmp(command, "logs") == 0) {
        print_logs();
    } else if (strcmp(command, "logout") == 0) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        state.admin_authenticated = false;
        xSemaphoreGive(state_mutex);
        printf("Administrator session ended\n");
    } else if (strcmp(command, "login") == 0) {
        char *pin = strtok(NULL, " \t\r\n");
        if (authenticate_admin_pin(pin)) {
            printf("Administrator authenticated for 5 minutes\n");
        }
    } else if (strcmp(command, "scan") == 0) {
        char *direction = strtok(NULL, " \t\r\n");
        char *uid = strtok(NULL, " \t\r\n");
        if (direction == NULL || uid == NULL ||
            (strcmp(direction, "in") != 0 && strcmp(direction, "out") != 0)) {
            printf("Usage: scan in|out <UID>\n");
        } else {
            /* Keep simulated scans and their SD logging from overlapping a
             * multi-transfer transaction in either physical RFID reader. */
            if (rfid_bus_task_mutex != NULL) {
                xSemaphoreTake(rfid_bus_task_mutex, portMAX_DELAY);
            }
            handle_scan(direction, uid);
            if (rfid_bus_task_mutex != NULL) {
                xSemaphoreGive(rfid_bus_task_mutex);
            }
        }
    } else if (strcmp(command, "schedule") == 0) {
        char *open_text = strtok(NULL, " \t\r\n");
        char *close_text = strtok(NULL, " \t\r\n");
        uint16_t open_minute;
        uint16_t close_minute;
        if (!require_admin()) {
            return;
        }
        if (open_text == NULL || close_text == NULL ||
            !parse_time(open_text, &open_minute) ||
            !parse_time(close_text, &close_minute) || open_minute == close_minute) {
            printf("Usage: schedule HH:MM HH:MM (different start/end times; overnight windows are supported)\n");
            return;
        }
        if (update_schedule(open_minute, close_minute)) {
            printf("Access schedule updated and saved\n");
        } else {
            printf("Access schedule updated for this run but not saved\n");
        }
    } else if (strcmp(command, "time") == 0) {
        char *time_text = strtok(NULL, " \t\r\n");
        uint16_t minute;
        if (!require_admin()) {
            return;
        }
        if (time_text == NULL || !parse_time(time_text, &minute)) {
            printf("Usage: time HH:MM\n");
            return;
        }
        if (update_demo_clock(minute)) {
            printf("Demonstration clock set\n");
        } else {
            printf("Demonstration clock set; RTC did not retain the change\n");
        }
    } else {
        printf("Unknown command. Type help.\n");
    }
}

static void run_serial_console(void)
{
    char line[96];
    size_t length = 0;

    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    printf("> ");
    fflush(stdout);
    while (true) {
        uint8_t ch;
        if (uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100)) <= 0) {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            if (length == 0) {
                continue;
            }
            putchar('\n');
            line[length] = '\0';
            handle_command(line);
            length = 0;
            printf("> ");
            fflush(stdout);
        } else if ((ch == 0x08 || ch == 0x7f) && length > 0) {
            length--;
            printf("\b \b");
            fflush(stdout);
        } else if (ch >= 32 && ch < 127 && length < sizeof(line) - 1) {
            line[length++] = (char)ch;
            putchar(ch);
            fflush(stdout);
        }
    }
}

/* Sekiranda Emma Michael - emergency egress and alarm */
static void servo_write(size_t door_index, int angle)
{
    uint32_t pulse_us = 500U + (uint32_t)angle * 2000U / 180U;
    uint32_t duty = pulse_us * 16384U / 20000U;
    ledc_channel_t channel = (ledc_channel_t)(LEDC_CHANNEL_1 + door_index);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

static void initialize_servos(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = 50,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer_config) != ESP_OK) {
        printf("Door servo timer setup failed; door LEDs only\n");
        return;
    }
    const gpio_num_t servo_pins[2] = { SERVO_ENTRY_PIN, SERVO_EXIT_PIN };
    for (size_t i = 0; i < 2; i++) {
        ledc_channel_config_t channel_config = {
            .gpio_num = servo_pins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)(LEDC_CHANNEL_1 + i),
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_1,
            .duty = 0,
            .hpoint = 0,
        };
        if (ledc_channel_config(&channel_config) != ESP_OK) {
            printf("Door servo %u setup failed\n", (unsigned)(i + 1));
            return;
        }
        servo_write(i, SERVO_CLOSED_ANGLE);
        door_last_level[i] = 0;
    }
    servo_ready = true;
    printf("Door servos ready (GPIO%d entry, GPIO%d exit)\n",
           (int)SERVO_ENTRY_PIN, (int)SERVO_EXIT_PIN);
}

/* Access and emergency outputs share these servo/indicator levels. */
static void door_output(gpio_num_t pin, bool open)
{
    size_t index = pin == door_pins[0] ? 0U : 1U;
    int level = open ? 1 : 0;
    gpio_set_level(pin, level);
    if (servo_ready && door_last_level[index] != level) {
        servo_write(index, open ? SERVO_OPEN_ANGLE : SERVO_CLOSED_ANGLE);
    }
    door_last_level[index] = level;
}

static void configure_gpio(void)
{
    uint64_t input_mask = 0;
    uint64_t output_mask = (1ULL << door_pins[0]) | (1ULL << door_pins[1]);
    for (size_t room = 0; room < ROOM_COUNT; room++) {
        input_mask |= 1ULL << pir_pins[room];
        output_mask |= 1ULL << light_pins[room];
    }

    gpio_config_t input_config = {
        .pin_bit_mask = input_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t output_config = {
        .pin_bit_mask = output_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t egress_config = {
        .pin_bit_mask = 1ULL << EMERGENCY_EGRESS_PIN,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    if (input_mask != 0) {
        ESP_ERROR_CHECK(gpio_config(&input_config));
    }
    ESP_ERROR_CHECK(gpio_config(&output_config));
    for (size_t i = 0; i < ROOM_COUNT; i++) {
        ESP_ERROR_CHECK(gpio_set_level(light_pins[i], 0));
    }
    for (size_t i = 0; i < 2; i++) {
        ESP_ERROR_CHECK(gpio_set_level(door_pins[i], 0));
    }
    ESP_ERROR_CHECK(gpio_config(&egress_config));
}

static void emergency_egress_task(void *argument)
{
    (void)argument;
    bool was_pressed = false;
    bool alarm_output_error_reported = false;
    TickType_t alarm_started = 0;
    while (true) {
        bool pressed = gpio_get_level(EMERGENCY_EGRESS_PIN) == 0;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        bool exit_pulse_active = state.exit_pulse_active;
        door_output(door_pins[1], pressed || exit_pulse_active);
        xSemaphoreGive(state_mutex);
        if (pressed && !was_pressed) {
            alarm_started = xTaskGetTickCount();
            printf("EMERGENCY EGRESS: exit released independently of RFID and schedule\n");
            lcd_message("EMERGENCY EXIT", "Door released");
            if (emergency_alarm_available) {
                printf("EMERGENCY ALARM: audible alert activated\n");
            } else {
                printf("WARNING: audible emergency alarm is unavailable\n");
            }
        } else if (!pressed && was_pressed && emergency_alarm_available) {
            printf("EMERGENCY ALARM: audible alert stopped\n");
        }
        if (emergency_alarm_available) {
            TickType_t elapsed_ms = pdTICKS_TO_MS(xTaskGetTickCount() - alarm_started);
            bool beep_on = pressed && (elapsed_ms % 600U) < 350U;
            esp_err_t result = ledc_set_duty(
                LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, beep_on ? 512 : 0);
            if (result == ESP_OK) {
                result = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
            }
            if (result != ESP_OK) {
                emergency_alarm_available = false;
                if (!alarm_output_error_reported) {
                    printf("Emergency alarm output failed: %s\n", esp_err_to_name(result));
                    alarm_output_error_reported = true;
                }
            }
        }
        was_pressed = pressed;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void initialize_emergency_alarm(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 2000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t result = ledc_timer_config(&timer_config);
    if (result != ESP_OK) {
        printf("Emergency alarm timer setup failed: %s\n", esp_err_to_name(result));
        return;
    }

    ledc_channel_config_t channel_config = {
        .gpio_num = EMERGENCY_ALARM_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    result = ledc_channel_config(&channel_config);
    if (result != ESP_OK) {
        printf("Emergency alarm output setup failed: %s\n", esp_err_to_name(result));
        return;
    }
    emergency_alarm_available = true;
    printf("Emergency buzzer ready (GPIO%d, 2 kHz pulsed alarm)\n",
           (int)EMERGENCY_ALARM_PIN);
}

/* Gwaiffe Mark - keypad and LCD administrator interface */
static esp_err_t lcd_write_nibble(uint8_t nibble)
{
    uint8_t value = (uint8_t)(((nibble & 0x0F) << 4) | LCD_BACKLIGHT_BIT);
    uint8_t transfer[] = { value, (uint8_t)(value | 0x04), value };
    return i2c_master_transmit(lcd_device, transfer, sizeof(transfer), 100);
}

static esp_err_t lcd_write_byte(uint8_t value, bool data)
{
    uint8_t mode = data ? 0x01 : 0x00;
    uint8_t high_nibble = (uint8_t)((value & 0xF0) | LCD_BACKLIGHT_BIT | mode);
    uint8_t low_nibble = (uint8_t)(((value << 4) & 0xF0) | LCD_BACKLIGHT_BIT | mode);
    uint8_t transfer[] = {
        high_nibble, (uint8_t)(high_nibble | 0x04), high_nibble,
        low_nibble, (uint8_t)(low_nibble | 0x04), low_nibble,
    };
    return i2c_master_transmit(lcd_device, transfer, sizeof(transfer), 100);
}

static size_t lcd_encode_byte(uint8_t value, bool data, uint8_t transfer[6])
{
    uint8_t mode = data ? 0x01 : 0x00;
    uint8_t high_nibble = (uint8_t)((value & 0xF0) | LCD_BACKLIGHT_BIT | mode);
    uint8_t low_nibble = (uint8_t)(((value << 4) & 0xF0) | LCD_BACKLIGHT_BIT | mode);
    transfer[0] = high_nibble;
    transfer[1] = (uint8_t)(high_nibble | 0x04);
    transfer[2] = high_nibble;
    transfer[3] = low_nibble;
    transfer[4] = (uint8_t)(low_nibble | 0x04);
    transfer[5] = low_nibble;
    return 6;
}

static void lcd_show_unlocked(const char *first_line, const char *second_line)
{
    if (!lcd_available) {
        return;
    }

    const char *lines[] = { first_line, second_line };
    const uint8_t row_addresses[] = { 0x80, 0xC0 };
    for (size_t row = 0; row < 2; row++) {
        uint8_t transfer[102];
        size_t transfer_length = lcd_encode_byte(row_addresses[row], false, transfer);
        size_t line_length = strlen(lines[row]);
        for (size_t column = 0; column < 16; column++) {
            char character = column < line_length ? lines[row][column] : ' ';
            transfer_length += lcd_encode_byte((uint8_t)character, true,
                                               transfer + transfer_length);
        }
        esp_err_t result = i2c_master_transmit(
            lcd_device, transfer, transfer_length, 100);
        if (result != ESP_OK) {
            printf("Admin LCD write failed on row %u: %s\n",
                   (unsigned)(row + 1), esp_err_to_name(result));
            lcd_available = false;
            return;
        }
    }
}

static void lcd_show(const char *first_line, const char *second_line)
{
    if (lcd_mutex != NULL) {
        xSemaphoreTake(lcd_mutex, portMAX_DELAY);
    }
    lcd_show_unlocked(first_line, second_line);
    if (lcd_mutex != NULL) {
        xSemaphoreGive(lcd_mutex);
    }
}

/* Shows a result (WELCOME, ACCESS DENIED...) and keeps the home screen from
 * overwriting it for MESSAGE_SHOW_MS. */
static void lcd_message(const char *first_line, const char *second_line)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    panel_message_until = xTaskGetTickCount() + pdMS_TO_TICKS(MESSAGE_SHOW_MS);
    xSemaphoreGive(state_mutex);
    lcd_show(first_line, second_line);
}

static bool initialize_lcd(void)
{
    if (i2c_bus == NULL) {
        printf("Admin LCD unavailable: I2C bus is not ready\n");
        return false;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LCD_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    esp_err_t result = i2c_master_bus_add_device(i2c_bus, &device_config, &lcd_device);
    if (result != ESP_OK) {
        printf("Admin LCD device setup failed: %s\n", esp_err_to_name(result));
        return false;
    }
    lcd_available = true;
    vTaskDelay(pdMS_TO_TICKS(50));

    const uint8_t startup_nibbles[] = { 0x03, 0x03, 0x03, 0x02 };
    for (size_t i = 0; i < sizeof(startup_nibbles); i++) {
        result = lcd_write_nibble(startup_nibbles[i]);
        if (result != ESP_OK) {
            printf("Admin LCD initialization failed: %s\n", esp_err_to_name(result));
            lcd_available = false;
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(i == 0 ? 5 : 1));
    }
    const uint8_t commands[] = { 0x28, 0x0C, 0x06, 0x01 };
    for (size_t i = 0; i < sizeof(commands); i++) {
        result = lcd_write_byte(commands[i], false);
        if (result != ESP_OK) {
            printf("Admin LCD initialization failed: %s\n", esp_err_to_name(result));
            lcd_available = false;
            return false;
        }
        if (commands[i] == 0x01) {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    printf("Admin LCD ready (I2C address 0x%02X)\n", LCD_I2C_ADDRESS);
    return true;
}

static bool keypad_expander_write(uint8_t reg, uint8_t value)
{
    uint8_t transfer[] = { reg, value };
    return i2c_master_transmit(
        keypad_expander_device, transfer, sizeof(transfer), 100) == ESP_OK;
}

static bool initialize_keypad(void)
{
    if (i2c_bus == NULL) {
        printf("Admin keypad unavailable: I2C bus is not ready\n");
        return false;
    }

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = KEYPAD_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    esp_err_t result = i2c_master_bus_add_device(
        i2c_bus, &device_config, &keypad_expander_device);
    if (result != ESP_OK) {
        printf("Admin keypad expander setup failed: %s\n", esp_err_to_name(result));
        return false;
    }
    if (!keypad_expander_write(MCP23017_IODIRA_REG, 0xF0) ||
        !keypad_expander_write(MCP23017_GPPUA_REG, 0xF0) ||
        !keypad_expander_write(MCP23017_GPIOA_REG, 0xFF)) {
        printf("Admin keypad expander initialization failed\n");
        return false;
    }
    keypad_available = true;
    printf("Admin keypad ready (MCP23017 I2C address 0x%02X)\n",
           KEYPAD_I2C_ADDRESS);
    return true;
}

static void initialize_admin_panel(void)
{
    bool display_ready = initialize_lcd();
    bool keypad_ready = initialize_keypad();
    if (display_ready && keypad_ready) {
        lcd_show("CoCIS Admin", "A:Admin  #=Status");
    } else {
        printf("Admin panel unavailable; use the serial console\n");
    }
}

static bool require_admin(void)
{
    bool authenticated;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    authenticated = state.admin_authenticated &&
                    (int32_t)(state.auth_expires - xTaskGetTickCount()) > 0;
    if (!authenticated) {
        state.admin_authenticated = false;
    }
    xSemaphoreGive(state_mutex);
    if (!authenticated) {
        printf("Administrator login required. Use: login <PIN>\n");
    }
    return authenticated;
}

static bool admin_session_active(void)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    bool authenticated = state.admin_authenticated &&
                         (int32_t)(state.auth_expires - xTaskGetTickCount()) > 0;
    if (!authenticated) {
        state.admin_authenticated = false;
    }
    xSemaphoreGive(state_mutex);
    return authenticated;
}

/* Checks the PIN. After LOGIN_MAX_FAILURES wrong tries, login is locked for
 * LOGIN_LOCKOUT_MS. Prints its own failure messages (serial and keypad share it). */
static bool authenticate_admin_pin(const char *pin)
{
    TickType_t now = xTaskGetTickCount();
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if ((int32_t)(lockout_until - now) > 0) {
        unsigned seconds = (unsigned)(pdTICKS_TO_MS(lockout_until - now) / 1000U) + 1U;
        xSemaphoreGive(state_mutex);
        printf("Login is locked. Try again in %u seconds.\n", seconds);
        return false;
    }
    if (pin != NULL && strcmp(pin, ADMIN_PIN) == 0) {
        failed_logins = 0;
        state.admin_authenticated = true;
        state.auth_expires = now + pdMS_TO_TICKS(300000);
        xSemaphoreGive(state_mutex);
        return true;
    }
    failed_logins++;
    bool locked = failed_logins >= LOGIN_MAX_FAILURES;
    unsigned tries = failed_logins;
    if (locked) {
        failed_logins = 0;
        lockout_until = now + pdMS_TO_TICKS(LOGIN_LOCKOUT_MS);
    }
    xSemaphoreGive(state_mutex);
    if (locked) {
        printf("%d wrong PINs. Login locked for %d seconds.\n",
               LOGIN_MAX_FAILURES, LOGIN_LOCKOUT_MS / 1000);
    } else {
        printf("Authentication failed (%u of %d)\n", tries, LOGIN_MAX_FAILURES);
    }
    return false;
}

static bool update_schedule(uint16_t open_minute, uint16_t close_minute)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.schedule.open_minute = open_minute;
    state.schedule.close_minute = close_minute;
    esp_err_t result = nvs_set_blob(nvs_store, "schedule", &state.schedule, sizeof(state.schedule));
    if (result == ESP_OK) {
        result = nvs_commit(nvs_store);
    }
    xSemaphoreGive(state_mutex);
    if (result != ESP_OK) {
        printf("Schedule is active until restart, but NVS save failed: %s\n",
               esp_err_to_name(result));
        return false;
    }
    return true;
}

static bool update_demo_clock(uint16_t minute)
{
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    state.clock_minute = minute;
    state.clock_tick = xTaskGetTickCount();
    xSemaphoreGive(state_mutex);
    if (rtc_available && !rtc_set_minute(minute)) {
        printf("Warning: RTC write failed; demonstration clock remains active\n");
        return false;
    }
    return true;
}

static bool read_keypad_key(char *key)
{
    static const char keys[4][4] = {
        { '1', '2', '3', 'A' },
        { '4', '5', '6', 'B' },
        { '7', '8', '9', 'C' },
        { '*', '0', '#', 'D' },
    };
    *key = '\0';
    for (uint8_t row = 0; row < 4; row++) {
        uint8_t output = (uint8_t)(0xF0 | (0x0F & ~(1U << row)));
        if (!keypad_expander_write(MCP23017_GPIOA_REG, output)) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
        uint8_t register_address = MCP23017_GPIOA_REG;
        uint8_t value;
        if (i2c_master_transmit_receive(
                keypad_expander_device, &register_address, 1, &value, 1, 100) != ESP_OK) {
            return false;
        }
        for (uint8_t column = 0; column < 4; column++) {
            if ((value & (uint8_t)(0x10U << column)) == 0) {
                *key = keys[row][column];
                return keypad_expander_write(MCP23017_GPIOA_REG, 0xFF);
            }
        }
    }
    return keypad_expander_write(MCP23017_GPIOA_REG, 0xFF);
}

static void panel_show_home(void)
{
    static bool closing_warning_printed;
    char now[6];
    bool access_open;
    int minutes_left = -1;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    if ((int32_t)(panel_message_until - xTaskGetTickCount()) > 0) {
        xSemaphoreGive(state_mutex);
        return;
    }
    uint16_t minute_now = current_minute_locked();
    minute_to_text(minute_now, now);
    access_open = is_access_time_locked();
    if (access_open) {
        minutes_left = ((int)state.schedule.close_minute + 1440 - (int)minute_now) % 1440;
    }
    xSemaphoreGive(state_mutex);

    char line[17];
    char line2[17];
    snprintf(line, sizeof(line), "%s ACCESS %s", now, access_open ? "ON" : "OFF");
    if (minutes_left >= 1 && minutes_left <= CLOSING_WARNING_MIN) {
        snprintf(line2, sizeof(line2), "Closes in %d min", minutes_left);
        if (!closing_warning_printed) {
            printf("WARNING: building access closes in %d minutes\n", minutes_left);
            closing_warning_printed = true;
        }
    } else {
        snprintf(line2, sizeof(line2), "A=Admin #=Status");
        closing_warning_printed = false;
    }
    lcd_show(line, line2);
}

static void panel_show_menu(void)
{
    lcd_show("A:Hours B:Status", "C:Clock D:Logout");
}

static void panel_show_status(void)
{
    char open_text[6];
    char close_text[6];
    bool access_open;
    size_t occupants;
    xSemaphoreTake(state_mutex, portMAX_DELAY);
    minute_to_text(state.schedule.open_minute, open_text);
    minute_to_text(state.schedule.close_minute, close_text);
    access_open = is_access_time_locked();
    occupants = state.inside_count;
    xSemaphoreGive(state_mutex);

    char line1[17];
    char line2[17];
    snprintf(line1, sizeof(line1), "%s %u inside",
             access_open ? "OPEN" : "CLOSED", (unsigned)occupants);
    snprintf(line2, sizeof(line2), "%s-%s", open_text, close_text);
    lcd_show(line1, line2);
}

static bool keypad_time_to_minute(const char input[5], uint16_t *minute)
{
    char time_text[6];
    snprintf(time_text, sizeof(time_text), "%.2s:%.2s", input, input + 2);
    return parse_time(time_text, minute);
}

static void panel_show_time_prompt(const char *title, const char input[5], size_t length)
{
    char shown[6] = "";
    char second_line[17];
    if (length == 4) {
        snprintf(shown, sizeof(shown), "%.2s:%.2s", input, input + 2);
    } else {
        memcpy(shown, input, length);
        shown[length] = '\0';
    }
    snprintf(second_line, sizeof(second_line), "%s #=OK *=Del", shown);
    lcd_show(title, second_line);
}

static void panel_show_pin(const char input[5], size_t length)
{
    char masked[5];
    memset(masked, '*', length);
    masked[length] = '\0';
    char second_line[17];
    snprintf(second_line, sizeof(second_line), "%s #=OK *=Back", masked);
    lcd_show("Admin PIN:", second_line);
}

static void keypad_append_digit(char input[5], size_t *length, char key)
{
    if (key >= '0' && key <= '9' && *length < 4) {
        input[(*length)++] = key;
        input[*length] = '\0';
    }
}

static void admin_panel_task(void *argument)
{
    (void)argument;
    panel_screen_t screen = PANEL_HOME;
    char input[5] = "";
    size_t input_length = 0;
    uint16_t proposed_open = 0;
    TickType_t last_home_refresh = 0;
    char previous_key = '\0';
    char candidate_key = '\0';
    unsigned stable_samples = 0;
    bool keypad_error_reported = false;
    panel_show_home();

    while (true) {
        if (!lcd_available) {
            printf("Admin panel task stopped because the LCD is unavailable\n");
            vTaskDelete(NULL);
        }
        if (screen == PANEL_HOME &&
            xTaskGetTickCount() - last_home_refresh >= pdMS_TO_TICKS(1000)) {
            panel_show_home();
            last_home_refresh = xTaskGetTickCount();
        }
        if ((screen == PANEL_MENU || screen == PANEL_OPEN_TIME ||
             screen == PANEL_CLOSE_TIME || screen == PANEL_CLOCK) &&
            !admin_session_active()) {
            screen = PANEL_HOME;
            panel_show_home();
        }

        char raw_key;
        if (!read_keypad_key(&raw_key)) {
            if (!keypad_error_reported) {
                printf("Admin keypad read failed; check the I2C expander and wiring\n");
                keypad_error_reported = true;
            }
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }
        keypad_error_reported = false;
        if (raw_key == candidate_key) {
            stable_samples++;
        } else {
            candidate_key = raw_key;
            stable_samples = 0;
        }
        if (stable_samples >= 1 && raw_key == '\0') {
            previous_key = '\0';
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }
        if (stable_samples < 1 || raw_key == previous_key || raw_key == '\0') {
            vTaskDelay(pdMS_TO_TICKS(40));
            continue;
        }
        previous_key = raw_key;

        if (screen == PANEL_HOME) {
            if (raw_key == 'A') {
                input[0] = '\0';
                input_length = 0;
                screen = PANEL_PIN;
                panel_show_pin(input, input_length);
            } else if (raw_key == '#') {
                screen = PANEL_STATUS;
                panel_show_status();
            }
        } else if (screen == PANEL_PIN) {
            if (raw_key == '*') {
                screen = PANEL_HOME;
                panel_show_home();
            } else if (raw_key == '#') {
                input[input_length] = '\0';
                if (input_length == 4 && authenticate_admin_pin(input)) {
                    screen = PANEL_MENU;
                    panel_show_menu();
                } else {
                    printf("Admin panel authentication failed\n");
                    lcd_show("Authentication", "failed");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    screen = PANEL_HOME;
                    panel_show_home();
                }
            } else {
                keypad_append_digit(input, &input_length, raw_key);
                panel_show_pin(input, input_length);
            }
        } else if (screen == PANEL_MENU) {
            if (raw_key == 'A') {
                input[0] = '\0';
                input_length = 0;
                screen = PANEL_OPEN_TIME;
                panel_show_time_prompt("Open HHMM:", input, input_length);
            } else if (raw_key == 'B') {
                screen = PANEL_STATUS;
                panel_show_status();
            } else if (raw_key == 'C') {
                input[0] = '\0';
                input_length = 0;
                screen = PANEL_CLOCK;
                panel_show_time_prompt("Set clock HHMM:", input, input_length);
            } else if (raw_key == 'D') {
                xSemaphoreTake(state_mutex, portMAX_DELAY);
                state.admin_authenticated = false;
                xSemaphoreGive(state_mutex);
                screen = PANEL_HOME;
                panel_show_home();
            }
        } else if (screen == PANEL_OPEN_TIME || screen == PANEL_CLOSE_TIME ||
                   screen == PANEL_CLOCK) {
            if (raw_key == '*') {
                if (input_length > 0) {
                    input_length = 0;
                    input[0] = '\0';
                    const char *title = screen == PANEL_OPEN_TIME ? "Open HHMM:" :
                                        screen == PANEL_CLOSE_TIME ? "Close HHMM:" :
                                        "Set clock HHMM:";
                    panel_show_time_prompt(title, input, input_length);
                } else {
                    screen = PANEL_MENU;
                    panel_show_menu();
                }
            } else if (raw_key == '#') {
                uint16_t minute;
                if (input_length != 4 || !keypad_time_to_minute(input, &minute)) {
                    lcd_show("Invalid time", "Use HHMM");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    const char *title = screen == PANEL_OPEN_TIME ? "Open HHMM:" :
                                        screen == PANEL_CLOSE_TIME ? "Close HHMM:" :
                                        "Set clock HHMM:";
                    panel_show_time_prompt(title, input, input_length);
                } else if (screen == PANEL_OPEN_TIME) {
                    proposed_open = minute;
                    input[0] = '\0';
                    input_length = 0;
                    screen = PANEL_CLOSE_TIME;
                    panel_show_time_prompt("Close HHMM:", input, input_length);
                } else if (screen == PANEL_CLOSE_TIME && minute == proposed_open) {
                    lcd_show("Hours must differ", "Schedule unchanged");
                    vTaskDelay(pdMS_TO_TICKS(1200));
                    screen = PANEL_MENU;
                    panel_show_menu();
                } else if (screen == PANEL_CLOSE_TIME) {
                    bool saved = update_schedule(proposed_open, minute);
                    lcd_show(saved ? "Schedule saved" : "Schedule active",
                             saved ? "Updated" : "NVS save failed");
                    vTaskDelay(pdMS_TO_TICKS(1200));
                    screen = PANEL_MENU;
                    panel_show_menu();
                } else {
                    bool clock_set = update_demo_clock(minute);
                    lcd_show(clock_set ? "Clock updated" : "RTC write failed",
                             "Demo clock active");
                    vTaskDelay(pdMS_TO_TICKS(1200));
                    screen = PANEL_MENU;
                    panel_show_menu();
                }
            } else {
                keypad_append_digit(input, &input_length, raw_key);
                const char *title = screen == PANEL_OPEN_TIME ? "Open HHMM:" :
                                    screen == PANEL_CLOSE_TIME ? "Close HHMM:" :
                                    "Set clock HHMM:";
                panel_show_time_prompt(title, input, input_length);
            }
        } else if (screen == PANEL_STATUS) {
            screen = admin_session_active() ? PANEL_MENU : PANEL_HOME;
            if (screen == PANEL_MENU) {
                panel_show_menu();
            } else {
                panel_show_home();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

/* Shared system startup and integration */
void app_main(void)
{
    /* Integration entry point: initialize shared services, then feature tasks. */
    memset(&state, 0, sizeof(state));
    initialize_storage();
    state_mutex = xSemaphoreCreateMutex();
    lcd_mutex = xSemaphoreCreateMutex();
    terminal_queue = xQueueCreate(8, sizeof(room_message_t));
    if (state_mutex == NULL) {
        printf("FATAL: could not create system-state mutex\n");
        return;
    }
    if (terminal_queue == NULL) {
        printf("FATAL: could not create room-terminal message queue\n");
        return;
    }
    state.clock_minute = 9 * 60;
    state.clock_tick = xTaskGetTickCount();
    initialize_rtc();
    if (lcd_mutex != NULL) {
        initialize_admin_panel();
    } else {
        printf("Admin panel unavailable: could not create LCD synchronization mutex\n");
    }
    configure_gpio();
    initialize_emergency_alarm();
    initialize_servos();
    bool spi_ready = initialize_shared_spi_bus();
    if (spi_ready) {
        mount_sd_card();
        initialize_rfid_readers();
    } else {
        printf("SD and RFID are disabled because the shared SPI bus failed to initialize\n");
    }
    if (xTaskCreate(entrance_link_task, "entrance_link", 3072, NULL, 5, NULL) != pdPASS) {
        printf("Room-terminal receiver task could not start; room status will remain offline\n");
    }
    for (size_t room = 0; room < ROOM_COUNT; room++) {
        if (xTaskCreate(room_monitor_task,
                        room == 0 ? "room1_terminal" : "room2_terminal",
                        3072, (void *)(uintptr_t)room, 5, NULL) != pdPASS) {
            printf("Room %u terminal task could not start; its light/status is unavailable\n",
                   (unsigned)(room + 1));
        }
    }
    if (xTaskCreate(emergency_egress_task, "emergency_egress", 2048, NULL, 6, NULL) != pdPASS) {
        printf("WARNING: emergency-egress task could not start\n");
    }
    if (xTaskCreate(rtc_sync_task, "rtc_sync", 2048, NULL, 3, NULL) != pdPASS) {
        printf("RTC synchronization task could not start; clock will use the fallback value\n");
    }
    if (lcd_available && keypad_available) {
        if (xTaskCreate(admin_panel_task, "admin_panel", 4096, NULL, 4, NULL) != pdPASS) {
            printf("Admin panel task could not start; use the serial console\n");
        }
    }

    printf("CoCIS Smart Control System ready | RTC %s | SD %s | entry RFID %s | exit RFID %s | admin panel %s\n",
           rtc_available ? "available" : "fallback clock",
           sd_available ? "mounted" : "NVS fallback",
           rfid_reader_ready[0] ? "ready" : "unavailable",
           rfid_reader_ready[1] ? "ready" : "unavailable",
           lcd_available && keypad_available ? "ready" : "unavailable");
    fflush(stdout);
    print_help();
    run_serial_console();
}