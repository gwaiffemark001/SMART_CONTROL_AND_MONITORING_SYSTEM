#!/usr/bin/env python3
"""Static project-integrity checks; these do not replace an ESP-IDF build or Wokwi run."""
from pathlib import Path
import json
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
MAIN = (ROOT / "src" / "main.c").read_text(encoding="utf-8")
DIAGRAM = json.loads((ROOT / "diagram.json").read_text(encoding="utf-8"))

checks = []
def check(name, condition, detail=""):
    checks.append((name, bool(condition), detail))

# Required project files and parseable simulator diagram.
for rel in ("CMakeLists.txt", "src/CMakeLists.txt", "platformio.ini", "wokwi.toml", "README.md"):
    check(f"project file exists: {rel}", (ROOT / rel).is_file())
check("diagram JSON parses", isinstance(DIAGRAM, dict))
wokwi_config = (ROOT / "wokwi.toml").read_text(encoding="utf-8")
check("Wokwi config targets PlatformIO firmware artifacts", 'firmware = ".pio/build/esp32dev/firmware.bin"' in wokwi_config and 'elf = ".pio/build/esp32dev/firmware.elf"' in wokwi_config)
root_cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
app_cmake = (ROOT / "src" / "CMakeLists.txt").read_text(encoding="utf-8")
check("root CMake loads the standalone ESP-IDF application", "project(cocis_smart_control)" in root_cmake and "idf_component_register" in app_cmake)
check(
    "ESP-IDF source component declares its driver and system dependencies",
    all(
        component in app_cmake
        for component in (
            "esp_driver_gpio",
            "esp_driver_i2c",
            "esp_driver_ledc",
            "esp_driver_sdspi",
            "esp_driver_spi",
            "esp_driver_uart",
            "esp_system",
            "fatfs",
            "freertos",
            "nvs_flash",
            "sdmmc",
        )
    ),
)
intellisense_config = json.loads((ROOT / ".vscode" / "c_cpp_properties.json").read_text(encoding="utf-8"))
intellisense_entry = intellisense_config["configurations"][0]
check(
    "C/C++ IntelliSense uses the project compile database, not stale host paths",
    intellisense_entry.get("compileCommands")
    == "${workspaceFolder}/.pio/build/esp32dev/compile_commands.json"
    and not any(
        "C:/Users/" in value
        for value in intellisense_entry.values()
        if isinstance(value, str)
    ),
)
for config_name in ("sdkconfig", "sdkconfig.esp32dev"):
    sdkconfig = (ROOT / config_name).read_text(encoding="utf-8")
    check(
        f"{config_name} flash size matches the 4MB esp32dev board",
        'CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y' in sdkconfig
        and 'CONFIG_ESPTOOLPY_FLASHSIZE="4MB"' in sdkconfig
        and "CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y" not in sdkconfig,
    )
parts = {part["id"]: part for part in DIAGRAM.get("parts", [])}
check("one ESP32 board is present", sum(p.get("type") == "board-esp32-devkit-c-v4" for p in parts.values()) == 1)
check("entry RFID reader present", parts.get("entry_rfid", {}).get("type") == "board-mfrc522")
check("exit RFID reader present", parts.get("exit_rfid", {}).get("type") == "board-mfrc522")
check(
    "both Blue Card UIDs match an authorized firmware UID",
    all(
        parts.get(reader, {}).get("attrs", {}).get("uid", "").replace(":", "").upper() in MAIN
        for reader in ("entry_rfid", "exit_rfid")
    ),
)
for part_id, part_type in [
    ("pir1", "wokwi-pir-motion-sensor"), ("pir2", "wokwi-pir-motion-sensor"),
    ("sd1", "wokwi-microsd-card"), ("rtc1", "wokwi-ds1307"),
    ("egress", "wokwi-pushbutton"),
    ("admin_lcd", "wokwi-lcd1602"),
    ("admin_keypad", "wokwi-membrane-keypad"),
    ("keypad_expander", "chip-mcp23017"),
    ("emergency_alarm", "wokwi-buzzer"),
    ("led1", "wokwi-led"), ("led2", "wokwi-led"),
    ("led3", "wokwi-led"), ("led4", "wokwi-led"),
    ("r1", "wokwi-resistor"), ("r2", "wokwi-resistor"),
    ("r3", "wokwi-resistor"), ("r4", "wokwi-resistor"),
]:
    check(f"required diagram part exists: {part_id}", parts.get(part_id, {}).get("type") == part_type)
check("four LED indicators have their intended colors", [parts.get(f"led{i}", {}).get("attrs", {}).get("color") for i in range(1, 5)] == ["yellow", "yellow", "green", "green"])

# Validate required wiring from diagram.json.
connections = {frozenset((c[0], c[1])) for c in DIAGRAM.get("connections", [])}
def wired(a, b):
    return frozenset((a, b)) in connections
for endpoint_a, endpoint_b in [
    ("entry_rfid:SDA", "esp:32"),
    ("exit_rfid:SDA", "esp:33"),
    ("entry_rfid:SCK", "esp:18"), ("exit_rfid:SCK", "esp:18"),
    ("entry_rfid:MISO", "esp:19"), ("exit_rfid:MISO", "esp:19"),
    ("entry_rfid:MOSI", "esp:23"), ("exit_rfid:MOSI", "esp:23"),
    ("esp:2", "r1:1"), ("esp:4", "r2:1"),
    ("esp:16", "r3:1"), ("esp:17", "r4:1"),
    ("pir1:VCC", "esp:3V3"), ("pir1:GND", "esp:GND.1"), ("pir1:OUT", "esp:27"),
    ("pir2:VCC", "esp:3V3"), ("pir2:GND", "esp:GND.1"), ("pir2:OUT", "esp:26"),
    ("esp:18", "sd1:SCK"), ("esp:19", "sd1:DO"), ("esp:23", "sd1:DI"),
    ("esp:5", "sd1:CS"), ("sd1:VCC", "esp:3V3"), ("sd1:GND", "esp:GND.1"),
    ("esp:21", "rtc1:SDA"), ("esp:22", "rtc1:SCL"),
    ("rtc1:5V", "esp:5V"), ("rtc1:GND", "esp:GND.1"),
    ("admin_lcd:VCC", "esp:3V3"), ("admin_lcd:GND", "esp:GND.1"),
    ("admin_lcd:SDA", "esp:21"), ("admin_lcd:SCL", "esp:22"),
    ("keypad_expander:SDA", "esp:21"), ("keypad_expander:SCL", "esp:22"),
    ("admin_keypad:R1", "keypad_expander:GPA0"),
    ("admin_keypad:R2", "keypad_expander:GPA1"),
    ("admin_keypad:R3", "keypad_expander:GPA2"),
    ("admin_keypad:R4", "keypad_expander:GPA3"),
    ("admin_keypad:C1", "keypad_expander:GPA4"),
    ("admin_keypad:C2", "keypad_expander:GPA5"),
    ("admin_keypad:C3", "keypad_expander:GPA6"),
    ("admin_keypad:C4", "keypad_expander:GPA7"),
    ("emergency_alarm:1", "esp:GND.2"),
    ("emergency_alarm:2", "esp:15"),
    ("esp:25", "egress:1.l"), ("egress:2.l", "esp:GND.1"),
    ("servo_entry:PWM", "esp:13"), ("servo_entry:V+", "esp:5V"),
    ("servo_entry:GND", "esp:GND.1"),
    ("servo_exit:PWM", "esp:14"), ("servo_exit:V+", "esp:5V"),
    ("servo_exit:GND", "esp:GND.1"),
    ("entry_rfid:3.3V", "esp:3V3"), ("entry_rfid:GND", "esp:GND.1"),
    ("exit_rfid:3.3V", "esp:3V3"), ("exit_rfid:GND", "esp:GND.1"),
    ("r1:2", "led1:A"), ("led1:C", "esp:GND.2"),
    ("r2:2", "led2:A"), ("led2:C", "esp:GND.2"),
    ("r3:2", "led3:A"), ("led3:C", "esp:GND.2"),
    ("r4:2", "led4:A"), ("led4:C", "esp:GND.2"),
]:
    check(f"wiring {endpoint_a} <-> {endpoint_b}", wired(endpoint_a, endpoint_b))
check(
    "MCP23017 Wokwi custom-chip dependency is pinned",
    DIAGRAM.get("dependencies", {}).get("chip-mcp23017")
    == "github:mohankumargupta/wokwi-mcp23017@0.0.1",
)

# Verify project configuration and source behaviour.
check("both RFID readers initialized", bool(re.search(r"for\s*\(size_t\s+reader\s*=\s*0;\s*reader\s*<\s*2;\s*reader\+\+\)", MAIN)))
check("custom RC522 driver checks SPI transaction results", "esp_err_t rc522_read(" in MAIN and "return spi_device_transmit(device, &transaction);" in MAIN)
check("custom RC522 driver performs REQA and UID anticollision", "const uint8_t request = 0x26;" in MAIN and "const uint8_t anticollision[2] = { 0x93, 0x20 };" in MAIN)
check("one RFID polling task serializes both readers on the SPI mutex", "rfid_bus_task_mutex = xSemaphoreCreateMutex();" in MAIN and "xSemaphoreTake(rfid_bus_task_mutex, portMAX_DELAY);" in MAIN and "rc522_read_uid(rfid_device[reader], uid)" in MAIN)
check("both RFID readers initialize before polling starts", MAIN.index("rc522_start_reader(rfid_device[reader])") < MAIN.index('xTaskCreate(rfid_poll_task, "rfid_poll"'))
check("console-simulated scans share the RFID bus mutex", MAIN.count("xSemaphoreTake(rfid_bus_task_mutex, portMAX_DELAY);") >= 2 and "handle_scan(direction, uid);" in MAIN)
check("SD log reads use the shared SPI mutex", "static void print_logs(void)" in MAIN and "fopen(SD_LOG_PATH, \"r\")" in MAIN and "xSemaphoreGive(rfid_bus_task_mutex);" in MAIN)
check("both RFID chip-select lines are set inactive", "gpio_set_level(ENTRY_RFID_CS_PIN, 1);" in MAIN and "gpio_set_level(EXIT_RFID_CS_PIN, 1);" in MAIN)
check("SD chip-select is also held inactive before card startup", "gpio_set_level(SD_CS_PIN, 1);" in MAIN and "(1ULL << SD_CS_PIN)" in MAIN)
app = MAIN[MAIN.index("void app_main(void)"):]
check("SD mounts before RFID reader startup", app.index("mount_sd_card();") < app.index("initialize_rfid_readers();"))
check("entry and exit ready messages are exposed", '"RFID %s reader READY' in MAIN and '"RFID readers: entry %s, exit %s' in MAIN)
check("RFID UID event is printed to terminal", '"RFID %s reader detected UID %s\\n"' in MAIN)
check("empty RFID polls are treated as expected timeouts", "result != ESP_ERR_TIMEOUT" in MAIN and "ESP_ERR_TIMEOUT" in MAIN)
check("empty RC522 FIFO is treated as no-card timeout", "if (count == 0)" in MAIN and "return ESP_ERR_TIMEOUT;" in MAIN[MAIN.index("size_t count = fifo_count;"):MAIN.index("result = rc522_read_fifo(device, reply, count);")])
check("RFID SPI failures are reported once per reader until recovery", "spi_error_reported[reader]" in MAIN and "reader communication failed: %s" in MAIN)
check("granted and denied outcomes are printed", '"ACCESS GRANTED at %s for UID %s\\n"' in MAIN and '"ACCESS DENIED at %s: %s\\n"' in MAIN)
check("schedule supports overnight windows", "return now >= opens || now < closes;" in MAIN)
check("access state is mutex protected", "xSemaphoreCreateMutex()" in MAIN and "xSemaphoreTake(state_mutex, portMAX_DELAY)" in MAIN)
check("NVS and SD logging exist", "nvs_set_blob(nvs_store, \"access_log\"" in MAIN and "esp_vfs_fat_sdspi_mount(" in MAIN)
check("PIR light auto-off hold is present", "MOTION_HOLD_MS 10000" in MAIN and "gpio_set_level(light_pins[room], light_on)" in MAIN and "(now - last_motion) >= pdMS_TO_TICKS(MOTION_HOLD_MS)" in MAIN)
check("emergency egress bypass remains present", "door_output(door_pins[1], pressed || exit_pulse_active);" in MAIN)
check("admin panel shares the RTC I2C bus", "LCD_I2C_ADDRESS 0x27" in MAIN and "KEYPAD_I2C_ADDRESS 0x20" in MAIN)
check("LCD row updates are batched into two I2C transfers", "uint8_t transfer[102];" in MAIN and "transfer_length, 100" in MAIN and "lcd_encode_byte((uint8_t)character" in MAIN)
check("keypad uses two-sample debounce at a responsive polling interval", "stable_samples < 1" in MAIN and "pdMS_TO_TICKS(40)" in MAIN)
check("keypad expander configures row outputs and column pull-ups", "MCP23017_IODIRA_REG, 0xF0" in MAIN and "MCP23017_GPPUA_REG, 0xF0" in MAIN)
check("keypad PIN login uses the shared administrator authentication", "authenticate_admin_pin(input)" in MAIN and "PANEL_PIN" in MAIN)
check("keypad admin menu updates schedule and demo clock", "update_schedule(proposed_open, minute)" in MAIN and "update_demo_clock(minute)" in MAIN)
check("emergency egress remains independent of the admin panel", "gpio_get_level(EMERGENCY_EGRESS_PIN) == 0" in MAIN and "door_output(door_pins[1], pressed || exit_pulse_active);" in MAIN)
check("emergency alarm uses a pulsed LEDC buzzer output", "EMERGENCY_ALARM_PIN GPIO_NUM_15" in MAIN and "freq_hz = 2000" in MAIN and "elapsed_ms % 600U" in MAIN)
check("emergency button activates alarm and release deactivates it", "if (pressed && !was_pressed)" in MAIN and "bool beep_on = pressed &&" in MAIN)
check("occupancy capacity uses one shared 64-person setting", "#define MAX_OCCUPANTS 64" in MAIN and "char inside[MAX_OCCUPANTS][UID_LENGTH]" in MAIN and "state.inside_count >= MAX_OCCUPANTS" in MAIN)
check("failed administrator PINs trigger a 30-second shared lockout", "#define LOGIN_MAX_FAILURES 3" in MAIN and "#define LOGIN_LOCKOUT_MS 30000" in MAIN and "lockout_until = now + pdMS_TO_TICKS(LOGIN_LOCKOUT_MS)" in MAIN and MAIN.count("authenticate_admin_pin(") >= 3)
check("console help does not disclose the configured PIN", '"  login <PIN>' in MAIN and '"  login 2468' not in MAIN)
check("LCD shows access outcome and reason", 'lcd_message("ACCESS DENIED", denial_short)' in MAIN and 'lcd_message(entering ? "WELCOME" : "GOODBYE", uid)' in MAIN)
check("LCD warns within the configured closing window", "#define CLOSING_WARNING_MIN 10" in MAIN and '"Closes in %d min"' in MAIN)
check("entry and exit servos use GPIO13 and GPIO14", "SERVO_ENTRY_PIN GPIO_NUM_13" in MAIN and "SERVO_EXIT_PIN GPIO_NUM_14" in MAIN and "wokwi-servo" in str([part.get("type") for part in parts.values()]))
check("room terminals publish status through a FreeRTOS queue", "xQueueCreate(8, sizeof(room_message_t))" in MAIN and "xQueueSend(terminal_queue, &message, 0)" in MAIN and "xQueueReceive(terminal_queue" in MAIN)
check("room task startup and online/offline status are exposed", '"room1_terminal"' in MAIN and '"room2_terminal"' in MAIN and '"Room %u: %s, light %s, terminal %s\\n"' in MAIN and "pdMS_TO_TICKS(3000)" in MAIN)
check("room-terminal queue creation failure is reported", '"FATAL: could not create room-terminal message queue' in MAIN)
check("old 16-person NVS snapshots migrate to the 64-person format", "#define LEGACY_MAX_OCCUPANTS 16" in MAIN and "legacy_occupant_snapshot_t" in MAIN and "Migrated saved occupants" in MAIN)
check("occupant NVS save and commit errors are reported", "Occupant snapshot save failed: %s" in MAIN and "Occupant snapshot migration save failed: %s" in MAIN)
check("room queue and task startup failures are reported", "status queue is full" in MAIN and "Room-terminal receiver task could not start" in MAIN and "terminal task could not start" in MAIN)
section_order = [
    MAIN.find("/* Talituleka Hope Mwelu"),
    MAIN.find("/* Ogesa Patrick Paul"),
    MAIN.find("/* Mugabe Gideon"),
    MAIN.find("/* Sekiranda Emma Michael"),
    MAIN.find("/* Gwaiffe Mark"),
    MAIN.find("/* Shared system startup and integration */"),
]
check("named contributor sections remain in chronological order", all(pos >= 0 for pos in section_order) and section_order == sorted(section_order))
sekiranda_section = MAIN[section_order[3]:section_order[4]]
check("servo and shared door-output helpers are grouped with egress work", all(name in sekiranda_section for name in ("servo_write(", "initialize_servos(", "door_output(")))

# Basic balanced-delimiter scan while ignoring comments and literals.
def strip_c_literals(source):
    out = []
    i = 0
    state = "code"
    while i < len(source):
        ch = source[i]
        nxt = source[i + 1] if i + 1 < len(source) else ""
        if state == "code":
            if ch == '/' and nxt == '/':
                state = "line_comment"; out.extend("  "); i += 2; continue
            if ch == '/' and nxt == '*':
                state = "block_comment"; out.extend("  "); i += 2; continue
            if ch == '"':
                state = "string"; out.append(' '); i += 1; continue
            if ch == "'":
                state = "char"; out.append(' '); i += 1; continue
            out.append(ch); i += 1; continue
        if state == "line_comment":
            if ch == '\n': state = "code"; out.append('\n')
            else: out.append(' ')
            i += 1; continue
        if state == "block_comment":
            if ch == '*' and nxt == '/': state = "code"; out.extend("  "); i += 2
            else:
                out.append('\n' if ch == '\n' else ' '); i += 1
            continue
        if state in ("string", "char"):
            delim = '"' if state == "string" else "'"
            if ch == '\\': out.extend("  "); i += 2; continue
            if ch == delim: state = "code"
            out.append(' '); i += 1; continue
    return ''.join(out)

clean = strip_c_literals(MAIN)
for left, right, label in [('{', '}', "braces"), ('(', ')', "parentheses"), ('[', ']', "brackets")]:
    depth = 0
    balanced = True
    for ch in clean:
        if ch == left: depth += 1
        elif ch == right:
            depth -= 1
            if depth < 0:
                balanced = False; break
    balanced = balanced and depth == 0
    check(f"C source {label} balanced", balanced)

failures = [item for item in checks if not item[1]]
for name, ok, detail in checks:
    print(f"{'PASS' if ok else 'FAIL'} - {name}" + (f" ({detail})" if detail else ""))
print(f"\nStatic checks: {len(checks) - len(failures)}/{len(checks)} passed.")
if failures:
    sys.exit(1)
print("NOTE: This is static validation only. It does not compile the firmware or execute a Wokwi simulation.")
