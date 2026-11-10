# Custom RC522 driver integration and test status

## What changed

The project now uses a small SPI-level RC522 driver in `src/main.c` instead of calling the bundled RC522 component. Both readers use chip select GPIO32/33 and share SCK/MISO/MOSI on GPIO18/19/23. The driver uses an MFRC522 soft reset, so the reader reset pins are not wired; GPIO13/14 now drive the entry/exit servos.

The driver sends REQA and anticollision commands, reads the FIFO in one burst, validates the four-byte UID block check character, and propagates SPI transaction errors. A completed poll with an empty FIFO is classified as a normal no-card timeout. Unexpected SPI errors are reported once per reader until a successful read.

## Verification state

- `pio run -e esp32dev`: passed; firmware image generated.
- `python3 tests/verify_static.py`: 142/142 passed, including diagram, wiring, and empty-FIFO timeout checks.
- Wokwi runtime test on 31 October 2026: both RC522 readers detected and granted the configured card; no `ESP_ERR_INVALID_SIZE` appeared during empty polling or after taps.
- The same run verified room PIR/light and queue updates, keypad/LCD login and status, PIN lockout, emergency alarm/egress, SD log reads, and the closing warning.
- Physical reader hardware: not tested.

The earlier `ESP_ERR_INVALID_SIZE` reports occurred when a poll completed without FIFO data. Treating an empty FIFO as a timeout removed those false communication errors in the updated Wokwi run. The driver still supports four-byte UIDs only.

## Wokwi test procedure

1. Build the project with `pio run -e esp32dev`, then open `diagram.json` in Wokwi.
2. Confirm startup reports the entry and exit readers `READY`, both room terminals online, and the admin panel ready.
3. Use **Tap** on the entry reader's Blue Card. Expect a UID detection, access decision, entry LED/servo movement, and `WELCOME` on the LCD for a granted scan.
4. Present the same card at entry again without exiting. Expect a duplicate-inside denial and no second occupant entry.
5. Use **Tap** on the exit reader. Expect the exit decision, exit LED/servo movement, `GOODBYE`, and an updated occupancy/log.
6. Type `status`; verify Room 1 and Room 2 show online while their queue messages are arriving. Test PIRs independently and confirm their own lights time out after ten seconds.
7. Test three incorrect administrator PIN attempts across serial/keypad; login should remain locked for 30 seconds. Verify the help output does not reveal the configured PIN.
8. Test the LCD result/reason messages and set the demonstration clock within ten minutes of closing to check the warning.
9. Press and release the emergency-egress button. Verify the exit servo/indicator and pulsed buzzer activate and stop as expected.

The serial `scan in UID` and `scan out UID` commands exercise access logic only; they do not test the physical/simulated RC522 transaction path. The current driver supports four-byte UIDs only. Out-of-schedule RFID denial, SD-card failure fallback, and NVS migration across restart were not part of the latest runtime test.
