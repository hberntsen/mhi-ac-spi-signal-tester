# mhi-ac-spi-signal-tester

This project is a stripped down version of [mhi-ac-ctrl-esp32-c3](https://github.com/hberntsen/mhi-ac-ctrl-esp32-c3). It can be used to check whether incoming SPI frames from the [mhi-ac-spi-signal-generator](https://github.com/hberntsen/mhi-ac-spi-signal-generator) are received correctly.

## Flashing

1. Make sure to set up the ESP-IDF v5.5.3 toolchain. See the [Espressif documentation](https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32s3/get-started/linux-macos-setup.html)
2. Run `idf.py set-target esp32s3`.
3. Switch the log to USB: `idf.py menuconfig`. Search for `ESP_CONSOLE_USB_SERIAL_JTAG` and select `USB Serial/JTAG Controller`.
4. Configure the CLK and MOSI pins in the `main.cpp` file.
5. Flash your board with `idf.py flash`
