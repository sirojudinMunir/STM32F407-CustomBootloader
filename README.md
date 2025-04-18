# STM32F407 Firmware Update via SD Card with Custom Bootloader

This system enables firmware updates on STM32F407-based devices simply by copying a `.bin` file to an SD card. The custom bootloader automatically reads and writes the new firmware to the MCU’s internal flash, without requiring a debugger or additional tools.

This system was tested using the project [STM32F407-LCD-Camera-SDCard](https://github.com/sirojudinMunir/STM32F407-LCD-Camera-SDCard) as the main application.

## Key Features

- Custom bootloader with `.bin` file support from SD card
- Automatic detection and update when a new firmware file is available

## Demo

https://youtu.be/p20jer7IenY?si=01RXiO1Az-_TIbf0

## Requirements

- STM32F407VGT6 Board (as per [schematic](https://github.com/sirojudinMunir/STM32F407-LCD-Camera-SDCard/blob/master/STM32-CAM_schematics.pdf))
- STM32CubeIDE
- SD card (FAT32)
- USB cable (for MSC connection)
- `.bin` file from the main application build

## How to Use

1. **Build** both `Bootloader` and `MainApp` projects.
2. **Flash** the bootloader to the STM32 internal flash using ST-Link or any debugger.
3. **Copy** the `.bin` file from the `MainApp` to the root directory of the SD card.
4. **Insert** the SD card into the board and power up the device.
5. The bootloader will detect and write the new firmware to the main application address.
6. The system will then automatically jump to the main application.

## Technical Notes

- The `.bin` file must be placed in the root directory and named appropriately (e.g., `firmware.bin`).
- The main application flash address can be configured in the linker script (e.g., `0x08008000`).
- After the update, the bootloader performs a clean vector reset jump to the main application.

## Author

**Moh Sirojudin Munir**  
Embedded systems developer  
[LinkedIn](https://www.linkedin.com/in/moh-sirojudin-munir-3b01561b1) | [GitHub](https://github.com/sirojudinMunir) | [YouTube](https://www.youtube.com/@srj4555)
