# SmartTrap Gen3
Econode SmartTrap Gen3 LoRa node

https://www.econode.nz/

![Gen3 Photo](https://raw.githubusercontent.com/econode/SmartTrap_Gen3/c9adf3386ccf46bc1ed8e167943ade23973ee94a/images/SmartTrap_Gen3_photo1.jpg)

Processor Atmel SAMD21G https://www.microchip.com/en-us/product/ATsamd21g18
CPU Clock 48Mhz
RAM 32KB
FLASH 256KB

Compatible with Adafruit M0 https://www.adafruit.com/product/3403

Power supply battery or USB, Absolute max 12volts / 150ma

Radio Lora / HopeRF RFM95, 868Mhz

To use sample code
 - Install VisualStudio
 - Install PlatformIO extension for VisualStudio
 - Clone this repository
 - Open example file in 'code' section


Gen3 node should have SAM-BA bootloader, double press of reset should enter bootloader mode.

Sensor connector is "Grove" compatible so should work with Grove sensors https://wiki.seeedstudio.com/Grove_System/
 
Programing and debugging with J-Link / SWD. Use 4 "prog pins", SWD/SWC/3V3/GND
