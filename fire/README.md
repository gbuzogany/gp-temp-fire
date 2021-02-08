# gp-temp-fire

This is the firmware for the device that's measuring the fire temperature.

This device has:

- ST7735S display (SPI)
- MAX31865 to read a Platinum RTD (SPI)
- DS3231 RTC (I2C)
- two user buttons
	- one to the display on/off
	- one to set it to manual mode

It hosts a BLE service using NUS (Nordic UART Service).