# gp-temp-garage

This is the firmware for the device that controls a heating element connected to the temp sensor in the garage.

This device has:

- 2x thermistors (analog inputs)
- 1x PTC heating element (5v, max 60˚, controlled by an IRFB8721)

It connects to `gp-temp-fire` via BLE using NUS (Nordic UART Service).