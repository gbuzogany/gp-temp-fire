# gp-temp-fire

- fire: nrf52 with an ST7735S display and a MAX31865 to read the temperature of a RTD sensor and receive/transmit temp measurements via BLE.
- garage: nrf52 with a heating element (HE) and a thermistor, reading temperatures from `fire` via BLE, setting the HE to the same temperature, and writing the HE temperature back to `fire`.
