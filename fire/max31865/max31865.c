#include "sdk_common.h"
#include "nrf_drv_spi.h"
#include "nrfx_spim.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include <math.h>
#include "boards.h"
#include "max31865.h"

#define MAX31865_CONFIG_REG             0x00
#define MAX31865_CONFIG_BIAS            0x80
#define MAX31865_CONFIG_MODEAUTO        0x40
#define MAX31865_CONFIG_MODEOFF         0x00
#define MAX31865_CONFIG_1SHOT           0x20
#define MAX31865_CONFIG_3WIRE           0x10
#define MAX31865_CONFIG_24WIRE          0x00
#define MAX31865_CONFIG_FAULTSTAT       0x02
#define MAX31865_CONFIG_FILT50HZ        0x01
#define MAX31865_CONFIG_FILT60HZ        0x00

#define MAX31865_RTDMSB_REG             0x01
#define MAX31865_RTDLSB_REG             0x02
#define MAX31865_HFAULTMSB_REG          0x03
#define MAX31865_HFAULTLSB_REG          0x04
#define MAX31865_LFAULTMSB_REG          0x05
#define MAX31865_LFAULTLSB_REG          0x06
#define MAX31865_FAULTSTAT_REG          0x07


#define MAX31865_FAULT_HIGHTHRESH       0x80
#define MAX31865_FAULT_LOWTHRESH        0x40
#define MAX31865_FAULT_REFINLOW         0x20
#define MAX31865_FAULT_REFINHIGH        0x10
#define MAX31865_FAULT_RTDINLOW         0x08
#define MAX31865_FAULT_OVUV             0x04

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

static volatile bool spi_xfer_done = false;
extern const nrfx_spim_t spi;

static void spim_event_handler(nrfx_spim_evt_t const * p_event, void *p_context)
{
    spi_xfer_done = true;
}

void readRegisterN(uint8_t addr, uint8_t *buffer, uint8_t n)
{
	addr &= 0x7F;
    uint8_t rx_buffer[n+1];

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TRX(&addr, 1, rx_buffer, n+1);
    APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));
    
    while (!spi_xfer_done) {}
    spi_xfer_done = false;

    memcpy(buffer, &rx_buffer[1], n);
}

void writeRegister8(uint8_t addr, uint8_t data) {
    addr |= 0x80;

    uint8_t buffer[2] = {addr, data};

    nrfx_spim_xfer_desc_t xfer_desc = NRFX_SPIM_XFER_TX(buffer, 2);
    APP_ERROR_CHECK(nrfx_spim_xfer(&spi, &xfer_desc, 0));

    while (!spi_xfer_done) {}
    spi_xfer_done = false;
}

uint8_t readRegister8(uint8_t addr) {
    uint8_t ret = 0;
    readRegisterN(addr, &ret, 1);

    return ret;
}

uint16_t readRegister16(uint8_t addr) {
    uint8_t buffer[2] = {0, 0};
    readRegisterN(addr, buffer, 2);

    uint16_t ret = buffer[0];
    ret <<= 8;
    ret |= buffer[1];

    return ret;
}

uint8_t readFault(void) {
  return readRegister8(MAX31865_FAULTSTAT_REG);
}

void clearFault(void) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    t &= ~0x2C;
    t |= MAX31865_CONFIG_FAULTSTAT;
    writeRegister8(MAX31865_CONFIG_REG, t);
}

void enable50Hz(bool b) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (b) {
        t |= MAX31865_CONFIG_FILT50HZ;
    } else {
        t &= ~MAX31865_CONFIG_FILT50HZ;
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}

void enableBias(bool b) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (b) {
        t |= MAX31865_CONFIG_BIAS;
    } else {
        t &= ~MAX31865_CONFIG_BIAS;
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}

void setWires(max31865_numwires_t wires) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (wires == MAX31865_3WIRE) {
        t |= MAX31865_CONFIG_3WIRE;
    } else {
        t &= ~MAX31865_CONFIG_3WIRE;
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}

void autoConvert(bool b) {
    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    if (b) {
        t |= MAX31865_CONFIG_MODEAUTO;
    } else {
        t &= ~MAX31865_CONFIG_MODEAUTO;
    }
    writeRegister8(MAX31865_CONFIG_REG, t);
}

uint16_t readRTD(void) {
    clearFault();
    enableBias(true);
    nrf_delay_ms(10);

    uint8_t t = readRegister8(MAX31865_CONFIG_REG);
    t |= MAX31865_CONFIG_1SHOT;
    writeRegister8(MAX31865_CONFIG_REG, t);
    nrf_delay_ms(65);

    uint16_t rtd = readRegister16(MAX31865_RTDMSB_REG);

    enableBias(false); // Disable bias current again to reduce selfheating.

    // remove fault
    rtd >>= 1;

    return rtd;
}

float max31865_temperature(float RTDnominal, float refResistor) {
  float Z1, Z2, Z3, Z4, Rt, temp;

  Rt = readRTD();
  Rt /= 32768;
  Rt *= refResistor;

  // Serial.print("\nResistance: "); Serial.println(Rt, 8);

  Z1 = -RTD_A;
  Z2 = RTD_A * RTD_A - (4 * RTD_B);
  Z3 = (4 * RTD_B) / RTDnominal;
  Z4 = 2 * RTD_B;

  temp = Z2 + (Z3 * Rt);
  temp = (sqrt(temp) + Z1) / Z4;

  if (temp >= 0)
    return temp;

  // ugh.
  Rt /= RTDnominal;
  Rt *= 100; // normalize to 100 ohm

  float rpoly = Rt;

  temp = -242.02;
  temp += 2.2228 * rpoly;
  rpoly *= Rt; // square
  temp += 2.5859e-3 * rpoly;
  rpoly *= Rt; // ^3
  temp -= 4.8260e-6 * rpoly;
  rpoly *= Rt; // ^4
  temp -= 2.8183e-8 * rpoly;
  rpoly *= Rt; // ^5
  temp += 1.5243e-10 * rpoly;

  return temp;
}

void max31865_init(void)
{
    setWires(MAX31865_3WIRE);
    enableBias(false);
    autoConvert(false);
    clearFault();
}

ret_code_t max31865_spi_init(void)
{
    ret_code_t err_code;
    spi_xfer_done = false;

    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_1M;
    spi_config.mode           = NRF_SPIM_MODE_1;
    spi_config.ss_pin         = MAX31865_SS_PIN;
    spi_config.miso_pin       = MAX31865_MISO_PIN;
    spi_config.mosi_pin       = MAX31865_MOSI_PIN;
    spi_config.sck_pin        = MAX31865_SCK_PIN;
    spi_config.bit_order      = NRF_SPIM_BIT_ORDER_MSB_FIRST;

    err_code = nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL);
    return err_code;
}

void max31865_spi_uninit(void) {
    nrfx_spim_uninit(&spi);
}