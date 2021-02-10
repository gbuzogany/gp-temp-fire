#include "sdk_common.h"
#include "nrf_drv_spi.h"
#include "nrfx_spim.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"

static volatile bool spi_xfer_done = false;
extern nrfx_spim_t spi;

static void spim_event_handler(nrfx_spim_evt_t const * p_event, void *p_context)
{
    spi_xfer_done = true;
}

ret_code_t max31865_init(void)
{
    ret_code_t err_code;
    spi_xfer_done = false;

    nrfx_spim_config_t spi_config = NRFX_SPIM_DEFAULT_CONFIG;
    spi_config.frequency      = NRF_SPIM_FREQ_1M;
    spi_config.mode           = NRF_SPIM_MODE_0;
    spi_config.ss_pin         = MAX31865_SS_PIN;
    spi_config.miso_pin       = MAX31865_MISO_PIN;
    spi_config.mosi_pin       = MAX31865_MOSI_PIN;
    spi_config.sck_pin        = MAX31865_SCK_PIN;
    spi_config.use_hw_ss      = true;
    spi_config.ss_active_high = false;
    spi_config.ss_duration    = 8;

    err_code = nrfx_spim_init(&spi, &spi_config, spim_event_handler, NULL);

    return err_code;
}

void max31865_uninit(void) {
    nrfx_spim_uninit(&spi);
}