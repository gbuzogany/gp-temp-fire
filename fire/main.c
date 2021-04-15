#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_rtc.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "boards.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_lbs.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_timer.h"

#include "max31865/max31865.h"

#include "lvgl/lvgl.h"
#include "nrf_gfx.h"
#include "nrf_delay.h"
#include <math.h>
#include "ble_temp.h"

extern const nrf_lcd_t nrf_lcd_st7735;
static const nrf_lcd_t * p_lcd = &nrf_lcd_st7735;

const nrf_drv_timer_t TEMP_TIMER = NRF_DRV_TIMER_INSTANCE(2);

#define PT100_RREF      430.0
#define PT100_RNOMINAL  100.0
#define FORCE_ON_MAX_DURATION 180

#define TX_POWER_LEVEL  (8)

// rtc
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
int _time_8hz = 0;
int _time_sec = 0;
int _time_min = 0;
// end rtc

static unsigned int _watchdog_clock = 0;
static unsigned int _last_update = 0;

static bool _force_heating = false;
static bool _connected = false;

// lvgl

lv_obj_t *fire_temp_label;
lv_obj_t *garage_temp_label;
lv_obj_t *fire_img;
lv_obj_t *garage_img;
// lv_obj_t *connected_icon;

LV_IMG_DECLARE(fire);
LV_IMG_DECLARE(garage);

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * LV_VER_RES_MAX / 10];                     /*Declare a buffer for 1/10 screen size*/

lv_disp_drv_t disp_drv;               /*Descriptor of a display driver*/
lv_indev_drv_t indev_drv;                  /*Descriptor of a input device driver*/

void my_disp_flush(lv_disp_drv_t * disp, const lv_area_t * area, lv_color_t * color_p)
{
    p_lcd->lcd_addr_window(area->x1, area->y1, area->x2, area->y2);
    size_t buf_len = (area->y2+1 - area->y1) * (area->x2+1 - area->x1) * 2;
    uint8_t buffer[buf_len];

    int32_t x, y, i = 0;
    for(y = area->y1; y <= area->y2; y++) {
        for(x = area->x1; x <= area->x2; x++) {
            buffer[i++] = color_p->full >> 8;
            buffer[i++] = color_p->full;
            color_p++;
        }
    }
    p_lcd->lcd_screen_flush(buffer, buf_len);

    lv_disp_flush_ready(disp);         /* Indicate you are ready with the flushing*/
}

bool my_touchpad_read(struct _lv_indev_drv_t * indev_drv, lv_indev_data_t * data)
{
    // data->state = touchpad_is_pressed() ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    // if(data->state == LV_INDEV_STATE_PR) touchpad_get_xy(&data->point.x, &data->point.y);

    return false; /*Return `false` because we are not buffering and no more data to read*/
}

static void event_handler_btn(lv_obj_t * obj, lv_event_t event){
    if(event == LV_EVENT_CLICKED) {

    }
}
// end lvgl

static void gfx_initialization(void)
{
    APP_ERROR_CHECK(nrf_gfx_init(p_lcd));
}

#define DEVICE_NAME                     "GP-Temp-Fire"                         /**< Name of device. Will be included in the advertising data. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                64                                      /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_DURATION                BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED   /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000)                  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50)                     /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {TEMP_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN }
};

BLE_TEMP_DEF(m_temp);
NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;                   /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];                    /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX];         /**< Buffer for storing an encoded scan data. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_scan_response_data,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX

    }
};

/**@brief Handler for shutdown preparation.
 */
bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    uint32_t err_code = NRF_SUCCESS;

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF:
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_SYSOFF");
            APP_ERROR_CHECK(err_code);
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_WAKEUP");
            UNUSED_VARIABLE(err_code);
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            return false;
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_RESET:
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_RESET");
            break;
    }

    err_code = app_timer_stop_all();
    APP_ERROR_CHECK(err_code);

    return true;
}

/**@brief Register application shutdown handler with priority 0. */
NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, 0);


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module, making it use the scheduler
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


static void switch_force_heating() {
    if (_force_heating == true) {
        _force_heating = false;
        NRF_LOG_INFO("Force heating OFF");
        lv_label_set_text(fire_temp_label, "OFF");
    }
    else {
        _force_heating = true;
        _time_8hz = 0;
        _time_sec = 0;
        _time_min = 0;
        lv_label_set_text(fire_temp_label, "ON");
    }
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    ret_code_t    err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&srdata, 0, sizeof(srdata));
    srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = m_adv_uuids;

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);

    ble_gap_adv_params_t adv_params;

    // Set advertising parameters.
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.primary_phy     = BLE_GAP_PHY_1MBPS;
    adv_params.duration        = APP_ADV_DURATION;
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;
    adv_params.p_peer_addr     = NULL;
    adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    adv_params.interval        = APP_ADV_INTERVAL;

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &adv_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the Custom Service Service events.
 *
 * @details This function will be called for all Custom Service events which are passed to
 *          the application.
 *
 * @param[in]   p_temp_service  Custom Service structure.
 * @param[in]   p_evt          Event received from the Custom Service.
 *
 */
static void on_temp_evt(ble_temp_t     * p_temp_service,
                       ble_temp_evt_t * p_evt)
{
    switch(p_evt->evt_type)
    {
        case BLE_TEMP_EVT_NOTIFICATION_ENABLED:
             break;

        case BLE_TEMP_EVT_NOTIFICATION_DISABLED:
            break;

        case BLE_TEMP_EVT_CONNECTED:
            break;

        case BLE_TEMP_EVT_DISCONNECTED:
              break;

        default:
              // No implementation needed.
              break;
    }
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_temp_init_t    temp_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    temp_init.evt_handler  = on_temp_evt;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.fire_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.fire_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.fire_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.garage_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.garage_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&temp_init.garage_md.write_perm);

    err_code = ble_temp_init(&m_temp, &temp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t           err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    // bsp_board_led_on(ADVERTISING_LED);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            _connected = true;
            // bsp_board_led_on(CONNECTED_LED);
            // bsp_board_led_off(ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            uint32_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, TX_POWER_LEVEL); 
            APP_ERROR_CHECK(err_code); 

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            _connected = false;
            // bsp_board_led_off(CONNECTED_LED);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);
            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
     if (button_action == APP_BUTTON_PUSH)
    {
        switch (pin_no)
        {
            case BSP_BUTTON_0:
                switch_force_heating();
                break;
            default:
                APP_ERROR_HANDLER(pin_no);
                break;
        }
    }
}

/**@brief Function for initializing the button handler module.
 */
static void buttons_init(void)
{
    ret_code_t err_code;

    //The array must be static because a pointer to it will be saved in the button handler module.
    static app_button_cfg_t buttons[] =
    {
        {BSP_BUTTON_0, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons),
                               BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    NRF_LOG_PROCESS();
    // if (NRF_LOG_PROCESS() == false)
    // {
    //     nrf_pwr_mgmt_run();
    // }
}

static bool should_read_temp = false;

void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    should_read_temp = true;
}

void timer_init() {
    uint32_t time_ms = 1000; 
    uint32_t time_ticks;
    uint32_t err_code;

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    err_code = nrf_drv_timer_init(&TEMP_TIMER, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_ms_to_ticks(&TEMP_TIMER, time_ms);
    nrf_drv_timer_extended_compare(
         &TEMP_TIMER, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TEMP_TIMER);
}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    _time_8hz++;
    if (_time_8hz >= 8) {
        _time_8hz = 0;
        _time_sec++;
        _watchdog_clock++;
    }
    if (_time_sec >= 60) {
        _time_sec = 0;
        _time_min++;
    }
    if (_time_min >= FORCE_ON_MAX_DURATION) {
        _time_min = 0;
        _force_heating = false;
        NRF_LOG_INFO("Force on turned OFF");
    }

    // if didn't run logic for more than 10 seconds
    if (_watchdog_clock - _last_update > 10) {
        // reset
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);
    }

}

static void rtc_config(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 4095;
    err_code = nrfx_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrfx_rtc_tick_enable(&rtc,true);

    //Power on RTC instance
    nrfx_rtc_enable(&rtc);
}

/**@brief Function for changing the tx power.
 */
static void tx_power_set(void)
{
    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, 0, TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */

int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    leds_init();
    timers_init();
    buttons_init();
    power_management_init();
    rtc_config();

    gfx_initialization();
    nrf_delay_ms(10);
    nrf_gfx_rotation_set(p_lcd, NRF_LCD_ROTATE_270);

    lv_init();

    static lv_style_t bigStyle;
    lv_style_init(&bigStyle);
    lv_style_set_text_font(&bigStyle, LV_STATE_DEFAULT, &lv_font_montserrat_36);

    lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * LV_VER_RES_MAX / 10);    /*Initialize the display buffer*/

    lv_disp_drv_init(&disp_drv);          /*Basic initialization*/
    disp_drv.flush_cb = my_disp_flush;    /*Set your driver function*/
    disp_drv.buffer = &disp_buf;          /*Assign the buffer to the display*/
    lv_disp_drv_register(&disp_drv);      /*Finally register the driver*/

    lv_indev_drv_init(&indev_drv);             /*Basic initialization*/
    indev_drv.type = LV_INDEV_TYPE_POINTER;    /*Touch pad is a pointer-like device*/
    indev_drv.read_cb = my_touchpad_read;      /*Set your driver function*/
    lv_indev_drv_register(&indev_drv);         /*Finally register the driver*/

    lv_obj_t *screenMain = lv_obj_create(NULL, NULL);
    fire_temp_label = lv_label_create(screenMain, NULL);
    lv_label_set_long_mode(fire_temp_label, LV_LABEL_LONG_BREAK);
    lv_label_set_text(fire_temp_label, "...");
    lv_label_set_align(fire_temp_label, LV_LABEL_ALIGN_RIGHT);
    lv_obj_set_size(fire_temp_label, 160, 40);
    lv_obj_set_pos(fire_temp_label, 0, 0);
    lv_obj_add_style(fire_temp_label, LV_LABEL_PART_MAIN, &bigStyle);

    garage_temp_label = lv_label_create(screenMain, NULL);
    lv_label_set_long_mode(garage_temp_label, LV_LABEL_LONG_BREAK);
    lv_label_set_text(garage_temp_label, "...");
    lv_label_set_align(garage_temp_label, LV_LABEL_ALIGN_RIGHT);
    lv_obj_set_size(garage_temp_label, 160, 40);
    lv_obj_set_pos(garage_temp_label, 0, 40);
    lv_obj_add_style(garage_temp_label, LV_LABEL_PART_MAIN, &bigStyle);

    lv_obj_t *fire_img = lv_img_create(screenMain, NULL);
    lv_img_set_src(fire_img, &fire);
    lv_obj_align(fire_img, NULL, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_pos(fire_img, 4, 0);

    lv_obj_t *garage_img = lv_img_create(screenMain, NULL);
    lv_img_set_src(garage_img, &garage);
    lv_obj_align(garage_img, NULL, LV_ALIGN_CENTER, 0, -20);
    lv_obj_set_pos(garage_img, 0, 40);

    // static lv_style_t obj_style;
    // lv_style_init(&obj_style);
    // lv_style_set_radius(&obj_style, LV_STATE_DEFAULT, 0);
    // lv_style_set_bg_color(&obj_style, LV_STATE_DEFAULT, LV_COLOR_RED);

    // connected_icon = lv_obj_create(lv_scr_act(), NULL);
    // lv_obj_add_style(connected_icon, LV_OBJ_PART_MAIN, &obj_style);
    // lv_obj_set_size(connected_icon, 40, 40);
    // lv_obj_set_pos(connected_icon, 40, 40);

    UNUSED_VARIABLE(event_handler_btn);

    lv_anim_t a;
    lv_anim_init(&a);

    lv_scr_load(screenMain);
    p_lcd->lcd_uninit();

    ble_stack_init();
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();

    // Start execution.
    NRF_LOG_INFO("GP-Temp-Fire started.");
    advertising_start();
    tx_power_set();

    APP_ERROR_CHECK(max31865_spi_init());
    max31865_init();
    max31865_spi_uninit();

    // Enter main loop.
    for (;;)
    {
        p_lcd->lcd_hw_init();

        lv_tick_inc(1);
        lv_task_handler();

        idle_state_handle();
        p_lcd->lcd_uninit();

        _last_update = _watchdog_clock;

        if (should_read_temp) {

            float temp;
            int16_t temp_i16;

            char buffer[64];
            if (_force_heating) {
                temp = 60.0f;
                sprintf(buffer, "ON");
                // sprintf(buffer, "%02d:%02d", _time_min, _time_sec);
                lv_label_set_text(fire_temp_label, buffer);
                temp_i16 = temp * 100.0f;
                if (_connected == true) {
                    ble_temp_fire_update(&m_temp, temp_i16);
                }
            }
            else {
                APP_ERROR_CHECK(max31865_spi_init());
                temp = max31865_temperature(PT100_RNOMINAL, PT100_RREF);
                max31865_spi_uninit();
                sprintf(buffer, "%.1f", temp);
                lv_label_set_text(fire_temp_label, buffer);
                temp_i16 = temp * 100.0f;
                if (_connected == true) {
                    ble_temp_fire_update(&m_temp, temp_i16);
                }
            }

            if (_connected == false) {
                lv_label_set_text(garage_temp_label, "--");
            }
            else {
                uint32_t err_code = ble_temp_garage_get(&m_temp, &temp_i16);
                if (err_code == NRF_SUCCESS) {
                    temp = temp_i16 / 100.0f;
                    sprintf(buffer, "%.1f", temp);
                    lv_label_set_text(garage_temp_label, buffer);
                }
            }
            
            should_read_temp = false;
        }
    }
}


/**
 * @}
 */
