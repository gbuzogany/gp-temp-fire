#include "sdk_common.h"
#include "ble_temp.h"
#include <string.h>
#include "ble_srv_common.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_temp       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_temp_t * p_temp, ble_evt_t const * p_ble_evt)
{
    p_temp->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    ble_temp_evt_t evt;

    evt.evt_type = BLE_TEMP_EVT_CONNECTED;

    p_temp->evt_handler(p_temp, &evt);
}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_temp_t * p_temp, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_temp->conn_handle = BLE_CONN_HANDLE_INVALID;
    
    ble_temp_evt_t evt;

    evt.evt_type = BLE_TEMP_EVT_DISCONNECTED;

    p_temp->evt_handler(p_temp, &evt);
}

/**@brief Function for handling the Write event.
 *
 * @param[in]   p_temp       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write(ble_temp_t * p_temp, ble_evt_t const * p_ble_evt)
{
    NRF_LOG_INFO("Write event");
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Custom Value Characteristic Written to.
    if (p_evt_write->handle == p_temp->characteristic_handles[TEMP_FIRE_CHAR])
    {
        NRF_LOG_INFO("Wrote to FIRE");
        // nrf_gpio_pin_toggle(LED_1);
        /*
        if(*p_evt_write->data == 0x01)
        {
            nrf_gpio_pin_clear(20); 
        }
        else if(*p_evt_write->data == 0x02)
        {
            nrf_gpio_pin_set(20); 
        }
        else
        {
          //Do nothing
        }
        */
    }

    if (p_evt_write->handle == p_temp->characteristic_handles[TEMP_GARAGE_CHAR])
    {
        NRF_LOG_INFO("Wrote to GARAGE %d", p_evt_write->len);
        NRF_LOG_HEXDUMP_INFO(p_evt_write->data, p_evt_write->len);
    }

    // Check if the Custom value CCCD is written to and that the value is the appropriate length, i.e 2 bytes.
    if ((p_evt_write->handle == p_temp->characteristic_handles[TEMP_FIRE_CCCD])
        && (p_evt_write->len == 2)
       )
    {
        NRF_LOG_INFO("Wrote to cccd_handle");
        // CCCD written, call application event handler
        if (p_temp->evt_handler != NULL)
        {
            ble_temp_evt_t evt;

            if (ble_srv_is_notification_enabled(p_evt_write->data))
            {
                evt.evt_type = BLE_TEMP_EVT_NOTIFICATION_ENABLED;
            }
            else
            {
                evt.evt_type = BLE_TEMP_EVT_NOTIFICATION_DISABLED;
            }
            // Call the application event handler.
            p_temp->evt_handler(p_temp, &evt);
        }
    }

}

void ble_temp_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_temp_t * p_temp = (ble_temp_t *) p_context;
    
    NRF_LOG_INFO("BLE event received. Event type = %d\r\n", p_ble_evt->header.evt_id); 
    if (p_temp == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_temp, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_temp, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_temp, p_ble_evt);
            break;
/* Handling this event is not necessary
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            NRF_LOG_INFO("EXCHANGE_MTU_REQUEST event received.\r\n");
            break;
*/
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for adding the Custom Value characteristic.
 *
 * @param[in]   p_temp        Battery Service structure.
 * @param[in]   p_temp_init   Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t custom_value_char_add(ble_temp_t * p_temp, const ble_temp_init_t * p_temp_init)
{
    uint32_t            err_code;
    
    ble_uuid_t          fire_ble_uuid;
    ble_gatts_char_md_t fire_char_md;
    ble_gatts_attr_md_t fire_cccd_md;
    ble_gatts_attr_t    fire_attr_char_value;
    ble_gatts_attr_md_t fire_attr_md;

    ble_uuid_t          garage_ble_uuid;
    ble_gatts_char_md_t garage_char_md;
    ble_gatts_attr_md_t garage_cccd_md;
    ble_gatts_attr_t    garage_attr_char_value;
    ble_gatts_attr_md_t garage_attr_md;

    ble_gatts_char_handles_t char_handles;
    
    // fire
    // Add Custom Value characteristic
    memset(&fire_cccd_md, 0, sizeof(fire_cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&fire_cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&fire_cccd_md.write_perm);
    
    fire_cccd_md.write_perm = p_temp_init->fire_md.cccd_write_perm;
    fire_cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&fire_char_md, 0, sizeof(fire_char_md));

    fire_char_md.char_props.read   = 1;
    fire_char_md.char_props.write  = 0;
    fire_char_md.char_props.notify = 1; 
    fire_char_md.p_char_user_desc  = NULL;
    fire_char_md.p_char_pf         = NULL;
    fire_char_md.p_user_desc_md    = NULL;
    fire_char_md.p_cccd_md         = &fire_cccd_md; 
    fire_char_md.p_sccd_md         = NULL;
		
    fire_ble_uuid.type = p_temp->uuid_type;
    fire_ble_uuid.uuid = FIRE_VALUE_CHAR_UUID;

    memset(&fire_attr_md, 0, sizeof(fire_attr_md));

    fire_attr_md.read_perm  = p_temp_init->fire_md.read_perm;
    fire_attr_md.write_perm = p_temp_init->fire_md.write_perm;
    fire_attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    fire_attr_md.rd_auth    = 0;
    fire_attr_md.wr_auth    = 0;
    fire_attr_md.vlen       = 0;

    memset(&fire_attr_char_value, 0, sizeof(fire_attr_char_value));

    fire_attr_char_value.p_uuid    = &fire_ble_uuid;
    fire_attr_char_value.p_attr_md = &fire_attr_md;
    fire_attr_char_value.init_len  = sizeof(int16_t);
    fire_attr_char_value.init_offs = 0;
    fire_attr_char_value.max_len   = sizeof(int16_t);

    err_code = sd_ble_gatts_characteristic_add(p_temp->service_handle, &fire_char_md,
                                               &fire_attr_char_value,
                                               &char_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    p_temp->characteristic_handles[TEMP_FIRE_CHAR] = char_handles.value_handle;
    p_temp->characteristic_handles[TEMP_FIRE_CCCD] = char_handles.cccd_handle;
    // end fire
    // garage

    // Add Custom Value characteristic
    memset(&garage_cccd_md, 0, sizeof(garage_cccd_md));

    //  Read  operation on cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&garage_cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&garage_cccd_md.write_perm);
    
    garage_cccd_md.write_perm = p_temp_init->garage_md.cccd_write_perm;
    garage_cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    memset(&garage_char_md, 0, sizeof(garage_char_md));

    garage_char_md.char_props.read   = 1;
    garage_char_md.char_props.write  = 1;
    garage_char_md.char_props.notify = 0; 
    garage_char_md.p_char_user_desc  = NULL;
    garage_char_md.p_char_pf         = NULL;
    garage_char_md.p_user_desc_md    = NULL;
    garage_char_md.p_cccd_md         = &garage_cccd_md; 
    garage_char_md.p_sccd_md         = NULL;
		
    garage_ble_uuid.type = p_temp->uuid_type;
    garage_ble_uuid.uuid = GARAGE_VALUE_CHAR_UUID;

    memset(&garage_attr_md, 0, sizeof(garage_attr_md));

    garage_attr_md.read_perm  = p_temp_init->garage_md.read_perm;
    garage_attr_md.write_perm = p_temp_init->garage_md.write_perm;
    garage_attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    garage_attr_md.rd_auth    = 0;
    garage_attr_md.wr_auth    = 0;
    garage_attr_md.vlen       = 0;

    memset(&garage_attr_char_value, 0, sizeof(garage_attr_char_value));

    garage_attr_char_value.p_uuid    = &garage_ble_uuid;
    garage_attr_char_value.p_attr_md = &garage_attr_md;
    garage_attr_char_value.init_len  = sizeof(int16_t);
    garage_attr_char_value.init_offs = 0;
    garage_attr_char_value.max_len   = sizeof(int16_t);

    err_code = sd_ble_gatts_characteristic_add(p_temp->service_handle, &garage_char_md,
                                               &garage_attr_char_value,
                                               &char_handles);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    p_temp->characteristic_handles[TEMP_GARAGE_CHAR] = char_handles.value_handle;

    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    uint8_t buffer[2];
    buffer[0] = 0;
    buffer[1] = 0;

    gatts_value.len     = sizeof(int16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = buffer;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_temp->conn_handle,
                                      p_temp->characteristic_handles[TEMP_FIRE_CHAR],
                                      &gatts_value);

    return NRF_SUCCESS;
}

uint32_t ble_temp_init(ble_temp_t * p_temp, const ble_temp_init_t * p_temp_init)
{
    if (p_temp == NULL || p_temp_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_temp->evt_handler               = p_temp_init->evt_handler;
    p_temp->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {TEMP_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_temp->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_temp->uuid_type;
    ble_uuid.uuid = TEMP_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_temp->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add Custom Value characteristic
    return custom_value_char_add(p_temp, p_temp_init);
}

uint32_t ble_temp_garage_get(ble_temp_t * p_temp, int16_t * value)
{
    ble_gatts_value_t gatts_value;

    uint8_t buffer[2];

    gatts_value.len     = sizeof(int16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = buffer;

    uint32_t err_code = sd_ble_gatts_value_get(p_temp->conn_handle,
                                      p_temp->characteristic_handles[TEMP_GARAGE_CHAR],
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    uint8_t *p_value = (uint8_t *)value;
    p_value[0] = gatts_value.p_value[1];
    p_value[1] = gatts_value.p_value[0];

    return err_code;
}

uint32_t ble_temp_fire_update(ble_temp_t * p_temp, int16_t value)
{
    NRF_LOG_INFO("In ble_temp_fire_update. \r\n"); 
    if (p_temp == NULL)
    {
        return NRF_ERROR_NULL;
    }

    uint32_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    uint8_t buffer[2];
    buffer[0] = ((uint8_t*)&value)[1];
    buffer[1] = ((uint8_t*)&value)[0];

    gatts_value.len     = sizeof(int16_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = buffer;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_temp->conn_handle,
                                      p_temp->characteristic_handles[TEMP_FIRE_CHAR],
                                      &gatts_value);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Send value if connected and notifying.
    if ((p_temp->conn_handle != BLE_CONN_HANDLE_INVALID)) 
    {
        ble_gatts_hvx_params_t hvx_params;

        memset(&hvx_params, 0, sizeof(hvx_params));

        hvx_params.handle = p_temp->characteristic_handles[TEMP_FIRE_CHAR];
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_temp->conn_handle, &hvx_params);
        NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code); 
    }
    else
    {
        err_code = NRF_ERROR_INVALID_STATE;
        NRF_LOG_INFO("sd_ble_gatts_hvx result: NRF_ERROR_INVALID_STATE. \r\n"); 
    }


    return err_code;
}
