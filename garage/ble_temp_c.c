/**
 * Copyright (c) 2012 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include "sdk_common.h"
#include "ble_temp_c.h"
#include "ble_db_discovery.h"
#include "ble_types.h"
#include "ble_gattc.h"


#define NRF_LOG_MODULE_NAME ble_temp_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

/**@brief Function for interception of the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
    ble_temp_c_t * p_ble_temp_c = (ble_temp_c_t *)p_ctx;

    NRF_LOG_INFO("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

    if (p_ble_temp_c->error_handler != NULL)
    {
        p_ble_temp_c->error_handler(nrf_error);
    }
}


/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function uses the Handle Value Notification received from the SoftDevice
 *            and checks whether it is a notification of the Temp measurement from the peer. If
 *            it is, this function decodes the Temp measurement and sends it to the
 *            application.
 *
 * @param[in] p_ble_temp_c Pointer to the Temp Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_temp_c_t * p_ble_temp_c, const ble_evt_t * p_ble_evt)
{
    // Check if the event is on the link for this instance.
    if (p_ble_temp_c->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        NRF_LOG_INFO("Received HVX on link 0x%x, not associated to this instance. Ignore.",
                      p_ble_evt->evt.gattc_evt.conn_handle);
        return;
    }

    NRF_LOG_INFO("Received HVX on link 0x%x, hrm_handle 0x%x",
    p_ble_evt->evt.gattc_evt.params.hvx.handle,
    p_ble_temp_c->peer_hrs_db.hrm_handle);

    // Check if this is a Temp notification.
    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_temp_c->peer_hrs_db.hrm_handle)
    {
        ble_temp_c_evt_t ble_temp_c_evt;

        ble_temp_c_evt.evt_type                    = BLE_TEMP_C_EVT_FIRE_NOTIFICATION;
        ble_temp_c_evt.conn_handle                 = p_ble_temp_c->conn_handle;

        uint8_t *temp = (uint8_t *) &ble_temp_c_evt.params.value_fire;
        
        NRF_LOG_HEXDUMP_INFO(p_ble_evt->evt.gattc_evt.params.hvx.data, p_ble_evt->evt.gattc_evt.params.hvx.len);

        if (p_ble_evt->evt.gattc_evt.params.hvx.len == 2) {
            temp[0] = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            temp[1] = p_ble_evt->evt.gattc_evt.params.hvx.data[0];

            p_ble_temp_c->evt_handler(p_ble_temp_c, &ble_temp_c_evt);
        }
    }
}


/**@brief     Function for handling Disconnected event received from the SoftDevice.
 *
 * @details   This function checks whether the disconnect event is happening on the link
 *            associated with the current instance of the module. If the event is happening, the function sets the instance's
 *            conn_handle to invalid.
 *
 * @param[in] p_ble_temp_c Pointer to the Temp Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_disconnected(ble_temp_c_t * p_ble_temp_c, const ble_evt_t * p_ble_evt)
{
    if (p_ble_temp_c->conn_handle == p_ble_evt->evt.gap_evt.conn_handle)
    {
        p_ble_temp_c->conn_handle                 = BLE_CONN_HANDLE_INVALID;
        p_ble_temp_c->peer_hrs_db.hrm_cccd_handle = BLE_GATT_HANDLE_INVALID;
        p_ble_temp_c->peer_hrs_db.hrm_handle      = BLE_GATT_HANDLE_INVALID;
    }
}


void ble_temp_on_db_disc_evt(ble_temp_c_t * p_ble_temp_c, const ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_HEXDUMP_INFO(&(p_evt->params.discovered_db.srv_uuid.uuid), 2);
    if (p_evt->evt_type == BLE_DB_DISCOVERY_SRV_NOT_FOUND) {
        NRF_LOG_INFO("BLE_DB_DISCOVERY_SRV_NOT_FOUND");
    }
    // Check if the Temp Service was discovered.
    if (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE &&
        p_evt->params.discovered_db.srv_uuid.uuid == TEMP_SERVICE_UUID &&
        p_evt->params.discovered_db.srv_uuid.type == BLE_UUID_TYPE_VENDOR_BEGIN)
    {
        // Find the CCCD Handle of the Temp Measurement characteristic.
        uint32_t i;

        ble_temp_c_evt_t evt;

        evt.evt_type    = BLE_TEMP_C_EVT_DISCOVERY_COMPLETE;
        evt.conn_handle = p_evt->conn_handle;

        for (i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            if (p_evt->params.discovered_db.charateristics[i].characteristic.uuid.uuid ==
                FIRE_VALUE_CHAR_UUID)
            {
                // Found Temp characteristic. Store CCCD handle and break.
                evt.params.peer_db.hrm_cccd_handle =
                    p_evt->params.discovered_db.charateristics[i].cccd_handle;
                evt.params.peer_db.hrm_handle =
                    p_evt->params.discovered_db.charateristics[i].characteristic.handle_value;
                break;
            }
        }

        NRF_LOG_INFO("Temp Service discovered at peer.");
        //If the instance has been assigned prior to db_discovery, assign the db_handles.
        if (p_ble_temp_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_ble_temp_c->peer_hrs_db.hrm_cccd_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_temp_c->peer_hrs_db.hrm_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_temp_c->peer_hrs_db = evt.params.peer_db;
            }
        }


        p_ble_temp_c->evt_handler(p_ble_temp_c, &evt);
    }
}


uint32_t ble_temp_c_init(ble_temp_c_t * p_ble_temp_c, ble_temp_c_init_t * p_ble_temp_c_init)
{
    VERIFY_PARAM_NOT_NULL(p_ble_temp_c);
    VERIFY_PARAM_NOT_NULL(p_ble_temp_c_init);

    ble_uuid128_t base_uuid = {TEMP_SERVICE_UUID_BASE};
    
    uint8_t uuid_type;

    ret_code_t err_code = sd_ble_uuid_vs_add(&base_uuid, &uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid_t    temp_uuid;
    temp_uuid.type = uuid_type;
    temp_uuid.uuid = TEMP_SERVICE_UUID;

    p_ble_temp_c->evt_handler                 = p_ble_temp_c_init->evt_handler;
    p_ble_temp_c->error_handler               = p_ble_temp_c_init->error_handler;
    p_ble_temp_c->p_gatt_queue                = p_ble_temp_c_init->p_gatt_queue;
    p_ble_temp_c->conn_handle                 = BLE_CONN_HANDLE_INVALID;
    p_ble_temp_c->peer_hrs_db.hrm_cccd_handle = BLE_GATT_HANDLE_INVALID;
    p_ble_temp_c->peer_hrs_db.hrm_handle      = BLE_GATT_HANDLE_INVALID;

    return ble_db_discovery_evt_register(&temp_uuid);
}

void ble_temp_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_temp_c_t * p_ble_temp_c = (ble_temp_c_t *)p_context;

    if ((p_ble_temp_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_temp_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnected(p_ble_temp_c, p_ble_evt);
            break;

        default:
            break;
    }
}


/**@brief Function for creating a message for writing to the CCCD.
 */
static uint32_t cccd_configure(ble_temp_c_t * p_ble_temp_c, bool enable)
{
    NRF_LOG_INFO("Configuring CCCD. CCCD Handle = %d, Connection Handle = %d",
                  p_ble_temp_c->peer_hrs_db.hrm_cccd_handle,
                  p_ble_temp_c->conn_handle);

    nrf_ble_gq_req_t hrs_c_req;
    uint8_t          cccd[BLE_CCCD_VALUE_LEN];
    uint16_t         cccd_val = enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    memset(&hrs_c_req, 0, sizeof(hrs_c_req));

    hrs_c_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    hrs_c_req.error_handler.cb            = gatt_error_handler;
    hrs_c_req.error_handler.p_ctx         = p_ble_temp_c;
    hrs_c_req.params.gattc_write.handle   = p_ble_temp_c->peer_hrs_db.hrm_cccd_handle;
    hrs_c_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
    hrs_c_req.params.gattc_write.p_value  = cccd;
    hrs_c_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;

    return nrf_ble_gq_item_add(p_ble_temp_c->p_gatt_queue, &hrs_c_req, p_ble_temp_c->conn_handle);
}


uint32_t ble_temp_c_fire_notif_enable(ble_temp_c_t * p_ble_temp_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_temp_c);

    return cccd_configure(p_ble_temp_c, true);
}


uint32_t ble_temp_c_handles_assign(ble_temp_c_t    * p_ble_temp_c,
                                  uint16_t         conn_handle,
                                  const temp_db_t * p_peer_temp_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_temp_c);

    p_ble_temp_c->conn_handle = conn_handle;
    if (p_peer_temp_handles != NULL)
    {
        p_ble_temp_c->peer_hrs_db = *p_peer_temp_handles;
    }

    return nrf_ble_gq_conn_handle_register(p_ble_temp_c->p_gatt_queue, conn_handle);
}
