#ifndef BLE_TEMP_H__
#define BLE_TEMP_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/**@brief   Macro for defining a ble_temp instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_TEMP_DEF(_name)                                                                          \
static ble_temp_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_TEMP_BLE_OBSERVER_PRIO,                                                     \
                     ble_temp_on_ble_evt, &_name)



// TEMP_SERVICE_UUID_BASE 855ea3d4-6d3a-11eb-9439-0242ac130002
//                          85 5e a3 d4 - 6d 3a - 11 eb - 94 39 - 02 42 ac 13 00 02

#define TEMP_SERVICE_UUID_BASE         {0x85, 0x5E, 0xA3, 0xD4, 0x6D, 0x3A, 0x11, 0xEB, \
                                          0x94, 0x39, 0x02, 0x42, 0xAC, 0x13, 0x00, 0x02}

#define TEMP_SERVICE_UUID               0x1400
#define FIRE_VALUE_CHAR_UUID            0x1401
#define GARAGE_VALUE_CHAR_UUID            0x1402

enum TEMP_CHARACTERISTICS
{
  TEMP_FIRE_CHAR,
  TEMP_FIRE_CCCD,
  TEMP_GARAGE_CHAR,
  TEMP_NUM_ATTRS
};

/**@brief Custom Service event type. */
typedef enum
{
    BLE_TEMP_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_TEMP_EVT_NOTIFICATION_DISABLED,                             /**< Custom value notification disabled event. */
    BLE_TEMP_EVT_DISCONNECTED,
    BLE_TEMP_EVT_CONNECTED
} ble_temp_evt_type_t;

/**@brief Custom Service event. */
typedef struct
{
    ble_temp_evt_type_t evt_type;                                  /**< Type of event. */
} ble_temp_evt_t;

// Forward declaration of the ble_temp_t type.
typedef struct ble_temp_s ble_temp_t;


/**@brief Custom Service event handler type. */
typedef void (*ble_temp_evt_handler_t) (ble_temp_t * p_bas, ble_temp_evt_t * p_evt);

/**@brief Battery Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_temp_evt_handler_t        evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint8_t                       initial_custom_value;           /**< Initial custom value */
    ble_srv_cccd_security_mode_t  fire_md;     /**< Initial security level for Custom characteristics attribute */
    ble_srv_cccd_security_mode_t  garage_md;     /**< Initial security level for Custom characteristics attribute */
} ble_temp_init_t;

/**@brief Custom Service structure. This contains various status information for the service. */
struct ble_temp_s
{
    ble_temp_evt_handler_t        evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                      service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    uint16_t                      characteristic_handles[TEMP_NUM_ATTRS];
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                       uuid_type; 
};

/**@brief Function for initializing the Custom Service.
 *
 * @param[out]  p_cus       Custom Service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_cus_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_temp_init(ble_temp_t * p_cus, const ble_temp_init_t * p_cus_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Battery Service.
 *
 * @note 
 *
 * @param[in]   p_cus      Custom Service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_temp_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context);

/**@brief Function for updating the custom value.
 *
 * @details The application calls this function when the cutom value should be updated. If
 *          notification has been enabled, the custom value characteristic is sent to the client.
 *
 * @note 
 *       
 * @param[in]   p_bas          Custom Service structure.
 * @param[in]   Custom value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */

uint32_t ble_temp_fire_update(ble_temp_t * p_cus, int16_t custom_value);
uint32_t ble_temp_garage_get(ble_temp_t * p_temp, int16_t * value);

#endif // BLE_TEMP_H__
