/**
 *
 *
 */

#ifndef W_BLE_H
#define W_BLE_H

#define APP_ADV_FAST_INTERVAL            MSEC_TO_UNITS(50, UNIT_0_625_MS)       /**< The fast advertising interval. */
#define APP_ADV_FAST_TIMEOUT_IN_SECONDS  30                                     /**< The fast advertising timeout in units of seconds. */
#define APP_ADV_SLOW_INTERVAL            MSEC_TO_UNITS(1200, UNIT_0_625_MS)     /**< The slow advertising interval. */
#define APP_ADV_SLOW_TIMEOUT_IN_SECONDS  0                                      /**< The slow advertising timeout in units of seconds. */


int w_ble_init(bool erase_bonds);

int w_ble_battery_level_update(uint8_t battery_level);

int w_ble_heart_rate_measurement_send(uint16_t heart_rate);

int w_ble_hrs_rr_interval_add(uint16_t rr_interval);

int w_ble_hrs_sensor_contact_detected_update(bool sensor_contact_detected);

int w_ble_disconnect(void);

int w_ble_advertising_restart_without_whitelist(void);

int w_ble_advertising_start(void);


#endif  // W_BLE_H