/**
 *
 *
 */

#ifndef W_BLE_H
#define W_BLE_H

int w_ble_init(bool erase_bonds);

int w_ble_battery_level_update(uint8_t battery_level);

int w_ble_heart_rate_measurement_send(uint16_t heart_rate);

int w_ble_hrs_rr_interval_add(uint16_t rr_interval);

int w_ble_hrs_sensor_contact_detected_update(bool sensor_contact_detected);

int w_ble_disconnect(void);

int w_ble_advertising_restart_without_whitelist(void);

int w_ble_advertising_start(void);


#endif  // W_BLE_H