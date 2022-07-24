/*
 * ble.h
 *
 *  Created on: 14/07/2022
 *      Author: Carl
 */

#ifndef VESC_BLE_H_
#define VESC_BLE_H_


typedef enum
{
    BLE_AT = 0,
    BLE_TRANSPARENT
} BLEMode;
typedef enum
{
    BLE_RESET = 0,
    BLE_NAME_ERROR,
    BLE_RENAMED,
    BLE_NORMAL,
    BLE_BUSY,
} BLEConfigStatus;
typedef enum
{
    BLE_DISCONNECTED = 0,
    BLE_CONNECTED
} BLEConnectionStatus;
typedef struct
{
    char device_name[12];
    BLEMode mode;
    BLEConfigStatus config_status;
    BLEConnectionStatus connect_status;
} BLEConfig;

#define BLE_DEBUG 0
#define BLE_DEFAULT_NAME "ABC-7XXXXX"

#define BLE_AT_MODE() GPIO_ResetBits(GPIOE, GPIO_Pin_7)
#define BLE_TRANSPARENT_MODE() GPIO_SetBits(GPIOE, GPIO_Pin_7)


void ble_init(void);
#endif /* VESC_BLE_H_ */
