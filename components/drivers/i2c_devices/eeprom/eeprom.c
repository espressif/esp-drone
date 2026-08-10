/**
 * ESP-Drone Firmware
 *
 * EEPROM compatibility API backed by one NVS blob. Unwritten bytes read as
 * 0xff, matching an erased serial EEPROM.
 */
#define DEBUG_MODULE "EEPROM"

#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"

#include "debug_cf.h"
#include "eeprom.h"

#define EEPROM_NVS_NAMESPACE "espdrone"
#define EEPROM_NVS_KEY "eeprom"

static bool isInit;
static nvs_handle_t nvsHandle;
static SemaphoreHandle_t storageMutex;

static bool rangeIsValid(size_t address, size_t length)
{
    return address <= EEPROM_SIZE_BYTES && length <= EEPROM_SIZE_BYTES - address;
}

bool eepromInit(I2C_Dev *i2cPort)
{
    (void)i2cPort;

    if (isInit) {
        return true;
    }

    storageMutex = xSemaphoreCreateMutex();
    if (storageMutex == NULL) {
        return false;
    }

    const esp_err_t err = nvs_open(EEPROM_NVS_NAMESPACE, NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        vSemaphoreDelete(storageMutex);
        storageMutex = NULL;
        DEBUG_PRINTW("NVS open failed: %s", esp_err_to_name(err));
        return false;
    }

    isInit = true;
    return true;
}

bool eepromTest(void)
{
    return isInit;
}

bool eepromTestConnection(void)
{
    return isInit;
}

bool eepromReadBuffer(uint8_t *buffer, size_t readAddr, size_t len)
{
    if (!isInit || !rangeIsValid(readAddr, len) || (buffer == NULL && len != 0)) {
        return false;
    }
    if (len == 0) {
        return true;
    }
    if (xSemaphoreTake(storageMutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    memset(buffer, 0xff, len);
    size_t storedLength = 0;
    esp_err_t err = nvs_get_blob(nvsHandle, EEPROM_NVS_KEY, NULL, &storedLength);
    if (err == ESP_ERR_NVS_NOT_FOUND || readAddr >= storedLength) {
        xSemaphoreGive(storageMutex);
        return true;
    }
    if (err != ESP_OK || storedLength > EEPROM_SIZE_BYTES) {
        xSemaphoreGive(storageMutex);
        return false;
    }

    uint8_t *storedData = malloc(storedLength);
    if (storedData == NULL) {
        xSemaphoreGive(storageMutex);
        return false;
    }
    err = nvs_get_blob(nvsHandle, EEPROM_NVS_KEY, storedData, &storedLength);
    if (err == ESP_OK) {
        const size_t available = storedLength - readAddr;
        const size_t copyLength = len < available ? len : available;
        memcpy(buffer, storedData + readAddr, copyLength);
    }

    free(storedData);
    xSemaphoreGive(storageMutex);
    return err == ESP_OK;
}

bool eepromWriteBuffer(const uint8_t *buffer, size_t writeAddr, size_t len)
{
    if (!isInit || !rangeIsValid(writeAddr, len) || (buffer == NULL && len != 0)) {
        return false;
    }
    if (len == 0) {
        return true;
    }
    if (xSemaphoreTake(storageMutex, portMAX_DELAY) != pdTRUE) {
        return false;
    }

    size_t storedLength = 0;
    esp_err_t err = nvs_get_blob(nvsHandle, EEPROM_NVS_KEY, NULL, &storedLength);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        storedLength = 0;
    } else if (err != ESP_OK || storedLength > EEPROM_SIZE_BYTES) {
        xSemaphoreGive(storageMutex);
        return false;
    }

    const size_t requiredLength = writeAddr + len;
    const size_t updatedLength = storedLength > requiredLength ? storedLength : requiredLength;
    uint8_t *updatedData = malloc(updatedLength);
    if (updatedData == NULL) {
        xSemaphoreGive(storageMutex);
        return false;
    }
    memset(updatedData, 0xff, updatedLength);

    if (storedLength != 0) {
        size_t readLength = storedLength;
        err = nvs_get_blob(nvsHandle, EEPROM_NVS_KEY, updatedData, &readLength);
    }
    if (err == ESP_OK || storedLength == 0) {
        memcpy(updatedData + writeAddr, buffer, len);
        err = nvs_set_blob(nvsHandle, EEPROM_NVS_KEY, updatedData, updatedLength);
        if (err == ESP_OK) {
            err = nvs_commit(nvsHandle);
        }
    }

    free(updatedData);
    xSemaphoreGive(storageMutex);
    return err == ESP_OK;
}

bool eepromWritePage(const uint8_t *buffer, uint16_t writeAddr)
{
    return eepromWriteBuffer(buffer, writeAddr, EEPROM_PAGE_SIZE);
}
