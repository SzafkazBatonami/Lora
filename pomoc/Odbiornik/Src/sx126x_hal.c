#include "sx126x_hal.h"

sx126x_hal_status_t sx126x_hal_init(sx126x_hal_context_t* context) {
    // Inicjalizacja GPIO i SPI może być tutaj
    // Ale zakładamy, że konfiguracja jest już zrobiona w HAL

    // Na wszelki wypadek, ustaw NSS na HIGH
    HAL_GPIO_WritePin(context->nss.port, context->nss.pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(context->reset.port, context->reset.pin, GPIO_PIN_RESET);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void* context) {
    const sx126x_hal_context_t* ctx = (const sx126x_hal_context_t*) context;

    // Ustaw reset pin na LOW
    HAL_GPIO_WritePin(ctx->reset.port, ctx->reset.pin, GPIO_PIN_RESET);
    HAL_Delay(1);  // Poczekaj 1 ms

    // Ustaw reset pin na HIGH
    HAL_GPIO_WritePin(ctx->reset.port, ctx->reset.pin, GPIO_PIN_SET);
    HAL_Delay(6);  // Poczekaj 6 ms

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_wakeup(const void* context) {
    const sx126x_hal_context_t* ctx = (const sx126x_hal_context_t*) context;

    // Wybudź radio przez krótkie obniżenie NSS
    HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_RESET);
    HAL_Delay(1);  // Poczekaj 1 ms
    HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_SET);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_write(const void* context, const uint8_t* command, const uint16_t command_length, const uint8_t* data, const uint16_t data_length) {
    const sx126x_hal_context_t* ctx = (const sx126x_hal_context_t*) context;

    // Czekaj, aż radio nie będzie zajęte
    while (HAL_GPIO_ReadPin(ctx->busy.port, ctx->busy.pin) == GPIO_PIN_SET);

    // NSS na LOW
    HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_RESET);

    // Wyślij komendę
    if (HAL_SPI_Transmit(ctx->spi, (uint8_t*) command, command_length, HAL_MAX_DELAY) != HAL_OK) {
        return SX126X_HAL_STATUS_ERROR;
    }

    // Wyślij dane
    if (data_length > 0) {
        if (HAL_SPI_Transmit(ctx->spi, (uint8_t*) data, data_length, HAL_MAX_DELAY) != HAL_OK) {
            return SX126X_HAL_STATUS_ERROR;
        }
    }

    // NSS na HIGH
    HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_SET);

    return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_read(const void* context, const uint8_t* command, const uint16_t command_length, uint8_t* data, const uint16_t data_length) {
    const sx126x_hal_context_t* ctx = (const sx126x_hal_context_t*) context;

    // Czekaj, aż radio nie będzie zajęte
    while (HAL_GPIO_ReadPin(ctx->busy.port, ctx->busy.pin) == GPIO_PIN_SET);

    // NSS na LOW
    HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_RESET);

    // Wyślij komendę
    if (HAL_SPI_Transmit(ctx->spi, (uint8_t*) command, command_length, HAL_MAX_DELAY) != HAL_OK) {
        return SX126X_HAL_STATUS_ERROR;
    }

    // Odbierz dane
    if (HAL_SPI_Receive(ctx->spi, data, data_length, HAL_MAX_DELAY) != HAL_OK) {
        return SX126X_HAL_STATUS_ERROR;
    }

    // NSS na HIGH
    HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_SET);

    return SX126X_HAL_STATUS_OK;
}
