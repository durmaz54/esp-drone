/*
 * ibus.c
 *
 *  Created on: 2 Şub 2022
 *      Author: Abdul Samet
 */
#include "dz_ibus.h"
#include "esp_log.h"
#include "driver/uart.h"


void dz_ibus_init() {
uart_config_t uart1_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
};
ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart1_config));

    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart1_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, 23, 22, 0, 0));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024, 1024, 0, NULL, 0));
}

void ibus_read(uint16_t *data) {
	uint8_t rxData[32];
	int8_t state;

	state = uart_read_bytes(UART_NUM_1, rxData, 20, 10);
		
	if (state != -1) {
		if ((rxData[1] == IBUS_LENGTH) && (rxData[2] == IBUS_COMMAND)) {
			for (uint8_t i = 0; i < IBUS_USER_CHANNELS * 2; i += 2) {

				data[(i / 2) + 1] = (rxData[4 + i] << 8) | rxData[3 + i];
				data[(i / 2) + 1] = data[(i / 2) + 1]
						- (data[(i / 2) + 1] % 10);
				if (data[1] > 2500) {
					data[0] = FAILSAFE_ACTIVE;
					ESP_LOGW("IBUS", "hata kumanda verisi 2500'den fazla %d\n", state);
				} else {
					data[0] = FAILSAFE_OFF;

				}
			}

		}
	} else {
		data[0] = FAILSAFE_ACTIVE;
		ESP_LOGW("IBUS", "hata hal =  %d\n", state);
	}

}
