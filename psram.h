/*
 * psram.h
 *
 *  Created on: 3 jun. 2023
 *      Author: Jose
 */

#ifndef INC_PSRAM_H_
#define INC_PSRAM_H_

#include <stdint.h>

#include "stm32g4xx_hal.h"

#define PSRAM_CMD_READ          (0x03)
#define PSRAM_CMD_FAST_READ     (0x0b)
#define PSRAM_CMD_QUAD_READ     (0xeb)
#define PSRAM_CMD_WRITE         (0x02)
#define PSRAM_CMD_QUAD_WRITE    (0x38)
#define PSRAM_CMD_QUAD_ON       (0x35)
#define PSRAM_CMD_QUAD_OFF      (0xf5)
#define PSRAM_CMD_RST_EN        (0x66)
#define PSRAM_CMD_RST           (0x99)
#define PSRAM_CMD_BURST_LEN     (0xc0)
#define PSRAM_CMD_READ_ID       (0x9f)

#define PSRAM_MANUFACTURER_ID	(0x0D)
#define PSRAM_KNOWN_GOOD_DIE	(0x5D)

struct psram
{
    GPIO_TypeDef *vcc_port;
    uint16_t vcc_pin;
    QSPI_HandleTypeDef *hqspi;
};
typedef struct psram psram_t;

void psram_init( psram_t *psram, QSPI_HandleTypeDef *hqspi, GPIO_TypeDef *vcc_port, uint16_t vcc_pin );
void psram_deinit( psram_t *psram );
void psram_power( psram_t *psram, uint8_t on );
uint8_t psram_reset( psram_t *psram );
uint8_t psram_read_id( psram_t *psram, uint16_t *id );
uint8_t psram_set_quad_mode( psram_t *psram, uint8_t status );
uint8_t psram_read_quad( psram_t *psram, uint32_t address, uint8_t *data, uint32_t size );
uint8_t psram_write_quad( psram_t *psram, uint32_t address, uint8_t *data, uint32_t size );
uint8_t psram_test( void );

#endif /* INC_PSRAM_H_ */
