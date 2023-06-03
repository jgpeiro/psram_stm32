/*
 * psram.c
 *
 *  Created on: 3 jun. 2023
 *      Author: Jose
 */


/*
                                 +----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
                                 | SPI Mode (QE=0)                                      | QUAD Mode (QE=1)                                     |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Command             | Code     | Cmd      | Addr     | Wait     | DIO      | Freq     | Cmd      | Addr     | Wait     | DIO      | Freq     |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Read                | 0x03     | S        | S        | 0        | S        | 33       | NA       | NA       | NA       | NA       | NA       |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Fast Read           | 0x0B     | S        | S        | 8        | S        | 133/84   | Q        | Q        | 6        | Q        | 133/84   |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Fast Read Quad      | 0xEB     | S        | Q        | 6        | Q        | 133/84   | Q        | Q        | 6        | Q        | 133/84   |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Write               | 0x02     | S        | S        | 0        | S        | 133/84   | Q        | Q        | 0        | Q        | 133/84   |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Quad Write          | 0x38     | S        | Q        | 0        | Q        | 133/84   | Q        | Q        | 0        | Q        | 133/84   |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Enter Quad Mode     | 0x35     | S        | -        | -        | -        | 133      | NA       | NA       | NA       | NA       | NA       |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Exit Quad Mode      | 0xF5     | NA       | NA       | NA       | NA       | NA       | Q        | -        | -        | -        | 133      |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Reset Enable        | 0x66     | S        | -        | -        | -        | 133      | Q        | -        | -        | -        | 133      |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Reset               | 0x99     | S        | -        | -        | -        | 133      | Q        | -        | -        | -        | 133      |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Wrap Boundary       | 0xC0     | S        | -        | -        | -        | 133      | Q        | -        | -        | -        | 133      |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
| Read ID             | 0x9F     | S        | S        | 0        | S        | 33       | NA       | NA       | NA       | NA       | NA       |
+---------------------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+----------+
*/

#include "psram.h"
void psram_init( psram_t *psram, QSPI_HandleTypeDef *hqspi, GPIO_TypeDef *vcc_port, uint16_t vcc_pin )
{
    psram->hqspi = hqspi;
    psram->vcc_port = vcc_port;
    psram->vcc_pin = vcc_pin;
}

void psram_deinit( psram_t *psram )
{
    psram_power( psram, 0 );
    psram->hqspi = NULL;
    psram->vcc_port = NULL;
    psram->vcc_pin = 0;
}

void psram_power( psram_t *psram, uint8_t on )
{
    HAL_GPIO_WritePin( psram->vcc_port, psram->vcc_pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET );
}

uint8_t psram_reset( psram_t *psram )
{
    QSPI_CommandTypeDef sCommand = {0};

    // Reset enable command in SPI mode:
    // Cmd=S, Addr=0, Wait=0, DIO=0
    sCommand.Instruction        = PSRAM_CMD_RST_EN;
    sCommand.InstructionMode    = QSPI_INSTRUCTION_1_LINE;  // Cmd=S
    sCommand.AddressMode        = QSPI_ADDRESS_NONE;        // Addr=0
    sCommand.Address            = 0x00000000;
    sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
    sCommand.DataMode           = QSPI_DATA_NONE;           // DIO=0
    sCommand.NbData             = 0;
    sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.AlternateBytes     = 0x00;
    sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    sCommand.DummyCycles        = 0x00;                     // Wait=0
    sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD; // instrucion every command

    sCommand.Instruction        = PSRAM_CMD_RST_EN;
    if( HAL_QSPI_Command( psram->hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK ) {
        return 0;
    }

    // Reset command in SPI mode:
    // Cmd=S, Addr=0, Wait=0, DIO=0
    sCommand.Instruction        = PSRAM_CMD_RST;
    if (HAL_QSPI_Command( psram->hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return 0;
    }

    return 1;
}

uint8_t psram_read_id( psram_t *psram, uint16_t *id )
{
    uint8_t buff[2];
    QSPI_CommandTypeDef sCommand = {0};

    // Read ID command in SPI mode:
    // Cmd=S, Addr=S, Wait=0, DIO=S
    sCommand.Instruction        = PSRAM_CMD_READ_ID;
    sCommand.InstructionMode    = QSPI_INSTRUCTION_1_LINE;  // Cmd=S
    sCommand.AddressMode        = QSPI_ADDRESS_1_LINE;      // Addr=S
    sCommand.Address            = 0x00000000;
    sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
    sCommand.DataMode           = QSPI_DATA_1_LINE;         // DIO=S
    sCommand.NbData             = sizeof(buff);             // 2 bytes
    sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.AlternateBytes     = 0x00;
    sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    sCommand.DummyCycles        = 0x00;                     // Wait=0
    sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD; // instrucion every command

    if( HAL_QSPI_Command( psram->hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK ) {
        return 0;
    }

    if( HAL_QSPI_Receive( psram->hqspi, buff, HAL_QSPI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK ) {
        return 0;
    }

    *id = (buff[1] << 8) | buff[0];

    return 1;
}


uint8_t psram_set_quad_mode( psram_t *psram, uint8_t status )
{
    QSPI_CommandTypeDef sCommand = {0};

    if( status )
    {
        // Enter Quad mode:
        // Cmd=S, Addr=0, Wait=0, DIO=0
        sCommand.Instruction        = PSRAM_CMD_QUAD_ON;
        sCommand.InstructionMode    = QSPI_INSTRUCTION_1_LINE;  // Cmd=S
        sCommand.AddressMode        = QSPI_ADDRESS_NONE;        // Addr=0
        sCommand.Address            = 0x00000000;
        sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
        sCommand.DataMode           = QSPI_DATA_NONE;           // DIO=0
        sCommand.NbData             = 0;
        sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
        sCommand.AlternateBytes     = 0x00;
        sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
        sCommand.DummyCycles        = 0x00;                     // Wait=0
        sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
        sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
        sCommand.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD; // instrucion every command
    }
    else
    {
        // Exit Quad mode:
        // Cmd=Q, Addr=0, Wait=0, DIO=0
        sCommand.Instruction        = PSRAM_CMD_QUAD_OFF;
        sCommand.InstructionMode    = QSPI_INSTRUCTION_4_LINES; // Cmd=Q
        sCommand.AddressMode        = QSPI_ADDRESS_NONE;        // Addr=0
        sCommand.Address            = 0x00000000;
        sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
        sCommand.DataMode           = QSPI_DATA_NONE;           // DIO=0
        sCommand.NbData             = 0;
        sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
        sCommand.AlternateBytes     = 0x00;
        sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
        sCommand.DummyCycles        = 0x00;                     // Wait=0
        sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
        sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
        sCommand.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD; // instrucion every command
    }

    if( HAL_QSPI_Command( psram->hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK ) {
        return 0;
    }

    return 1;
}


uint8_t psram_read_quad( psram_t *psram, uint32_t address, uint8_t *data, uint32_t size )
{
    QSPI_CommandTypeDef sCommand = {0};

    // Fast Read Quad:
    // Cmd=Q, Addr=Q, Wait=6, DIO=Q
    sCommand.Instruction        = PSRAM_CMD_QUAD_READ;
    sCommand.InstructionMode    = QSPI_INSTRUCTION_4_LINES; // Cmd=Q
    sCommand.AddressMode        = QSPI_ADDRESS_4_LINES;     // Addr=Q
    sCommand.Address            = address;
    sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
    sCommand.DataMode           = QSPI_DATA_4_LINES;        // DIO=Q
    sCommand.NbData             = size;
    sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.AlternateBytes     = 0x00;
    sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    sCommand.DummyCycles        = 0x06;                     // Wait=6
    sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode           = QSPI_SIOO_INST_ONLY_FIRST_CMD; // instrucion only fisrt command

    if( HAL_QSPI_Command( psram->hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK ) {
        return 0;
    }

    if( HAL_QSPI_Receive( psram->hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK ) {
        return 0;
    }

    return 1;
}

uint8_t psram_write_quad( psram_t *psram, uint32_t address, uint8_t *data, uint32_t size )
{
    QSPI_CommandTypeDef sCommand = {0};

    // Quad Write:
    // Cmd=Q, Addr=Q, Wait=0, DIO=Q
    sCommand.Instruction        = PSRAM_CMD_QUAD_WRITE;
    sCommand.InstructionMode    = QSPI_INSTRUCTION_4_LINES; // Cmd=Q
    sCommand.AddressMode        = QSPI_ADDRESS_4_LINES;     // Addr=Q
    sCommand.Address            = address;
    sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
    sCommand.DataMode           = QSPI_DATA_4_LINES;        // DIO=Q
    sCommand.NbData             = size;
    sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
    sCommand.AlternateBytes     = 0x00;
    sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
    sCommand.DummyCycles        = 0x00;                     // Wait=0
    sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
    sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
    sCommand.SIOOMode           = QSPI_SIOO_INST_ONLY_FIRST_CMD; // instrucion only fisrt command

    if( HAL_QSPI_Command( psram->hqspi, &sCommand, HAL_QSPI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK ) {
        return 0;
    }

    if( HAL_QSPI_Transmit( psram->hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE ) != HAL_OK ) {
        return 0;
    }

    return 1;
}




#define DATA_LEN 1024
#define N_COPIES 10


uint8_t psram_test( void )
{
    extern QSPI_HandleTypeDef hqspi1;
    uint16_t id;
    uint16_t i, j;
    psram_t psram;
    uint32_t address = 0;
    uint8_t buff[ DATA_LEN ];

    //MX_QUADSPI1_Init();

    // Initialization
    psram_init( &psram, &hqspi1, GPIOA, GPIO_PIN_0 );
    psram_power( &psram, 0 );
    HAL_Delay(1);
    psram_power( &psram, 1 );
    HAL_Delay(1);
    psram_reset( &psram );
    HAL_Delay(1);
    psram_read_id( &psram, &id );
    if( (((id>>0)&0xFF) != PSRAM_MANUFACTURER_ID) ||
    	(((id>>8)&0xFF) != PSRAM_KNOWN_GOOD_DIE) )
    {
        return 0;
    }

    // Enter quad mode
    psram_set_quad_mode( &psram, 1 );

    // Fill the data
    for( i = 0 ; i < DATA_LEN ; i++ ) {
        buff[ i ] = i;
    }
    // Write the data
    address = 0;
    for( i = 0 ; i < N_COPIES ; i++ ) {
        psram_write_quad( &psram, address, buff, DATA_LEN );
        address += DATA_LEN;
    }

    // Read the data
    address = 0;
    for( i = 0 ; i < N_COPIES ; i++ ) {
        // Erase the buffer before reading
        for( j = 0 ; j < DATA_LEN ; j++ ) {
            buff[ j ] = 0;
        }

    	psram_read_quad( &psram, address, buff, DATA_LEN );
        // verify the data
        for( j = 0 ; j < DATA_LEN ; j++ ) {
            if( buff[ j ] != j ) {
                return 0;
            }
        }
        address += DATA_LEN;
    }

    return 1;
}
