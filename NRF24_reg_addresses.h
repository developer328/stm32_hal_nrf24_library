/*
 * 25-JUL-2024
 * STM32 HAL NRF24 LIBRARY
 */

#ifndef _NRF_24_REG_ADDRESSES_H
#define _NRF_24_REG_ADDRESSES_H

// NRF24L01 Register Addresses
#define CONFIG          0x00
#define EN_AA           0x01
#define EN_RXADDR       0x02
#define SETUP_AW        0x03
#define SETUP_RETR      0x04
#define RF_CH           0x05
#define RF_SETUP        0x06
#define STATUS          0x07
#define OBSERVE_TX      0x08
#define RPD             0x09
#define RX_ADDR_P0      0x0A
#define RX_ADDR_P1      0x0B
#define RX_ADDR_P2      0x0C
#define RX_ADDR_P3      0x0D
#define RX_ADDR_P4      0x0E
#define RX_ADDR_P5      0x0F
#define TX_ADDR         0x10
#define RX_PW_P0        0x11
#define RX_PW_P1        0x12
#define RX_PW_P2        0x13
#define RX_PW_P3        0x14
#define RX_PW_P4        0x15
#define RX_PW_P5        0x16
#define FIFO_STATUS     0x17
#define DYNPD           0x1C
#define FEATURE         0x1D

#define TX_PLD         0xA0    // TX payload register address
#define RX_PLD_P0      0x61    // RX payload register address for pipe 0
#define RX_PLD_P1      0x62    // RX payload register address for pipe 1
#define RX_PLD_P2      0x63    // RX payload register address for pipe 2
#define RX_PLD_P3      0x64    // RX payload register address for pipe 3
#define RX_PLD_P4      0x65    // RX payload register address for pipe 4
#define RX_PLD_P5      0x66    // RX payload register address for pipe 5

// Bit Positions in Registers
// CONFIG Register
#define MASK_RX_DR      6
#define MASK_TX_DS      5
#define MASK_MAX_RT     4
#define EN_CRC          3
#define CRCO            2
#define PWR_UP          1
#define PRIM_RX         0

// EN_AA Register
#define ENAA_P5         5
#define ENAA_P4         4
#define ENAA_P3         3
#define ENAA_P2         2
#define ENAA_P1         1
#define ENAA_P0         0

// EN_RXADDR Register
#define ERX_P5          5
#define ERX_P4          4
#define ERX_P3          3
#define ERX_P2          2
#define ERX_P1          1
#define ERX_P0          0

// SETUP_AW Register
#define AW              0  // 2 bits

// SETUP_RETR Register
#define ARD             4  // 4 bits
#define ARC             0  // 4 bits

// RF_SETUP Register
#define CONT_WAVE       7
#define RF_DR_LOW       5
#define PLL_LOCK        4
#define RF_DR_HIGH      3
#define RF_PWR          1

// STATUS Register
#define RX_DR           6
#define TX_DS           5
#define MAX_RT          4
#define RX_P_NO         1  // 3 bits
#define TX_FULL         0

// OBSERVE_TX Register
#define PLOS_CNT        4  // 4 bits
#define ARC_CNT         0  // 4 bits

// FIFO_STATUS Register
#define TX_REUSE        6
#define TX_FULL_FIFO    5
#define TX_EMPTY        4
#define RX_FULL         1
#define RX_EMPTY        0

// DYNPD Register
#define DPL_P5          5
#define DPL_P4          4
#define DPL_P3          3
#define DPL_P2          2
#define DPL_P1          1
#define DPL_P0          0

// FEATURE Register
#define EN_DPL          2
#define EN_ACK_PAY      1
#define EN_DYN_ACK      0

// Commands
#define W_REGISTER 0b100000
#define R_REGISTER 0b000000
#define R_RX_PAYLOAD 0b01100001
#define W_TX_PAYLOAD 0b10100000
#define FLUSH_TX 0b11100001
#define FLUSH_RX 0b11100010
#define REUSE_TX_PL 0b11100011
#define R_RX_PL_WID 0b01100000
#define W_TX_PAYLOAD_NOACK 0b10110000
#define W_ACK_PAYLOAD 0b10101000
#define NOP_CMD 0b11111111

#endif

