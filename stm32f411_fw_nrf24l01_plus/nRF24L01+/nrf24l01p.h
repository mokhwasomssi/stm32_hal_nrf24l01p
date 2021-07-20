  
/*
 *  nrf24l01_plus.h
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#ifndef __NRF24L01P_H__
#define __NRF24L01P_H__


#include "spi.h"    // header from stm32cubemx code generate


/* nRF24L01+ SPI Interface */
#define NRF24L01P_SPI                     (&hspi1)
#define NRF24L01P_SPI_CS_PIN_PORT         GPIOA 
#define NRF24L01P_SPI_CS_PIN_NUMBER       GPIO_PIN_4

/* nRF24L01+ Chip Enable Pin */
#define NRF24L01P_CE_PIN_PORT             GPIOA
#define NRF24L01P_CE_PIN_NUMBER           GPIO_PIN_3


/* nRF24L01+ Commands */
#define NRF24L01P_CMD_R_REGISTER                  0b00000000
#define NRF24L01P_CMD_W_REGISTER                  0b00100000
#define NRF24L01P_CMD_R_RX_PAYLOAD                0b01100001
#define NRF24L01P_CMD_W_TX_PAYLOAD                0b10100000
#define NRF24L01P_CMD_FLUSH_TX                    0b11100001
#define NRF24L01P_CMD_FLUSH_RX                    0b11100010
#define NRF24L01P_CMD_REUSE_TX_PL                 0b11100011
#define NRF24L01P_CMD_R_RX_PL_WID                 0b01100000
#define NRF24L01P_CMD_W_ACK_PAYLOAD               0b10101000
#define NRF24L01P_CMD_W_TX_PAYLOAD_NOACK          0b10110000
#define NRF24L01P_CMD_NOP                         0b11111111    // No Operation. Might be used to read the STATUS register

/* nRF24L01+ Registers */
#define NRF24L01P_REG_CONFIG            0x00
#define NRF24L01P_REG_EN_AA             0x01
#define NRF24L01P_REG_EN_RXADDR         0x02
#define NRF24L01P_REG_SETUP_AW          0x03
#define NRF24L01P_REG_SETUP_RETR        0x04
#define NRF24L01P_REG_RF_CH             0x05
#define NRF24L01P_REG_RF_SETUP          0x06
#define NRF24L01P_REG_STATUS            0x07
#define NRF24L01P_REG_OBSERVE_TX        0x08
#define NRF24L01P_REG_RPD               0x09
#define NRF24L01P_REG_RX_ADDR_P0        0x0A
#define NRF24L01P_REG_RX_ADDR_P1        0x0B
#define NRF24L01P_REG_RX_ADDR_P2        0x0C
#define NRF24L01P_REG_RX_ADDR_P3        0x0D
#define NRF24L01P_REG_RX_ADDR_P4        0x0E
#define NRF24L01P_REG_RX_ADDR_P5        0x0F
#define NRF24L01P_REG_TX_ADDR           0x10
#define NRF24L01P_REG_RX_PW_P0          0x11
#define NRF24L01P_REG_RX_PW_P1          0x12
#define NRF24L01P_REG_RX_PW_P2          0x13
#define NRF24L01P_REG_RX_PW_P3          0x14
#define NRF24L01P_REG_RX_PW_P4          0x15
#define NRF24L01P_REG_RX_PW_P5          0x16
#define NRF24L01P_REG_FIFO_STATUS       0x17
#define NRF24L01P_REG_DYNPD             0x1C
#define NRF24L01P_REG_FEATURE           0x1D


/* nRF24L01+ Functions */

// Commands List
uint8_t nrf24l01p_cmd_r_register(uint8_t reg);
uint8_t nrf24l01p_cmd_w_register(uint8_t reg, uint8_t value);

void nrf24l01p_cmd_r_rx_payload(uint8_t rx_byte);
void nrf24l01p_cmd_w_tx_payload(uint8_t tx_byte);

void nrf24l01p_cmd_flush_tx();
void nrf24l01p_cmd_flush_rx();

void nrf24l01p_cmd_reuse_tx_pl();
void nrf24l01p_cmd_r_rx_pl_wid();
void nrf24l01p_cmd_w_ack_payload(uint8_t pipe, uint8_t ack_byte);
void nrf24l01p_cmd_w_tx_payload_noack(uint8_t tx_byte);
void nrf24l01p_cmd_nop();


// Settings
void nrf24l01p_power_up();
void nrf24l01p_power_down();    // reset

// RX/TX control
void nrf24l01p_prx();
void nrf24l01p_ptx();   // reset

void nrf24l01p_enable_auto_acknowledgment();    // reset
void nrf24l01p_disable_auto_acknowledgment();   // for compatible with nRF24L01

void nrf24l01p_enable_data_pipe(uint8_t pipe);



void nrf24l01p_reset(); // 어떻게 자동 초기화가 안되냐
void nrf24l01p_init();


#endif /* __NRF24L01P_H__ */
