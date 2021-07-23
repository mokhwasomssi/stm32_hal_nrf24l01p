  
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
#include "stdbool.h"


/* User Configurations */

// nRF24L01+ Mode 
#define RECEIVER
//#define TRANSMITTER

// nRF24L01+ Automatic Protocol
//#define ENHANCED_SHOCKBURST
#define SHOCKBURST

// nRF24L01+ SPI Interface
#define NRF24L01P_SPI                     (&hspi1)
#define NRF24L01P_SPI_CS_PIN_PORT         GPIOA 
#define NRF24L01P_SPI_CS_PIN_NUMBER       GPIO_PIN_4

// nRF24L01+ Chip Enable Pin
#define NRF24L01P_CE_PIN_PORT             GPIOA
#define NRF24L01P_CE_PIN_NUMBER           GPIO_PIN_3


/* nRF24L01+ Main Functions */

void nrf24l01p_reset();

#ifdef RECEIVER
void nrf24l01p_rx_init();
void nrf24l01p_rx_read(uint8_t* rx_payroad);
#endif

#ifdef TRANSMITTER
void nrf24l01p_tx_init();
void nrf24l01p_tx_write(uint8_t* tx_data);
#endif

/* nRF24L01+ Read/Write Registers */

uint8_t nrf24l01p_read_register(uint8_t reg);
uint8_t nrf24l01p_write_register(uint8_t reg, uint8_t value);

/* nRF24L01+ Status */

uint8_t nrf24l01p_cmd_nop();
void nrf24l01p_reset_status();
uint8_t nrf24l01p_get_status();
uint8_t nrf24l01p_get_fifo_status();
bool nrf24l01p_is_rx_empty();
bool nrf24l01p_is_rx_full();
bool nrf24l01p_is_tx_empty();
bool nrf24l01p_is_tx_full();
bool nrf24l01p_is_tx_reuse();

/* nRF24L01+ RX/TX FIFO */

#ifdef RECEIVER
void nrf24l01p_flush_rx();
void nrf24l01p_read_rx_payload(uint8_t length);
void nrf24l01p_write_tx_payload_with_ack(uint8_t data_pipe, uint8_t length);
uint8_t nrf24l01p_read_rx_payload_width();
#endif

#ifdef TRANSMITTER
void nrf24l01p_flush_tx();
void nrf24l01p_write_tx_payload(uint8_t length);
void nrf24l01p_write_tx_payload_with_noack(uint8_t length);
void nrf24l01p_reuse_tx_payload();
#endif

/* nRF24L01+ RX/TX Mode */

void nrf24l01p_reset_mode();
void nrf24l01p_prx_mode();
void nrf24l01p_ptx_mode();

/* nRF24L01+ Power Mode */

void nrf24l01p_reset_power();
void nrf24l01p_power_up();
void nrf24l01p_power_down();

/* nRF24L01+ CRC */

void nrf24l01_reset_crc();
void nrf24l01p_enable_crc();
void nrf24l01p_disable_crc();
void nrf24l01p_crc_length(uint8_t length);

/* nRF24L01+ Interrupt */

void nrf24l01p_reset_interrupt();
void nrf24l01p_enable_all_interrupt();
void nrf24l01p_disable_rx_dr_interrupt();
void nrf24l01p_disable_tx_ds_interrupt();
void nrf24l01p_disable_max_rt_interrupt();

/* nRF24L01+ Enhanced Shockburst */

void nrf24l01p_reset_enhanced_shockburst();
void nrf24l01p_enable_enhanced_shockburst();   
void nrf24l01p_disable_enhanced_shockburst();

/* nRF24L01+ Address */

void nrf24l01p_reset_rx_address();
void nrf24l01p_set_rx_address(uint8_t data_pipe);
void nrf24l01p_reset_address_widths();
void nrf24l01p_set_address_widths(uint8_t widths);

/* nRF24L01+ Auto Retransmission */

void nrf24l01p_reset_auto_retransmit();
void nrf24l01p_disable_auto_retransmit();
void nrf24l01p_auto_retransmit_count(uint8_t count);
void nrf24l01p_auto_retransmit_delay(uint8_t delay);

/* nRF24L01+ RF */

void nrf24l01p_reset_rf();
void nrf24l01p_set_rf_channel(uint16_t channel);
void nrf24l01p_set_rf_channel_2mbps(uint8_t channel);
void nrf24l01p_set_rf_output_power(uint8_t output_power);
void nrf24l01p_set_rf_data_rate(uint8_t data_rate);

/* nRF24L01+ Payload */

void nrf24l01p_reset_payload();

#ifdef RECEIVER
void nrf24l01p_rx_set_static_payload(uint8_t data_pipe, uint8_t length);
void nrf24l01p_rx_set_dynamic_payload(uint8_t data_pipe);
#endif

#ifdef TRANSMITTER
void nrf24l01p_tx_set_static_payload();
void nrf24l01p_tx_set_dynamic_payload();
#endif


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
#define NRF24L01P_CMD_NOP                         0b11111111    

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


#endif /* __NRF24L01P_H__ */


