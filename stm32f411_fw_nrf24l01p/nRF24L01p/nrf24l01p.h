  
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

// nRF24L01+ SPI Interface
#define NRF24L01P_SPI                     (&hspi2)

// nRF24L01+ CS Pin
#define NRF24L01P_SPI_CS_PIN_PORT         GPIOB 
#define NRF24L01P_SPI_CS_PIN_NUMBER       GPIO_PIN_12

// nRF24L01+ CE Pin
#define NRF24L01P_CE_PIN_PORT             GPIOB
#define NRF24L01P_CE_PIN_NUMBER           GPIO_PIN_13

// nRF24L01+ IRQ Pin
#define NRF24L01P_IRQ_PIN_PORT            GPIOB
#define NRF24L01P_IRQ_PIN_NUMBER          GPIO_PIN_2

// nRF24L01+ Payload Length
#define NRF24L01P_PAYLOAD_LENGTH          8     // 1 - 32bytes


/* nRF24L01+ typedefs */

typedef uint8_t read_reg;
typedef uint8_t status_reg;
typedef uint8_t write_reg;
typedef uint8_t count;
typedef uint8_t widths;
typedef uint8_t length;
typedef uint16_t delay;
typedef uint16_t channel;

typedef enum
{
    DATA_PIPE_0,
    DATA_PIPE_1,
    DATA_PIPE_2,
    DATA_PIPE_3,
    DATA_PIPE_4,
    DATA_PIPE_5
} data_pipe;

typedef enum
{
    _250kbps = 2,
    _1Mbps   = 0,
    _2Mbps   = 1
} air_data_rate;

typedef enum
{
    _0dBm  = 3,
    _6dBm  = 2,
    _12dBm = 1,
    _18dBm = 0
} output_power;


/* nRF24L01+ Main Functions */
void nrf24l01p_rx_init(channel MHz, air_data_rate bps);
void nrf24l01p_tx_init(channel MHz, air_data_rate bps);

void nrf24l01p_rx_receive();
void nrf24l01p_tx_transmit();


/* nRF24L01+ Sub Functions */
void nrf24l01p_reset();

uint8_t nrf24l01p_read_register(uint8_t reg);
uint8_t nrf24l01p_write_register(uint8_t reg, uint8_t value);

uint8_t nrf24l01p_rx_read_payload(uint8_t* rx_payload);
uint8_t nrf24l01p_tx_write_payload(uint8_t* tx_payload);

uint8_t nrf24l01p_get_status();
uint8_t nrf24l01p_get_fifo_status();

void nrf24l01p_rx_interrupt_flag_clear();
void nrf24l01p_tx_interrupt_flag_clear();

void nrf24l01p_clear_max_rt();

bool nrf24l01p_is_rx_data_ready();
bool nrf24l01p_is_tx_data_sent();
bool nrf24l01p_is_tx_max_rt();
data_pipe nrf24l01p_rx_data_pipe_num();

bool nrf24l01p_is_rx_fifo_empty();
bool nrf24l01p_is_rx_fifo_full();
bool nrf24l01p_is_tx_fifo_empty();
bool nrf24l01p_is_tx_fifo_full();
bool nrf24l01p_is_tx_payload_reused();

void nrf24l01p_prx_mode();
void nrf24l01p_ptx_mode();

void nrf24l01p_power_up();
void nrf24l01p_power_down();

void nrf24l01p_disable_crc();
void nrf24l01p_crc_length(length bytes);

void nrf24l01p_disable_data_ready_interrupt();
void nrf24l01p_disable_data_sent_interrupt();
void nrf24l01p_disable_max_retransmit_interrupt();

void nrf24l01p_disable_enhanced_shockburst();

void nrf24l01p_enable_rx_address(data_pipe pipe_num);
void nrf24l01p_set_address_widths(widths bytes);

void nrf24l01p_auto_retransmit_count(count cnt);
void nrf24l01p_auto_retransmit_delay(delay us);
void nrf24l01p_disable_auto_retransmit();

void nrf24l01p_set_rf_channel(channel MHz);
void nrf24l01p_set_rf_channel_2mbps(channel MHz);
void nrf24l01p_set_rf_tx_output_power(output_power dBm);
void nrf24l01p_set_rf_air_data_rate(air_data_rate bps);
 
void nrf24l01p_enable_rx_dynamic_payload_length(uint8_t data_pipe);
void nrf24l01p_enable_tx_dynamic_payload_length();
status_reg nrf24l01p_read_rx_payload_width();

void nrf24l01p_rx_set_payload_widths(data_pipe pipe_num, widths bytes);
void nrf24l01p_tx_set_static_payload();

void nrf24l01p_flush_rx();
void nrf24l01p_flush_tx();

void nrf24l01p_write_rx_payload_with_ack(data_pipe pipe_num, length bytes);
void nrf24l01p_write_tx_payload_with_noack(uint8_t length);
void nrf24l01p_reuse_tx_payload();


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
#define NRF24L01P_REG_OBSERVE_TX        0x08    // Read-Only
#define NRF24L01P_REG_RPD               0x09    // Read-Only
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
