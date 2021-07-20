  
/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#include "nrf24l01p.h"


uint8_t rx_payroad[32]; // 1-32 bytes.
uint8_t tx_payroad[32]; // 1-32 bytes.
uint8_t ack_payroad[32];    // in RX mode


/* Write CS Pin */
static void cs_high()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}


/* Write CE Pin */
static void ce_high()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

static void ce_low()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}


/* nRF24L01+ Functions */
uint8_t nrf24l01p_cmd_r_register(uint8_t reg)
{
    uint8_t command = NRF24L01P_CMD_R_REGISTER | reg;
    uint8_t status;
    uint8_t read_data;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, &read_data, 1, 2000);
    cs_high();

    return read_data;
}

uint8_t nrf24l01p_cmd_w_register(uint8_t reg, uint8_t value)
{
    uint8_t command = NRF24L01P_CMD_W_REGISTER | reg;
    uint8_t status;
    uint8_t write_data = value;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, &write_data, 1, 2000);
    cs_high();

    return write_data;
} 

void nrf24l01p_cmd_r_rx_payload(uint8_t rx_byte)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, rx_payroad, rx_byte, 2000);
    cs_high();
}

void nrf24l01p_cmd_w_tx_payload(uint8_t tx_byte)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payroad, tx_byte, 2000);
    cs_high(); 
}

void nrf24l01p_cmd_flush_tx()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_cmd_flush_rx()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_RX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_cmd_reuse_tx_pl()
{
    uint8_t command = NRF24L01P_CMD_REUSE_TX_PL;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_cmd_w_ack_payload(uint8_t pipe, uint8_t ack_byte)
{
    uint8_t command = NRF24L01P_CMD_W_ACK_PAYLOAD | pipe;
    uint8_t status;


    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, ack_payroad, ack_byte, 2000);
    cs_high(); 
}

void nrf24l01p_cmd_w_tx_payload_noack(uint8_t tx_byte)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD_NOACK;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payroad, tx_byte, 2000);
    cs_high(); 
}

void nrf24l01p_cmd_nop()
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high(); 
}

void nrf24l01p_power_up()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 1;

    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_prx()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;

    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_ptx()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;

    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_enable_auto_acknowledgment(uint8_t pipe)
{
    nrf24l01p_cmd_w_register(NRF24L01P_REG_EN_AA, 0x3F);
}

void nrf24l01p_disable_auto_acknowledgment()
{
    nrf24l01p_cmd_w_register(NRF24L01P_REG_EN_AA, 0x00);
}


