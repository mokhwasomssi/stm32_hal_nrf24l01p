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


/* nRF24L01+ CS, CE Status */

static void cs_high()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_SET);
}

static void cs_low()
{
    HAL_GPIO_WritePin(NRF24L01P_SPI_CS_PIN_PORT, NRF24L01P_SPI_CS_PIN_NUMBER, GPIO_PIN_RESET);
}

static void ce_high()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_SET);
}

static void ce_low()
{
    HAL_GPIO_WritePin(NRF24L01P_CE_PIN_PORT, NRF24L01P_CE_PIN_NUMBER, GPIO_PIN_RESET);
}


/* nRF24L01+ Main Functions */

void nrf24l01p_init()
{
    // STATUS, FIFO_STATUS register
    nrf24l01p_reset_status();

    // CONFIG register
    nrf24l01p_reset_mode();
    nrf24l01p_reset_power();
    nrf24l01_reset_crc();
    nrf24l01p_reset_interrupt();

    // EN_AA register
    nrf24l01p_reset_enhanced_shockburst();

    // EN_RXADDR register
    nrf24l01p_reset_rx_address();

    // SETUP_AW register
    nrf24l01p_reset_address_widths();

    // SETUP_RETR register
    nrf24l01p_reset_auto_retransmit();

    // RF_CH, RF_SETUP register
    nrf24l01p_reset_rf();

    // RX_PW_Px, DYNPD ,FEATURE register
    nrf24l01p_reset_payload();

    // Clear FIFO
    nrf24l01p_cmd_flush_tx();
    nrf24l01p_cmd_flush_rx();

    // CS Pin Idle
    cs_high();
}


/* nRF24L01+ Command Functions */

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

void nrf24l01p_cmd_r_rx_payload(uint8_t length)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, rx_payroad, length, 2000);
    cs_high();
}

void nrf24l01p_cmd_w_tx_payload(uint8_t length)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payroad, length, 2000);
    cs_high(); 
}

/**
 * @brief  Flush TX FIFO, used in TX mode
 */
void nrf24l01p_cmd_flush_tx()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

/**
 * @brief  Flush RX FIFO, used in RX mode
 */
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

void nrf24l01p_cmd_w_ack_payload(uint8_t data_pipe, uint8_t length)
{
    uint8_t command = NRF24L01P_CMD_W_ACK_PAYLOAD | data_pipe;
    uint8_t status;


    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, ack_payroad, length, 2000);
    cs_high(); 
}

void nrf24l01p_cmd_w_tx_payload_noack(uint8_t length)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD_NOACK;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payroad, length, 2000);
    cs_high(); 
}

/**
  * @brief  No Operation. Might be used to read the STATUS register
  * @retval STATUS register value
  */
uint8_t nrf24l01p_cmd_nop()
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high(); 

    return status;
}


/* nRF24L01+ Status */

/**
 * @brief  Reset STATUS and FIFO_STATUS registers
 */
void nrf24l01p_reset_status()
{
    uint8_t reset_status = 0x0E;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_STATUS, reset_status);

    uint8_t reset_fifo_status = 0x11;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_FIFO_STATUS, reset_fifo_status);
}

uint8_t nrf24l01p_get_status()
{
    return nrf24l01p_cmd_nop();
}

uint8_t nrf24l01p_get_fifo_status()
{
    return nrf24l01p_cmd_r_register(NRF24L01P_REG_FIFO_STATUS);
}


/* nRF24L01+ Interrupt */

/**
 * @brief  Reset MASK bits in CONFIG register
 */
void nrf24l01p_reset_interrupt()
{
    nrf24l01p_enable_all_interrupt();
}

/**
 * @brief  All IRQ sources(TX_DX, RX_DR, MAX_RT) are enabled
 */
void nrf24l01p_enable_all_interrupt()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config &= 0x0F;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

/**
 * @brief  Disable Receive Data Ready interrupt
 */
void nrf24l01p_disable_rx_dr_interrupt()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 6;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

/**
 * @brief  Disable Transmit Data Sent interrupt
 */
void nrf24l01p_disable_tx_ds_interrupt()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 5;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

/**
 * @brief  Disable Maximum number of TX retransmits interrupt
 */
void nrf24l01p_disable_max_rt_interrupt()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 4;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

/* nRF24L01+ CRC */

/**
 * @brief  Reset EN_CRC, CRCO bits in CONFIG register
 */
void nrf24l01_reset_crc()
{
    nrf24l01p_enable_crc();
    nrf24l01p_crc_length(1);
}

/**
 * @brief  Enable CRC(Cyclic Redundancy Check)
 */
void nrf24l01p_enable_crc()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 3;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

/**
 * @brief Disable CRC(Cyclic Redundancy Check)
 */
void nrf24l01p_disable_crc()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xF7;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

/**
 * @brief  
 * @param byte 1byte(reset), 2bytes
 */
void nrf24l01p_crc_length(uint8_t length)
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    
    switch(length)
    {
        // CRCO bit in CONFIG resiger set 0
        case 1:
            new_config &= 0xFB;
            break;
        // CRCO bit in CONFIG resiger set 1
        case 2:
            new_config |= 1 << 2;
            break;
    }

    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}


/* nRF24L01+ Power Mode */

void nrf24l01p_reset_power()
{
    nrf24l01p_power_down();
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


/* nRF24L01+ RX/TX Mode */

void nrf24l01p_reset_mode()
{
    nrf24l01p_ptx_mode();
}

void nrf24l01p_prx_mode()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;

    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_ptx_mode()
{
    uint8_t new_config = nrf24l01p_cmd_r_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;

    nrf24l01p_cmd_w_register(NRF24L01P_REG_CONFIG, new_config);
}


/* nRF24L01+ Address */

/**
 * @brief  Reset SETUP_AW register
 */
void nrf24l01p_reset_address_widths()
{
    nrf24l01p_set_address_widths(5);
}

/**
  * @brief  RX/TX Address field width
  * @param  widths 3bytes, 4bytes, 5bytes(reset)
  */
void nrf24l01p_set_address_widths(uint8_t widths)
{
    nrf24l01p_cmd_w_register(NRF24L01P_REG_SETUP_AW, widths - 2);
}

/**
 * @brief  Reset EN_RXADDR register
 */
void nrf24l01p_reset_rx_address()
{
    nrf24l01p_cmd_w_register(NRF24L01P_REG_EN_RXADDR, 0x03);
}

/**
 * @brief  Enable data pipe
 * @param  data_pipe 0 - 5
 */
void nrf24l01p_set_rx_address(uint8_t data_pipe)
{
    uint8_t new_en_rxaddr = nrf24l01p_cmd_r_register(NRF24L01P_REG_EN_RXADDR);
    new_en_rxaddr |= 1 << data_pipe;

    nrf24l01p_cmd_w_register(NRF24L01P_REG_EN_RXADDR, new_en_rxaddr);
}


/* nRF24L01+ Enhanced Shockburst */

/**
 * @brief  Reset EN_AA register
 */
void nrf24l01p_reset_enhanced_shockburst()
{
    nrf24l01p_enable_enhanced_shockburst();
}

/**
 * @brief  Enable 'Auto Acknowledgment Function
 */
void nrf24l01p_enable_enhanced_shockburst()
{
    nrf24l01p_cmd_w_register(NRF24L01P_REG_EN_AA, 0x3F);
}

/**
 * @brief  Disable this functionality to be compatible with nRF24L01
 */
void nrf24l01p_disable_enhanced_shockburst()
{
    nrf24l01p_cmd_w_register(NRF24L01P_REG_EN_AA, 0x00);
}


/* nRF24L01+ Auto Retransmission */

/**
 * @brief  Reset SETUP_RETR register
 */
void nrf24l01p_reset_auto_retransmit()
{
    nrf24l01p_auto_retransmit_count(3);
    nrf24l01p_auto_retransmit_delay(250);
}

/**
 * @brief  Re-Transmit disabled
 */
void nrf24l01p_disable_auto_retransmit()
{
    uint8_t new_setup_retr = 0x00;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

/**
 * @brief  Count of Re-Transmit on fail of Auto Acknowledgment
 * @param  count 1 - 15, 3(reset)
 */
void nrf24l01p_auto_retransmit_count(uint8_t count)
{
    uint8_t new_setup_retr = nrf24l01p_cmd_r_register(NRF24L01P_REG_SETUP_RETR);
    
    // Reset ARC register 0
    new_setup_retr |= 0xF0;
    new_setup_retr |= count;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

/**
 * @brief  Delay from end of transmission to start of next transmission
 * @param  delay 250us, 500us, 750us, ..., 4000us
 */
void nrf24l01p_auto_retransmit_delay(uint8_t delay)
{
    uint8_t new_setup_retr = nrf24l01p_cmd_r_register(NRF24L01P_REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr |= 0x0F;
    new_setup_retr |= ((delay / 250) - 1) << 4;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}


/* nRF24L01+ RF */

/**
 * @brief   Reset RF_CH, RF_SETUP registers
 */
void nrf24l01p_reset_rf()
{
    nrf24l01p_set_rf_channel(2402);
    nrf24l01p_set_rf_output_power(3);
    nrf24l01p_set_rf_data_rate(1);
}

/**
 * @brief   Set RF Channel at 250kbps and 1Mbps
 * @details Channel spacing is 1MHz
 * @param   channel 2400MHz - 2525MHz, 2402MHz(reset)
 */
void nrf24l01p_set_rf_channel(uint16_t channel)
{
	uint16_t new_rf_ch = channel - 2400;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

/**
 * @brief  Set RF Channel at 2Mbps
 * @details Channel spacing is 2MHz
 * @param  channel 2400MHz - 2525MHz, 2402MHz(reset)
 */
void nrf24l01p_set_rf_channel_2mbps(uint8_t channel)
{
    uint8_t new_rf_ch = channel - 2400;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

/**
 * @brief  Set RF output power in TX mode
 * @param  output_power 0(-18dBm), 1(-12dBm), 2(-6dBm), 3(0dBm, reset)
 */
void nrf24l01p_set_rf_output_power(uint8_t output_power)
{
    uint8_t new_rf_setup = nrf24l01p_cmd_r_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (output_power << 1);

    nrf24l01p_cmd_w_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

/**
 * @brief  Set RF Date Rate
 * @param  data_rate 0(1Mbps), 1(2Mbps, reset), 2(250kbps)
 */
void nrf24l01p_set_rf_data_rate(uint8_t data_rate)
{
    // Set value to 0
    uint8_t new_rf_setup = nrf24l01p_cmd_r_register(NRF24L01P_REG_RF_SETUP) & 0xD7;
    
    switch(data_rate)
    {
        case 0: // 1Mbps
            break;
        case 1: // 2Mbps
            new_rf_setup |= 1 << 3;
            break;
        case 2: // 250kbps
            new_rf_setup |= 1 << 5;
            break;
    }

    nrf24l01p_cmd_w_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}


/* nRF24L01+ Payload */

/**
 * @brief  Reset RX_PW_Px, DYNPD ,FEATURE register
 */
void nrf24l01p_reset_payload()
{
    for(int i = 0; i < 6; i++)
    {
        // RX_PW_P0 - RX_PW_P5
        nrf24l01p_cmd_w_register(NRF24L01P_REG_RX_PW_P0 + i, 0);
    }
    
    uint8_t reset_dynpd = 0x00;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_DYNPD, reset_dynpd);

    uint8_t reset_feature = 0x00;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_FEATURE, reset_feature);
}

/**
 * @brief  Number of bytes in RX payload in data pipe
 * @param  data_pipe 0 - 5
 * @param  length 1 - 32, 0(reset)
 */
void nrf24l01p_rx_set_static_payload(uint8_t data_pipe, uint8_t length)
{
    nrf24l01p_cmd_w_register(NRF24L01P_REG_RX_PW_P0 + data_pipe, length);
}

void nrf24l01p_tx_set_static_payload()
{
    // nothing?
}

/**
 * @brief  Enable dynamic payload length in RX mode  
 * @param  data_pipe 0 - 5
 */
void nrf24l01p_rx_set_dynamic_payload(uint8_t data_pipe)
{
    // Enable Dynamic Payload Length
    uint8_t new_feature = nrf24l01p_cmd_r_register(NRF24L01P_REG_FEATURE);
    new_feature |= 1 << 3;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_FEATURE, new_feature);

    // Enable data pipe
    uint8_t new_dynpd = nrf24l01p_cmd_r_register(NRF24L01P_REG_DYNPD);
    new_dynpd |= 1 << data_pipe;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_DYNPD, new_dynpd);
}

/**
 * @brief  Enable dynamic payload length in TX mode  
 */
void nrf24l01p_tx_set_dynamic_payload()
{
    // Enable Dynamic Payload Length
    uint8_t new_feature = nrf24l01p_cmd_r_register(NRF24L01P_REG_FEATURE);
    new_feature |= 1 << 3;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_FEATURE, new_feature);

    // Set DPL_P0 bit in DYNPD register
    uint8_t new_dynpd = nrf24l01p_cmd_r_register(NRF24L01P_REG_DYNPD);
    new_dynpd |= 1 << 0;
    nrf24l01p_cmd_w_register(NRF24L01P_REG_DYNPD, new_dynpd);
}
