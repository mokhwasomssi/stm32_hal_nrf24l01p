/*
 *  nrf24l01_plus.c
 *
 *  Created on: 2021. 7. 20.
 *      Author: mokhwasomssi
 * 
 */


#include "nrf24l01p.h"


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
void nrf24l01p_rx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    nrf24l01p_prx_mode();
    nrf24l01p_power_up();

    nrf24l01p_rx_set_payload_widths(DATA_PIPE_0, NRF24L01P_PAYLOAD_LENGTH);

    nrf24l01p_flush_rx();

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    
    ce_high();
}

void nrf24l01p_tx_init(channel MHz, air_data_rate bps)
{
    nrf24l01p_reset();

    nrf24l01p_ptx_mode();
    nrf24l01p_power_up();

    nrf24l01p_flush_tx();

    nrf24l01p_set_rf_channel(MHz);
    nrf24l01p_set_rf_air_data_rate(bps);
    
    ce_high();
}

void nrf24l01p_rx_receive(uint8_t* rx_payload)
{
    nrf24l01p_rx_read_payload(rx_payload);
    nrf24l01p_rx_interrupt_flag_clear();
}

void nrf24l01p_tx_transmit(uint8_t* tx_payload)
{
    nrf24l01p_tx_write_payload(tx_payload);
    while(!nrf24l01p_is_tx_data_sent());
}


/* nRF24L01+ Sub Functions */
void nrf24l01p_reset()
{
    // Reset pins
    cs_high();
    ce_low();

    // Reset registers
    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, 0x08);
    nrf24l01p_write_register(NRF24L01P_REG_EN_AA, 0x3F);
    nrf24l01p_write_register(NRF24L01P_REG_EN_RXADDR, 0x03);
    nrf24l01p_write_register(NRF24L01P_REG_SETUP_AW, 0x03);
    nrf24l01p_write_register(NRF24L01P_REG_SETUP_RETR, 0x03);
    nrf24l01p_write_register(NRF24L01P_REG_RF_CH, 0x02);
    nrf24l01p_write_register(NRF24L01P_REG_RF_SETUP, 0x07);
    nrf24l01p_write_register(NRF24L01P_REG_STATUS, 0x7E);
    nrf24l01p_write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    nrf24l01p_write_register(NRF24L01P_REG_RX_PW_P0, 0x00);
    nrf24l01p_write_register(NRF24L01P_REG_RX_PW_P1, 0x00);
    nrf24l01p_write_register(NRF24L01P_REG_RX_PW_P2, 0x00);
    nrf24l01p_write_register(NRF24L01P_REG_RX_PW_P3, 0x00);
    nrf24l01p_write_register(NRF24L01P_REG_RX_PW_P4, 0x00);
    nrf24l01p_write_register(NRF24L01P_REG_RX_PW_P5, 0x00);
    nrf24l01p_write_register(NRF24L01P_REG_FIFO_STATUS, 0x11);
    nrf24l01p_write_register(NRF24L01P_REG_DYNPD, 0x00);
    nrf24l01p_write_register(NRF24L01P_REG_FEATURE, 0x00);
}

uint8_t nrf24l01p_read_register(uint8_t reg)
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

uint8_t nrf24l01p_write_register(uint8_t reg, uint8_t value)
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

uint8_t nrf24l01p_get_status()
{
    uint8_t command = NRF24L01P_CMD_NOP;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high(); 

    return status;
}

uint8_t nrf24l01p_get_fifo_status()
{
    return nrf24l01p_read_register(NRF24L01P_REG_FIFO_STATUS);
}

void nrf24l01p_rx_interrupt_flag_clear()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status &= 0x3F;

    nrf24l01p_write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_tx_interrupt_flag_clear()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status &= 0xCF;

    nrf24l01p_write_register(NRF24L01P_REG_STATUS, new_status);
}

void nrf24l01p_clear_max_rt()
{
    uint8_t new_status = nrf24l01p_get_status();
    new_status &= 0x6F;

    nrf24l01p_write_register(NRF24L01P_REG_STATUS, new_status); 
}

bool nrf24l01p_is_rx_data_ready()
{
    uint8_t rx_dr = nrf24l01p_get_status();
    rx_dr &= 0x40;

    if(rx_dr)
        return true;
    else
        return false;
}

bool nrf24l01p_is_tx_data_sent()
{
    uint8_t tx_ds = nrf24l01p_get_status();
    tx_ds &= 0x20;

    if(tx_ds)
        return true;
    else
        return false;
}

bool nrf24l01p_is_tx_max_rt()
{
    uint8_t max_rt = nrf24l01p_get_status();
    max_rt &= 0x10;

    if(max_rt)
        return true;
    else
        return false;
}

data_pipe nrf24l01p_rx_data_pipe_num()
{
    uint8_t rx_p_no = nrf24l01p_get_status();
    rx_p_no &= 0x0E;
    rx_p_no >>= 1;

    return rx_p_no;
}

bool nrf24l01p_is_rx_fifo_empty()
{
    uint8_t rx_empty = nrf24l01p_get_fifo_status();
    rx_empty &= 0x01;

    if(rx_empty)
        return true;
    else
        return false;
}

bool nrf24l01p_is_rx_fifo_full()
{
    uint8_t rx_full = nrf24l01p_get_fifo_status();
    rx_full &= 0x02;

    if(rx_full)
        return true;
    else
        return false;
}

bool nrf24l01p_is_tx_fifo_empty()
{
    uint8_t tx_empty = nrf24l01p_get_fifo_status();
    tx_empty &= 0x10;

    if(tx_empty)
        return true;
    else
        return false;
}

bool nrf24l01p_is_tx_fifo_full()
{
    uint8_t tx_full = nrf24l01p_get_fifo_status();
    tx_full &= 0x20;

    if(tx_full)
        return true;
    else
        return false;
}

bool nrf24l01p_is_tx_payload_reused()
{
    uint8_t tx_reuse = nrf24l01p_get_fifo_status();
    tx_reuse &= 0x40;

    if(tx_reuse)
        return true;
    else
        return false;
}

void nrf24l01p_prx_mode()
{
    uint8_t new_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 0;

    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_ptx_mode()
{
    uint8_t new_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFE;

    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_reset_power()
{
    nrf24l01p_power_down();
}

void nrf24l01p_power_up()
{
    uint8_t new_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    new_config |= 1 << 1;

    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_power_down()
{
    uint8_t new_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xFD;

    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01_reset_crc()
{
    uint8_t reset_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    reset_config |= 1 << 3;
    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, reset_config);

    nrf24l01p_crc_length(1);
}

void nrf24l01p_disable_crc()
{
    uint8_t new_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    new_config &= 0xF7;
    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_crc_length(length bytes)
{
    uint8_t new_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    
    switch(bytes)
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

    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_reset_interrupt()
{
    uint8_t reset_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    reset_config &= 0x0F;
    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, reset_config);
}

void nrf24l01p_disable_data_ready_interrupt()
{
    uint8_t new_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    new_config |= 0x40;
    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_disable_data_sent_interrupt()
{
    uint8_t new_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    new_config |= 0x20;
    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_disable_max_retransmit_interrupt()
{
    uint8_t new_config = nrf24l01p_read_register(NRF24L01P_REG_CONFIG);
    new_config |= 0x10;
    nrf24l01p_write_register(NRF24L01P_REG_CONFIG, new_config);
}

void nrf24l01p_reset_enhanced_shockburst()
{
    nrf24l01p_write_register(NRF24L01P_REG_EN_AA, 0x3F);
}

void nrf24l01p_disable_enhanced_shockburst()
{
    nrf24l01p_write_register(NRF24L01P_REG_EN_AA, 0x00);
}


void nrf24l01p_reset_enable_rx_address()
{
    nrf24l01p_write_register(NRF24L01P_REG_EN_RXADDR, 0x03);
}


void nrf24l01p_enable_rx_address(data_pipe pipe_num)
{
    uint8_t new_en_rxaddr = nrf24l01p_read_register(NRF24L01P_REG_EN_RXADDR);
    new_en_rxaddr |= 1 << pipe_num;

    nrf24l01p_write_register(NRF24L01P_REG_EN_RXADDR, new_en_rxaddr);
}

void nrf24l01p_set_address_widths(widths bytes)
{
    nrf24l01p_write_register(NRF24L01P_REG_SETUP_AW, bytes - 2);
}

void nrf24l01p_disable_auto_retransmit()
{
    uint8_t new_setup_retr = 0x00;
    nrf24l01p_write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_auto_retransmit_count(count cnt)
{
    uint8_t new_setup_retr = nrf24l01p_read_register(NRF24L01P_REG_SETUP_RETR);
    
    // Reset ARC register 0
    new_setup_retr |= 0xF0;
    new_setup_retr |= cnt;
    nrf24l01p_write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_auto_retransmit_delay(delay us)
{
    uint8_t new_setup_retr = nrf24l01p_read_register(NRF24L01P_REG_SETUP_RETR);

    // Reset ARD register 0
    new_setup_retr |= 0x0F;
    new_setup_retr |= ((us / 250) - 1) << 4;
    nrf24l01p_write_register(NRF24L01P_REG_SETUP_RETR, new_setup_retr);
}

void nrf24l01p_set_rf_channel(channel MHz)
{
	uint16_t new_rf_ch = MHz - 2400;
    nrf24l01p_write_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_channel_2mbps(channel MHz)
{
    uint8_t new_rf_ch = MHz - 2400;
    nrf24l01p_write_register(NRF24L01P_REG_RF_CH, new_rf_ch);
}

void nrf24l01p_set_rf_tx_output_power(output_power dBm)
{
    uint8_t new_rf_setup = nrf24l01p_read_register(NRF24L01P_REG_RF_SETUP) & 0xF9;
    new_rf_setup |= (dBm << 1);

    nrf24l01p_write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_set_rf_air_data_rate(air_data_rate bps)
{
    // Set value to 0
    uint8_t new_rf_setup = nrf24l01p_read_register(NRF24L01P_REG_RF_SETUP) & 0xD7;
    
    switch(bps)
    {
        case _1Mbps: 
            break;
        case _2Mbps: 
            new_rf_setup |= 1 << 3;
            break;
        case _250kbps:
            new_rf_setup |= 1 << 5;
            break;
    }
    nrf24l01p_write_register(NRF24L01P_REG_RF_SETUP, new_rf_setup);
}

void nrf24l01p_enable_rx_dynamic_payload_length(uint8_t data_pipe)
{
    // Enable Dynamic Payload Length
    uint8_t new_feature = nrf24l01p_read_register(NRF24L01P_REG_FEATURE);
    new_feature |= 1 << 3;
    nrf24l01p_write_register(NRF24L01P_REG_FEATURE, new_feature);

    // Enable data pipe
    uint8_t new_dynpd = nrf24l01p_read_register(NRF24L01P_REG_DYNPD);
    new_dynpd |= 1 << data_pipe;
    nrf24l01p_write_register(NRF24L01P_REG_DYNPD, new_dynpd);
}

void nrf24l01p_enable_tx_dynamic_payload_length()
{
    // Enable Dynamic Payload Length
    uint8_t new_feature = nrf24l01p_read_register(NRF24L01P_REG_FEATURE);
    new_feature |= 1 << 3;
    nrf24l01p_write_register(NRF24L01P_REG_FEATURE, new_feature);

    // Set DPL_P0 bit in DYNPD register
    uint8_t new_dynpd = nrf24l01p_read_register(NRF24L01P_REG_DYNPD);
    new_dynpd |= 1 << 0;
    nrf24l01p_write_register(NRF24L01P_REG_DYNPD, new_dynpd);
}

uint8_t nrf24l01p_read_rx_payload_width()
{
    uint8_t command = NRF24L01P_CMD_R_RX_PL_WID;
    uint8_t status;
    uint8_t rx_payload_width;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, &rx_payload_width, 1, 2000);
    cs_high();

    return rx_payload_width;
}

void nrf24l01p_rx_set_payload_widths(data_pipe pipe_num, widths bytes)
{
    nrf24l01p_write_register(NRF24L01P_REG_RX_PW_P0 + pipe_num, bytes);
}

void nrf24l01p_tx_set_static_payload()
{
    // nothing?
}

void nrf24l01p_flush_tx()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_TX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

void nrf24l01p_flush_rx()
{
    uint8_t command = NRF24L01P_CMD_FLUSH_RX;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

uint8_t nrf24l01p_rx_read_payload(uint8_t* rx_payload)
{
    uint8_t command = NRF24L01P_CMD_R_RX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Receive(NRF24L01P_SPI, rx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high();

    return status;
}

uint8_t nrf24l01p_tx_write_payload(uint8_t* tx_payload)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payload, NRF24L01P_PAYLOAD_LENGTH, 2000);
    cs_high(); 

    return status;
}

/*
void nrf24l01p_write_rx_payload_with_ack(data_pipe pipe_num, length bytes)
{
    uint8_t command = NRF24L01P_CMD_W_ACK_PAYLOAD | pipe_num;
    uint8_t status;


    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, ack_payroad, bytes, 2000);
    cs_high(); 
}

void nrf24l01p_write_tx_payload_with_noack(uint8_t length)
{
    uint8_t command = NRF24L01P_CMD_W_TX_PAYLOAD_NOACK;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    HAL_SPI_Transmit(NRF24L01P_SPI, tx_payload, length, 2000);
    cs_high(); 
}
*/

void nrf24l01p_reuse_tx_payload()
{
    uint8_t command = NRF24L01P_CMD_REUSE_TX_PL;
    uint8_t status;

    cs_low();
    HAL_SPI_TransmitReceive(NRF24L01P_SPI, &command, &status, 1, 2000);
    cs_high();
}

