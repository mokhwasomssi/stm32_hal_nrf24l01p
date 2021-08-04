# stm32_hal_nrf24l01p

## Brief

__nRF24L01+ is a single chip 2.4GHz transceiver.__  
___TAG__ - `nRF24L01+` `SPI` `STM32 HAL`_  

## Library Features
- Only consider 1:1 transaction
    - Not support `MultiCeiver`
- Use Enhanced ShockBurst
    - TX : Auto Retransmit
    - RX : Auto Acknowledgement
- Use IRQ Pin


## Dev Environment
- STM32CubeIDE
- STM32F411CEU6

## nRF24L01+ Module
[NRF24L01+PA+LNA 2.4GHz Wireless RF Transceiver Module](https://electropeak.com/nrf24l01-pa-lna-wireless-module)

- Pin
    |Name|Description|
    |:---:|:---:|
    |VCC, GND|Power Supply, 1.9V - 3.6V|
    |MOSI, MISO, SCK, CSN|SPI|
    |CE|Chip Enable|
    |IRQ|Interrupt Pin, Active Low|

- Pinout

    ![image](https://user-images.githubusercontent.com/48342925/126250540-7b1a6722-91dc-422f-b028-bfee02d0f004.png)
