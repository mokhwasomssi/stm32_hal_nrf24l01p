# stm32_hal_nrf24l01p

__nRF24L01+, SPI, STM32 HAL__

## 0. Development Environment

- MCU : STM32F411CEU6
- IDE : STM32CubeIDE
- nRF24L01+ module : [NRF24L01+PA+LNA 2.4GHz Wireless RF Transceiver Module](https://electropeak.com/nrf24l01-pa-lna-wireless-module)

    - Pin
        |Name|Description|
        |:---:|:---:|
        |VCC, GND|Power Supply, 1.9V - 3.6V|
        |MOSI, MISO, SCK, CSN|SPI|
        |CE|Chip Enable|
        |IRQ|Interrupt Pin, Active Low|

    <img src = "https://user-images.githubusercontent.com/48342925/127821579-90bb10aa-f2a8-499f-a3e3-e9a086c06564.png" width = "50%" height = "50%">

    ![image](https://user-images.githubusercontent.com/48342925/126250540-7b1a6722-91dc-422f-b028-bfee02d0f004.png)







![image](https://user-images.githubusercontent.com/48342925/126868830-8b13179c-a50b-4d94-b3e5-aab5d05a12bf.png)
