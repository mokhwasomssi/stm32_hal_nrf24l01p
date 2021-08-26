# stm32_hal_nrf24l01p

## Brief

__nRF24L01+ is a single chip 2.4GHz transceiver.__  
___TAG__ - `nRF24L01+` `SPI` `STM32 HAL`_  

<img src = "https://user-images.githubusercontent.com/48342925/128331144-e377e215-7260-4fe3-9fbd-e04e87840eb9.gif" width = "70%" height = "70%">

## Library Features
- 1:1 transaction
- Static payload lengths (1 - 32bytes)
- Use IRQ Pin

## Dev Environment
- STM32CubeIDE
- STM32 HAL driver
- STM32F411
- nRF24L01+ Module ([NRF24L01+PA+LNA 2.4GHz Wireless RF Transceiver Module](https://electropeak.com/nrf24l01-pa-lna-wireless-module))
    - Pinout

        ![image](https://user-images.githubusercontent.com/48342925/126250540-7b1a6722-91dc-422f-b028-bfee02d0f004.png)
        |Name|Description|
        |:---:|:---:|
        |VCC, GND|Power Supply, 1.9V - 3.6V|
        |MOSI, MISO, SCK, CSN|SPI|
        |CE|Chip Enable|
        |IRQ|Interrupt Pin, Active Low|

## STM32CubeMX
- Project Manager
    ![image](https://user-images.githubusercontent.com/48342925/124620697-a34acd00-deb4-11eb-8b47-8fe5a3dad001.png)

- SPI
    ![image](https://user-images.githubusercontent.com/48342925/131006561-9bbc5104-e4de-4a15-ae27-574336a0de55.png)

- CSN, CE (GPIO_OUTPUT)
    ![image](https://user-images.githubusercontent.com/48342925/131006997-57bd58fb-5bff-4456-bf0e-cf5c19686191.png)

    ![image](https://user-images.githubusercontent.com/48342925/131007147-c884ab5d-bb96-42f9-ad39-97577d89ffc0.png)

- IRQ (GPIO_EXIT)
    ![image](https://user-images.githubusercontent.com/48342925/131007248-468264f7-698f-40ae-8797-60b361eba55c.png)

    ![image](https://user-images.githubusercontent.com/48342925/131014420-b07a3373-198a-4d0c-a455-06387abaf43b.png)


## Example

### nrf24l01p.h
- SPI2, PB13 (CSN), PB12 (CE), PA8 (IRQ), Payload length 8

```c
/* User Configurations */
#define NRF24L01P_SPI                     (&hspi2)

#define NRF24L01P_SPI_CS_PIN_PORT         GPIOB 
#define NRF24L01P_SPI_CS_PIN_NUMBER       GPIO_PIN_13

#define NRF24L01P_CE_PIN_PORT             GPIOB
#define NRF24L01P_CE_PIN_NUMBER           GPIO_PIN_12

#define NRF24L01P_IRQ_PIN_PORT            GPIOA
#define NRF24L01P_IRQ_PIN_NUMBER          GPIO_PIN_8

#define NRF24L01P_PAYLOAD_LENGTH          8     // 1 - 32bytes
```

### main.c
- Only contain relative things

#### Transmitter
```c
#include "nrf24l01p.h"

// data array to be sent
uint8_t tx_data[NRF24L01P_PAYLOAD_LENGTH] = {0, 1, 2, 3, 4, 5, 6, 7};

// for rx interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

int main(void)
{
    nrf24l01p_tx_init(2500, _1Mbps);
 
    while (1)
    {
        // change tx datas
        for(int i= 0; i < 8; i++)
            tx_data[i]++;

        // transmit
        nrf24l01p_tx_transmit(tx_data);
        HAL_Delay(100);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
		nrf24l01p_tx_irq(); // clear interrupt flag
}
```

#### Receiver
```c
#include "nrf24l01p.h"

// data array to be read
uint8_t rx_data[NRF24L01P_PAYLOAD_LENGTH] = { 0, };

// for tx interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

int main(void)
{
    nrf24l01p_rx_init(2500, _1Mbps);
 
    while (1)
    {
        // Nothing to do
        HAL_Delay(100);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF24L01P_IRQ_PIN_NUMBER)
		nrf24l01p_rx_receive(rx_data); // read data when data ready flag is set
}
```