#include "main.h"

#include "cmsis_os.h"

#include "adc.h"
#include "tim.h"
#include "usbd_cdc_if.h"
#include "ws2812.h"

// uint8_t r = 1;
// uint8_t g = 1;
// uint8_t b = 1;

// uint32_t adc_raw_data[2]; __attribute__((section(".ARM.__at_0x24000000")));
// uint16_t adc_val_dma[2];
// uint16_t adc_val[2];
// uint8_t usb_rx_buffer[64];
// uint8_t usb_tx_buffer[64];
// float vbus;
// float voltage;

// uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch);
// float get_battery_voltage(void);

void StartDefaultTask(void* argument)
{
    /* init code for USB_DEVICE */
    osDelay(1000);
    // HAL_GPIO_WritePin(GPIOC, POWER_OUT2_Pin|POWER_OUT1_Pin, GPIO_PIN_RESET);
    // HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
    // TIM12->CCR2 = 50;
    /* USER CODE BEGIN StartDefaultTask */
    /* Infinite loop */
    for (;;)
    {
        // HAL_GPIO_WritePin(GPIOC, POWER_OUT2_Pin|POWER_OUT1_Pin, GPIO_PIN_SET);

        // WS2812_Ctrl(100, 0, 0);
        // osDelay(100);

        // HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2);
        // TIM12->CCR2 = 50;

        WS2812_Ctrl(0, 100, 0);
        osDelay(1000);

        // WS2812_Ctrl(0, 0, 100);
        // osDelay(100);

        // HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2);
        // TIM12->CCR2 = 0;

        WS2812_Ctrl(0, 0, 0);
        osDelay(1000);

        // vofa_start();
        // voltage = get_battery_voltage();
        // adc_val_dma[0] = adc_raw_data[0] & 0xffff;
        // adc_val_dma[0] = adc_val_dma[0] >> 16;
        // vbus = (adc_val_dma[0]*3.3f/65535)*11.0f;
        // sprintf((char*)usb_tx_buffer, "voltage: %4f \r\n", vbus);
        // CDC_Transmit_HS(usb_tx_buffer, strlen((char*)usb_tx_buffer));
    }
    /* USER CODE END StartDefaultTask */
}
