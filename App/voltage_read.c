#include "main.h"

#include "cmsis_os.h"

#include "adc.h"
#include "usbd_cdc_if.h"

float voltage;

uint8_t usb_rx_buffer[64];
uint8_t usb_tx_buffer[64];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    /* Prevent unused argument(s) compilation warning */
    voltage = HAL_ADC_GetValue(hadc);
    // voltage = (float)voltage * voltage_vrefint_proportion * 11.0f;
    voltage = (voltage * 3.3f / 65535.0f) * 11.0f;
}

void VoltReadTask(void* argument)
{
    /* USER CODE BEGIN VoltReadTask */
    // HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    // HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, 1);
    HAL_ADC_Start_IT(&hadc1);
    // osDelay(1500);
    /* Infinite loop */
    for (;;)
    {
        // voltage = get_battery_voltage();
        sprintf((char*)usb_tx_buffer, "voltage %4f \r\n", voltage);
        CDC_Transmit_HS(usb_tx_buffer, strlen((char*)usb_tx_buffer));
        osDelay(1000);
    }
    /* USER CODE END VoltReadTask */
}
