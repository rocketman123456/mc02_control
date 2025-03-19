#include "main.h"

#include "cmsis_os.h"

#include "math.h"
#include "tim.h"

// dm motor
#include "bsp_fdcan.h"
#include "dm_motor_ctrl.h"
#include "dm_motor_drv.h"

// rs motor
#include "robstrite.h"

extern osSemaphoreId_t can1BinarySemHandle;
extern osSemaphoreId_t can2BinarySemHandle;
extern osMutexId_t     motorDataMutexHandle;

RobStrite_Motor RobStrite_01(0x02, &hfdcan2);
RobStrite_Motor RobStrite_02(0x04, &hfdcan2);

uint16_t motor_ids[] = {Motor1, Motor2};

#ifdef __cplusplus
extern "C"
{
#endif

    void CAN1_CtrlTask(void* argument)
    {
        /* USER CODE BEGIN CAN1_CtrlTask */
        dm_motor_init();
        osDelay(1000);

        // write_motor_data(motor[Motor1].id, 10, 1, 0, 0, 0);
        // osDelay(1);
        // save_motor_data(motor[Motor1].id, 10);
        // osDelay(1);
        for (int i = 0; i < 2; ++i)
        {
            dm_motor_disable(&hfdcan1, &motor[motor_ids[i]]);
            osDelay(100);

            write_motor_data(motor[motor_ids[i]].id, 10, mit_mode, 0, 0, 0);
            osDelay(100);

            save_motor_data(motor[motor_ids[i]].id, 10);
            osDelay(100);

            dm_motor_enable(&hfdcan1, &motor[motor_ids[i]]);
            osDelay(1);
            // dm_motor_enable(&hfdcan1, &motor[motor_ids[i]]);
            // osDelay(1);
        }
        osDelay(1000);
        HAL_TIM_Base_Start_IT(&htim2);
        /* Infinite loop */
        float phase = 0.0;
        for (;;)
        {
            osSemaphoreAcquire(can1BinarySemHandle, osWaitForever);

            phase += 0.01f;

            for (int i = 0; i < 2; ++i)
            {
                motor[motor_ids[i]].ctrl.pos_set = sin(phase);
                motor[motor_ids[i]].ctrl.vel_set = 0.0;
                motor[motor_ids[i]].ctrl.kp_set  = 2.0;
                motor[motor_ids[i]].ctrl.kd_set  = 0.5;
                motor[motor_ids[i]].ctrl.tor_set = 0.0;
                mit_ctrl(
                    &hfdcan1,
                    &motor[motor_ids[i]],
                    motor[motor_ids[i]].id,
                    motor[motor_ids[i]].ctrl.pos_set,
                    motor[motor_ids[i]].ctrl.vel_set,
                    motor[motor_ids[i]].ctrl.kp_set,
                    motor[motor_ids[i]].ctrl.kd_set,
                    motor[motor_ids[i]].ctrl.tor_set
                );
            }
        }
        /* USER CODE END CAN1_CtrlTask */
    }

    void CAN2_CtrlTask(void* argument)
    {
        /* USER CODE BEGIN CAN2_CtrlTask */
        osDelay(1000);
        // RobStrite_01.RobStrite_Get_CAN_ID();  //电机获取设备ID和MCU，请按照需要使用
        // osDelay(1);
        // RobStrite_01.Set_CAN_ID(0x02);				//电机设置CANID，请按照需要使用
        // osDelay(1);
        // RobStrite_01.Set_ZeroPos();					  //设置当前位置为电机机械零位，请按照需要使用
        RobStrite_01.Enable_Motor(); // 电机使能
        RobStrite_02.Enable_Motor();
        osDelay(1);
        RobStrite_01.Enable_Motor();
        RobStrite_02.Enable_Motor();
        osDelay(1);
        // for(int i = 0; i < 2; ++i)
        //{
        //	//write_motor_data(motor[motor_ids[i]].id, 10, mit_mode, 0, 0, 0);
        //	//osDelay(100);
        //
        //  dm_motor_enable(&hfdcan2, &motor[motor_ids[i]]);
        //  osDelay(1);
        //  dm_motor_enable(&hfdcan2, &motor[motor_ids[i]]);
        //  osDelay(1);
        //}
        osDelay(1000);
        // HAL_TIM_Base_Start_IT(&htim3);
        /* Infinite loop */
        float phase = 0.0;
        for (;;)
        {
            osSemaphoreAcquire(can2BinarySemHandle, osWaitForever);

            phase += 0.01f;

            //*********运控模式*************//
            float T     = 0.0;        // 扭矩
            float Angle = sin(phase); // 要转到的位置
            float Speed = sin(phase); // 目标速度
            // float Pacceleration = 1.0; //加速度
            float Kp = 1.0; // Kp
            float Kd = 0.5; // Kd
            RobStrite_01.RobStrite_Motor_move_control(T, Angle, Speed, Kp, Kd);
            RobStrite_02.RobStrite_Motor_move_control(T, -Angle, -Speed, Kp, Kd);

            // RobStrite_01.Disenable_Motor(0);	//电机失能，谨慎使用

            // for(int i = 0; i < 2; ++i)
            //{
            //   motor[motor_ids[i]].ctrl.pos_set = sin(phase);
            //   motor[motor_ids[i]].ctrl.vel_set = 0.0;
            //   motor[motor_ids[i]].ctrl.kp_set = 2.0;
            //   motor[motor_ids[i]].ctrl.kd_set = 0.5;
            //   motor[motor_ids[i]].ctrl.tor_set = 0.0;
            //   mit_ctrl(
            //     &hfdcan2,
            //     &motor[motor_ids[i]],
            //     motor[motor_ids[i]].id,
            //     motor[motor_ids[i]].ctrl.pos_set,
            //     motor[motor_ids[i]].ctrl.vel_set,
            //     motor[motor_ids[i]].ctrl.kp_set,
            //     motor[motor_ids[i]].ctrl.kd_set,
            //     motor[motor_ids[i]].ctrl.tor_set
            //   );
            // }
        }
        /* USER CODE END CAN2_CtrlTask */
    }

    // void can1_rx_callback(void)
    //{
    //   uint16_t rec_id;
    //   uint8_t  rx_data[8] = {0};
    //   canx_receive(&hcan1, &rec_id, rx_data);
    //   switch (rec_id)
    //   {
    //		case 0x81: dm_motor_fbdata(&motor[Motor1], rx_data); receive_motor_data(&motor[Motor1], rx_data); break;
    //		case 0x83: dm_motor_fbdata(&motor[Motor2], rx_data); receive_motor_data(&motor[Motor2], rx_data); break;
    //   }
    // }
    //
    // void can2_rx_callback(void)
    //{
    //   CAN_RxHeaderTypeDef rx_header;
    //   uint8_t rx_data[8] = {0};
    //	uint8_t rec_id;
    //   if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&rx_header,rx_data) == HAL_OK)
    //   {
    //     if (rx_header.IDE == CAN_ID_EXT)
    //     {
    //			rec_id = uint8_t((rx_header.ExtId&0xFF00)>>8);
    //			if(rec_id == 0x02)
    //			{
    //				RobStrite_01.RobStrite_Motor_Analysis(rx_data, rx_header.ExtId);
    //			}
    //			if(rec_id == 0x04)
    //			{
    //				RobStrite_02.RobStrite_Motor_Analysis(rx_data, rx_header.ExtId);
    //			}
    //     }
    //   }
    // }

    void fdcan1_rx_callback(void)
    {
        uint16_t rec_id;
        uint8_t  rx_data[8] = {0};
        fdcanx_receive(&hfdcan1, &rec_id, rx_data);
        switch (rec_id)
        {
            case 0x81:
                dm_motor_fbdata(&motor[Motor1], rx_data);
                receive_motor_data(&motor[Motor1], rx_data);
                break;
            case 0x83:
                dm_motor_fbdata(&motor[Motor2], rx_data);
                receive_motor_data(&motor[Motor2], rx_data);
                break;
        }
    }

    void fdcan2_rx_callback(void)
    {
        uint32_t ext_id;
        uint16_t rec_id;
        uint8_t  rx_data[8] = {0};
        // fdcanx_receive(&hfdcan2, &rec_id, rx_data);
        // switch (rec_id)
        //{
        //	case 0x81: dm_motor_fbdata(&motor[Motor1], rx_data); receive_motor_data(&motor[Motor1], rx_data); break;
        //	case 0x83: dm_motor_fbdata(&motor[Motor2], rx_data); receive_motor_data(&motor[Motor2], rx_data); break;
        // }
        FDCAN_RxHeaderTypeDef pRxHeader;

        if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &pRxHeader, rx_data) == HAL_OK)
        {
            ext_id = pRxHeader.Identifier;
            rec_id = uint8_t((ext_id & 0xFF00) >> 8);
            if (rec_id == 0x02)
            {
                RobStrite_01.RobStrite_Motor_Analysis(rx_data, ext_id);
            }
            if (rec_id == 0x04)
            {
                RobStrite_02.RobStrite_Motor_Analysis(rx_data, ext_id);
            }
        }
    }

#ifdef __cplusplus
}
#endif
