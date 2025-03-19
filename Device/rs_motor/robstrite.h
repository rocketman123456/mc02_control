#ifndef __ROBSTRITE_H__
#define __ROBSTRITE_H__

#ifdef __cplusplus
 extern "C" {
#endif

#ifdef __cplusplus
 }
#endif


#include "main.h"
#include "fdcan.h"
 
#define Set_mode 		  'j'		  //���ÿ���ģʽ
#define Set_parameter 'p'			//���ò���
//���ֿ���ģʽ
#define move_control_mode  0	//�˿�ģʽ
#define Pos_control_mode   1	//λ��ģʽ
#define Speed_control_mode 2  //�ٶ�ģʽ
#define Elect_control_mode 3  //����ģʽ
#define Set_Zero_mode      4  //���ģʽ

//ͨ�ŵ�ַ
#define Communication_Type_Get_ID 0x00     					//��ȡ�豸��ID��64λMCUΨһ��ʶ��`
#define Communication_Type_MotionControl 0x01 			//�˿�ģʽ�������������Ϳ���ָ��
#define Communication_Type_MotorRequest 0x02				//���������������������״̬
#define Communication_Type_MotorEnable 0x03					//���ʹ������
#define Communication_Type_MotorStop 0x04						//���ֹͣ����
#define Communication_Type_SetPosZero 0x06					//���õ����е��λ
#define Communication_Type_Can_ID 0x07							//���ĵ�ǰ���CAN_ID
#define Communication_Type_Control_Mode 0x12				//���õ��ģʽ
#define Communication_Type_GetSingleParameter 0x11	//��ȡ��������
#define Communication_Type_SetSingleParameter 0x12	//�趨��������
#define Communication_Type_ErrorFeedback 0x15				//���Ϸ���֡

class data_read_write_one
{
	public:
		uint16_t index;
		float data;
};
static const uint16_t Index_List[] = {0X7005, 0X7006, 0X700A, 0X700B, 0X7010, 0X7011, 0X7014, 0X7016, 0X7017, 0X7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D};
//18ͨ�����Ϳ���д��Ĳ����б�
//����������			������ַ			����			����			�ֽ���			��λ/˵��
class data_read_write			//�ɶ�д�Ĳ���
{
	public:
		data_read_write_one run_mode;				//0:�˿�ģʽ 1:λ��ģʽ 2:�ٶ�ģʽ 3:����ģʽ 4:���ģʽ uint8  1byte
		data_read_write_one iq_ref;					//����ģʽIqָ��   				float 	4byte 	-23~23A
		data_read_write_one spd_ref;				//ת��ģʽת��ָ�� 				float 	4byte 	-30~30rad/s 
		data_read_write_one imit_torque;		//ת������ 								float 	4byte 	0~12Nm  
		data_read_write_one cur_kp;					//������ Kp 							float 	4byte 	Ĭ��ֵ 0.125  
		data_read_write_one cur_ki;					//������ Ki 							float 	4byte 	Ĭ��ֵ 0.0158  
		data_read_write_one cur_filt_gain;	//�����˲�ϵ��filt_gain 	float 	4byte 	0~1.0��Ĭ��ֵ0.1  
		data_read_write_one loc_ref;				//λ��ģʽ�Ƕ�ָ��				float 	4byte 	rad  
		data_read_write_one limit_spd;			//λ��ģʽ�ٶ�����				float 	4byte 	0~30rad/s  
		data_read_write_one limit_cur;			//�ٶ�λ��ģʽ�������� 		float 	4byte 	0~23A
		//����ֻ�ɶ�
		data_read_write_one mechPos;				//���ض˼�Ȧ��е�Ƕ�			float 	4byte 	rad
		data_read_write_one iqf;						//iq �˲�ֵ 							float 	4byte 	-23~23A
		data_read_write_one	mechVel;				//���ض�ת��							float 	4byte 	-30~30rad/s 	
		data_read_write_one	VBUS;						//ĸ�ߵ�ѹ								float 	4byte 	V	
		data_read_write_one	rotation;				//Ȧ�� 										int16 	2byte   Ȧ��
		data_read_write(const uint16_t *index_list=Index_List);
};

typedef struct
{
	float Angle;
	float Speed;
	float acceleration;
	float Torque;
	float Temp;
	int pattern; //���ģʽ��0��λ1�궨2���У�
}Motor_Pos_RobStrite_Info;

typedef struct
{
	int set_motor_mode;
	float set_current;
	float set_speed;
	float set_acceleration;
	float set_Torque;
	float set_angle;
	float set_limit_cur;
	float set_Kp;
	float set_Ki;
	float set_Kd;
}Motor_Set;

//RobStrite_Motor���
class RobStrite_Motor
{
private:		
	uint8_t CAN_ID;						//CAN ID   (Ĭ��127(0x7f) ����ͨ����λ����ͨ������1�鿴)
	uint16_t Master_CAN_ID;		//����ID  �����ڳ�ʼ���������趨Ϊ0x1F��
	float (*Motor_Offset_MotoFunc)(float Motor_Tar);

	Motor_Set Motor_Set_All;	//�趨ֵ
	uint8_t error_code;

	//CAN_HandleTypeDef* hcan;
	FDCAN_HandleTypeDef* hfdcan;

public:
	float output;
	int Can_Motor;
	Motor_Pos_RobStrite_Info Pos_Info; //�ش�ֵ
	data_read_write drw;      				 //�������

	RobStrite_Motor(uint8_t CAN_Id, FDCAN_HandleTypeDef* hfdcan);
	RobStrite_Motor(float (*Offset_MotoFunc)(float Motor_Tar) , uint8_t CAN_Id, FDCAN_HandleTypeDef* hfdcan);

	void RobStrite_Get_CAN_ID();
	void Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode);
	void Get_RobStrite_Motor_parameter(uint16_t Index);
	void RobStrite_Motor_Analysis(uint8_t *DataFrame,uint32_t ID_ExtId);

	void RobStrite_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd);
	void RobStrite_Motor_Pos_control( float Speed,float acceleration, float Angle);
	void RobStrite_Motor_Speed_control(float Speed,float acceleration, float limit_cur);
	void RobStrite_Motor_current_control( float current);

	void RobStrite_Motor_Set_Zero_control();
	void Enable_Motor();
	void Disenable_Motor( uint8_t clear_error);
	\
	void Set_CAN_ID(uint8_t Set_CAN_ID);
	void Set_ZeroPos();
};

#endif
