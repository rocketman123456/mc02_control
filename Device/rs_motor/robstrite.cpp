#include "robstrite.h"

#include "string.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -44.0f 
#define V_MAX 44.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -17.0f
#define T_MAX 17.0f

uint32_t Mailbox; // �����������

/*******************************************************************************
* @����     		: RobStrite���ʵ�����Ĺ��캯��
* @����         : CAN ID
* @����ֵ 			: void
* @����  				: ��ʼ�����ID��
*******************************************************************************/
RobStrite_Motor::RobStrite_Motor(uint8_t CAN_Id, FDCAN_HandleTypeDef* hcan)
{
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0x40 + CAN_Id;
	Motor_Set_All.set_motor_mode = move_control_mode;
	this->hfdcan = hcan;
}

RobStrite_Motor::RobStrite_Motor(float (*Offset_MotoFunc)(float Motor_Tar) , uint8_t CAN_Id, FDCAN_HandleTypeDef* hcan)
{
	CAN_ID = CAN_Id;	
	Master_CAN_ID = 0x40 + CAN_Id;
	Motor_Set_All.set_motor_mode = move_control_mode;
	Motor_Offset_MotoFunc = Offset_MotoFunc;
	this->hfdcan = hcan;
}

/*******************************************************************************
* @����     		: uint16_t��תfloat�͸�����
* @����1        : ��Ҫת����ֵ
* @����2        : x����Сֵ
* @����3        : x�����ֵ
* @����4        : ��Ҫת���Ľ�����
* @����ֵ 			: ʮ���Ƶ�float�͸�����
* @����  				: None
*******************************************************************************/
float uint16_to_float(uint16_t x,float x_min,float x_max,int bits)
{
    uint32_t span = (1 << bits) - 1;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}
/*******************************************************************************
* @����     		: float������תint��
* @����1        : ��Ҫת����ֵ
* @����2        : x����Сֵ
* @����3        : x�����ֵ
* @����4        : ��Ҫת���Ľ�����
* @����ֵ 			: ʮ���Ƶ�int������
* @����  				: None
*******************************************************************************/
int float_to_uint(float x,float x_min,float x_max,int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	if(x > x_max) x = x_max;
	else if(x < x_min) x = x_min;
	return (int) ((x - offset)*((float)((1<<bits)-1))/span);
}
/*******************************************************************************
* @����     		: uint8_t����תfloat������
* @����        	: ��Ҫת��������
* @����ֵ 			: ʮ���Ƶ�float�͸�����
* @����  				: None
*******************************************************************************/
float Byte_to_float(uint8_t* bytedata)  
{  
	uint32_t data = bytedata[7]<<24|bytedata[6]<<16|bytedata[5]<<8|bytedata[4];
	float data_float = *(float*)(&data);
  return data_float;  
}  
//int count_num = 0 ;
/*******************************************************************************
* @����     		: ���մ�����		��ͨ������2 17Ӧ��֡ 0Ӧ��֡��
* @����1        : ���յ�������
* @����2        : ���յ���CANID
* @����ֵ 			: None
* @����  				: drwֻ��ͨ��ͨ��17�����Ժ����ֵ
*******************************************************************************/
void RobStrite_Motor::RobStrite_Motor_Analysis(uint8_t *DataFrame,uint32_t ID_ExtId)
{
	if (uint8_t((ID_ExtId&0xFF00)>>8) == CAN_ID)
	{		
		//count_num++;
		if (int((ID_ExtId&0x3F000000)>>24) == 2)
		{
			Pos_Info.Angle =  uint16_to_float(DataFrame[0]<<8|DataFrame[1],P_MIN,P_MAX,16);
			Pos_Info.Speed =  uint16_to_float(DataFrame[2]<<8|DataFrame[3],V_MIN,V_MAX,16);			
			Pos_Info.Torque = uint16_to_float(DataFrame[4]<<8|DataFrame[5],T_MIN,T_MAX,16);				
			Pos_Info.Temp = (DataFrame[6]<<8|DataFrame[7])*0.1;
			error_code = uint8_t((ID_ExtId&0x3F0000)>>16);
			Pos_Info.pattern = uint8_t((ID_ExtId&0xC00000)>>22);
		}
		else if (int((ID_ExtId&0x3F000000)>>24) == 17)
		{
			for (int index_num = 0; index_num <= 13; index_num++)
			{
				if ((DataFrame[1]<<8|DataFrame[0]) == Index_List[index_num])
					switch(index_num)
					{
						case 0:
							drw.run_mode.data = uint8_t(DataFrame[4]);
							break;
						case 1:
							drw.iq_ref.data = Byte_to_float(DataFrame);
							break;
						case 2:
							drw.spd_ref.data = Byte_to_float(DataFrame);
							break;
						case 3:
							drw.imit_torque.data = Byte_to_float(DataFrame);
							break;
						case 4:
							drw.cur_kp.data = Byte_to_float(DataFrame);
							break;
						case 5:
							drw.cur_ki.data = Byte_to_float(DataFrame);
							break;
						case 6:
							drw.cur_filt_gain.data = Byte_to_float(DataFrame);
							break;
						case 7:
							drw.loc_ref.data = Byte_to_float(DataFrame);
							break;
						case 8:
							drw.limit_spd.data = Byte_to_float(DataFrame);
							break;
						case 9:
							drw.limit_cur.data = Byte_to_float(DataFrame);
							break;	
						case 10:
							drw.mechPos.data = Byte_to_float(DataFrame);
							break;	
						case 11:
							drw.iqf.data = Byte_to_float(DataFrame);
							break;	
						case 12:
							drw.mechVel.data =Byte_to_float(DataFrame);
							break;	
						case 13:
							drw.VBUS.data = Byte_to_float(DataFrame);
							break;	
					}	
			}
		}
		else if ((uint8_t)((ID_ExtId & 0xFF)) == 0xFE)
		{
			CAN_ID = uint8_t((ID_ExtId & 0xFF00)>>8);	
		}	
	}
}
/*******************************************************************************
* @����     		: RobStrite�����ȡ�豸ID��MCU��ͨ������0��
* @����         : None
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStrite_Motor::RobStrite_Get_CAN_ID()
{
	uint8_t txdata[8] = {0};				//��������
	
	FDCAN_TxHeaderTypeDef pTxHeader;

	pTxHeader.Identifier  = Communication_Type_Get_ID<<24|Master_CAN_ID <<8|CAN_ID;
	pTxHeader.IdType      = FDCAN_EXTENDED_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength  = 8;
	
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
	pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker       = 0;
	
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, txdata);
	
	//CAN_TxHeaderTypeDef TxMessage; 	//��������
	//TxMessage.IDE = CAN_ID_EXT;
	//TxMessage.RTR = CAN_RTR_DATA;
	//TxMessage.DLC = 8;
	//TxMessage.ExtId = Communication_Type_Get_ID<<24|Master_CAN_ID <<8|CAN_ID;
  //HAL_CAN_AddTxMessage(hcan, &TxMessage, txdata, &Mailbox); // ����CAN��Ϣ
}
/*******************************************************************************
* @����     		: RobStrite����˿�ģʽ  ��ͨ������1��
* @����1        : ���أ�-4Nm~4Nm��
* @����2        : Ŀ��Ƕ�(-4��~4��)
* @����3        : Ŀ����ٶ�(-30rad/s~30rad/s)
* @����4        : Kp(0.0~500.0)
* @����5        : Kp(0.0~5.0)
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStrite_Motor::RobStrite_Motor_move_control(float Torque, float Angle, float Speed, float Kp, float Kd)
{
	uint8_t txdata[8] = {0};						   	//��������
	//CAN_TxHeaderTypeDef TxMessage; 					//��������
	
	Motor_Set_All.set_Torque = Torque;
	Motor_Set_All.set_angle = Angle;	
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_Kp = Kp;
	Motor_Set_All.set_Kd = Kd;
	if (drw.run_mode.data != 0 && Pos_Info.pattern == 2)
	{
		Set_RobStrite_Motor_parameter(0X7005, move_control_mode, Set_mode);		//���õ��ģʽ
		Get_RobStrite_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = move_control_mode;
	}
	//TxMessage.IDE = CAN_ID_EXT;
	//TxMessage.RTR = CAN_RTR_DATA;
	//TxMessage.DLC = 8;
	//TxMessage.ExtId = Communication_Type_MotionControl<<24|float_to_uint(Motor_Set_All.set_Torque,T_MIN,T_MAX,16)<<8|CAN_ID;
	txdata[0] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16)>>8; 
	txdata[1] = float_to_uint(Motor_Set_All.set_angle, P_MIN,P_MAX, 16); 
	txdata[2] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16)>>8; 
	txdata[3] = float_to_uint(Motor_Set_All.set_speed, V_MIN,V_MAX, 16); 
	txdata[4] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16)>>8; 
	txdata[5] = float_to_uint(Motor_Set_All.set_Kp,KP_MIN, KP_MAX, 16); 
	txdata[6] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16)>>8; 
	txdata[7] = float_to_uint(Motor_Set_All.set_Kd,KD_MIN, KD_MAX, 16); 
	
	FDCAN_TxHeaderTypeDef pTxHeader;
	pTxHeader.Identifier  = Communication_Type_MotionControl<<24|float_to_uint(Motor_Set_All.set_Torque,T_MIN,T_MAX,16)<<8|CAN_ID;
	pTxHeader.IdType      = FDCAN_EXTENDED_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength  = 8;
	
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
	pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker       = 0;
	
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, txdata);
  //HAL_CAN_AddTxMessage(hcan, &TxMessage, txdata, &Mailbox); // ����CAN��Ϣ
}
/*******************************************************************************
* @����     		: RobStrite���λ��ģʽ 
* @����1        : Ŀ����ٶ�(-30rad/s~30rad/s)
* @����2        : Ŀ��Ƕ�(-4��~4��)
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStrite_Motor::RobStrite_Motor_Pos_control(float Speed, float acceleration, float Angle)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_angle = Angle;
	if (drw.run_mode.data != 1 && Pos_Info.pattern == 2)
	{
		Set_RobStrite_Motor_parameter(0X7005, Pos_control_mode, Set_mode);		//���õ��ģʽ
		Get_RobStrite_Motor_parameter(0x7005);
		Motor_Set_All.set_motor_mode = Pos_control_mode;
	}
	Set_RobStrite_Motor_parameter(0X7017, Motor_Set_All.set_speed, Set_parameter);
	Set_RobStrite_Motor_parameter(0X7025, Motor_Set_All.set_acceleration, Set_parameter);	
	Set_RobStrite_Motor_parameter(0X7016, Motor_Set_All.set_angle, Set_parameter);
}
/*******************************************************************************
* @����     		: RobStrite����ٶ�ģʽ 
* @����1        : Ŀ����ٶ�(-30rad/s~30rad/s)
* @����2        : Ŀ���������(0~23A)
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
uint8_t count_set_motor_mode_Speed = 0;
void RobStrite_Motor::RobStrite_Motor_Speed_control(float Speed, float acceleration, float limit_cur)
{
	Motor_Set_All.set_speed = Speed;
	Motor_Set_All.set_limit_cur = limit_cur;
	Motor_Set_All.set_motor_mode = Speed_control_mode;
	Set_RobStrite_Motor_parameter(0X7005, Speed_control_mode, Set_mode);		//���õ��ģʽ
	count_set_motor_mode_Speed++;
	Set_RobStrite_Motor_parameter(0X7018, Motor_Set_All.set_limit_cur, Set_parameter);
  Set_RobStrite_Motor_parameter(0X7022, Motor_Set_All.set_acceleration, Set_parameter);	
	Set_RobStrite_Motor_parameter(0X700A, Motor_Set_All.set_speed, Set_parameter);
}
/*******************************************************************************
* @����     		: RobStrite�������ģʽ
* @����         : Ŀ�����(-23~23A)
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
uint8_t count_set_motor_mode = 0;
void RobStrite_Motor::RobStrite_Motor_current_control(float current)
{
	Motor_Set_All.set_current = current;
	output = Motor_Set_All.set_current;
	if (Pos_Info.pattern == 2 && Motor_Set_All.set_motor_mode != 3)
	{
		Set_RobStrite_Motor_parameter(0X7005, Elect_control_mode, Set_mode);		//���õ��ģʽ
		Motor_Set_All.set_motor_mode = Elect_control_mode;
	}
	if (count_set_motor_mode % 50 == 0)
		Set_RobStrite_Motor_parameter(0X7005, Elect_control_mode, Set_mode);		//���õ��ģʽ
	count_set_motor_mode++;
	Set_RobStrite_Motor_parameter(0X7006, Motor_Set_All.set_current, Set_parameter);
}
/*******************************************************************************
* @����     		: RobStrite������ģʽ
* @����         : None
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStrite_Motor::RobStrite_Motor_Set_Zero_control()
{
	Set_RobStrite_Motor_parameter(0X7005, Set_Zero_mode, Set_mode);					//���õ��ģʽ
}
/*******************************************************************************
* @����     		: RobStrite���ʹ�� ��ͨ������3��
* @����         : None
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStrite_Motor::Enable_Motor()
{
	uint8_t txdata[8] = {0};				//��������
	//CAN_TxHeaderTypeDef TxMessage; 	//��������
	//
	//TxMessage.IDE = CAN_ID_EXT;
	//TxMessage.RTR = CAN_RTR_DATA;
	//TxMessage.DLC = 8;
	//TxMessage.ExtId = Communication_Type_MotorEnable<<24|Master_CAN_ID<<8|CAN_ID;
  //HAL_CAN_AddTxMessage(hcan, &TxMessage, txdata, &Mailbox); // ����CAN��Ϣ
	
	FDCAN_TxHeaderTypeDef pTxHeader;

	pTxHeader.Identifier  = Communication_Type_MotorEnable<<24|Master_CAN_ID <<8|CAN_ID;
	pTxHeader.IdType      = FDCAN_EXTENDED_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength  = 8;
	
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
	pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker       = 0;
	
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, txdata);
}
/*******************************************************************************
* @����     		: RobStrite���ʧ�� ��ͨ������4��
* @����         : �Ƿ��������λ��0����� 1�����
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStrite_Motor::Disenable_Motor(uint8_t clear_error)
{
	uint8_t txdata[8] = {0};					   	//��������
	//CAN_TxHeaderTypeDef TxMessage; 	//��������
	//
	txdata[0] = clear_error;
	//TxMessage.IDE = CAN_ID_EXT;
	//TxMessage.RTR = CAN_RTR_DATA;
	//TxMessage.DLC = 8;
	//TxMessage.ExtId = Communication_Type_MotorStop<<24|Master_CAN_ID<<8|CAN_ID;
	//
  //HAL_CAN_AddTxMessage(hcan, &TxMessage, txdata, &Mailbox); // ����CAN��Ϣ
	
	FDCAN_TxHeaderTypeDef pTxHeader;

	pTxHeader.Identifier  = Communication_Type_MotorStop<<24|Master_CAN_ID <<8|CAN_ID;
	pTxHeader.IdType      = FDCAN_EXTENDED_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength  = 8;
	
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
	pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker       = 0;
	
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, txdata);
}
/*******************************************************************************
* @����     		: RobStrite���д����� ��ͨ������18��
* @����1        : ������ַ
* @����2        : ������ֵ
* @����3        : ѡ���Ǵ������ģʽ ������������ ��Set_mode���ÿ���ģʽ Set_parameter���ò�����
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStrite_Motor::Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode)
{
	uint8_t txdata[8] = {0};						   	//��������
	//CAN_TxHeaderTypeDef TxMessage; 	//��������
	//TxMessage.IDE = CAN_ID_EXT;
	//TxMessage.RTR = CAN_RTR_DATA;
	//TxMessage.DLC = 8;
	//TxMessage.ExtId = Communication_Type_SetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = Index;
	txdata[1] = Index>>8;
	txdata[2] = 0x00;
	txdata[3] = 0x00;	
	if (Value_mode == 'p')
	{
		memcpy(&txdata[4],&Value,4);
	}
	else if (Value_mode == 'j')
	{
		Motor_Set_All.set_motor_mode = int(Value);
		txdata[4] = (uint8_t)Value;
		txdata[5] = 0x00;	
		txdata[6] = 0x00;	
		txdata[7] = 0x00;	
	}
	
	FDCAN_TxHeaderTypeDef pTxHeader;

	pTxHeader.Identifier  = Communication_Type_SetSingleParameter<<24|Master_CAN_ID <<8|CAN_ID;
	pTxHeader.IdType      = FDCAN_EXTENDED_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength  = 8;
	
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
	pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker       = 0;
	
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, txdata);
	
  //HAL_CAN_AddTxMessage(hcan, &TxMessage, txdata, &Mailbox); // ����CAN��Ϣ
}
/*******************************************************************************
* @����     		: RobStrite�������������ȡ ��ͨ������17��
* @����         : ������ַ
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStrite_Motor::Get_RobStrite_Motor_parameter(uint16_t Index)
{
	uint8_t txdata[8] = {0};						   	//��������
	//CAN_TxHeaderTypeDef TxMessage; 	//��������
	txdata[0] = Index;
	txdata[1] = Index>>8;
	//TxMessage.IDE = CAN_ID_EXT;
	//TxMessage.RTR = CAN_RTR_DATA;
	//TxMessage.DLC = 8;
	//TxMessage.ExtId = Communication_Type_GetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
  //HAL_CAN_AddTxMessage(hcan, &TxMessage, txdata, &Mailbox); // ����CAN��Ϣ
	
	FDCAN_TxHeaderTypeDef pTxHeader;

	pTxHeader.Identifier  = Communication_Type_GetSingleParameter<<24|Master_CAN_ID<<8|CAN_ID;
	pTxHeader.IdType      = FDCAN_EXTENDED_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength  = 8;
	
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
	pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker       = 0;
	
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, txdata);
}
/*******************************************************************************
* @����     		: RobStrite�������CAN_ID ��ͨ������7��
* @����         : �޸ĺ�Ԥ�裩CANID
* @����ֵ 			: void
* @����  				: None
*******************************************************************************/
void RobStrite_Motor::Set_CAN_ID(uint8_t Set_CAN_ID)
{
	Disenable_Motor(0);
	uint8_t txdata[8] = {0};						   	//��������
	//CAN_TxHeaderTypeDef TxMessage; 	//��������
	//TxMessage.IDE = CAN_ID_EXT;
	//TxMessage.RTR = CAN_RTR_DATA;
	//TxMessage.DLC = 8;
	//TxMessage.ExtId = Communication_Type_Can_ID<<24|Set_CAN_ID<<16|Master_CAN_ID<<8|CAN_ID;
  //HAL_CAN_AddTxMessage(hcan, &TxMessage, txdata, &Mailbox); // ����CAN��Ϣ
	
	FDCAN_TxHeaderTypeDef pTxHeader;

	pTxHeader.Identifier  = Communication_Type_Can_ID<<24|Set_CAN_ID<<16|Master_CAN_ID<<8|CAN_ID;
	pTxHeader.IdType      = FDCAN_EXTENDED_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength  = 8;
	
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
	pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker       = 0;
	
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, txdata);
}
/*******************************************************************************
* @����     		: RobStrite������û�е��� ��ͨ������6��
* @����         : None
* @����ֵ 			: void
* @����  				: ��ѵ�ǰ���λ����Ϊ��е��λ�� ����ʧ�ܵ��, ��ʹ�ܵ��
*******************************************************************************/
void RobStrite_Motor::Set_ZeroPos()
{
	Disenable_Motor(0);							//ʧ�ܵ��
	uint8_t txdata[8] = {0};						   	//��������
	//CAN_TxHeaderTypeDef TxMessage; 	//��������
	//TxMessage.IDE = CAN_ID_EXT;
	//TxMessage.RTR = CAN_RTR_DATA;
	//TxMessage.DLC = 8;
	//TxMessage.ExtId = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|CAN_ID;
	txdata[0] = 1;
  //HAL_CAN_AddTxMessage(hcan, &TxMessage, txdata, &Mailbox); // ����CAN��Ϣ
	
	FDCAN_TxHeaderTypeDef pTxHeader;

	pTxHeader.Identifier  = Communication_Type_SetPosZero<<24|Master_CAN_ID<<8|CAN_ID;
	pTxHeader.IdType      = FDCAN_EXTENDED_ID;
	pTxHeader.TxFrameType = FDCAN_DATA_FRAME;
	pTxHeader.DataLength  = 8;
	
	pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
	pTxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
	pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
	pTxHeader.MessageMarker       = 0;
	
	HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, txdata);
	
	Enable_Motor();
}

/*******************************************************************************
* @����     		: RobStrite������ݵĲ�����ַ��ʼ��
* @����         : ���ݵĲ�����ַ����
* @����ֵ 			: void
* @����  				: ���ڴ��������ʱ�Զ�����
*******************************************************************************/
data_read_write::data_read_write(const uint16_t *index_list)
{
	run_mode.index      = index_list[0];
	iq_ref.index        = index_list[1];
	spd_ref.index       = index_list[2];
	imit_torque.index   = index_list[3];
	cur_kp.index        = index_list[4];
	cur_ki.index        = index_list[5];
	cur_filt_gain.index = index_list[6];
	loc_ref.index       = index_list[7];
	limit_spd.index     = index_list[8];
	limit_cur.index     = index_list[9];
	mechPos.index       = index_list[10];
	iqf.index           = index_list[11];
	mechVel.index       = index_list[12];
	VBUS.index          = index_list[13];	
	rotation.index      = index_list[14];
}
