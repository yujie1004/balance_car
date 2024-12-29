#include "Data_Analysis.h"

uint8_t Data_receive[3];
/*
	֡ͷΪ0xAA��֡βΪ0xBB��
	�м�ֵData_receive[1]��
	ֹͣ��0x00
	ǰ����0x01
	���ˣ�0x02
	��ת��0x03
	��ת��0x04
*/
void Data_rec(void)
{
	xQueueReceive(xQueue_rec,Data_receive,0);
	if(Data_receive[0] == 0xAA && Data_receive[2] == 0xBB)
	{
		switch (Data_receive[1])
		{
			case 0x00:
				velocity = 0;
				angle_turn = 0;
				//OLED_ShowString(0,0,"0",16);
				break;
			case 0x01:
				velocity = 20;
//				velocity+=2;
//			    velocity = velocity>20 ? 20 : velocity;//�޷�
				//OLED_ShowString(0,0,"1",16);
				break;
			case 0x02:
				velocity = -20;
//				velocity-=2;
//			    velocity = velocity<-20 ? -20 : velocity;//�޷�
				//OLED_ShowString(0,0,"2",16);
				break;
			case 0x03:
				angle_turn = 35;
//				angle_turn+=10;
//			    angle_turn = angle_turn>20 ? 20 : angle_turn;//�޷�
				//OLED_ShowString(0,0,"3",16);
				break;
			case 0x04:
				angle_turn = -35; 
			
//				angle_turn-=10;
//			    angle_turn = angle_turn<-20 ? -20 : angle_turn;//�޷�
				//OLED_ShowString(0,0,"4",16);
				break;
			default:
				//OLED_ShowString(0,0,"error",16);
				break;
		}
	}
//	else
//		OLED_ShowString(0,6,"error",16);
}

