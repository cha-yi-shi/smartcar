#include "encoder.h"

int32 encoder1,encoder2;
int32 cnt1,cnt2;
int temp_EX=0;//实例变量（文档教学）



void INT2_Isr() interrupt 10
{
	INT2_CLEAR_FLAG;  //清除中断标志
	if(0 == P50)
		cnt1++;
	else
		cnt1--;
}
void INT3_Isr() interrupt 11
{
	INT3_CLEAR_FLAG;  //清除中断标志
	if(0 == P51)
		cnt2++;
	else
		cnt2--;
}

void encoder_init(encoder_index_enum encoder_n)
{
	if(encoder_n == ENCODER1)
	{
		EX2 = 1;    	//INT2 Enable P3.6
	}
	if(encoder_n == ENCODER2)
	{
		EX3 = 1;    	//INT3 Enable P3.7
	}
}

int32 encoder_get_count(encoder_index_enum encoder_n,uint8 reversal)
{
	if(encoder_n == ENCODER1)
	{
		encoder1 = cnt1;
		cnt1 = 0;
		return (reversal?(-encoder1):encoder1);
	}
	if(encoder_n == ENCODER2)
	{
		encoder2 = cnt2;
		cnt2 = 0;
		return (reversal?(-encoder2):encoder2);
	}
	return 0;
}