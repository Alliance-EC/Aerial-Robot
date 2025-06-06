#include "tfminiplus.h"
#include "stdlib.h"
#include "bsp_dwt.h"

static TFMiniInstance *tfmini;

TFMiniInstance *TFMiniRegister(I2C_HandleTypeDef *hi2c)
{
	tfmini = (TFMiniInstance *)malloc(sizeof(TFMiniInstance));
	IIC_Init_Config_s conf = {
		.handle = hi2c,
		.dev_address = 0x10,
		.id = tfmini,
		.work_mode = IIC_BLOCK_MODE,
	};
	tfmini->iic = IICRegister(&conf);
	DWT_Delay(0.5);
	return tfmini;
}

float GetDistance(TFMiniInstance *p_tfmini)
{
	uint8_t cmd_buf[5] = {0x5A, 0x05, 0x00, 0x06, 0x65}; 
	IICTransmit(p_tfmini->iic, cmd_buf, 5, IIC_SEQ_RELEASE);
	IICReceive(p_tfmini->iic, p_tfmini->buf, 11, IIC_SEQ_RELEASE);
	p_tfmini->Mode = p_tfmini->buf[6];
	p_tfmini->Dist = (uint16_t)p_tfmini->buf[2] + (((uint16_t)p_tfmini->buf[3]) << 8);
	p_tfmini->Strength = p_tfmini->buf[4] | (p_tfmini->buf[5] << 8);
	return p_tfmini->Dist;
}
