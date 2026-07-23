#include "jy901.h"
#include "stdlib.h"
#include "stdio.h"
#include "usart.h"

void JY901_Get_Angle(JY901 *this)
{
    uint8_t chr;
    uint8_t raw_data[11];
    short roll_raw, pitch_raw, yaw_raw;
    float roll, pitch, yaw;

    if(HAL_UART_Receive(&huart3, &chr, 1, 0) == HAL_OK){
		if(chr == 0x55){
			raw_data[0] = 0x55;
		    if(HAL_UART_Receive(&huart3, &raw_data[1], 10, 100) == HAL_OK){
		    	if(raw_data[1] == 0x53 && raw_data[11] == (raw_data[0] + raw_data[1] + raw_data[2] + raw_data[3] + raw_data[4] + raw_data[5] + raw_data[6] + raw_data[7] + raw_data[8] + raw_data[9] + raw_data[10])){
		    	    short roll_raw  = (short)(raw_data[3] << 8 | raw_data[2]);
		            short pitch_raw = (short)(raw_data[5] << 8 | raw_data[4]);
		            short yaw_raw   = (short)(raw_data[7] << 8 | raw_data[6]);

		            float roll  = (float)roll_raw  / 32768.0f * 180.0f;
		            float pitch = (float)pitch_raw / 32768.0f * 180.0f;
		            float yaw   = (float)yaw_raw   / 32768.0f * 180.0f;

		            // printf("Roll:%.2f Pitch:%.2f Yaw:%.2f\r\n", roll, pitch, yaw);
                    this->roll = roll;
                    this->pitch = pitch;
                    this->yaw = yaw;
		        }
		    }
		}
	}
}