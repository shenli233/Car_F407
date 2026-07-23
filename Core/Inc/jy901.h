#include "main.h"

typedef struct JY901 {
    //返回的角度(欧拉角描述法)
    float yaw;       // 偏航角
    float roll;      // 翻滚角
    float pitch;     // 俯仰角
} JY901;

void JY901_Get_Angle(JY901 *this);