#include "AirDebug.h"

#if DebugInfo 
AirDebug Debug;
BluetoothSerial BT;


void AirDebug::SendInfo()
{
    outbuf[0] = 255;
    outbuf[1] = 255;
    outbuf[2] = 255;
    
    // Линиb
    outbuf [3] = 0;
    outbuf [4] = 0;
    for (int i = 0; i < 8; ++i){
        outbuf [3] |= line_sensor[i] << i;
        outbuf [4] |= line_sensor[i+8] << i;
    }
    
    outbuf [5] =  (line_angle & mask1) >> 8;
    outbuf [6] =  line_angle & mask2;

    outbuf [7] =  (ball_angle & mask1) >> 8;
    outbuf [8] =  ball_angle & mask2;

    outbuf [9] =  (gyroskope_angle & mask1) >> 8;
    outbuf [10] =  gyroskope_angle & mask2;

    outbuf [11] = speed_rotate;
    outbuf [12] = speed_x;
    outbuf [13] = speed_y;

    outbuf [14] = dribl_speed;

    //Байт состояний
    outbuf[15] = state;
    
    // Данные камеры
    outbuf [16] =  (((int16_t)camDataOmni.gates[0].left_angle) & mask1) >> 8;
    outbuf [17] =  ((int16_t)camDataOmni.gates[0].left_angle) & mask2;
    // Serial.print(outbuf [16]);
    // Serial.print(outbuf [17]);

    outbuf [18] =  (camDataOmni.gates[0].right_angle & mask1) >> 8;
    outbuf [19] =  camDataOmni.gates[0].right_angle & mask2;

    outbuf [20] =  (camDataOmni.gates[0].center_angle & mask1) >> 8;
    outbuf [21] =  camDataOmni.gates[0].center_angle & mask2;
    
    outbuf [22] =  (camDataOmni.gates[0].width & mask1) >> 8;
    outbuf [23] =  camDataOmni.gates[0].width & mask2;
    
    outbuf [24] =  (camDataOmni.gates[0].distance & mask1) >> 8;
    outbuf [25] =  camDataOmni.gates[0].distance & mask2;

    outbuf [26] =  (camDataOmni.gates[0].height & mask1) >> 8;
    outbuf [27] =  camDataOmni.gates[0].height & mask2;
    // голубые ворота
    outbuf [28] =  (camDataOmni.gates[1].left_angle & mask1) >> 8;
    outbuf [29] =  camDataOmni.gates[1].left_angle & mask2;

    outbuf [30] =  (camDataOmni.gates[1].right_angle & mask1) >> 8;
    outbuf [31] =  camDataOmni.gates[1].right_angle & mask2;

    outbuf [32] =  (camDataOmni.gates[1].center_angle & mask1) >> 8;
    outbuf [33] =  camDataOmni.gates[1].center_angle & mask2;
    
    outbuf [34] =  (camDataOmni.gates[1].width & mask1) >> 8;
    outbuf [35] =  camDataOmni.gates[1].width & mask2;
    
    outbuf [36] =  (camDataOmni.gates[1].distance & mask1) >> 8;
    outbuf [37] =  camDataOmni.gates[1].distance & mask2;

    outbuf [38] =  (camDataOmni.gates[1].height & mask1) >> 8;
    outbuf [39] =  camDataOmni.gates[1].height & mask2;

    BT.write(outbuf, 40);
    //Отсылка текста
    //Заглушка
    BT.write('\'');
}

#endif