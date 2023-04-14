#include "Serial_Send.h"


Serial_Send_Msg::Serial_Send_Msg() {

    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    sp.setPort(PORT);
    sp.setBaudrate(115200);
    sp.setTimeout(to);
}

bool Serial_Send_Msg::Serial_Open() {

    try {
        sp.open();
    }
    catch(serial::IOException &e) {
        
        return false;
    }
    
    if(sp.isOpen()) {

    }
    else {
        return false;
    }

    return true;
}

void Serial_Send_Msg::Serial_Close() {

    sp.close();
}

// void Serial_Send_Msg::Send_Speed(int8_t *speed) {

//     uint8_t Buff[7];
//     Buff[0] = HEAD_DATA;
//     Buff[1] = CHECK_DATA1;
//     Buff[2] = CHECK_DATA2;
//     Buff[3] = speed[0] + SPEED_ADD;
//     Buff[4] = speed[1] + SPEED_ADD;
//     Buff[5] = speed[2] + SPEED_ADD;
//     Buff[6] = speed[3] + SPEED_ADD;    

//     sp.write(Buff, 7);
// }

void Serial_Send_Msg::Send_Speed(int8_t *speed) {

    uint8_t Buff[7];
    Buff[0] = HEAD_DATA;
    Buff[1] = CHECK_DATA1;
    Buff[2] = CHECK_DATA2;
    Buff[3] = speed[0] + SPEED_ADD;
    Buff[4] = speed[1] + SPEED_ADD;
    Buff[5] = speed[2] + SPEED_ADD;
    Buff[6] = speed[3] + SPEED_ADD;    

    sp.write(Buff, 7);
}

