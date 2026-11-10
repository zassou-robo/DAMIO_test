#include "mbed.h"
#include "DAMIAO.hpp"

DM3520::Range motor_range = {
    12.5f,
    30.0f,
    10.0f
};

DM3520 motor1(
    1,
    DM3520::Mode::VELOCITY,
    motor_range
);

int main()
{
    printf("Program start\r\n");

    printf(
        "CAN Init: %s\r\n",
        DM3520::InitBus() ? "OK" : "NG"
    );

    bool init_ok = motor1.Init();

    printf(
        "Motor Init: %s\r\n",
        init_ok ? "OK" : "NG"
    );

    if (!init_ok) {
        while (true) {
            ThisThread::sleep_for(1s);
        }
    }

    while (true) {

        // 受信処理
        DM3520::Receive();

        // 速度指令
        bool send_ok = motor1.SendVelocity(0.5f);

        const auto& fb = motor1.GetFeedback();

        printf(
            "Send:%d Pos:%.3f Vel:%.3f Torque:%.3f Error:%u Valid:%d\r\n",
            send_ok,
            fb.position,
            fb.velocity,
            fb.torque,
            fb.error,
            fb.valid
        );

        ThisThread::sleep_for(10ms);
    }
}