#include "mbed.h"
#include "DAMIAO.hpp"

// モーター設定
DM3520::Range motor_range = {
    12.5f,  // Position Max
    30.0f,  // Velocity Max
    10.0f   // Torque Max
};

DM3520 motor(
    1,
    DM3520::Mode::VELOCITY,
    motor_range
);

int main()
{
    printf("===== DM3520 Auto Recovery Test =====\r\n");

    // CAN初期化
    if (!DM3520::InitBus()) {
        printf("CAN Init: Failed\r\n");
        while (true) {
            ThisThread::sleep_for(1000ms);
        }
    }
    printf("CAN Init: OK\r\n");

    // 初回のモード設定とEnable
    if (!motor.Init()) {
        printf("Motor Init: Failed\r\n");
        while (true) {
            ThisThread::sleep_for(1000ms);
        }
    }

    printf("Motor Init: OK\r\n");
    printf("Automatic recovery: ENABLED\r\n");
    printf("Velocity command: DISABLED\r\n");

    // フィードバックが500ms途絶えたら復旧処理を開始
    Timer feedback_timeout;
    Timer retry_interval;
    feedback_timeout.start();
    retry_interval.start();

    bool recovery_attempted = false;

    while (true) {
        // 受信できたフレームが対象モーターのフィードバックなら監視時間を更新
        if (DM3520::Receive()) {
            feedback_timeout.reset();
            recovery_attempted = false;
        }

        const bool feedback_lost =
            feedback_timeout.elapsed_time() >= 500ms;

        const bool retry_ready =
            !recovery_attempted || retry_interval.elapsed_time() >= 1000ms;

        if (feedback_lost && retry_ready) {
            printf("\r\n[RECOVERY] Feedback timeout\r\n");

            // CANリセット + 1Mbps再設定
            if (!DM3520::RecoverBus()) {
                printf("[RECOVERY] CAN recovery failed\r\n");
            } else {
                ThisThread::sleep_for(10ms);

                // Enableのみ送信。速度指令は送らない
                if (motor.EnableMotor()) {
                    printf("[RECOVERY] Enable: Send OK\r\n");
                } else {
                    printf("[RECOVERY] Enable: Send Failed\r\n");
                }
            }

            recovery_attempted = true;
            retry_interval.reset();
        }

        ThisThread::sleep_for(10ms);
    }
}
