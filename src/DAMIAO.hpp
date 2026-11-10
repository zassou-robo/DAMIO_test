#ifndef DAMIAO_HPP
#define DAMIAO_HPP

#include "mbed.h"
#include <cstdint>

class DM3520 {
public:

    // 制御モード
    enum class Mode : uint8_t {
        MIT = 1,
        POSITION_VELOCITY = 2,
        VELOCITY = 3
    };

    // 各モーターのデータ変換範囲
    struct Range {
    float position_max;
    float velocity_max;
    float torque_max;

    Range(
        float p = 12.5f,
        float v = 30.0f,
        float t = 10.0f
    )
        : position_max(p),
          velocity_max(v),
          torque_max(t)
    {}
    };

    // フィードバック
    struct Feedback {
        float position = 0.0f;
        float velocity = 0.0f;
        float torque = 0.0f;

        uint8_t mos_temperature = 0;
        uint8_t motor_temperature = 0;
        uint8_t error = 0;

        bool valid = false;
    };

    // コンストラクタ
    // コンストラクタ
    DM3520(uint8_t id, Mode mode);
    DM3520(uint8_t id, Mode mode, Range range);

    // CAN初期化（全モーター共通）
    static bool InitBus();

    // モーター初期化・モード設定
    bool Init();

    // 制御
    bool SendVelocity(float velocity);

    bool SendPosition(
        float position,
        float max_velocity
    );

    bool SendMIT(
        float position,
        float velocity,
        float kp,
        float kd,
        float torque
    );

    // トルク指令（MITモード）
    bool SendTorque(float torque);

    // フィードバック
    const Feedback& GetFeedback() const;

    // CAN受信処理
    static void Receive();

    // モーター情報
    uint8_t GetID() const;
    Mode GetMode() const;

private:
    bool EnableMotor();

    static DM3520* instances_[16];
    uint8_t id_;
    Mode mode_;
    Range range_;

    Feedback feedback_;

    static CAN can_;
    static bool bus_initialized_;

    bool WriteCAN(
        uint16_t can_id,
        const uint8_t* data,
        uint8_t length
    );

    bool SetMode();

    static uint32_t FloatToUint(
        float value,
        float min,
        float max,
        uint8_t bits
    );

    static float UintToFloat(
        uint32_t value,
        float min,
        float max,
        uint8_t bits
    );

    bool ParseFeedback(const CANMessage& msg);
};

#endif //DAMIAO_HPP