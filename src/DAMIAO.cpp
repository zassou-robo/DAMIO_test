#include <mbed.h>
#include "DAMIAO.hpp"

#include <cstring>
#include <cmath>
#include <algorithm>

// CAN共通インスタンス
CAN DM3520::can_(PB_12,PB_13,(int)1e6);

bool DM3520::bus_initialized_ = false;
DM3520* DM3520::instances_[16] = {};


// コンストラクタ
DM3520::DM3520(
    uint8_t id,
    Mode mode
)
    : DM3520(id, mode, Range{})
{
}

DM3520::DM3520(
    uint8_t id,
    Mode mode,
    Range range
)
    : id_(id),
      mode_(mode),
      range_(range)
{
    if (id_ < 16) {
        instances_[id_] = this;
    }
}

// モーター有効化
bool DM3520::EnableMotor()
{
    uint8_t data[8];

    for (int i = 0; i < 7; i++) {
        data[i] = 0xFF;
    }

    data[7] = 0xFC;

    return WriteCAN(
        id_,
        data,
        8
    );
}

// CAN初期化
bool DM3520::InitBus()
{
    if (bus_initialized_) {
        return true;
    }

    can_.frequency((int)1e6);

    bus_initialized_ = true;

    return true;
}


// CAN送信
bool DM3520::WriteCAN(
    uint16_t can_id,
    const uint8_t* data,
    uint8_t length
)
{
    if (!bus_initialized_ || length > 8) {
        return false;
    }

    CANMessage msg(
        can_id,
        reinterpret_cast<const char*>(data),
        length,
        CANData,
        CANStandard
    );

    return can_.write(msg);
}


// モーター初期化
bool DM3520::Init()
{
    if (!bus_initialized_) {
        return false;
    }

    // 先に制御モードを設定
    if (!SetMode()) {
        return false;
    }

    ThisThread::sleep_for(100ms);

    // 最後にモーターを有効化
    if (!EnableMotor()) {
        return false;
    }

    ThisThread::sleep_for(100ms);

    return true;
}


// モード設定
bool DM3520::SetMode()
{
    uint8_t data[8] = {};

    // 対象モーターID
    data[0] = id_ & 0xFF;
    data[1] = (id_ >> 8) & 0x07;

    // レジスタ書き込みコマンド
    data[2] = 0x55;

    // CTRL_MODE = 0x0A
    data[3] = 0x0A;

    // モード値（uint32_t little endian）
    uint32_t mode = static_cast<uint32_t>(mode_);

    data[4] = mode & 0xFF;
    data[5] = (mode >> 8) & 0xFF;
    data[6] = (mode >> 16) & 0xFF;
    data[7] = (mode >> 24) & 0xFF;

    return WriteCAN(0x7FF, data, 8);
}


// float → uint
uint32_t DM3520::FloatToUint(
    float value,
    float min,
    float max,
    uint8_t bits
)
{
    if (max <= min || bits == 0 || bits > 16) {
        return 0;
    }

    value = std::clamp(value, min, max);

    const uint32_t max_int =
        (1UL << bits) - 1;

    return static_cast<uint32_t>(
        (value - min) / (max - min) * max_int
    );
}


// uint → float
float DM3520::UintToFloat(
    uint32_t value,
    float min,
    float max,
    uint8_t bits
)
{
    const uint32_t max_int =
        (1UL << bits) - 1;

    return static_cast<float>(value) /
        max_int * (max - min) + min;
}


// 速度制御
bool DM3520::SendVelocity(float velocity)
{
    if (mode_ != Mode::VELOCITY) {
        return false;
    }

    uint8_t data[8] = {};

    memcpy(&data[0], &velocity, sizeof(float));

    return WriteCAN(
        0x200 + id_,
        data,
        8
    );
}


// 位置・速度制御
bool DM3520::SendPosition(
    float position,
    float max_velocity
)
{
    if (mode_ != Mode::POSITION_VELOCITY) {
        return false;
    }

    uint8_t data[8] = {};

    memcpy(&data[0], &position, sizeof(float));

    memcpy(
        &data[4],
        &max_velocity,
        sizeof(float)
    );

    return WriteCAN(
        0x100 + id_,
        data,
        8
    );
}


// MIT制御
bool DM3520::SendMIT(
    float position,
    float velocity,
    float kp,
    float kd,
    float torque
)
{
    if (mode_ != Mode::MIT) {
        return false;
    }

    uint16_t p = FloatToUint(
        position,
        -range_.position_max,
        range_.position_max,
        16
    );

    uint16_t v = FloatToUint(
        velocity,
        -range_.velocity_max,
        range_.velocity_max,
        12
    );

    uint16_t kp_int = FloatToUint(
        kp,
        0.0f,
        500.0f,
        12
    );

    uint16_t kd_int = FloatToUint(
        kd,
        0.0f,
        5.0f,
        12
    );

    uint16_t t = FloatToUint(
        torque,
        -range_.torque_max,
        range_.torque_max,
        12
    );

    uint8_t data[8];

    data[0] = (p >> 8) & 0xFF;
    data[1] = p & 0xFF;

    data[2] = (v >> 4) & 0xFF;

    data[3] =
        ((v & 0x0F) << 4) |
        ((kp_int >> 8) & 0x0F);

    data[4] = kp_int & 0xFF;

    data[5] = (kd_int >> 4) & 0xFF;

    data[6] =
        ((kd_int & 0x0F) << 4) |
        ((t >> 8) & 0x0F);

    data[7] = t & 0xFF;

    return WriteCAN(
        id_,
        data,
        8
    );
}


// トルク制御
bool DM3520::SendTorque(float torque)
{
    if (mode_ != Mode::MIT) {
        return false;
    }

    return SendMIT(
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        torque
    );
}


// フィードバック取得
const DM3520::Feedback& DM3520::GetFeedback() const
{
    return feedback_;
}


uint8_t DM3520::GetID() const
{
    return id_;
}


DM3520::Mode DM3520::GetMode() const
{
    return mode_;
}


// CAN受信処理
void DM3520::Receive()
{
    CANMessage msg;
    while (can_.read(msg)) {
    }
}