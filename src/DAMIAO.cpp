
#include "DAMIAO.hpp"

#include <cstring>
#include <cmath>

// CAN共通インスタンス
// 現在使用しているCAN1のピン配置を維持
CAN DM3520::can_(PA_11, PA_12, 1000000);

bool DM3520::bus_initialized_ = false;

DM3520* DM3520::instances_[MAX_MOTORS] = {};


// コンストラクタ
DM3520::DM3520(
    uint8_t id,
    Mode mode,
    Range range
)
    : id_(id),
      master_id_(0),
      mode_(mode),
      range_(range)
{
    if (id_ < MAX_MOTORS) {
        instances_[id_] = this;
    }
}


// CAN初期化
bool DM3520::InitBus()
{
    if (bus_initialized_) {
        return true;
    }

    can_.frequency(1000000);

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


// モーター有効化
bool DM3520::EnableMotor()
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFC
    };

    return WriteCAN(id_, data, 8);
}


// モーター無効化
bool DM3520::DisableMotor()
{
    uint8_t data[8] = {
        0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF, 0xFF, 0xFD
    };

    return WriteCAN(id_, data, 8);
}


// モード設定
bool DM3520::SetMode()
{
    uint8_t data[8] = {};

    data[0] = id_ & 0xFF;
    data[1] = (id_ >> 8) & 0xFF;

    data[2] = 0x55;
    data[3] = 0x0A;

    uint32_t mode = static_cast<uint32_t>(mode_);

    data[4] = mode & 0xFF;
    data[5] = (mode >> 8) & 0xFF;
    data[6] = (mode >> 16) & 0xFF;
    data[7] = (mode >> 24) & 0xFF;

    return WriteCAN(0x7FF, data, 8);
}


// モーター初期化
bool DM3520::Init()
{
    if (!bus_initialized_) {
        return false;
    }

    // モード設定
    if (!SetMode()) {
        return false;
    }

    ThisThread::sleep_for(100ms);

    // モーター有効化
    if (!EnableMotor()) {
        return false;
    }

    ThisThread::sleep_for(100ms);

    return true;
}


// Master ID設定
void DM3520::SetMasterID(uint8_t master_id)
{
    master_id_ = master_id;
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

    if (value < min) value = min;
    if (value > max) value = max;

    const uint32_t max_int = (1UL << bits) - 1;

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
    const uint32_t max_int = (1UL << bits) - 1;

    return static_cast<float>(value) /
        max_int * (max - min) + min;
}


// 速度制御
bool DM3520::SendVelocity(float velocity)
{
    if (mode_ != Mode::VELOCITY) {
        return false;
    }

    uint8_t data[4] = {};

    std::memcpy(data, &velocity, sizeof(float));

    // 参考実装ではDLC = 4
    return WriteCAN(
        0x200 + id_,
        data,
        4
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

    std::memcpy(
        &data[0],
        &position,
        sizeof(float)
    );

    std::memcpy(
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
        kp, 0.0f, 500.0f, 12
    );

    uint16_t kd_int = FloatToUint(
        kd, 0.0f, 5.0f, 12
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

    return WriteCAN(id_, data, 8);
}


// トルク指令
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


// フィードバック解析
bool DM3520::ParseFeedback(const CANMessage& msg)
{
    if (msg.len < 8) {
        return false;
    }

    if (msg.id != master_id_) {
        return false;
    }

    // data[1]上位4bit: エラー状態
    feedback_.error =
        (msg.data[1] >> 4) & 0x0F;

    // 位置16bit
    uint16_t pos_int =
        (static_cast<uint16_t>(msg.data[2]) << 8) |
        msg.data[3];

    // 速度12bit
    uint16_t vel_int =
        (static_cast<uint16_t>(msg.data[4]) << 4) |
        ((msg.data[5] >> 4) & 0x0F);

    // トルク12bit
    uint16_t torque_int =
        (static_cast<uint16_t>(msg.data[5] & 0x0F) << 8) |
        msg.data[6];

    feedback_.position = UintToFloat(
        pos_int,
        -range_.position_max,
        range_.position_max,
        16
    );

    feedback_.velocity = UintToFloat(
        vel_int,
        -range_.velocity_max,
        range_.velocity_max,
        12
    );

    feedback_.torque = UintToFloat(
        torque_int,
        -range_.torque_max,
        range_.torque_max,
        12
    );

    // 温度
    feedback_.mos_temperature = msg.data[7];

    feedback_.valid = true;

    return true;
}


// CAN受信処理
void DM3520::Receive()
{
    CANMessage msg;

    // 受信バッファ内の全メッセージを処理
    while (can_.read(msg)) {

        if (msg.len < 8) {
            continue;
        }

        // 登録されたモーターを検索
        for (uint8_t i = 0; i < MAX_MOTORS; i++) {

            DM3520* motor = instances_[i];

            if (motor == nullptr) {
                continue;
            }

            if (motor->ParseFeedback(msg)) {
                break;
            }
        }
    }
}


// フィードバック取得
const DM3520::Feedback& DM3520::GetFeedback() const
{
    return feedback_;
}


// モーターID
uint8_t DM3520::GetID() const
{
    return id_;
}


// Master ID
uint8_t DM3520::GetMasterID() const
{
    return master_id_;
}


// モード
DM3520::Mode DM3520::GetMode() const
{
    return mode_;
}
