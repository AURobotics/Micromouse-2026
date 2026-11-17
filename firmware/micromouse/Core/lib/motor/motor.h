#pragma once
#include <algorithm>
#include <app_main.h>
#include <cfloat>
#include <main.h>


class Motor
{
public:
    enum class Decay_mode : uint8_t
    {
        slow,
        fast
    };
    class Encoder
    {
    public:
        explicit constexpr Encoder(TIM_HandleTypeDef* encoder_timer) :
            m_encoder_timer(encoder_timer)
        {
        }
        Encoder() = default;
        ~Encoder() = default;

        void init() const { HAL_TIM_Encoder_Start(m_encoder_timer, TIM_CHANNEL_ALL); }

        [[nodiscard]] int32_t delta()
        {
            uint32_t now = __HAL_TIM_GET_COUNTER(m_encoder_timer);
            int32_t delta = static_cast<int16_t>(static_cast<uint16_t>(now - m_last));
            m_last = now;
            return delta;
        }

        void reset() { __HAL_TIM_SET_COUNTER(m_encoder_timer, 0); }

    private:
        TIM_HandleTypeDef* m_encoder_timer{};
        uint32_t m_last{};
    };

    constexpr Motor(TIM_HandleTypeDef* htim, uint32_t channel_1, uint32_t channel_2,
                    TIM_HandleTypeDef* encoder_timer, Decay_mode decay_mode = Decay_mode::slow) :
        m_htim(htim),
        m_channel_1(channel_1),
        m_channel_2(channel_2),
        m_encoder(encoder_timer),
        m_decay_mode(decay_mode)
    {
    }

    constexpr Motor(TIM_HandleTypeDef* htim, uint32_t channel_1, uint32_t channel_2,
                Decay_mode decay_mode = Decay_mode::slow) :
        m_htim(htim),
        m_channel_1(channel_1),
        m_channel_2(channel_2),
        m_decay_mode(decay_mode)
    {
    }

    ~Motor() = default;

    void init()
    {
        HAL_TIM_PWM_Start(m_htim, m_channel_1);
        HAL_TIM_PWM_Start(m_htim, m_channel_2);
    }


    void move(float speed)
    {
        speed = std::clamp(speed, -100.0f, 100.0f);
        auto pwm = static_cast<uint32_t>(std::fabs(speed) * (MOTOR_PWM_MAX_CCR / 100.0f));

        uint32_t active_ch = (speed >= FLT_EPSILON) ? m_channel_1 : m_channel_2;
        uint32_t idle_ch = (speed >= FLT_EPSILON) ? m_channel_2 : m_channel_1;

        bool fast = (m_decay_mode == Decay_mode::fast);
        __HAL_TIM_SET_COMPARE(m_htim, active_ch, fast ? pwm : MOTOR_PWM_MAX_CCR - pwm);
        __HAL_TIM_SET_COMPARE(m_htim, idle_ch, fast ? 0 : MOTOR_PWM_MAX_CCR);
    }

    Encoder& encoder() { return m_encoder; }
    [[nodiscard]] const Encoder& encoder() const { return m_encoder; }

private:
    TIM_HandleTypeDef* m_htim{};
    uint32_t m_channel_1{};
    uint32_t m_channel_2{};
    Encoder m_encoder{};
    Decay_mode m_decay_mode{};
};
