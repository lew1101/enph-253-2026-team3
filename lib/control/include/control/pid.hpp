#pragma once

#include <Arduino.h>
#include <cfloat>

namespace control {
class PID {
  public:
    explicit PID(float kp, float ki, float kd)
        : _kp(kp)
        , _ki(ki)
        , _kd(kd)
    {
    }

    explicit PID(float kp, float ki, float kd, float integral_limit, float min_output, float max_output)
        : _kp(kp)
        , _ki(ki)
        , _kd(kd)
        , _integral_limit(integral_limit)
        , _min_output(min_output)
        , _max_output(max_output)
    {
    }

    virtual float update(float ref, float meas, float dt_s);
    inline void set_gain(float kp, float ki, float kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }
    virtual void reset();

  protected:
    float _kp;
    float _ki;
    float _kd;

    float _integral_limit = FLT_MAX;
    float _min_output = -FLT_MAX;
    float _max_output = FLT_MAX;

    float _integral = 0.0f;
    float _prev_err = 0.0f;
    bool _initialized = false;
};
} // namespace control
