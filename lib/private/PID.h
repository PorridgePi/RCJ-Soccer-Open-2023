#include <Arduino.h>
class PID {
    public:
        PID(float kp, float ki, float kd, float timeStep) :
            _timeStep(timeStep), _kp(kp), _ki(ki), _kd(kd) {
        }

        float compute(float goal, float actual) {
            unsigned long now = micros();
            if (now - lastTime > _timeStep) {
                double dt = (now - lastTime) / 1000;
                lastTime  = now;

                double error = goal - actual; // Floats are not precise enough

                float output = 0;

                if (_kp) { // Proportional component
                    output += _kp * error;
                };

                if (_ki) { // Integral component
                    _integral += error * dt;
                    output += _ki * _integral;
                };

                if (_kd) { // Derivative component
                    output += _kd * (error - _lastError) / dt;
                    _lastError = error;
                };
                _lastOutput = output;
                return output; // constrain(output, -1, 1)

            } else
                return _lastOutput;
        } // MMMM I LOVE WRITING CODE ON MY PHONE

        void setConfig(float kp, float ki, float kd) {
            _kp = kp;
            _ki = ki;
            _kd = kd;
        };

    private:
        float         _kp;
        float         _ki;
        float         _kd;
        float         _integral;
        double        _lastError;
        unsigned long lastTime;
        float         _lastOutput;
        const float   _timeStep;
};