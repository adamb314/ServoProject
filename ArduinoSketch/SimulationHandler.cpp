#include "SimulationHandler.h"

SimulationHandler::SimulationHandler()
{
    Eigen::Matrix3f a;
    a << 1.0f, 0.0005963881634343555f, 1.6030177484362372e-05f,
        0.0f, 0.9879847514731415f, 0.05332648997363798f,
        0.0f, 0.0f, 1.0f;

    Eigen::Vector3f b;
    b << 1.6030177484362372e-05f,
        0.05332648997363798f,
        0.0f;

    a01 = a(0, 1) * fixedPoint * fixedPoint;
    a11 = a(1, 1) * fixedPoint;
    b0 = b[0] * fixedPoint * fixedPoint;
    b1 = b[1] * fixedPoint;

    pos = 0;
    vel = 0;
}

float SimulationHandler::getPosition()
{
    update();
    return pos * (1.0f / fixedPoint);
}

void SimulationHandler::setPwm(int16_t pwm)
{
    newCycle = true;

    this->pwm = pwm;
}

void SimulationHandler::update()
{
    if (!newCycle)
    {
        return;
    }
    newCycle = false;

    auto pwmToTorque = [&](int16_t in)
    {
        constexpr static int16_t maxPwm = 1023;

        if (in == 0)
        {
            return 0;
        }

        int16_t out = std::max(0, std::abs(in) - pwmOffset) * maxPwm / (maxPwm - pwmOffset);
        return out * adam_std::sign(in);
    };

    int32_t uSim = pwmToTorque(pwm);

    uSim = uSim + ((backEmfCurrent * vel) / fixedPoint * std::abs(uSim)) / (fixedPoint * fixedPoint);

    int32_t uSimFric = uSim - friction * adam_std::sign(vel);

    int32_t newVel = (a11 * vel) / fixedPoint + b1 * uSimFric;

    if ((uSim == 0 || (uSim < 0) != (uSimFric < 0)) &&
            (newVel < 0) != (vel < 0))
    {
        newVel = 0;
    }
    else
    {
        pos += ((a01 * vel) / fixedPoint + b0 * uSimFric) / fixedPoint;
    }

    vel = newVel;
}

constexpr std::array<uint16_t, 2048> SimulationHandler::aVec;
constexpr std::array<uint16_t, 2048> SimulationHandler::bVec;

OpticalEncoderSim::OpticalEncoderSim(SimulationHandler& simHandler,
            const std::array<uint16_t, vecSize>& aVec, const std::array<uint16_t, vecSize>& bVec,
            int16_t sensor1Pin, int16_t sensor2Pin, float unitsPerRev) :
    OpticalEncoderHandler(aVec, bVec, sensor1Pin, sensor2Pin, unitsPerRev),
    simHandler(simHandler),
    gearingInv(vecSize / unitsPerRev)
{
}

void OpticalEncoderSim::init()
{
}

void OpticalEncoderSim::triggerSample()
{
    newData = true;
}

float OpticalEncoderSim::getValue()
{
    if (newData)
    {
        newData = false;

        int32_t motorPos = static_cast<int32_t>(simHandler.getPosition() * gearingInv);
        motorPos = adam_std::wrapAround<vecSize>(motorPos);

        uint16_t sensor1Value = simHandler.aVec[motorPos];
        uint16_t sensor2Value = simHandler.bVec[motorPos];
        
        value = OpticalEncoderHandler::getValue(sensor1Value, sensor2Value);
    }

    return value;
}

ResistiveEncoderSim::ResistiveEncoderSim(SimulationHandler& simHandler, int16_t pin, float unitsPerRev,
        const std::array<int16_t, vecSize>& compVec) :
    ResistiveEncoderHandler(pin, unitsPerRev, compVec),
    simHandler(simHandler),
    scalingInv{4096.0f * 16 / unitsPerRev}
{
}

void ResistiveEncoderSim::init()
{
}

void ResistiveEncoderSim::triggerSample()
{
    newData = true;
}

float ResistiveEncoderSim::getValue()
{
    if (newData)
    {
        newData = false;
        uint16_t sensorValue = static_cast<uint16_t>(simHandler.getPosition() * scalingInv + (1 << 15)) / 16;

        value = ResistiveEncoderHandler::getValue(sensorValue);
    }
    return value;
}

PwmHandlerSim::PwmHandlerSim(SimulationHandler& simHandler, LinearizeFunctionType linearizeFunction) :
    simHandler(simHandler), linearizeFunction(linearizeFunction) {}

int PwmHandlerSim::setOutput(int output)
{
    output = std::min(std::max(output, -1023), 1023);
    if (output >= 0)
    {
        simHandler.setPwm(linearizeFunction(output));
    }
    else
    {
        simHandler.setPwm(-linearizeFunction(-output));
    }
    return output;
}

void PwmHandlerSim::activateBrake()
{
    simHandler.setPwm(0);
}
