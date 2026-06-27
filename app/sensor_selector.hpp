// sensor_selector.hpp
#pragma once
#include "sensor.hpp"

namespace app
{

class SensorSelector
{
  public:
    enum class SensorType
    {
        OpenLoop,
        Encoder,
        EmkObserver
    };

    SensorSelector(ISensor& openLoopSensor,
                   ISensor& encoderSensor,
                   ISensor& emkObserver,
                   float pwmPeriod_s)
        : mOpenLoopSensor(openLoopSensor), mEncoderSensor(encoderSensor), mEmkObserver(emkObserver),
          mActiveSensor(&openLoopSensor), mPwmPeriod_s(pwmPeriod_s)
    {
    }

    void selectSensor(SensorType type)
    {
        mSelectedType = type;
        switch (type)
        {
            case SensorType::OpenLoop:
                mActiveSensor = &mOpenLoopSensor;
                break;
            case SensorType::Encoder:
                mActiveSensor = &mEncoderSensor;
                break;
            case SensorType::EmkObserver:
                mActiveSensor = &mEmkObserver;
                break;
        }
    }

    void updateAllSensors()
    {
        mOpenLoopSensor.update();
        mEncoderSensor.update();
        mEmkObserver.update();
    }

    void updateActiveSensor()
    {
        if (mActiveSensor != nullptr)
        {
            mActiveSensor->update();
        }
    }

    float getActiveTheta_rad() const
    {
        return mActiveSensor ? mActiveSensor->getTheta_rad() : 0.0f;
    }

    float getActiveThetaPredicted_rad() const
    {
        return mActiveSensor
                   ? mActiveSensor->getTheta_rad() + 1.5f * mPwmPeriod_s * getActiveOmega_rad_Hz()
                   : 0.0f;
    }

    float getActiveOmega_rad_Hz() const
    {
        return mActiveSensor ? mActiveSensor->getOmega_rad_Hz() : 0.0f;
    }

    SensorType getSelectedType() const
    {
        return mSelectedType;
    }

  private:
    ISensor& mOpenLoopSensor;
    ISensor& mEncoderSensor;
    ISensor& mEmkObserver;

    ISensor* mActiveSensor{nullptr};
    SensorType mSelectedType{SensorType::OpenLoop};
    float mPwmPeriod_s{};
};

} // namespace app