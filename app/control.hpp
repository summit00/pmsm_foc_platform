#pragma once
#include "foc.hpp"
#include "interfaces.hpp"
#include "motor_params.hpp"
#include "sensor.hpp"
#include "transform.hpp"
#include "user_interface.hpp"
#include <cstdint>

namespace app
{
class Control
{
  public:
    enum class Mode : uint8_t
    {
        OPENLOOP = 0,
        CLOSEDLOOP = 1,
        AUTOSETUP = 2,
    };

    explicit Control(ICurrentSense& current_sense,
                     IInverter& inverter,
                     IEnableOutput& gate_enable,
                     IEncoder& encoder,
                     MotorParams& motor_params,
                     UserInterface& ui)
        : current_sense(current_sense), inverter(inverter), gate_enable(gate_enable),
          encoder(encoder), motor_params(motor_params), foc(motor_params), ui(ui)
    {
    }

    void run_isr()
    {
        readUserCommands();

        PhaseCurrents currents = current_sense.read_amps();
        float ia_A = currents.ia_A;
        float ic_A = currents.ic_A;

        if (cmd_motor_enabled != motor_enabled)
        {
            motor_enabled = cmd_motor_enabled;
            gate_enable.set_enable(motor_enabled);

            if (!motor_enabled)
            {
                foc.resetFoc();
                return;
            }
        }

        sensor.runSensors(2000.0f, mOmegaRef_rad_Hz, motor_enabled);

        auto [Ialpha_A, IBeta_A] = transforms.clarke(ia_A, ic_A);
        std::tie(mId_A, mIq_A) = transforms.park(Ialpha_A, IBeta_A, sensor.getThetaOpenLoop());

        if (mMode == Mode::OPENLOOP)
        {
            mIdRef_A = mIsAbs_A;
            mIqRef_A = 0.0f;
            std::tie(mUd_V, mUq_V) = foc.runCurrentControl(mIdRef_A,
                                                           mIqRef_A,
                                                           mId_A,
                                                           mIq_A,
                                                           sensor.getOmegaOpenLoop_rad_Hz(),
                                                           -Udc_V,
                                                           Udc_V,
                                                           motor_enabled);
        }
        else if (mMode == Mode::CLOSEDLOOP)
        {
            std::tie(mIdRef_A, mIqRef_A) = foc.runSpeedControl(
                mOmegaRef_rad_Hz, sensor.getOmegaEncoder_rad_Hz(), -mIsAbs_A, mIsAbs_A);

            std::tie(mUd_V, mUq_V) = foc.runCurrentControl(mIdRef_A,
                                                           mIqRef_A,
                                                           mId_A,
                                                           mIq_A,
                                                           sensor.getOmegaEncoder_rad_Hz(),
                                                           -Udc_V,
                                                           Udc_V,
                                                           motor_enabled);
        }
        else if (mMode == Mode::AUTOSETUP)
        {
            // to do: implement auto setup routine.
        }

        auto [Ualpha_V, Ubeta_V] = transforms.inversePark(mUd_V, mUq_V, sensor.getThetaOpenLoop());
        auto [Va_V, Vb_V, Vc_V] = transforms.inverseClarke(Ualpha_V, Ubeta_V);

        inverter.set_phase_voltages(Va_V, Vb_V, Vc_V, Udc_V, motor_enabled);

        if (mTelemetryCounter++ >= 50)
        {
            mTelemetryCounter = 0;
            writeUserTelemetry();
        }
    }

    void readUserCommands()
    {
        cmd_motor_enabled = static_cast<bool>(ui.mEnable);
        mMode = static_cast<Mode>(ui.mMode);
        mOmegaRef_rad_Hz =
            sensor.mech_rpm_to_elec_rad_per_sec(ui.targetSpeed_rpm, motor_params.polePairs);
        mIsAbs_A = ui.mIsAbs_mA / 1000.0f;
        mAcceleration_rad_Hz2 =
            sensor.mech_rpm_to_elec_rad_per_sec(ui.mAcceleration_rpm_s, motor_params.polePairs);
    }

    void writeUserTelemetry()
    {
        ui.actualSpeed_rpm =
            sensor.getOmegaOpenLoop_rad_Hz() * 30.0f / (math::PI * motor_params.polePairs);
        ui.actualSpeedEncoder_rpm =
            sensor.getOmegaEncoder_rad_Hz() * 30.0f / (math::PI * motor_params.polePairs);
        ui.ThetaEncoder_deg = sensor.getThetaEncoder() * 180.0f / math::PI;
        ui.ThetaOpenLoop_deg = sensor.getThetaOpenLoop() * 180.0f / math::PI;
        ui.busVoltage_V = Udc_V;
        ui.Id_A = mId_A;
        ui.Iq_A = mIq_A;
        ui.IdRef_A = mIdRef_A;
        ui.IqRef_A = mIqRef_A;
    }

  private:
    ICurrentSense& current_sense;
    IInverter& inverter;
    IEnableOutput& gate_enable;
    IEncoder& encoder;
    MotorParams& motor_params;

    Sensor sensor{encoder, 4.0f};
    FOC foc;
    Transforms transforms;

    UserInterface& ui;

    float mOmegaRef_rad_Hz{0.0f};
    float mIdRef_A{0.0f};
    float mIqRef_A{0.0f};
    float mId_A{0.0f};
    float mIq_A{0.0f};
    float mUd_V{0.0f};
    float mUq_V{0.0f};
    float mIsAbs_A{0.0f};
    Mode mMode{0};

    float Udc_V = 24.0f;

    bool motor_enabled = false;
    bool cmd_motor_enabled = false;
    float mAcceleration_rad_Hz2{0.0f};
    uint8_t mTelemetryCounter = 0;
};

} // namespace app