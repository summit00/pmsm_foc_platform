#pragma once

// Core App
#include "control.hpp"
#include "motor_params.hpp"
#include "user_interface.hpp"

// Simulation Models & Hardware
#include "csv_logger.hpp"
#include "motor_model.hpp"
#include "motor_parameter_sets.hpp"
#include "sim_adc.hpp"
#include "sim_encoder.hpp"
#include "sim_gate_enable.hpp"
#include "sim_inverter.hpp"
#include "solver.hpp"

#include <string>

namespace sim
{

class SimRunner
{
  public:
    // Setup the simulation environment (e.g., 20kHz control, 10x physics oversampling)
    SimRunner(const SimMotorParams& simParams,
              const ILoadModel& externalLoad,
              const std::string& logFilename = "sim_output.csv",
              uint32_t physicsOversample_count = 10,
              float ctrlFreq_Hz = 20000.0f)
        : mCtrlFreq_Hz(20000.0f), mPhysicsDt_s(1.0f / (ctrlFreq_Hz * physicsOversample_count)),
          mStepsPerIsr_count(physicsOversample_count),

          // Initialize App Parameters from Sim Parameters
          mAppMotorParams{simParams.Rs_ohm,
                          simParams.Ld_H,
                          simParams.Lq_H,
                          simParams.fluxPm_Wb,
                          simParams.polePairs_count},

          // Initialize Physics
          mMotor(simParams, mSolver, externalLoad), mLogger(logFilename),

          // Initialize Mock Hardware
          mSimAdc(mMotor), mSimInverter(),
          mSimEncoder(mMotor, 2000, simParams.polePairs_count), // Assuming 2000 CPR = 8000 ticks
          mSimGateEnable(),

          // Initialize the actual Controller!
          mControl(mSimAdc, mSimInverter, mSimGateEnable, mSimEncoder, mAppMotorParams, mUi)
    {
    }

    // Run the simulation for a specific amount of simulated time
    void run(float duration_s, float loadTorque_Nm = 0.0f)
    {
        uint32_t totalPhysicsSteps = static_cast<uint32_t>(duration_s / mPhysicsDt_s);

        for (uint32_t step = 0; step < totalPhysicsSteps; ++step)
        {
            // 1. Advance Time (for logging)
            mCurrentTime_s += mPhysicsDt_s;

            mSimInverter.update_latched_voltages();

            // 2. Step the Physics Engine (using voltages sitting in the mock inverter)
            mMotor.stepSimulation(
                mPhysicsDt_s, mSimInverter.getVu(), mSimInverter.getVv(), mSimInverter.getVw());

            // 3. Run the Controller ISR (only on specific physics ticks)
            if (step % mStepsPerIsr_count == 0)
            {
                mControl.run_isr();
            }

            // 4. Log the state
            mLogger.logStep(mCurrentTime_s,
                            mMotor,
                            mControl,
                            mSimInverter.getVu(),
                            mSimInverter.getVv(),
                            mSimInverter.getVw());
        }
    }

    // Access to the UI so tests can send commands
    app::UserInterface& getUi()
    {
        return mUi;
    }

    // Access to the motor state so Catch2 can use REQUIRE() on final speeds/currents
    MotorState getFinalState() const
    {
        return mMotor.getState();
    }

  private:
    float mCtrlFreq_Hz;
    float mPhysicsDt_s;
    uint32_t mStepsPerIsr_count;
    float mCurrentTime_s{0.0f};

    // Physics
    // EulerSolver mSolver;
    RK4Solver mSolver;
    MotorModel mMotor;
    CsvLogger mLogger;

    // App Data
    app::MotorParams mAppMotorParams;
    app::UserInterface mUi;

    // Mock Hardware
    SimADC mSimAdc;
    SimInverter mSimInverter;
    SimEncoder mSimEncoder;
    SimGateEnable mSimGateEnable;

    // The System Under Test
    app::Control mControl;
};

} // namespace sim