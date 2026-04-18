#pragma once
#include "control.hpp"
#include "motor_model.hpp"
#include <fstream>
#include <numbers>
#include <string>
#include <vector>

namespace sim
{

class CsvLogger
{
  public:
    explicit CsvLogger(const std::string& filename) : mFilename(filename)
    {
        mFile.open(mFilename);
    }

    ~CsvLogger()
    {
        if (mFile.is_open())
            mFile.close();
    }

    // High-level helper: This is what your SimRunner calls
    void logStep(float time_s,
                 const MotorModel& motor,
                 const app::Control& control,
                 float vu_V,
                 float vv_V,
                 float vw_V)
    {
        if (!mFile.is_open())
            return;

        MotorState simState = motor.getState();
        PhaseCurrents phaseCurrents = motor.getPhaseCurrents();
        float simSpeed_rpm = simState.mOmegaMech_rad_s * (30.0f / std::numbers::pi_v<float>);

        // --- STEP 1: Record everything by name ---
        // The order here determines the order in the CSV
        record("Time_s", time_s);

        // Phase Currents
        record("Iu_A", phaseCurrents.mIu_A);
        record("Iv_A", phaseCurrents.mIv_A);
        record("Iw_A", phaseCurrents.mIw_A);

        // FOC Internals
        record("foc.Mode", static_cast<float>(control.getMode()));
        record("foc.Enabled", static_cast<float>(control.getIsEnabled()));
        record("foc.Fault", static_cast<float>(control.getFaultStatus()));
        record("foc.OmegaRef_rpm", control.getOmegaRef_rpm());
        record("foc.IdRef_A", control.getIdRef());
        record("foc.IqRef_A", control.getIqRef());
        record("foc.Id_A", control.getId_A());
        record("foc.Iq_A", control.getIq_A());
        record("foc.PIId_Kp", control.getIdController().getKp());
        record("foc.PIId_Ki", control.getIdController().getKi());
        record("foc.Ud_V", control.getUd_V());
        record("foc.Uq_V", control.getUq_V());
        record("foc.OpenLoopTheta_rad", control.getOpenLoopTheta_rad());
        record("foc.EncoderTheta_rad", control.getEncoderTheta_rad());

        // Simulation Physics (The Ground Truth)
        record("sim.Id_A", simState.mId_A);
        record("sim.Iq_A", simState.mIq_A);
        record("sim.ThetaMech_rad", simState.mThetaMech_rad);
        record("sim.OmegaMech_rad_s", simState.mOmegaMech_rad_s);
        record("sim.Speed_rpm", simSpeed_rpm);

        // Inverter Voltages
        record("Vu_V", vu_V);
        record("Vv_V", vv_V);
        record("Vw_V", vw_V);

        // --- STEP 2: Finalize the row ---
        finishStep();
    }

  private:
    // Low-level: Adds a data point to the current row
    void record(const std::string& label, float value)
    {
        if (!mHeaderLocked)
        {
            mLabels.push_back(label);
        }
        mCurrentRow.push_back(value);
    }

    // Low-level: Writes the row to file and handles the header
    void finishStep()
    {
        if (!mHeaderLocked)
        {
            for (size_t i = 0; i < mLabels.size(); ++i)
            {
                mFile << mLabels[i] << (i == mLabels.size() - 1 ? "" : ",");
            }
            mFile << "\n";
            mHeaderLocked = true;
        }

        for (size_t i = 0; i < mCurrentRow.size(); ++i)
        {
            mFile << mCurrentRow[i] << (i == mCurrentRow.size() - 1 ? "" : ",");
        }
        mFile << "\n";

        mCurrentRow.clear();
    }

    std::string mFilename;
    std::ofstream mFile;
    bool mHeaderLocked{false};
    std::vector<std::string> mLabels;
    std::vector<float> mCurrentRow;
};

} // namespace sim