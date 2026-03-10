#include <cmath>
#include <tuple>

class DQLimiter
{
  public:
    static std::tuple<float, float> applyLimit(float d_ref, float q_ref, float x_max)
    {
        // Calculate magnitude: mag_ref = sqrt((d_ref)^2 + (q_ref)^2)
        float mag_ref = std::sqrt(d_ref * d_ref + q_ref * q_ref);

        float d_sat;
        float q_sat;

        // When mag_ref > x_max: scale down proportionally
        if (mag_ref > x_max)
        {
            float scale = x_max / mag_ref;
            d_sat = d_ref * scale;
            q_sat = q_ref * scale;
        }
        // When mag_ref <= x_max: no saturation needed
        else
        {
            d_sat = d_ref;
            q_sat = q_ref;
        }

        return {d_sat, q_sat};
    }
};
