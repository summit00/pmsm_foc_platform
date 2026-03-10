#include <algorithm>
#include <cmath>
#include <tuple>

class SpaceVectorModulation
{
  public:
    std::tuple<float, float, float> applyModulation(float va, float vb, float vc)
    {
        float minV = std::min({va, vb, vc});
        float maxV = std::max({va, vb, vc});
        float vOffset = (minV + maxV) / 2.0f;

        float scale = 2.0f / std::sqrt(3.0f);

        float va_out = scale * (va - vOffset);
        float vb_out = scale * (vb - vOffset);
        float vc_out = scale * (vc - vOffset);

        return {va_out, vb_out, vc_out};
    }
};