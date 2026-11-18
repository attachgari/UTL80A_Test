/* UTL80A-0.6-fxy-drift. Fx/Fy Fz-hold drift regularization; signed Fz and Tz. */
#ifndef UTL80A_CALIBRATION_LAB_H
#define UTL80A_CALIBRATION_LAB_H
#include <stdint.h>
#define UTL80A_OUTPUT_COUNT 6
static const char *const UTL80A_OUTPUT_AXES[UTL80A_OUTPUT_COUNT] = {"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"};
static const float UTL80A_C[UTL80A_OUTPUT_COUNT][8] = {
    {2.6643069958e-04f, -1.0983346948e-03f, -2.5782258281e-03f, 2.1484567188e-03f, -3.3452570078e-04f, -9.0732583596e-04f, 1.7183919035e-03f, -1.1142281659e-03f},
    {1.1384514748e-03f, -8.4109859599e-04f, -1.2498139309e-03f, 7.1396786549e-04f, -2.6504024013e-03f, 2.0433837054e-03f, -1.4096458416e-04f, -1.2616985374e-04f},
    {-1.1748208418e-03f, 2.8839875516e-04f, -3.0546026151e-04f, 1.2468643759e-04f, -1.1865717183e-03f, 1.6543166460e-05f, 1.6472905554e-04f, -4.3530714602e-05f},
    {-7.1886891668e-05f, 1.0713488178e-04f, 9.4977541452e-06f, -9.3245412827e-06f, -5.9322632027e-05f, 2.6497679330e-05f, -3.9008543780e-06f, 6.6437842880e-06f},
    {-8.8081741575e-05f, 9.0484512721e-05f, -2.4877253512e-05f, 3.3028960671e-05f, -5.4532049280e-06f, -9.5248129850e-06f, -8.2229135529e-07f, -6.8640433103e-06f},
    {-9.6895682530e-05f, 1.5823820570e-04f, 4.7785739944e-05f, -3.6820564648e-05f, -7.5381129946e-05f, 8.2211484775e-05f, 1.5116498748e-05f, -2.7130915672e-05f}
};
static inline void utl80a_predict_delta(const float delta[8], float wrench[UTL80A_OUTPUT_COUNT]) {
    for (unsigned a = 0; a < UTL80A_OUTPUT_COUNT; ++a) {
        float value = 0.0f;
        for (unsigned ch = 0; ch < 8; ++ch) value += UTL80A_C[a][ch] * delta[ch];
        wrench[a] = value;
    }
}
/* tare_raw must be the local mean under the intended unloaded fixture condition. */
static inline void utl80a_predict(const uint32_t raw[8], const double tare_raw[8],
                                  float wrench[UTL80A_OUTPUT_COUNT]) {
    float delta[8];
    for (unsigned ch = 0; ch < 8; ++ch) delta[ch] = (float)((double)raw[ch] - tare_raw[ch]);
    utl80a_predict_delta(delta, wrench);
}
#endif
