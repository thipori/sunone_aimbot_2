#ifndef JITTER_AIM_H
#define JITTER_AIM_H

#include <cstdint>

// Jitter aim — keeps the camera rotating faster than Apex's recoil-smoothing
// threshold (~3.74 deg/s) with a small fast oscillation while firing, which
// zeroes vertical recoil. All output is mouse counts on the configured device
// (MAKCU), so there is no injected-input flag anywhere.
//
// Math (see features/jitter_aim.md):
//   deg_per_count = 0.022 * sens * ads_mult
//   v_required    = 3.74 * margin / deg_per_count          [counts/s]
//   square orbit / zigzag: |v| = 4 * A * f
//   circle:                |v| = 2*pi * A * f
// DPI is NOT part of the degrees math (the game only sees counts); it is kept
// for plausibility/telemetry only.

struct JitterAimSettings
{
    bool   enabled      = false;
    double sens         = 1.0;    // in-game sensitivity
    double ads_mult     = 1.0;    // optic multiplier while scoped
    double margin       = 2.0;    // safety factor over the threshold ("strength")
    double freq_hz      = 15.0;   // loops per second ("shake speed")
    int    shape        = 0;      // 0 = square orbit, 1 = circle, 2 = zigzag
    double noise_pct    = 40.0;   // humanization amount, 0..100
    bool   adaptive     = false;  // emit only the velocity deficit (advanced)
    double game_fps     = 144.0;  // per-frame quantization floor; <= 0 disables
};

// Shared formulas, used by both the module and the overlay so they cannot drift
// apart. speed = counts/s, size = excursion in counts, freq = loops/s.
double JitterAimShapeGain(int shape);
double JitterAimRequiredSpeed(const JitterAimSettings& settings);
double JitterAimSizeForFreq(const JitterAimSettings& settings, double freqHz);
double JitterAimFreqForSize(const JitterAimSettings& settings, double sizeCounts);

class JitterAim
{
public:
    // Emits integer counts for this iteration. aim_move_counts = magnitude of
    // the movement the aim path applied this iteration (counts), used by the
    // adaptive mode. Call with enabled=false (or reset()) to stop cleanly.
    void step(const JitterAimSettings& settings,
              double dt,
              double aim_move_counts,
              int& out_dx,
              int& out_dy);

    void reset();

    // Telemetry for the overlay.
    double lastVelocityCountsPerSec() const { return last_velocity_; }
    double lastAmplitude() const { return last_amplitude_; }
    double lastFrequencyHz() const { return last_frequency_; }
private:
    double rnd01();

    uint64_t rng_ = 0x9E3779B97F4A7C15ull;

    double step_accum_  = 0.0;  // time inside the current step
    int    step_idx_    = 0;    // square-orbit corner index (0 = down)
    int    zig_dir_     = 1;    // zigzag direction
    double phase_       = 0.0;  // circle phase
    double carry_x_     = 0.0;  // fractional count accumulator
    double carry_y_     = 0.0;

    double noise_timer_ = 0.0;
    double amp_noise_   = 1.0;
    double freq_drift_  = 1.0;
    double step_jitter_ = 1.0;
    int    loop_dir_    = 1;
    double onset_left_  = 0.0;
    double ramp_        = 0.0;
    bool   started_     = false;

    double last_velocity_  = 0.0;
    double last_amplitude_ = 0.0;
    double last_frequency_ = 0.0;
    double aim_rate_ema_   = 0.0;  // smoothed aim speed (counts/s) for adaptive mode
};

#endif // JITTER_AIM_H
