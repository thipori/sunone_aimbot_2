#include "jitter_aim.h"

#include <algorithm>
#include <cmath>

namespace
{
constexpr double kPi                  = 3.14159265358979323846;
constexpr double kApexYawDegPerCount  = 0.022;  // deg per count at sens 1.0
constexpr double kThresholdDegPerSec  = 3.74;   // community-measured recoil-smoothing threshold
constexpr double kMaxAmplitudeCounts  = 64.0;   // plausibility cap
constexpr double kFreqMinHz           = 8.0;    // slowest loop we allow
constexpr double kFreqMaxHz           = 64.0;   // fastest loop we allow
constexpr double kMinCountsPerFrame   = 3.0;    // per-frame floor (see JitterAimRequiredSpeed)
constexpr double kAimRateTimeConstantSec = 0.15; // adaptive smoothing
constexpr double kRampSeconds         = 0.08;   // onset ramp (humanization)
constexpr double kNoisePeriodSeconds  = 0.10;   // noise refresh interval
constexpr double kOnsetDelayMinSec    = 0.010;  // 10 ms
constexpr double kOnsetDelayRangeSec  = 0.050;  // .. 60 ms

// Square orbit corner directions, in emission order: down -> right -> up -> left.
constexpr int kOrbitX[4] = {  0, +1,  0, -1 };
constexpr int kOrbitY[4] = { +1,  0, -1,  0 };
}

double JitterAimShapeGain(int shape)
{
    return (std::clamp(shape, 0, 2) == 1) ? (2.0 * kPi) : 4.0;   // circle vs square/zigzag
}

double JitterAimRequiredSpeed(const JitterAimSettings& s)
{
    const double deg_per_count =
        kApexYawDegPerCount * std::max(0.01, s.sens) * std::max(0.01, s.ads_mult);

    // Rate interpretation: 3.74 deg/s, community-measured (at 60 fps).
    double v = (kThresholdDegPerSec * std::max(1.0, s.margin)) / deg_per_count;

    if (s.game_fps > 1.0)
    {
        // Per-frame floor. 3.74 deg/s measured at 60 fps is 0.0623 deg/frame,
        // which is 2.83 counts/frame at sens 1.0. If the engine really checks a
        // per-frame delta instead of a rate, then the required rate grows with
        // fps and a sub-3-counts-per-frame jitter does nothing at all. Requiring
        // >= 3 counts/frame covers both readings and the count rounding.
        v = std::max(v, kMinCountsPerFrame * s.game_fps);
    }
    return v;
}

double JitterAimSizeForFreq(const JitterAimSettings& s, double freqHz)
{
    const double f = std::clamp(std::max(1.0, freqHz), kFreqMinHz, kFreqMaxHz);
    return JitterAimRequiredSpeed(s) / (JitterAimShapeGain(s.shape) * f);
}

double JitterAimFreqForSize(const JitterAimSettings& s, double sizeCounts)
{
    const double size = std::clamp(sizeCounts, 0.5, kMaxAmplitudeCounts);
    const double f = JitterAimRequiredSpeed(s) / (JitterAimShapeGain(s.shape) * size);
    return std::clamp(f, kFreqMinHz, kFreqMaxHz);
}

double JitterAim::rnd01()
{
    // xorshift64* — cheap, no allocation, good enough for humanization noise.
    uint64_t x = rng_;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    rng_ = x;
    const uint64_t r = x * 0x2545F4914F6CDD1Dull;
    return static_cast<double>(r >> 11) / 9007199254740992.0;
}

void JitterAim::reset()
{
    step_accum_  = 0.0;
    step_idx_    = 0;
    zig_dir_     = 1;
    phase_       = 0.0;
    carry_x_     = 0.0;
    carry_y_     = 0.0;
    noise_timer_ = 0.0;
    amp_noise_   = 1.0;
    freq_drift_  = 1.0;
    step_jitter_ = 1.0;
    loop_dir_    = 1;
    onset_left_  = 0.0;
    ramp_        = 0.0;
    started_     = false;
    last_velocity_  = 0.0;
    last_amplitude_ = 0.0;
    last_frequency_ = 0.0;
    aim_rate_ema_   = 0.0;
}

void JitterAim::step(const JitterAimSettings& s,
                     double dt,
                     double aim_move_counts,
                     int& out_dx,
                     int& out_dy)
{
    out_dx = 0;
    out_dy = 0;

    if (!s.enabled || dt <= 0.0)
    {
        reset();
        return;
    }

    // Random onset so firing does not instantly produce a machine-perfect pattern.
    if (!started_)
    {
        started_    = true;
        onset_left_ = kOnsetDelayMinSec + kOnsetDelayRangeSec * rnd01();
        ramp_       = 0.0;
    }
    if (onset_left_ > 0.0)
    {
        onset_left_ -= dt;
        return;
    }

    ramp_ = std::min(1.0, ramp_ + dt / kRampSeconds);

    // --- humanization noise (never allowed to break the velocity guarantee) ---
    const double noise = std::clamp(s.noise_pct, 0.0, 100.0) / 100.0;
    noise_timer_ += dt;
    if (noise_timer_ >= kNoisePeriodSeconds)
    {
        noise_timer_ = 0.0;
        amp_noise_   = 1.0 + noise * (rnd01() * 2.0 - 1.0) * 0.30;
        freq_drift_  = 1.0 + noise * (rnd01() * 2.0 - 1.0) * 0.15;
        step_jitter_ = 1.0 + noise * (rnd01() * 2.0 - 1.0) * 0.15;
        if (noise > 0.0 && rnd01() < 0.05 * noise)
        {
            loop_dir_ = -loop_dir_;
        }
    }

    // --- required velocity in counts/s ---
    const double v_required = JitterAimRequiredSpeed(s);

    // Adaptive (off by default): subtract the aim's own speed, smoothed. A
    // single 1-count correction in one millisecond is 1000 counts/s and would
    // otherwise switch the jitter off for that iteration and starve it.
    const double aim_rate = (dt > 0.0) ? (aim_move_counts / dt) : 0.0;
    const double ema_alpha = std::min(1.0, dt / kAimRateTimeConstantSec);
    aim_rate_ema_ += (aim_rate - aim_rate_ema_) * ema_alpha;

    const double v_have = s.adaptive ? aim_rate_ema_ : 0.0;
    const double v_def  = v_required - v_have;
    if (v_def <= 0.0)
    {
        // The aim itself is already fast enough to trigger recoil smoothing.
        carry_x_ = 0.0;
        carry_y_ = 0.0;
        last_velocity_  = 0.0;
        last_amplitude_ = 0.0;
        last_frequency_ = 0.0;
        return;
    }

    // --- frequency is the master knob ("shake speed"); the excursion ("shake
    //     width") falls out of it: A = v / (gain * f).
    const int shape = std::clamp(s.shape, 0, 2);
    const double gain = JitterAimShapeGain(shape);

    double f = std::clamp(std::max(1.0, s.freq_hz) * freq_drift_, kFreqMinHz, kFreqMaxHz);

    double amplitude = v_def / (gain * f);
    amplitude *= amp_noise_ * ramp_;
    amplitude = std::clamp(amplitude, 0.0, kMaxAmplitudeCounts);

    // --- oscillator: emit velocity * dt, kept as fractional counts ---
    double vx = 0.0;
    double vy = 0.0;
    double v_counts = 0.0;

    if (shape == 0) // square orbit: down -> right -> up -> left (net zero per cycle)
    {
        const double step_duration = (1.0 / (4.0 * f)) * step_jitter_;
        step_accum_ += dt;
        while (step_duration > 0.0 && step_accum_ >= step_duration)
        {
            step_accum_ -= step_duration;
            step_idx_ = (step_idx_ + loop_dir_ + 4) & 3;
        }
        v_counts = 4.0 * amplitude * f;
        vx = kOrbitX[step_idx_] * v_counts;
        vy = kOrbitY[step_idx_] * v_counts;
    }
    else if (shape == 2) // zigzag: back and forth on X
    {
        const double half_duration = (1.0 / (2.0 * f)) * step_jitter_;
        step_accum_ += dt;
        while (half_duration > 0.0 && step_accum_ >= half_duration)
        {
            step_accum_ -= half_duration;
            zig_dir_ = -zig_dir_;
        }
        v_counts = 4.0 * amplitude * f;
        vx = zig_dir_ * v_counts;
    }
    else // circle
    {
        phase_ += 2.0 * kPi * f * dt;
        v_counts = 2.0 * kPi * amplitude * f;
        vx = -v_counts * std::sin(phase_);
        vy =  v_counts * std::cos(phase_);
    }

    carry_x_ += vx * dt;
    carry_y_ += vy * dt;

    const int emit_x = static_cast<int>(std::trunc(carry_x_));
    const int emit_y = static_cast<int>(std::trunc(carry_y_));
    carry_x_ -= static_cast<double>(emit_x);
    carry_y_ -= static_cast<double>(emit_y);

    out_dx = emit_x;
    out_dy = emit_y;

    last_velocity_  = v_counts;
    last_amplitude_ = amplitude;
    last_frequency_ = f;
}
