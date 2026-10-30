#define _WINSOCKAPI_
#include <winsock2.h>
#include <Windows.h>

#include <algorithm>
#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "capture.h"
#include "mouse.h"
#include "config.h"
#include "detection_buffer.h"
#include "runtime/input_state.h"
#include "runtime/thread_loops.h"

#ifndef AIMBOT_TEST_NO_JITTER
#include "jitter_aim.h"
#endif

// Pause flag is now in detectionBuffer.isPaused() (upstream commit f9c65319).

namespace
{
constexpr int kPredictedOnlyMoveGraceFrames = 3;
constexpr double kPredictedOnlyMoveGraceSec =
    static_cast<double>(kPredictedOnlyMoveGraceFrames) / 60.0;
constexpr int kPredictedOnlyMoveStalePadMs = 16;

double trackerFrameIntervalSec(int captureFpsValue)
{
    const double fps = std::clamp(
        static_cast<double>((captureFpsValue > 0) ? captureFpsValue : 60),
        15.0,
        500.0);
    return 1.0 / fps;
}

bool allowPredictedOnlyMove(
    int activeTrackId,
    bool hasActiveTarget,
    const LockedTargetInfo& lockInfo,
    int captureFpsValue)
{
    if (activeTrackId != lockInfo.trackId ||
        !hasActiveTarget ||
        lockInfo.missedFrames <= 0)
    {
        return false;
    }

    const double frameDtSec = trackerFrameIntervalSec(captureFpsValue);
    const double missedSec = static_cast<double>(lockInfo.missedFrames) * frameDtSec;
    return missedSec <= kPredictedOnlyMoveGraceSec + frameDtSec * 0.51;
}

int trackerStaleTimeoutMs(int captureFpsValue)
{
    const int fps = std::max(1, captureFpsValue);
    const int frameBasedMs = 2000 / fps;
    const int graceBasedMs =
        static_cast<int>(kPredictedOnlyMoveGraceSec * 1000.0 + 0.5) +
        kPredictedOnlyMoveStalePadMs;
    return std::clamp(std::max(frameBasedMs, graceBasedMs), 25, 180);
}
}

void createInputDevices();
void assignInputDevices();
void handleEasyNoRecoil(MouseThread& mouseThread)
{
    bool easyNoRecoil = false;
    int recoil_compensation = 0;
    {
        std::lock_guard<std::mutex> cfgLock(configMutex);
        easyNoRecoil = config.easynorecoil;
        recoil_compensation = static_cast<int>(config.easynorecoilstrength);
    }

    if (easyNoRecoil && shooting.load() && zooming.load())
    {
        mouseThread.moveRelative(0, recoil_compensation);
    }
}

#ifndef AIMBOT_TEST_NO_JITTER
// Jitter aim — see features/jitter_aim.md. Oscillates the crosshair just enough
// to keep the camera above Apex's recoil-smoothing threshold (~3.74 deg/s)
// while firing. Only runs while the aim key is held, because MAKCU's injected
// frame replaces the physical delta for that report (stock firmware), so the
// user's own hand input would otherwise be overwritten during hip-fire.
void handleJitterAim(MouseThread& mouseThread)
{
    static JitterAim jitter;
    static std::chrono::steady_clock::time_point lastTick{};

    const auto now = std::chrono::steady_clock::now();
    double dt = 0.001;
    if (lastTick.time_since_epoch().count() != 0)
    {
        dt = std::chrono::duration<double>(now - lastTick).count();
    }
    lastTick = now;
    dt = std::clamp(dt, 0.0002, 0.010);

    JitterAimSettings settings;
    {
        std::lock_guard<std::mutex> cfgLock(configMutex);
        settings.enabled   = config.jitter_aim;
        settings.sens      = config.jitter_aim_sens;
        settings.ads_mult  = config.jitter_aim_ads_mult;
        settings.margin    = config.jitter_aim_margin;
        settings.freq_hz   = config.jitter_aim_freq;
        settings.shape     = config.jitter_aim_shape;
        settings.noise_pct = config.jitter_aim_noise;
        settings.adaptive  = config.jitter_aim_adaptive;
        settings.game_fps  = config.jitter_aim_game_fps;
    }

    if (!settings.enabled || detectionBuffer.isPaused() || !shooting.load() || !aiming.load())
    {
        jitter.reset();
        g_jitterVelocity.store(0.0);
        g_jitterAmplitude.store(0.0);
        g_jitterFrequency.store(0.0);
        return;
    }

    int dx = 0;
    int dy = 0;
    jitter.step(settings, dt, mouseThread.getLastAimMoveCounts(), dx, dy);
    if (dx != 0 || dy != 0)
    {
        // Quiet move: the injected shake must not be recorded as camera movement,
        // otherwise the aim compensates for its own jitter (feedback loop).
        mouseThread.moveRelativeQuiet(dx, dy);
    }

    g_jitterVelocity.store(jitter.lastVelocityCountsPerSec());
    g_jitterAmplitude.store(jitter.lastAmplitude());
    g_jitterFrequency.store(jitter.lastFrequencyHz());
}
#endif // !AIMBOT_TEST_NO_JITTER

void mouseThreadFunction(MouseThread& mouseThread)
{
    struct StopOnExit
    {
        MouseThread& mouse;
        ~StopOnExit() { mouse.stop(); }
    } stopOnExit{ mouseThread };

    int lastVersion = -1;
    std::vector<cv::Rect> boxes;
    std::vector<int> classes;
    std::chrono::steady_clock::time_point detectionTimestamp{};
    MultiTargetTracker targetTracker;
    std::optional<AimbotTarget> activeTarget;
    int activeTrackId = -1;
    bool activeTargetObserved = false;
    bool wasAiming = false;
    int appliedDetectionResolution = -1;
    bool appliedTrackerEnabled = true;
    std::uint64_t controlGeneration = detectionBuffer.currentGeneration();
    auto lastTrackerUpdate = std::chrono::steady_clock::time_point::min();

    auto resetActiveTarget = [&]() {
        activeTarget.reset();
        activeTrackId = -1;
        activeTargetObserved = false;
        mouseThread.clearFuturePositions();
        mouseThread.resetPrediction();
    };

    while (!shouldExit)
    {
        if (mouseThread.applyPendingConfig())
            resetActiveTarget();

        bool hasNewDetection = false;
        bool hasAimObservation = false;
        int detectionResolution = 0;
        bool disableHeadshot = false;
        bool trackerEnabled = true;
        bool trackerPredictedOnly = false;
        int predictionFuturePositions = 0;
        bool autoShoot = false;
        bool pausedNow = false;
        std::uint64_t observationGeneration = controlGeneration;

        {
            std::lock_guard<std::mutex> cfgLock(configMutex);
            detectionResolution = config.detection_resolution;
            disableHeadshot = config.disable_headshot;
            trackerEnabled = config.tracker_enabled;
            trackerPredictedOnly = config.tracker_predicted_only;
            predictionFuturePositions = config.prediction_futurePositions;
            autoShoot = config.auto_shoot;
        }

        const bool aimingNow = aiming.load();
        if (aimingNow != wasAiming)
        {
            resetActiveTarget();
            wasAiming = aimingNow;
        }

        {
            std::unique_lock<std::mutex> lock(detectionBuffer.mutex);
            detectionBuffer.cv.wait_for(lock, std::chrono::milliseconds(1), [&] {
                return detectionBuffer.version > lastVersion || shouldExit;
                }
            );

            if (shouldExit) break;

            pausedNow = detectionBuffer.paused;
            observationGeneration = detectionBuffer.generation;

            if (detectionBuffer.version > lastVersion)
            {
                boxes = detectionBuffer.boxes;
                classes = detectionBuffer.classes;
                detectionTimestamp = detectionBuffer.frameTimestamp;
                lastVersion = detectionBuffer.version;
                hasNewDetection = true;
            }
        }

        mouseThread.setDetectionGeneration(observationGeneration);
        if (observationGeneration != controlGeneration)
        {
            controlGeneration = observationGeneration;
            targetTracker.reset();
            resetActiveTarget();
            mouseThread.releaseMouse();
            std::lock_guard<std::mutex> lk(g_trackerDebugMutex);
            g_trackerDebugTracks.clear();
            g_trackerLockedId = -1;
        }

        if (input_method_changed.exchange(false))
        {
            createInputDevices();
            assignInputDevices();
        }

        if (detection_resolution_changed.load() || detectionResolution != appliedDetectionResolution)
        {
            {
                std::lock_guard<std::mutex> cfgLock(configMutex);
                appliedDetectionResolution = config.detection_resolution;
                mouseThread.updateConfig(config);
            }
            mouseThread.applyPendingConfig();
            targetTracker.reset();
            {
                std::lock_guard<std::mutex> lk(g_trackerDebugMutex);
                g_trackerDebugTracks.clear();
                g_trackerLockedId = -1;
            }
            resetActiveTarget();
        }

        if (pausedNow)
        {
            mouseThread.releaseMouse();
            continue;
        }

        if (trackerEnabled != appliedTrackerEnabled)
        {
            appliedTrackerEnabled = trackerEnabled;
            targetTracker.reset();
            {
                std::lock_guard<std::mutex> lk(g_trackerDebugMutex);
                g_trackerDebugTracks.clear();
                g_trackerLockedId = -1;
            }
            resetActiveTarget();
        }

        if (hasNewDetection)
        {
            if (trackerEnabled)
            {
                targetTracker.update(
                    boxes,
                    classes,
                    detectionResolution,
                    detectionResolution,
                    disableHeadshot,
                    aimingNow,
                    detectionTimestamp
                );
                lastTrackerUpdate = std::chrono::steady_clock::now();
                {
                    std::lock_guard<std::mutex> lk(g_trackerDebugMutex);
                    g_trackerDebugTracks = targetTracker.getDebugTracks();
                    g_trackerLockedId = targetTracker.getLockedTrackId();
                }

                LockedTargetInfo lockInfo;
                if (targetTracker.getLockedTarget(lockInfo))
                {
                    const int previousActiveTrackId = activeTrackId;
                    const bool hadActiveTarget = activeTarget.has_value();
                    if (activeTrackId != -1 && activeTrackId != lockInfo.trackId)
                    {
                        mouseThread.resetPrediction();
                        mouseThread.clearFuturePositions();
                    }

                    activeTarget = lockInfo.target;
                    activeTrackId = lockInfo.trackId;
                    activeTargetObserved = lockInfo.observedThisFrame;
                    mouseThread.setTargetDetected(true);

                    if (lockInfo.observedThisFrame)
                    {
                        hasAimObservation = true;
                        mouseThread.setLastTargetTime(std::chrono::steady_clock::now());

                        auto futurePositions = mouseThread.predictFuturePositions(
                            activeTarget->pivotX,
                            activeTarget->pivotY,
                            predictionFuturePositions
                        );
                        mouseThread.storeFuturePositions(futurePositions);
                    }
                    else if (trackerPredictedOnly && allowPredictedOnlyMove(
                        previousActiveTrackId,
                        hadActiveTarget,
                        lockInfo,
                        captureFps.load()))
                    {
                        hasAimObservation = true;

                        auto futurePositions = mouseThread.predictFuturePositions(
                            activeTarget->pivotX,
                            activeTarget->pivotY,
                            predictionFuturePositions
                        );
                        mouseThread.storeFuturePositions(futurePositions);
                    }
                }
                else
                {
                    resetActiveTarget();
                }
            }
            else
            {
                targetTracker.reset();
                {
                    std::lock_guard<std::mutex> lk(g_trackerDebugMutex);
                    g_trackerDebugTracks.clear();
                    g_trackerLockedId = -1;
                }

                std::unique_ptr<AimbotTarget> selected(
                    sortTargets(
                        boxes,
                        classes,
                        detectionResolution,
                        detectionResolution,
                        disableHeadshot));
                lastTrackerUpdate = std::chrono::steady_clock::now();

                if (selected)
                {
                    activeTarget = *selected;
                    activeTrackId = -1;
                    activeTargetObserved = true;
                    hasAimObservation = true;
                    mouseThread.setTargetDetected(true);
                    mouseThread.setLastTargetTime(std::chrono::steady_clock::now());

                    auto futurePositions = mouseThread.predictFuturePositions(
                        activeTarget->pivotX,
                        activeTarget->pivotY,
                        predictionFuturePositions
                    );
                    mouseThread.storeFuturePositions(futurePositions);
                }
                else
                {
                    resetActiveTarget();
                }
            }
        }

        if (activeTarget)
        {
            const int staleMs = trackerStaleTimeoutMs(captureFps.load());
            if (std::chrono::steady_clock::now() - lastTrackerUpdate > std::chrono::milliseconds(staleMs))
            {
                resetActiveTarget();
            }
        }

        if (aimingNow)
        {
            if (activeTarget && hasAimObservation)
            {
                mouseThread.moveMousePivot(activeTarget->pivotX, activeTarget->pivotY, detectionTimestamp);
            }
            else
            {
                if (!activeTarget || !activeTargetObserved)
                {
                    mouseThread.clearQueuedMoves();
                }
            }

            // Keep our press across polling timeouts while the observed target
            // remains valid. Disabling auto-shoot always releases our button.
            if (autoShoot && activeTarget && activeTargetObserved)
                mouseThread.pressMouse(*activeTarget);
            else
                mouseThread.releaseMouse();
        }
        else
        {
            mouseThread.clearQueuedMoves();
            mouseThread.releaseMouse();
        }

        handleEasyNoRecoil(mouseThread);
#ifndef AIMBOT_TEST_NO_JITTER
        handleJitterAim(mouseThread);
#endif

        mouseThread.checkAndResetPredictions();
    }
}


