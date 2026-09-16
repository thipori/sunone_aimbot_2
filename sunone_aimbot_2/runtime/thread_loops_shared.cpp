#include "runtime/thread_loops.h"
#include "Game_overlay.h"

Game_overlay* gameOverlayPtr = nullptr;
std::thread gameOverlayThread;
std::atomic<bool> gameOverlayShouldExit(false);

std::mutex g_trackerDebugMutex;
std::vector<TrackDebugInfo> g_trackerDebugTracks;
int g_trackerLockedId = -1;

std::atomic<double> g_jitterVelocity(0.0);
std::atomic<double> g_jitterAmplitude(0.0);
std::atomic<double> g_jitterFrequency(0.0);
