#ifndef OPENSIM_JAM_VISUALIZER_CLOSER_H_
#define OPENSIM_JAM_VISUALIZER_CLOSER_H_
/* -------------------------------------------------------------------------- *
 *                   OpenSim:  VisualizerCloser.h                             *
 * -------------------------------------------------------------------------- *
 * Closes the visualizer windows of finished simulations a fixed time after   *
 * each one is done (auto_close_visualizer of the COMAK tools).               *
 * -------------------------------------------------------------------------- */

#include <OpenSim/Common/Logger.h>
#include <OpenSim/Simulation/Model/Model.h>

#include <algorithm>
#include <chrono>
#include <set>
#include <thread>
#include <vector>


#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <tlhelp32.h>
#endif

namespace OpenSim {


#ifdef _WIN32
// Ids of the simbody-visualizer processes (the windows) started by this process
inline std::vector<unsigned long> visualizerChildProcesses() {
    std::vector<unsigned long> ids;
    HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snapshot == INVALID_HANDLE_VALUE) { return ids; }
    PROCESSENTRY32W entry;
    entry.dwSize = sizeof(entry);
    const DWORD me = GetCurrentProcessId();
    if (Process32FirstW(snapshot, &entry)) {
        do {
            if (entry.th32ParentProcessID == me &&
                    _wcsnicmp(entry.szExeFile, L"simbody-visualizer", 18) ==
                            0) {
                ids.push_back(entry.th32ProcessID);
            }
        } while (Process32NextW(snapshot, &entry));
    }
    CloseHandle(snapshot);
    return ids;
}

// Ends one of the visualizer processes of this process (checked again, so an
// id that was reused by another process is never ended)
inline void terminateVisualizerProcess(unsigned long id) {
    const std::vector<unsigned long> ids = visualizerChildProcesses();
    if (std::find(ids.begin(), ids.end(), id) == ids.end()) { return; }
    HANDLE process = OpenProcess(PROCESS_TERMINATE, FALSE, (DWORD)id);
    if (process != nullptr) {
        TerminateProcess(process, 0);
        CloseHandle(process);
    }
}
#else
inline std::vector<unsigned long> visualizerChildProcesses() { return {}; }
inline void terminateVisualizerProcess(unsigned long) {}
#endif

// Closes the visualizer windows of the finished settling and sweep simulations
// a fixed time after each one is done, without holding up the next one
// (auto_close_visualizer). A window is closed with Visualizer::shutdown() on a
// copy of the model's Visualizer (copies share the same window), so it does not
// matter that the Model replaces its own visualizer at the next initSystem().
// Windows that are still waiting when this object is destroyed (e.g. after an
// exception) are closed at once.
class VisualizerCloser {
public:
    using Clock = std::chrono::steady_clock;

    VisualizerCloser(bool enabled, double delay_seconds)
        : _enabled(enabled),
          _delay(std::chrono::duration_cast<Clock::duration>(
                  std::chrono::duration<double>(delay_seconds))) {}
    VisualizerCloser(const VisualizerCloser&) = delete;
    VisualizerCloser& operator=(const VisualizerCloser&) = delete;
    ~VisualizerCloser() { closeAll(); }

    // The simulation that used the model's current window is done: close it
    // after the delay.
    void closeLater(Model& model) {
        if (!_enabled || !model.hasVisualizer()) {
            log_debug("auto_close_visualizer: nothing to close (enabled {}, "
                    "model has a visualizer {}).", _enabled,
                    model.hasVisualizer());
            return;
        }
        // The window of this phase is the visualizer process that was not
        // seen at an earlier call (0 if it cannot be identified).
        unsigned long process_id = 0;
        for (unsigned long id : visualizerChildProcesses()) {
            if (_known_processes.insert(id).second) { process_id = id; }
        }
        _pending.push_back({model.updVisualizer().updSimbodyVisualizer(),
                Clock::now() + _delay, process_id});
        log_info("auto_close_visualizer: this window closes in {} s.",
                std::chrono::duration<double>(_delay).count());
    }

    // Close the windows whose delay has passed (call from the running loops).
    void closeDue() {
        if (_pending.empty()) { return; }
        const Clock::time_point now = Clock::now();
        for (auto it = _pending.begin(); it != _pending.end();) {
            if (it->due <= now) {
                close(*it);
                it = _pending.erase(it);
            } else {
                ++it;
            }
        }
    }

    // Wait until the delay of every waiting window has passed, then close it.
    void waitAndCloseAll() {
        for (const Pending& p : _pending) { std::this_thread::sleep_until(p.due); }
        closeAll();
    }

private:
    struct Pending {
        SimTK::Visualizer viz;
        Clock::time_point due;
        unsigned long process_id;
    };

    // Visualizer::shutdown() did not close the window with the visualizer
    // that is installed here, so the process of the window is ended as well
    // (on Windows, where it can be identified).
    void close(Pending& p) {
        log_info("auto_close_visualizer: closing a window.");
        p.viz.shutdown();
        if (p.process_id != 0) { terminateVisualizerProcess(p.process_id); }
    }

    void closeAll() {
        for (Pending& p : _pending) { close(p); }
        _pending.clear();
    }

    bool _enabled;
    std::set<unsigned long> _known_processes;
    Clock::duration _delay;
    std::vector<Pending> _pending;
};

// seconds a window stays open after its simulation is done (auto_close_visualizer)
inline constexpr double kAutoCloseVisualizerDelay = 10.0;

} // namespace OpenSim

#endif
