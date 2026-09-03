// C ABI bridge for the SG100 SDK (V1.5_NO_TACTILE, ~/SG100_SDK_Publish).
//
// The integration uses the vendor's native RS485 ACTIVE mode end to end:
// one SG100Device owns the serial port, its CommScheduler drives the
// 10 ms realtime exchange, and control/status flow exclusively through the
// public C++ API:
//
//   connect            SetConfig + SetConfigSearchDir + Init +
//                      RegisterHandStatusCallback/RegisterErrorCallback +
//                      Start()            (scheduler thread starts)
//   set_hand_control   Hand_Control_t (id=i+1, caller mode, user DEG)
//                      -> SG100Device::SetHandControl
//                      (RS485: updates cached_control_; the scheduler
//                      encodes clamp+map+deg->rad and sends 0x05 at 100 Hz)
//   switch_to_realtime SG100Device::SwitchToRealtimeMode
//                      (0x25 -> 0xA5 ACK -> scheduler restart)
//   request_status     SG100Device::RequestHandStatus (0x26 query)
//   poll_status        latest callback snapshot (SDK user DEG) + sequence
//
// There is deliberately NO raw protocol, CRC, framing, mode-query, or
// transport code here: those were workarounds for the previous
// CANFD-passive architecture and are not part of the new backend.
//
// Public position semantics (docs/Realtime_Performance_Analysis.md 2.2,
// docs/SG100_SDK_User_Guide.md 6.2): every public position API and the
// status callback are in USER DEGREES; the SDK applies joint_angle_map.json
// (clamp + a*x + b) and converts to wire radians internally.

#include "sg100_device.h"
#include "sg100_types.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>

namespace {

std::mutex g_device_mutex;
SG100Device* g_device = nullptr;
bool g_started = false;

struct StatusSnapshot {
    double position_deg[JOINT_COUNT]{};
    double current[JOINT_COUNT]{};
    uint8_t mode[JOINT_COUNT]{};
    uint64_t received_monotonic_ns = 0;
    uint64_t sequence = 0;
    bool valid = false;
};

std::mutex g_status_mutex;
StatusSnapshot g_latest;
std::atomic<uint64_t> g_error_frames{0};
std::atomic<uint64_t> g_hand_status_callbacks{0};
std::atomic<uint64_t> g_hand_control_calls{0};
std::atomic<uint64_t> g_switch_to_realtime_calls{0};

uint64_t monotonic_ns_now() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

// Invoked on the SDK communication thread.  Copy the snapshot and return.
void on_hand_status(const Hand_Status_t& status) {
    std::lock_guard<std::mutex> lock(g_status_mutex);
    for (int i = 0; i < JOINT_COUNT; ++i) {
        const uint8_t id = status.joints[i].id;
        if (id < 1 || id > JOINT_COUNT) {
            return;  // malformed frame: drop the whole snapshot
        }
        g_latest.position_deg[id - 1] =
            static_cast<double>(status.joints[i].position);
        g_latest.current[id - 1] = static_cast<double>(
            SG100_Quant::decodeCurQ(status.joints[i].cur_q));
        g_latest.mode[id - 1] = status.joints[i].mode;
    }
    g_latest.received_monotonic_ns = monotonic_ns_now();
    ++g_latest.sequence;
    g_latest.valid = true;
    ++g_hand_status_callbacks;
}

void on_error(uint8_t, uint8_t) {
    ++g_error_frames;
}

// Build the whole-hand control frame exactly as the vendor examples do
// (teaching_mode_example.cpp): id=i+1, caller-selected mode, position in
// USER DEGREES, quantized PID/output gains shared by all joints.
Hand_Control_t build_hand_control(const float* positions_deg,
                                  uint8_t mode,
                                  uint16_t p, uint16_t i, uint16_t d,
                                  uint16_t output_limit) {
    Hand_Control_t control{};
    for (int j = 0; j < JOINT_COUNT; ++j) {
        control.joints[j].id = static_cast<uint8_t>(j + 1);
        control.joints[j].mode = mode;
        control.joints[j].position = positions_deg[j];
        control.joints[j].p = p;
        control.joints[j].i = i;
        control.joints[j].d = d;
        control.joints[j].output_limit = output_limit;
    }
    return control;
}

}  // namespace

extern "C" {

// -- lifecycle ---------------------------------------------------------------

// Open the vendor RS485 active transport.  config_dir is passed to
// SetConfigSearchDir so joint_angle_map.json resolves next to the SDK.
// Returns 0 on success, negative vendor/connect error otherwise.
int sg100_connect(const char* port, int baudrate, const char* config_dir) {
    std::lock_guard<std::mutex> lock(g_device_mutex);
    if (g_device || !port || baudrate <= 0) {
        return -1;
    }
    {
        std::lock_guard<std::mutex> status_lock(g_status_mutex);
        g_latest = StatusSnapshot{};
    }
    g_error_frames.store(0);
    g_hand_status_callbacks.store(0);
    g_hand_control_calls.store(0);
    g_switch_to_realtime_calls.store(0);

    g_device = new SG100Device("VPT_SG100");
    if (config_dir && config_dir[0] != '\0') {
        g_device->SetConfigSearchDir(std::string(config_dir));
    }
    SG100Config config;
    config.port = std::string(port);
    config.baudrate = static_cast<unsigned long>(baudrate);
    config.databyte = 8;
    config.parity = 0;
    config.stopbyte = 1;
    config.flowcontrol = 0;
    config.timeout_ms = 100;
    if (!g_device->SetConfig(config) || !g_device->Init()) {
        delete g_device;
        g_device = nullptr;
        return -2;
    }
    // A static C++ function is copied into the SDK's std::function; it has
    // no Python/GIL lifetime and stays valid until device destruction.
    g_device->RegisterHandStatusCallback(on_hand_status);
    g_device->RegisterErrorCallback(on_error);
    if (!g_device->Start()) {
        delete g_device;
        g_device = nullptr;
        return -3;
    }
    g_started = true;
    return 0;
}

int sg100_disconnect(void) {
    std::lock_guard<std::mutex> lock(g_device_mutex);
    if (!g_device) {
        return 0;
    }
    g_device->Stop();  // joins the scheduler thread, closes the adapter
    delete g_device;
    g_device = nullptr;
    g_started = false;
    return 0;
}

int sg100_is_connected(void) {
    std::lock_guard<std::mutex> lock(g_device_mutex);
    return g_device && g_device->IsRunning() && g_device->IsSerialOpen()
        ? 1
        : 0;
}

// -- feedback ----------------------------------------------------------------

// Copy the newest callback snapshot.  Returns its sequence number, or 0
// when no hand status has been received yet.  Sequence changes are the
// freshness signal: same value means the cache did not refresh.
uint64_t sg100_poll_status(double* position_deg_out, double* current_out,
                           uint8_t* mode_out, uint64_t* recv_ns_out) {
    std::lock_guard<std::mutex> lock(g_status_mutex);
    if (!g_latest.valid || !position_deg_out || !current_out ||
        !mode_out || !recv_ns_out) {
        return 0;
    }
    std::memcpy(position_deg_out, g_latest.position_deg,
                sizeof(g_latest.position_deg));
    std::memcpy(current_out, g_latest.current, sizeof(g_latest.current));
    std::memcpy(mode_out, g_latest.mode, sizeof(g_latest.mode));
    *recv_ns_out = g_latest.received_monotonic_ns;
    return g_latest.sequence;
}

// Vendor GetStatusRecvCount: increments on every palm status frame; a
// communication-health signal that does not depend on our snapshot.
uint32_t sg100_status_recv_count(void) {
    std::lock_guard<std::mutex> lock(g_device_mutex);
    return g_device ? g_device->GetStatusRecvCount() : 0;
}

// Queue an explicit whole-hand status query (0x26).  In RS485 active mode
// the scheduler delivers it; the response reaches the callback snapshot.
int sg100_request_hand_status(void) {
    std::lock_guard<std::mutex> lock(g_device_mutex);
    if (!g_device || !g_started) {
        return 0;
    }
    return g_device->RequestHandStatus() ? 1 : 0;
}

// -- control -----------------------------------------------------------------

// Whole-hand realtime control through the vendor production API.
// positions_deg: 11 USER-DEGREE values; mode: CtrlMode_e byte
// (1=MODE_POSITION, 7=MODE_JOINT_POSITION, 0=MODE_NULL); p/i/d and
// output_limit are pre-quantized u16 (SG100_Quant::encodePid/encodeOutLim).
// RS485 semantics: updates cached_control_ and returns; the 10 ms
// scheduler encodes and transmits.  Returns 1 when accepted.
int sg100_set_hand_control(const float* positions_deg, int count,
                           uint8_t mode,
                           uint16_t p, uint16_t i, uint16_t d,
                           uint16_t output_limit) {
    std::lock_guard<std::mutex> lock(g_device_mutex);
    if (!g_device || !g_started || !positions_deg ||
        count != JOINT_COUNT) {
        return 0;
    }
    for (int j = 0; j < JOINT_COUNT; ++j) {
        if (!std::isfinite(positions_deg[j])) {
            return 0;
        }
    }
    const Hand_Control_t control = build_hand_control(
        positions_deg, mode, p, i, d, output_limit);
    ++g_hand_control_calls;
    return g_device->SetHandControl(control) ? 1 : 0;
}

// Vendor realtime mode entry: stops the scheduler, flushes the passthrough
// queue, sends 0x25, waits for the 0xA5 ACK and the status flow to resume,
// then restarts the scheduler.  The scheduler restart re-transmits
// cached_control_ at 100 Hz, so callers must seed the cache (typically
// MODE_NULL) BEFORE switching — that closes the zero-target window.
int sg100_switch_to_realtime_mode(void) {
    std::lock_guard<std::mutex> lock(g_device_mutex);
    if (!g_device || !g_started) {
        return 0;
    }
    ++g_switch_to_realtime_calls;
    return g_device->SwitchToRealtimeMode() ? 1 : 0;
}

// Host work mode mirror (0=REALTIME, 1=PASSTHROUGH).  Diagnostics only:
// the vendor rewrites it from every 0x27 response, so it reflects the palm
// state, not a send capability.
int sg100_get_work_mode(void) {
    std::lock_guard<std::mutex> lock(g_device_mutex);
    if (!g_device) {
        return -1;
    }
    return static_cast<int>(g_device->GetWorkMode());
}

// -- diagnostics -------------------------------------------------------------

uint64_t sg100_error_frames(void) {
    return g_error_frames.load();
}
uint64_t sg100_hand_status_callbacks(void) {
    return g_hand_status_callbacks.load();
}
uint64_t sg100_hand_control_calls(void) {
    return g_hand_control_calls.load();
}
uint64_t sg100_switch_to_realtime_calls(void) {
    return g_switch_to_realtime_calls.load();
}

// -- offline ABI/construction verification (no device, no serial) ------------

// Sizes of the vendor structs this integration relies on.  The Python side
// asserts them against the values documented in sg100_types.h so a future
// SDK layout change fails loudly instead of corrupting frames.
int sg100_test_struct_sizes(int* out, int count) {
    static const std::array<int, 6> sizes = {
        static_cast<int>(sizeof(Joint_Control_t)),
        static_cast<int>(sizeof(Joint_Status_t)),
        static_cast<int>(sizeof(Hand_Control_t)),
        static_cast<int>(sizeof(Hand_Status_t)),
        static_cast<int>(sizeof(MotorDown_t)),
        static_cast<int>(sizeof(MotorUp_t)),
    };
    if (!out || count != static_cast<int>(sizes.size())) {
        return 0;
    }
    std::copy(sizes.begin(), sizes.end(), out);
    return 1;
}

// Serialize the exact Hand_Control_t this wrapper would hand to
// SetHandControl, so tests can verify id/mode/position/gain placement
// against the published struct layout without any hardware.
int sg100_test_serialize_hand_control(const float* positions_deg, int count,
                                      uint8_t mode,
                                      uint16_t p, uint16_t i, uint16_t d,
                                      uint16_t output_limit,
                                      uint8_t* out, int capacity) {
    if (!positions_deg || count != JOINT_COUNT || !out ||
        capacity < static_cast<int>(sizeof(Hand_Control_t))) {
        return 0;
    }
    const Hand_Control_t control = build_hand_control(
        positions_deg, mode, p, i, d, output_limit);
    std::memcpy(out, &control, sizeof(control));
    return static_cast<int>(sizeof(control));
}

}  // extern "C"
