#include "application.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "led/gpio_led.h"
#include "wifi_board.h"

#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <driver/i2s_std.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <atomic>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
#include "aec_watch.h"
#endif

#if CONFIG_RESPEAKER_XVF3800_TUNE_UDP_PORT
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace {

constexpr char kTag[] = "XiaoReSpeakerXVF3800";
constexpr gpio_num_t kLedPin = GPIO_NUM_21;
constexpr gpio_num_t kBootButtonPin = GPIO_NUM_0;
constexpr gpio_num_t kI2cSdaPin = GPIO_NUM_5;
constexpr gpio_num_t kI2cSclPin = GPIO_NUM_6;
constexpr gpio_num_t kI2sBclkPin = GPIO_NUM_8;
constexpr gpio_num_t kI2sWsPin = GPIO_NUM_7;
constexpr gpio_num_t kI2sDoutPin = GPIO_NUM_44;
constexpr gpio_num_t kI2sDinPin = GPIO_NUM_43;
constexpr uint8_t kXvf3800I2cAddress = 0x2C;
// Seeed's XIAO I2C example uses the Wire default of 100 kHz; the XMOS INT-device
// documentation lists 100 kbps as the control-interface maximum.
constexpr uint32_t kXvf3800I2cSpeedHz = 100000;

// RMS (16-bit units) of the most recent I2S input chunk, for the barge-in monitor.
std::atomic<uint32_t> g_input_rms{0};
// Extra output gain (Q16, 65536 = unity) set by the barge-in monitor while it ducks a reply.
std::atomic<int32_t> g_output_duck_q16{65536};
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
// The input since the barge-in monitor's previous poll. g_input_rms is one 10 ms chunk,
// and a poll comes about every 72 ms, so on its own it catches a keystroke or the gap
// between two syllables by chance.
struct InputShape {
    uint32_t chunks = 0;
    uint32_t samples = 0;
    uint32_t loud_chunks = 0;  // chunks with an RMS of at least BARGE_IN_MIN_RMS
    uint32_t max_chunk_rms = 0;
    uint32_t peak = 0;       // largest |sample|
    uint32_t crossings = 0;  // sign changes
    uint64_t square_sum = 0;
    uint64_t diff_square_sum = 0;  // sum of (x[n] - x[n-1])^2: high frequencies weigh more
};
InputShape g_input_shape;
// The latest input samples, for the monitor's voicing check (16 chunks of 10 ms).
constexpr int kInputRingSamples = 2560;
int16_t g_input_ring[kInputRingSamples];
int g_input_ring_pos = 0;  // where the next sample goes
portMUX_TYPE g_input_shape_lock = portMUX_INITIALIZER_UNLOCKED;  // guards the shape and the ring
#endif
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
// What arrived on both channels and what went to the XMOS since the monitor last looked (about a
// second), guarded by g_input_shape_lock.
struct AecWatchCounts {
    uint32_t rx_frames = 0;
    uint32_t tx_frames = 0;
    uint64_t left_square = 0;
    uint64_t right_square = 0;
    uint64_t tx_square = 0;
};
AecWatchCounts g_aec_watch_counts;
#endif
#if CONFIG_RESPEAKER_I2S_EXTERNAL_CLOCK_48K
constexpr int kAudioSampleRate = 48000;
constexpr i2s_role_t kI2sRole = I2S_ROLE_SLAVE;
constexpr char kI2sRoleName[] = "slave (external clocks)";
#else
constexpr int kAudioSampleRate = 16000;
constexpr i2s_role_t kI2sRole = I2S_ROLE_MASTER;
constexpr char kI2sRoleName[] = "master";
#endif

#if CONFIG_RESPEAKER_I2S_CLOCK_PROBE_ONLY
[[noreturn]] void ProbeExternalI2sClock() {
    gpio_config_t config = {};
    config.pin_bit_mask = (1ULL << kI2sBclkPin) | (1ULL << kI2sWsPin);
    config.mode = GPIO_MODE_INPUT;
    config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    config.intr_type = GPIO_INTR_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&config));
    ESP_LOGW(kTag, "CLOCK PROBE ONLY: GPIO8 BCLK and GPIO7 WS are inputs; I2S/chat disabled");
    ESP_LOGW(kTag, "Polling reports approximate WS frequency, not a precise clock measurement");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        int previous_ws = gpio_get_level(kI2sWsPin);
        int previous_bclk = gpio_get_level(kI2sBclkPin);
        uint32_t ws_rising_edges = 0;
        uint32_t bclk_edges = 0;
        const int64_t start = esp_timer_get_time();
        int64_t elapsed = 0;
        do {
            const int ws = gpio_get_level(kI2sWsPin);
            const int bclk = gpio_get_level(kI2sBclkPin);
            ws_rising_edges += ws != 0 && previous_ws == 0;
            bclk_edges += bclk != previous_bclk;
            previous_ws = ws;
            previous_bclk = bclk;
            elapsed = esp_timer_get_time() - start;
        } while (elapsed < 20000);

        ESP_LOGI(kTag,
                 "CLOCK PROBE: window_us=%lld ws_rises=%lu approx_ws_hz=%lu "
                 "sampled_bclk_edges=%lu WS=%d BCLK=%d",
                 static_cast<long long>(elapsed), static_cast<unsigned long>(ws_rising_edges),
                 static_cast<unsigned long>(ws_rising_edges * 1000000ULL / elapsed),
                 static_cast<unsigned long>(bclk_edges), previous_ws, previous_bclk);
    }
}
#endif

i2c_master_bus_handle_t Xvf3800I2cBus() {
    static i2c_master_bus_handle_t bus = nullptr;
    if (bus != nullptr) {
        return bus;
    }
    i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = kI2cSdaPin,
        .scl_io_num = kI2cSclPin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags =
            {
                .enable_internal_pullup = true,
            },
    };

    const esp_err_t err = i2c_new_master_bus(&config, &bus);
    if (err != ESP_OK) {
        ESP_LOGW(kTag, "Could not initialize I2C bus: %s", esp_err_to_name(err));
        bus = nullptr;
    }
    return bus;
}

void ProbeXvf3800() {
    i2c_master_bus_handle_t bus = Xvf3800I2cBus();
    if (bus == nullptr) {
        return;
    }

    const esp_err_t err = i2c_master_probe(bus, kXvf3800I2cAddress, pdMS_TO_TICKS(200));
    if (err == ESP_OK) {
        ESP_LOGI(kTag, "XVF3800 detected at I2C address 0x%02X", kXvf3800I2cAddress);
    } else {
        ESP_LOGW(kTag, "XVF3800 not detected at I2C address 0x%02X: %s", kXvf3800I2cAddress,
                 esp_err_to_name(err));
    }
}

#if CONFIG_RESPEAKER_XVF3800_BARGE_IN || CONFIG_RESPEAKER_XVF3800_CONTROL_DIAGNOSTICS
// XVF3800 control interface over I2C. Resource and command IDs follow the "Control
// Commands" appendix of the XMOS XVF3800 user guide (v3.2.1) and Seeed's I2C command
// list for the reSpeaker XVF3800; the wire format follows Seeed's XIAO example:
//   write: [resid][cmd][len][payload...]
//   read:  write [resid][cmd | 0x80][len + 1], then read len + 1 bytes; byte 0 is the
//          status (0 = success, 64 = servicer busy, retry), the rest is the payload.
// Multi-byte values are little-endian, like the ESP32-S3.
class Xvf3800Control {
public:
    static constexpr uint8_t kResPp = 17;        // post-processing servicer (PP_*)
    static constexpr uint8_t kResGpo = 20;       // GPO servicer (Seeed LED and DoA commands)
    static constexpr uint8_t kResAec = 33;       // AEC servicer (AEC_*)
    static constexpr uint8_t kResAudioMgr = 35;  // audio manager (AUDIO_MGR_*)
    static constexpr uint8_t kResApp = 48;       // application servicer (VERSION, ...)
    static constexpr uint8_t kCmdVersion = 0;         // kResApp: 3 x uint8 MAJOR MINOR PATCH
    static constexpr uint8_t kCmdReboot = 7;          // kResApp: uint8, write-only, any value
    static constexpr uint8_t kCmdAecPathChange = 0;   // kResAec: int32, AEC_AECPATHCHANGE
    static constexpr uint8_t kCmdAecConverged = 3;    // kResAec: int32
    static constexpr uint8_t kCmdSpeechEnergy = 80;   // kResAec: 4 x float, AEC_SPENERGY_VALUES
    static constexpr uint8_t kCmdAgcGain = 13;        // kResPp: float, PP_AGCGAIN
    static constexpr uint8_t kStatusSuccess = 0;
    static constexpr uint8_t kStatusRetry = 64;

    bool Init() {
        if (device_ != nullptr) {
            return true;
        }
        i2c_master_bus_handle_t bus = Xvf3800I2cBus();
        if (bus == nullptr) {
            return false;
        }
        i2c_device_config_t config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = kXvf3800I2cAddress,
            .scl_speed_hz = kXvf3800I2cSpeedHz,
            .scl_wait_us = 0,
            .flags =
                {
                    .disable_ack_check = 0,
                },
        };
        const esp_err_t err = i2c_master_bus_add_device(bus, &config, &device_);
        if (err != ESP_OK) {
            ESP_LOGW(kTag, "XVF3800 control: could not add I2C device: %s", esp_err_to_name(err));
            device_ = nullptr;
            return false;
        }
        return true;
    }

    // Reads `length` payload bytes of command `cmd` on resource `resid` into `out`.
    bool Read(uint8_t resid, uint8_t cmd, void* out, size_t length) {
        if (device_ == nullptr || length == 0 || length > kMaxPayload) {
            return false;
        }
        const uint8_t header[3] = {resid, static_cast<uint8_t>(cmd | 0x80),
                                   static_cast<uint8_t>(length + 1)};
        uint8_t buffer[kMaxPayload + 1];
        for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
            esp_err_t err = i2c_master_transmit(device_, header, sizeof(header), kTimeoutMs);
            if (err == ESP_OK) {
                err = i2c_master_receive(device_, buffer, length + 1, kTimeoutMs);
            }
            if (err != ESP_OK) {
                LogError(resid, cmd, "I2C %s", esp_err_to_name(err));
                return false;
            }
            if (buffer[0] == kStatusSuccess) {
                memcpy(out, buffer + 1, length);
                return true;
            }
            if (buffer[0] != kStatusRetry) {
                LogError(resid, cmd, "status %u", static_cast<unsigned>(buffer[0]));
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(kRetryDelayMs));
        }
        LogError(resid, cmd, "still busy after %d attempts", kMaxAttempts);
        return false;
    }

    // Writes `length` payload bytes to command `cmd` on resource `resid`. The XMOS does
    // not acknowledge writes in-band; callers read the value back to confirm.
    bool Write(uint8_t resid, uint8_t cmd, const void* payload, size_t length) {
        if (device_ == nullptr || length == 0 || length > kMaxPayload) {
            return false;
        }
        uint8_t buffer[3 + kMaxPayload] = {resid, cmd, static_cast<uint8_t>(length)};
        memcpy(buffer + 3, payload, length);
        const esp_err_t err = i2c_master_transmit(device_, buffer, 3 + length, kTimeoutMs);
        if (err != ESP_OK) {
            ESP_LOGW(kTag, "XVF3800 control write %u/%u failed: %s", static_cast<unsigned>(resid),
                     static_cast<unsigned>(cmd), esp_err_to_name(err));
            return false;
        }
        return true;
    }

private:
    static constexpr size_t kMaxPayload = 48;
    // Servicers answer asynchronously: a read may return status 64 until the DSP thread
    // has processed the command (Seeed's xvf_host.py allows 100 attempts 10 ms apart).
    static constexpr int kMaxAttempts = 20;
    static constexpr int kRetryDelayMs = 5;
    static constexpr int kTimeoutMs = 50;

    template <typename... Args>
    void LogError(uint8_t resid, uint8_t cmd, const char* format, Args... args) {
        const int64_t now = esp_timer_get_time();
        if (now - last_error_us_ < 5000000) {
            return;
        }
        last_error_us_ = now;
        char detail[64];
        snprintf(detail, sizeof(detail), format, args...);
        ESP_LOGW(kTag, "XVF3800 control read %u/%u failed: %s", static_cast<unsigned>(resid),
                 static_cast<unsigned>(cmd), detail);
    }

    i2c_master_dev_handle_t device_ = nullptr;
    int64_t last_error_us_ = -5000000;
};

// Logs the firmware version and the tuning parameters that decide what the ESP32-S3
// receives (output routing, gains, AEC and echo-suppression settings).
// Returns the firmware version packed as 0xMMmmpp, or 0 if the control interface
// does not answer.
uint32_t LogXvf3800Settings(Xvf3800Control& control) {
    uint8_t version[3] = {};
    if (!control.Read(Xvf3800Control::kResApp, Xvf3800Control::kCmdVersion, version,
                      sizeof(version))) {
        ESP_LOGW(kTag, "XVF3800 VERSION read failed; control interface unavailable");
        return 0;
    }
    ESP_LOGI(kTag, "XVF3800 firmware v%u.%u.%u", version[0], version[1], version[2]);

    struct Setting {
        const char* name;
        uint8_t resid;
        uint8_t cmd;
        char type;  // 'f' float, 'i' int32, 'u' uint8
        uint8_t count;
    };
    static const Setting kSettings[] = {
        {"AUDIO_MGR_OP_L", 35, 15, 'u', 2},     {"AUDIO_MGR_OP_R", 35, 19, 'u', 2},
        {"AEC_ASROUTONOFF", 33, 35, 'i', 1},    {"AEC_ASROUTGAIN", 33, 36, 'f', 1},
        {"AUDIO_MGR_MIC_GAIN", 35, 0, 'f', 1},  {"AUDIO_MGR_REF_GAIN", 35, 1, 'f', 1},
        {"AUDIO_MGR_SYS_DELAY", 35, 26, 'i', 1}, {"AEC_HPFONOFF", 33, 1, 'i', 1},
        {"AEC_NUM_FARENDS", 33, 72, 'i', 1},    {"AEC_AECCONVERGED", 33, 3, 'i', 1},
        {"PP_AGCONOFF", 17, 10, 'i', 1},        {"PP_AGCMAXGAIN", 17, 11, 'f', 1},
        {"PP_AGCDESIREDLEVEL", 17, 12, 'f', 1}, {"PP_AGCGAIN", 17, 13, 'f', 1},
        {"PP_LIMITONOFF", 17, 19, 'i', 1},      {"PP_LIMITPLIMIT", 17, 20, 'f', 1},
        {"PP_MIN_NS", 17, 21, 'f', 1},          {"PP_MIN_NN", 17, 22, 'f', 1},
        {"PP_ECHOONOFF", 17, 23, 'i', 1},       {"PP_GAMMA_E", 17, 24, 'f', 1},
        {"PP_GAMMA_ETAIL", 17, 25, 'f', 1},     {"PP_GAMMA_ENL", 17, 26, 'f', 1},
        {"PP_NLATTENONOFF", 17, 27, 'i', 1},    {"PP_MGSCALE", 17, 29, 'f', 3},
        {"PP_FMIN_SPEINDEX", 17, 30, 'f', 1},   {"PP_DTSENSITIVE", 17, 31, 'i', 1},
        {"PP_ATTNS_MODE", 17, 32, 'i', 1},
    };
    for (const Setting& setting : kSettings) {
        union {
            float f[3];
            int32_t i[3];
            uint8_t u[12];
        } value = {};
        const size_t length = setting.type == 'u' ? setting.count : setting.count * 4U;
        if (!control.Read(setting.resid, setting.cmd, &value, length)) {
            ESP_LOGW(kTag, "XVF3800 %s read failed", setting.name);
            continue;
        }
        char text[48] = {};
        if (setting.type == 'f' && setting.count == 3) {
            // PP_MGSCALE: max, min and the value in use
            snprintf(text, sizeof(text), "%g, %g, %g", static_cast<double>(value.f[0]),
                     static_cast<double>(value.f[1]), static_cast<double>(value.f[2]));
        } else if (setting.type == 'f') {
            snprintf(text, sizeof(text), "%g", static_cast<double>(value.f[0]));
        } else if (setting.type == 'i') {
            snprintf(text, sizeof(text), "%d", static_cast<int>(value.i[0]));
        } else {
            snprintf(text, sizeof(text), "%u, %u", static_cast<unsigned>(value.u[0]),
                     static_cast<unsigned>(value.u[1]));
        }
        ESP_LOGI(kTag, "XVF3800 %s = %s", setting.name, text);
    }
    return (static_cast<uint32_t>(version[0]) << 16) | (static_cast<uint32_t>(version[1]) << 8) |
           version[2];
}

// Polls the XVF3800 while the device is speaking. Near-end speech reported by the DSP
// (which already subtracted what it is playing) ends the reply on the device: the
// local playback queue is dropped and an abort is sent to the server, so interruption
// does not depend on residual echo reaching the server's gate.
class Xvf3800Monitor {
public:
    void Start() {
        xTaskCreatePinnedToCore(
            [](void* arg) {
                static_cast<Xvf3800Monitor*>(arg)->Run();
                vTaskDelete(nullptr);
            },
            "xvf3800_ctl", 4096, this, 3, nullptr, 1);
    }

private:
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
    static constexpr int64_t kGraceUs = CONFIG_RESPEAKER_XVF3800_BARGE_IN_GRACE_MS * 1000LL;
    // Sliding window of polls: trigger once kHoldPolls of the last kWindowPolls saw speech.
    static constexpr int kWindowPolls = std::max(
        1, CONFIG_RESPEAKER_XVF3800_BARGE_IN_WINDOW_MS / CONFIG_RESPEAKER_XVF3800_CONTROL_POLL_MS);
    static constexpr int kHoldPolls = std::max(
        1, std::min(kWindowPolls, (CONFIG_RESPEAKER_XVF3800_BARGE_IN_HOLD_MS +
                                   CONFIG_RESPEAKER_XVF3800_CONTROL_POLL_MS - 1) /
                                      CONFIG_RESPEAKER_XVF3800_CONTROL_POLL_MS));
    static_assert(kWindowPolls <= 64, "window must fit in the 64-bit history");
    // Fast path: a few loud polls in the same window. XMOS post-processing chops a
    // near-end talker into short bursts during playback, which can starve the window.
    // Its own, shorter window: a knock is one loud poll, a word is several in a row.
    static constexpr int kStrongWindowPolls = std::max(
        1, std::min(64, (CONFIG_RESPEAKER_XVF3800_BARGE_IN_STRONG_WINDOW_MS > 0
                             ? CONFIG_RESPEAKER_XVF3800_BARGE_IN_STRONG_WINDOW_MS
                             : CONFIG_RESPEAKER_XVF3800_BARGE_IN_WINDOW_MS) /
                            CONFIG_RESPEAKER_XVF3800_CONTROL_POLL_MS));
    static constexpr int kStrongPolls =
        std::max(1, std::min(kStrongWindowPolls, CONFIG_RESPEAKER_XVF3800_BARGE_IN_STRONG_POLLS));
    // Voicing: XMOS reports speech energy on keystrokes as well, but a vowel repeats at
    // its pitch period and a keystroke dies away within one or two. Largest r(lag) / r(0)
    // over 32 ms frames, lags of 2..12.5 ms (500..80 Hz), as is and scaled for the shorter
    // overlap at longer lags.
    static constexpr int kVoicedPolls = CONFIG_RESPEAKER_XVF3800_BARGE_IN_VOICED_POLLS;
    static_assert(kVoicedPolls <= kWindowPolls,
                  "VOICED_POLLS counts polls of the speech window (WINDOW_MS / CONTROL_POLL_MS)");
    static constexpr int kVoicingFrame = 512;
    static constexpr int kVoicingHop = 256;
    static constexpr int kVoicingMinLag = 32;
    static constexpr int kVoicingMaxLag = 200;
    static constexpr int kVoicingLeadSamples = 320;  // two chunks before the poll's own
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
    // Duck instead of interrupting: a backchannel ("en", "dui dui") is as loud as the first
    // word of a command and only shorter, so the fast path lowers the volume and the reply is
    // interrupted once the burst outlasts a backchannel.
    static constexpr int32_t kDuckQ16 = CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK_PERCENT * 65536 / 100;
    static constexpr int kDuckConfirmPolls = CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK_CONFIRM_POLLS;
    static constexpr int64_t kDuckReleaseUs = CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK_RELEASE_MS * 1000LL;
#endif
#if CONFIG_RESPEAKER_XVF3800_ECHO_GUARD
    // Echo guard. A reply that carries near-full-scale input on two polls within its
    // first 800 ms is the device hearing itself: on 11-02 one overload during playback
    // left the XMOS echo canceller diverged, every later reply started at 19k..25k RMS,
    // the server took the echo for the user and cut each reply within a second, so the
    // canceller never got the playback it needs to reconverge, and ESP resets do not
    // reset the XMOS. After a few such replies in a row the XMOS is rebooted (it comes
    // back with fresh filters and defaults, so the overrides are written again).
    static constexpr int64_t kEchoGuardWindowUs = 800000;
    static constexpr int kEchoGuardPolls = 2;
    static constexpr int64_t kEchoGuardCooldownUs = 60000000;
#endif
#if CONFIG_RESPEAKER_XVF3800_CONTROL_DIAGNOSTICS
    static constexpr bool kDiagnostics = true;
#else
    static constexpr bool kDiagnostics = false;
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_TRACE
    static constexpr bool kTrace = true;
#else
    static constexpr bool kTrace = false;
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_LOUDEST_CHUNK
    static constexpr bool kLoudestChunk = true;
#else
    static constexpr bool kLoudestChunk = false;
#endif

    void Run() {
        // Give the XMOS time to boot its own firmware before the first control read.
        vTaskDelay(pdMS_TO_TICKS(3000));
        if (!control_.Init()) {
            ESP_LOGW(kTag, "XVF3800 control unavailable; device-side barge-in disabled");
            return;
        }
        const uint32_t version = LogXvf3800Settings(control_);
        if (version == 0) {
            return;
        }
        // Seeed I2S firmware: DOA_VALUE (angle, speech flag) exists from v1.0.6 as
        // command 19 and moved to command 18 in v1.0.8.
        doa_cmd_ = version >= 0x010008 ? 18 : (version >= 0x010006 ? 19 : 0);
        ApplyOverrides();
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
        const float threshold = strtof(CONFIG_RESPEAKER_XVF3800_BARGE_IN_THRESHOLD, nullptr);
        ESP_LOGI(kTag,
                 "XVF3800 barge-in: speech energy > %g with input rms >= %d, or input rms > %d, in %d of "
                 "the last %d polls (%d ms), or input rms > %d in %d of the last %d polls (0 = off), "
                 "%d ms grace",
                 static_cast<double>(threshold), CONFIG_RESPEAKER_XVF3800_BARGE_IN_MIN_RMS,
                 CONFIG_RESPEAKER_XVF3800_BARGE_IN_INPUT_RMS, kHoldPolls, kWindowPolls,
                 CONFIG_RESPEAKER_XVF3800_CONTROL_POLL_MS, CONFIG_RESPEAKER_XVF3800_BARGE_IN_STRONG_RMS,
                 kStrongPolls, kStrongWindowPolls, CONFIG_RESPEAKER_XVF3800_BARGE_IN_GRACE_MS);
        const float voicing_threshold = strtof(CONFIG_RESPEAKER_XVF3800_BARGE_IN_VOICING, nullptr);
        const float scaled_threshold = strtof(CONFIG_RESPEAKER_XVF3800_BARGE_IN_VOICING_UNBIASED, nullptr);
        ESP_LOGI(kTag,
                 "XVF3800 barge-in voicing: interrupt only with %d voiced polls (periodicity >= %g, or >= %g "
                 "scaled for the overlap, 0 = off) among the last %d (0 = off)",
                 kVoicedPolls, static_cast<double>(voicing_threshold), static_cast<double>(scaled_threshold),
                 kWindowPolls);
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
        ESP_LOGI(kTag,
                 "XVF3800 barge-in duck: the fast path (or %d speech polls of its last %d, 0 = off) ducks the "
                 "reply to %d%%; interrupt after %d speech polls in the ducked burst (0 = speech window only), "
                 "volume back after %d ms without speech; speech votes check the %s chunk",
                 duck_speech_polls_, kStrongWindowPolls, CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK_PERCENT,
                 kDuckConfirmPolls, CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK_RELEASE_MS,
                 kLoudestChunk ? "loudest" : "latest");
#endif
#if CONFIG_RESPEAKER_XVF3800_ECHO_GUARD
        ESP_LOGI(kTag,
                 "XVF3800 echo guard: reboot the XMOS after %d replies in a row with input rms > %d "
                 "on %d polls in their first %lld ms",
                 CONFIG_RESPEAKER_XVF3800_ECHO_GUARD_REPLIES, CONFIG_RESPEAKER_XVF3800_ECHO_GUARD_RMS,
                 kEchoGuardPolls, static_cast<long long>(kEchoGuardWindowUs / 1000));
#endif

#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
        ESP_LOGI(kTag,
                 "XVF3800 aec watch: reboot the XMOS when right / playback stays >= %g over %d s of playback, "
                 "at most %d times a day, then ask for a power cycle",
                 static_cast<double>(strtof(CONFIG_RESPEAKER_XVF3800_AEC_WATCH_RATIO, nullptr)),
                 CONFIG_RESPEAKER_XVF3800_AEC_WATCH_WINDOWS, CONFIG_RESPEAKER_XVF3800_AEC_WATCH_MAX_REBOOTS);
#endif

        auto& app = Application::GetInstance();
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(CONFIG_RESPEAKER_XVF3800_CONTROL_POLL_MS));
#if CONFIG_RESPEAKER_XVF3800_TUNE_UDP_PORT
            PollTuneCommand();
#endif
            const int64_t now = esp_timer_get_time();
            const bool speaking = app.GetDeviceState() == kDeviceStateSpeaking;
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
            if (AecWatchTick(now, speaking)) {
                continue;  // the XMOS was rebooted, which took seconds: start over with fresh values
            }
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
            portENTER_CRITICAL(&g_input_shape_lock);
            const InputShape shape = g_input_shape;
            g_input_shape = InputShape{};
            recent_count_ = 0;
            if (speaking) {
                // The samples since the previous poll and two chunks before them, oldest first.
                recent_count_ = static_cast<int>(
                    std::min<uint32_t>(shape.samples + kVoicingLeadSamples, kInputRingSamples));
                int start = g_input_ring_pos - recent_count_;
                start += start < 0 ? kInputRingSamples : 0;
                const int first = std::min(recent_count_, kInputRingSamples - start);
                memcpy(recent_, g_input_ring + start, first * sizeof(int16_t));
                memcpy(recent_ + first, g_input_ring, (recent_count_ - first) * sizeof(int16_t));
            }
            portEXIT_CRITICAL(&g_input_shape_lock);
#endif
            if (!speaking) {
#if CONFIG_RESPEAKER_XVF3800_ECHO_GUARD
                if (speaking_since_us_ != 0) {
                    EchoGuardReplyEnded();  // a reply just ended (or was aborted)
                }
#endif
                speaking_since_us_ = 0;
                history_ = 0;
                strong_history_ = 0;
                voiced_history_ = 0;
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
                duck_history_ = 0;
                // GPT-Live's replies arrive as segments split at 0.7 s of silence, so a segment
                // can end in the middle of a ducked burst; the burst then carries over a short gap
                // and the next segment starts ducked. After an interruption, or once the burst
                // is over, the next reply starts at full volume.
                if (!armed_ || now - last_speech_us_ >= kDuckReleaseUs) {
                    SetDucked(false);
                    burst_polls_ = 0;
                    last_speech_us_ = 0;
                }
#endif
                armed_ = true;
            } else if (speaking_since_us_ == 0) {
                speaking_since_us_ = now;
                history_ = 0;
                strong_history_ = 0;
                voiced_history_ = 0;
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
                duck_history_ = 0;
#endif
            }
            const bool report =
                kDiagnostics && now - last_report_us_ >= (speaking ? 200000 : 500000);
            if (!speaking && !report) {
                continue;
            }

            float energy[4] = {};
            if (!control_.Read(Xvf3800Control::kResAec, Xvf3800Control::kCmdSpeechEnergy, energy,
                               sizeof(energy))) {
                continue;
            }
            [[maybe_unused]] const float peak =
                std::max(std::max(energy[0], energy[1]), std::max(energy[2], energy[3]));
            const uint32_t input_rms = g_input_rms.load(std::memory_order_relaxed);

#if CONFIG_RESPEAKER_XVF3800_ECHO_GUARD
            if (speaking && now - speaking_since_us_ <= kEchoGuardWindowUs &&
                input_rms > static_cast<uint32_t>(CONFIG_RESPEAKER_XVF3800_ECHO_GUARD_RMS)) {
                ++reply_loud_polls_;
            }
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
            [[maybe_unused]] float voicing = -1.0f;  // not computed
            [[maybe_unused]] float voicing_scaled = -1.0f;
            [[maybe_unused]] int voicing_us = 0;
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
            // A burst carried over from the previous segment is still being spoken: no grace.
            const bool counting = now - speaking_since_us_ >= kGraceUs ||
                                  (last_speech_us_ != 0 && now - last_speech_us_ < kDuckReleaseUs);
#else
            const bool counting = now - speaking_since_us_ >= kGraceUs;
#endif
            if (speaking && armed_ && counting) {
                const uint32_t vote_rms = kLoudestChunk ? std::max(input_rms, shape.max_chunk_rms) : input_rms;
                const bool energy_vote =
                    peak > threshold &&
                    vote_rms >= static_cast<uint32_t>(CONFIG_RESPEAKER_XVF3800_BARGE_IN_MIN_RMS);
                const bool level_vote =
                    CONFIG_RESPEAKER_XVF3800_BARGE_IN_INPUT_RMS > 0 &&
                    input_rms > static_cast<uint32_t>(CONFIG_RESPEAKER_XVF3800_BARGE_IN_INPUT_RMS);
                const bool speech = energy_vote || level_vote;
                // A loud poll also needs the DSP's speech energy: typing next to the
                // board gave consecutive polls above 2000 RMS with all four energies 0,
                // while every recorded barge-in (and every knock) carried energy.
                const bool strong =
                    CONFIG_RESPEAKER_XVF3800_BARGE_IN_STRONG_RMS > 0 &&
                    input_rms > static_cast<uint32_t>(CONFIG_RESPEAKER_XVF3800_BARGE_IN_STRONG_RMS) &&
                    peak > threshold;
                // Typing next to the board also carries speech energy (11-03: polls of
                // 2000..20000 RMS that filled the window), so an interruption also needs
                // voiced polls among the window's speech polls.
                if (speech && (kVoicedPolls > 0 || kTrace)) {
                    const int64_t started = esp_timer_get_time();
                    Voicing(&voicing, &voicing_scaled);
                    voicing_us = static_cast<int>(esp_timer_get_time() - started);
                }
                // A low voice repeats at a long lag, where r(lag) / r(0) cannot reach the
                // threshold (below 0.75 for pitch under 125 Hz); the scaled score can.
                const bool voiced = speech && (voicing >= voicing_threshold ||
                                               (scaled_threshold > 0.0f && voicing_scaled >= scaled_threshold));
                history_ = (history_ << 1) | (speech ? 1ULL : 0ULL);
                strong_history_ = (strong_history_ << 1) | (strong ? 1ULL : 0ULL);
                voiced_history_ = (voiced_history_ << 1) | (voiced ? 1ULL : 0ULL);
                if (kWindowPolls < 64) {
                    history_ &= (1ULL << kWindowPolls) - 1;
                    voiced_history_ &= (1ULL << kWindowPolls) - 1;
                }
                if (kStrongWindowPolls < 64) {
                    strong_history_ &= (1ULL << kStrongWindowPolls) - 1;
                }
                const bool fast = __builtin_popcountll(strong_history_) >= kStrongPolls;
                const bool window = __builtin_popcountll(history_) >= kHoldPolls;
                const int voiced_polls = __builtin_popcountll(voiced_history_);
                const bool voiced_enough = kVoicedPolls == 0 || voiced_polls >= kVoicedPolls;
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
                // A burst is a run of speech polls with gaps shorter than the release time.
                if (speech) {
                    if (now - last_speech_us_ >= kDuckReleaseUs) {
                        burst_polls_ = 0;
                    }
                    ++burst_polls_;
                    last_speech_us_ = now;
                }
                // A soft talker never reaches STRONG_RMS during playback: a few speech polls within
                // the fast path's window duck too, so less echo reaches the XMOS for the rest of
                // the phrase.
                duck_history_ = (duck_history_ << 1) | (speech ? 1ULL : 0ULL);
                if (kStrongWindowPolls < 64) {
                    duck_history_ &= (1ULL << kStrongWindowPolls) - 1;
                }
                const bool speech_run = duck_speech_polls_ > 0 &&
                                        __builtin_popcountll(duck_history_) >= duck_speech_polls_;
                if ((fast || speech_run) && !ducked_) {
                    strong_history_ = 0;  // a second loud burst ducks again, it does not escalate
                    duck_history_ = 0;
                    duck_since_us_ = now;
                    SetDucked(true);
                    ESP_LOGI(kTag,
                             "XVF3800 %s during playback (input rms %u, speech poll %d of the burst); "
                             "ducking to %d%%",
                             fast ? "loud near-end" : "near-end speech polls", static_cast<unsigned>(input_rms),
                             burst_polls_, CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK_PERCENT);
                }
                const bool confirmed =
                    ducked_ && kDuckConfirmPolls > 0 && burst_polls_ >= kDuckConfirmPolls;
                if ((window || confirmed) && voiced_enough) {
                    history_ = 0;
                    strong_history_ = 0;
                    voiced_history_ = 0;
                    duck_history_ = 0;
                    BargeIn(energy, voiced_polls, input_rms, window ? "speech window" : "ducked burst");
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_LOG_ONLY
                    // The reply goes on: count the next burst from scratch, at full volume.
                    burst_polls_ = 0;
                    last_speech_us_ = 0;
                    SetDucked(false);
#else
                    armed_ = false;
#endif
                } else if (ducked_ && now - last_speech_us_ >= kDuckReleaseUs) {
                    SetDucked(false);
                    ESP_LOGI(kTag,
                             "XVF3800 near-end burst ended after %d speech polls; volume back after "
                             "%lld ms ducked",
                             burst_polls_, static_cast<long long>((now - duck_since_us_) / 1000));
                }
#else
                if ((fast || window) && voiced_enough) {
#if !CONFIG_RESPEAKER_XVF3800_BARGE_IN_LOG_ONLY
                    armed_ = false;
#endif
                    history_ = 0;
                    strong_history_ = 0;
                    voiced_history_ = 0;
                    BargeIn(energy, voiced_polls, input_rms, fast ? "loud polls" : "speech window");
                }
#endif
            }
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_TRACE
            if (speaking) {
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
                const char* duck_tag = ducked_ ? " duck" : "";
#else
                const char* duck_tag = "";
#endif
                // rms / peak: the latest chunk and the largest speech energy, as voted on.
                // The rest covers every chunk since the previous poll: count, RMS, loudest
                // chunk, chunks of at least MIN_RMS, largest |sample|, high-frequency weight
                // (difference energy over energy, percent), sign changes per 1000 samples and,
                // on speech polls, the voicing in percent (-1 = not computed), its CPU time and,
                // after the duck tag, the voicing scaled for the overlap.
                const uint64_t samples = std::max<uint32_t>(shape.samples, 1);
                ESP_LOGI(kTag,
                         "XVF3800 poll +%lld rms=%u peak=%.3g n=%u avg=%u max=%u loud=%u pk=%u hf=%u zc=%u voi=%d "
                         "vus=%d%s vu=%d",
                         static_cast<long long>((now - speaking_since_us_) / 1000),
                         static_cast<unsigned>(input_rms), static_cast<double>(peak),
                         static_cast<unsigned>(shape.chunks),
                         static_cast<unsigned>(std::sqrt(static_cast<double>(shape.square_sum) / samples)),
                         static_cast<unsigned>(shape.max_chunk_rms), static_cast<unsigned>(shape.loud_chunks),
                         static_cast<unsigned>(shape.peak),
                         static_cast<unsigned>(shape.square_sum == 0 ? 0 : shape.diff_square_sum * 100 / shape.square_sum),
                         static_cast<unsigned>(shape.crossings * 1000ULL / samples),
                         voicing < 0.0f ? -1 : static_cast<int>(voicing * 100.0f), voicing_us, duck_tag,
                         voicing_scaled < 0.0f ? -1 : static_cast<int>(voicing_scaled * 100.0f));
            }
#endif
            if (report) {
                Report(speaking, energy, input_rms);
                last_report_us_ = now;
            }
        }
    }

    // XMOS control writes are not persisted, so the tuning chosen for this product is
    // re-sent at every boot and read back for the log.
    void ApplyOverrides() {
        if (CONFIG_RESPEAKER_XVF3800_PP_DTSENSITIVE >= 0) {
            const int32_t value = CONFIG_RESPEAKER_XVF3800_PP_DTSENSITIVE;
            int32_t readback = -1;
            const bool ok = control_.Write(Xvf3800Control::kResPp, 31, &value, sizeof(value)) &&
                            control_.Read(Xvf3800Control::kResPp, 31, &readback, sizeof(readback));
            ESP_LOGI(kTag, "XVF3800 PP_DTSENSITIVE := %d (read back %d, %s)", static_cast<int>(value),
                     static_cast<int>(readback), ok && readback == value ? "ok" : "FAILED");
        }
        if (CONFIG_RESPEAKER_XVF3800_MIC_GAIN[0] != '\0') {
            const float gain = strtof(CONFIG_RESPEAKER_XVF3800_MIC_GAIN, nullptr);
            float readback = NAN;
            const bool ok = control_.Write(Xvf3800Control::kResAudioMgr, 0, &gain, sizeof(gain)) &&
                            control_.Read(Xvf3800Control::kResAudioMgr, 0, &readback, sizeof(readback));
            ESP_LOGI(kTag, "XVF3800 AUDIO_MGR_MIC_GAIN := %g (read back %g, %s)", static_cast<double>(gain),
                     static_cast<double>(readback), ok && readback == gain ? "ok" : "FAILED");
        }
        if (CONFIG_RESPEAKER_XVF3800_PP_AGC_FIXED_GAIN[0] != '\0') {
            const float gain = strtof(CONFIG_RESPEAKER_XVF3800_PP_AGC_FIXED_GAIN, nullptr);
            const int32_t off = 0;
            float gain_back = NAN;
            int32_t on_back = -1;
            const bool ok = control_.Write(Xvf3800Control::kResPp, 13, &gain, sizeof(gain)) &&
                            control_.Write(Xvf3800Control::kResPp, 10, &off, sizeof(off)) &&
                            control_.Read(Xvf3800Control::kResPp, 13, &gain_back, sizeof(gain_back)) &&
                            control_.Read(Xvf3800Control::kResPp, 10, &on_back, sizeof(on_back));
            ESP_LOGI(kTag, "XVF3800 PP_AGCGAIN := %g, PP_AGCONOFF := 0 (read back %g / %d, %s)",
                     static_cast<double>(gain), static_cast<double>(gain_back),
                     static_cast<int>(on_back), ok && on_back == 0 ? "ok" : "FAILED");
        }
        if (CONFIG_RESPEAKER_XVF3800_PP_MGSCALE[0] != '\0') {
            float scale[3] = {};
            char* rest = nullptr;
            scale[0] = strtof(CONFIG_RESPEAKER_XVF3800_PP_MGSCALE, &rest);
            scale[1] = strtof(rest, nullptr);
            scale[2] = scale[1];
            float readback[3] = {NAN, NAN, NAN};
            const bool ok = control_.Write(Xvf3800Control::kResPp, 29, scale, sizeof(scale)) &&
                            control_.Read(Xvf3800Control::kResPp, 29, readback, sizeof(readback));
            ESP_LOGI(kTag, "XVF3800 PP_MGSCALE := %g %g (read back %g %g %g, %s)", static_cast<double>(scale[0]),
                     static_cast<double>(scale[1]), static_cast<double>(readback[0]),
                     static_cast<double>(readback[1]), static_cast<double>(readback[2]),
                     ok && readback[0] == scale[0] && readback[1] == scale[1] ? "ok" : "FAILED");
        }
        if (CONFIG_RESPEAKER_XVF3800_PP_FMIN_SPEINDEX[0] != '\0') {
            const float value = strtof(CONFIG_RESPEAKER_XVF3800_PP_FMIN_SPEINDEX, nullptr);
            float readback = NAN;
            const bool ok = control_.Write(Xvf3800Control::kResPp, 30, &value, sizeof(value)) &&
                            control_.Read(Xvf3800Control::kResPp, 30, &readback, sizeof(readback));
            ESP_LOGI(kTag, "XVF3800 PP_FMIN_SPEINDEX := %g (read back %g, %s)",
                     static_cast<double>(value), static_cast<double>(readback),
                     ok && readback == value ? "ok" : "FAILED");
        }
    }

#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
    // Once a second: feed the watch. Reboots the XMOS when the watch asks for it and the device has
    // not played anything for a while (GPT-Live splits a reply at 0.7 s of silence). Returns true
    // if it rebooted. A failed reboot only costs an attempt: the watch counts it and then gives up.
    bool AecWatchTick(int64_t now, bool speaking) {
        auto& app = Application::GetInstance();
        if (speaking) {
            aec_last_speaking_us_ = now;
        }
        if (now - aec_last_second_us_ >= 1000000) {
            aec_last_second_us_ = now;
            portENTER_CRITICAL(&g_input_shape_lock);
            const AecWatchCounts counts = g_aec_watch_counts;
            g_aec_watch_counts = AecWatchCounts{};
            portEXIT_CRITICAL(&g_input_shape_lock);
            const auto rms = [](uint64_t square, uint32_t frames) {
                return frames == 0 ? 0U : static_cast<uint32_t>(std::sqrt(static_cast<double>(square) / frames));
            };
            const uint32_t seen_before = aec_watch_.seconds_seen();
            const auto event = aec_watch_.AddSecond(rms(counts.left_square, counts.rx_frames),
                                                    rms(counts.right_square, counts.rx_frames),
                                                    rms(counts.tx_square, counts.tx_frames),
                                                    counts.tx_frames >= kAudioSampleRate * 3 / 4, now / 1000000);
            const auto median = static_cast<double>(aec_watch_.Median());
            if (aec_watch_.seconds_seen() != seen_before &&
                aec_watch_.seconds_seen() % CONFIG_RESPEAKER_XVF3800_AEC_WATCH_WINDOWS == 0) {
                ESP_LOGI(kTag, "XVF3800 aec watch: right / playback median %.3f over %d s of playback", median,
                         aec_watch_.count());
            }
            switch (event) {
                case AecWatch::Event::kBroken:
                    ESP_LOGW(kTag, "XVF3800 aec watch: median %.3f, the echo canceller looks broken", median);
                    break;
                case AecWatch::Event::kRebootFailed:
                    ESP_LOGW(kTag, "XVF3800 aec watch: median %.3f, the reboot did not help", median);
                    break;
                case AecWatch::Event::kRecovered:
                    ESP_LOGW(kTag, "XVF3800 aec watch: median %.3f, recovered", median);
                    break;
                case AecWatch::Event::kNeedsPowerCycle:
                    ESP_LOGW(kTag, "XVF3800 aec watch: median %.3f after %d reboots today, needs a power cycle", median,
                             aec_watch_.RebootsInLastDay(now / 1000000));
                    aec_told_this_session_ = false;
                    break;
                case AecWatch::Event::kNone:
                    break;
            }
        }
        const auto state = app.GetDeviceState();
        if (state != kDeviceStateListening && state != kDeviceStateSpeaking) {
            aec_told_this_session_ = false;
        } else if (aec_watch_.needs_power_cycle() && !aec_told_this_session_) {
            // The server says it aloud: the loudspeaker still works, and only a person can help.
            aec_told_this_session_ = true;
            char payload[128];
            snprintf(payload, sizeof(payload),
                     "{\"jsonrpc\":\"2.0\",\"method\":\"notifications/device_health\","
                     "\"params\":{\"aec\":\"needs_power_cycle\",\"median\":%.2f}}",
                     static_cast<double>(aec_watch_.Median()));
            app.SendMcpMessage(payload);
            ESP_LOGW(kTag, "XVF3800 aec watch: told the server that a power cycle is needed");
        }
        if (aec_watch_.reboot_wanted() && !speaking && now - aec_last_speaking_us_ >= kAecWatchQuietUs) {
            ESP_LOGW(kTag, "XVF3800 aec watch: rebooting the XMOS (reboot %d today)",
                     aec_watch_.RebootsInLastDay(now / 1000000) + 1);
            RebootXmos();
            return true;
        }
        return false;
    }
#endif

    void Report(bool speaking, const float* energy, uint32_t input_rms) {
        uint16_t doa[2] = {};
        const bool doa_ok = doa_cmd_ != 0 &&
                            control_.Read(Xvf3800Control::kResGpo, doa_cmd_, doa, sizeof(doa));
        float agc_gain = NAN;
        control_.Read(Xvf3800Control::kResPp, Xvf3800Control::kCmdAgcGain, &agc_gain,
                      sizeof(agc_gain));
        int32_t converged = -1;
        control_.Read(Xvf3800Control::kResAec, Xvf3800Control::kCmdAecConverged, &converged,
                      sizeof(converged));
        int32_t path_change = -1;
        control_.Read(Xvf3800Control::kResAec, Xvf3800Control::kCmdAecPathChange, &path_change,
                      sizeof(path_change));
        ESP_LOGI(kTag,
                 "XVF3800 %s spenergy=%.3g/%.3g/%.3g/%.3g in_rms=%u vad=%d doa=%u speech=%u "
                 "agc_gain=%.1f converged=%d path_change=%d",
                 speaking ? "speaking" : "idle", static_cast<double>(energy[0]),
                 static_cast<double>(energy[1]), static_cast<double>(energy[2]),
                 static_cast<double>(energy[3]), static_cast<unsigned>(input_rms),
                 Application::GetInstance().IsVoiceDetected() ? 1 : 0,
                 doa_ok ? static_cast<unsigned>(doa[0]) : 0U,
                 doa_ok ? static_cast<unsigned>(doa[1]) : 0U, static_cast<double>(agc_gain),
                 static_cast<int>(converged), static_cast<int>(path_change));
    }

#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
    void SetDucked(bool ducked) {
        ducked_ = ducked;
        g_output_duck_q16.store(ducked ? kDuckQ16 : 65536, std::memory_order_relaxed);
    }
#endif

#if CONFIG_RESPEAKER_XVF3800_TUNE_UDP_PORT
    // Evaluation only (see RESPEAKER_XVF3800_TUNE_UDP_PORT): at most one command per poll,
    // run in this task because it owns the control interface.
    void PollTuneCommand() {
        if (tune_socket_ < 0) {
            const int sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock < 0) {
                return;
            }
            sockaddr_in address = {};
            address.sin_family = AF_INET;
            address.sin_port = htons(CONFIG_RESPEAKER_XVF3800_TUNE_UDP_PORT);
            address.sin_addr.s_addr = htonl(INADDR_ANY);
            if (bind(sock, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
                close(sock);
                return;
            }
            tune_socket_ = sock;
            ESP_LOGW(kTag, "XVF3800 tune: listening on UDP port %d", CONFIG_RESPEAKER_XVF3800_TUNE_UDP_PORT);
        }
        char line[64];
        const int length = recv(tune_socket_, line, sizeof(line) - 1, MSG_DONTWAIT);
        if (length <= 0) {
            return;
        }
        line[length] = '\0';
        int duck = 0;
        if (sscanf(line, "duck %d", &duck) == 1) {
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
            duck_speech_polls_ = std::max(0, std::min(kStrongWindowPolls, duck));
            ESP_LOGW(kTag, "XVF3800 tune: duck on %d speech polls of %d", duck_speech_polls_, kStrongWindowPolls);
#endif
            return;
        }
        // "r <resid> <cmd> <f|i|u>[count]" or "w <resid> <cmd> <f|i|u> <v1> [v2] [v3]" (float, int32 or
        // uint8, up to 3 values: PP_MGSCALE is 3 floats, REBOOT is one uint8); the read-back uses the same count.
        unsigned resid = 0;
        unsigned cmd = 0;
        char type = 0;
        int count = 1;
        char text[3][16] = {};
        const char op = line[0];
        int fields = 0;
        if (op == 'r') {
            fields = sscanf(line + 1, "%u %u %c%d", &resid, &cmd, &type, &count);
        } else if (op == 'w') {
            fields = sscanf(line + 1, "%u %u %c %15s %15s %15s", &resid, &cmd, &type, text[0], text[1], text[2]);
            count = fields - 3;
        }
        if ((op != 'r' && op != 'w') || fields < 3 || (op == 'w' && count < 1) || count < 1 || count > 3 ||
            (type != 'f' && type != 'i' && type != 'u') || resid > 255 || cmd > 127) {
            line[strcspn(line, "\r\n")] = '\0';
            ESP_LOGW(kTag, "XVF3800 tune: bad command \"%s\"", line);
            return;
        }
        union {
            float f[3];
            int32_t i[3];
            uint8_t u[12];
        } value = {};
        const size_t bytes = static_cast<size_t>(count) * (type == 'u' ? 1U : 4U);
        if (op == 'w') {
            for (int k = 0; k < count; ++k) {
                if (type == 'f') {
                    value.f[k] = strtof(text[k], nullptr);
                } else if (type == 'u') {
                    value.u[k] = static_cast<uint8_t>(strtol(text[k], nullptr, 10));
                } else {
                    value.i[k] = static_cast<int32_t>(strtol(text[k], nullptr, 10));
                }
            }
            if (!control_.Write(resid, cmd, &value, bytes)) {
                return;  // Write() logs the failure
            }
        }
        decltype(value) back = {};
        const bool ok = control_.Read(resid, cmd, &back, bytes);
        char shown[64] = {};
        for (int k = 0, used = 0; k < count && ok && used < static_cast<int>(sizeof(shown)); ++k) {
            const int value_k = type == 'u' ? back.u[k] : back.i[k];
            used += type == 'f' ? snprintf(shown + used, sizeof(shown) - used, "%s%g", k ? " " : "",
                                           static_cast<double>(back.f[k]))
                                : snprintf(shown + used, sizeof(shown) - used, "%s%d", k ? " " : "", value_k);
        }
        ESP_LOGW(kTag, "XVF3800 tune: %u/%u %s %s", resid, cmd, op == 'w' ? "written, now" : "=",
                 ok ? shown : "(read failed)");
    }

    int tune_socket_ = -1;
#endif

#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
    // Largest r(lag) / r(0) over recent_, in 512-sample frames stepping back from the newest
    // sample by kVoicingHop, for lags of kVoicingMinLag..kVoicingMaxLag. Each lag sums over
    // the frame's overlap with itself, so a sound that dies away within the lag scores low.
    // *scaled is the same with r(lag) scaled to the whole frame (at most 1): a periodic frame
    // scores about 1 at any pitch, where r(lag) / r(0) stays below overlap / kVoicingFrame.
    void Voicing(float* biased, float* scaled) {
        float best = 0.0f;
        float best_scaled = 0.0f;
        for (int start = recent_count_ - kVoicingFrame; start >= 0; start -= kVoicingHop) {
            float mean = 0.0f;
            for (int i = 0; i < kVoicingFrame; ++i) {
                mean += recent_[start + i];
            }
            mean /= kVoicingFrame;
            float r0 = 0.0f;
            for (int i = 0; i < kVoicingFrame; ++i) {
                frame_[i] = recent_[start + i] - mean;
                r0 += frame_[i] * frame_[i];
            }
            if (r0 <= 0.0f) {
                continue;
            }
            for (int lag = kVoicingMinLag; lag <= kVoicingMaxLag; ++lag) {
                // Four partial sums: the size-optimised build does not unroll this loop, and
                // one accumulator waits on the FPU latency at every step.
                const float* head = frame_;
                const float* tail = frame_ + lag;
                const int count = kVoicingFrame - lag;
                float sum[4] = {};
                int i = 0;
                for (; i + 4 <= count; i += 4) {
                    sum[0] += head[i] * tail[i];
                    sum[1] += head[i + 1] * tail[i + 1];
                    sum[2] += head[i + 2] * tail[i + 2];
                    sum[3] += head[i + 3] * tail[i + 3];
                }
                float r = (sum[0] + sum[1]) + (sum[2] + sum[3]);
                for (; i < count; ++i) {
                    r += head[i] * tail[i];
                }
                const float ratio = r / r0;
                best = std::max(best, ratio);
                best_scaled = std::max(best_scaled, std::min(1.0f, ratio * kVoicingFrame / count));
            }
        }
        *biased = best;
        *scaled = best_scaled;
    }

    void BargeIn(const float* energy, int voiced_polls, uint32_t input_rms, const char* path) {
        ESP_LOGI(kTag,
                 "XVF3800 near-end speech during playback (spenergy %.3g/%.3g/%.3g/%.3g, %d voiced polls, "
                 "input rms %u, %s); aborting",
                 static_cast<double>(energy[0]), static_cast<double>(energy[1]),
                 static_cast<double>(energy[2]), static_cast<double>(energy[3]), voiced_polls,
                 static_cast<unsigned>(input_rms), path);
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_LOG_ONLY
        ESP_LOGI(kTag, "XVF3800 barge-in log only: the reply keeps playing");
        return;
#endif
        // Same steps as the server's "tts stop" plus the wake-word path's queue flush,
        // executed in the main task: tell the server, drop queued audio, start listening.
        Application::GetInstance().Schedule([]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() != kDeviceStateSpeaking) {
                return;
            }
            app.AbortSpeaking(kAbortReasonNone);
            app.GetAudioService().ResetDecoder();
            app.SetDeviceState(kDeviceStateListening);
        });
    }
#endif

#if CONFIG_RESPEAKER_XVF3800_ECHO_GUARD
    // Called on the first poll after a reply ended, so the XMOS (which also drives the
    // loudspeaker) is never rebooted in the middle of playback.
    void EchoGuardReplyEnded() {
        const bool loud_start = reply_loud_polls_ >= kEchoGuardPolls;
        reply_loud_polls_ = 0;
        echo_replies_ = loud_start ? echo_replies_ + 1 : 0;
        if (echo_replies_ < CONFIG_RESPEAKER_XVF3800_ECHO_GUARD_REPLIES) {
            return;
        }
        echo_replies_ = 0;
        const int64_t now = esp_timer_get_time();
        if (now - last_reboot_us_ < kEchoGuardCooldownUs) {
            ESP_LOGW(kTag, "XVF3800 echo guard: replies still start with loud echo, but the XMOS "
                           "was rebooted less than a minute ago");
            return;
        }
        last_reboot_us_ = now;
        ESP_LOGW(kTag,
                 "XVF3800 echo guard: %d replies in a row started with input rms > %d; the echo "
                 "canceller looks diverged, rebooting the XMOS",
                 CONFIG_RESPEAKER_XVF3800_ECHO_GUARD_REPLIES, CONFIG_RESPEAKER_XVF3800_ECHO_GUARD_RMS);
        RebootXmos();
    }
#endif

#if CONFIG_RESPEAKER_XVF3800_ECHO_GUARD || CONFIG_RESPEAKER_XVF3800_AEC_WATCH
    void RebootXmos() {
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
        aec_watch_.OnReboot(esp_timer_get_time() / 1000000);
#endif
        const uint8_t any = 1;
        if (!control_.Write(Xvf3800Control::kResApp, Xvf3800Control::kCmdReboot, &any, sizeof(any))) {
            ESP_LOGW(kTag, "XVF3800 reboot: REBOOT write failed");
            return;
        }
        // Same allowance as at power-up, then wait for the application servicer to answer.
        vTaskDelay(pdMS_TO_TICKS(3000));
        uint8_t version[3] = {};
        bool back = false;
        for (int attempt = 0; attempt < 10 && !back; ++attempt) {
            back = control_.Read(Xvf3800Control::kResApp, Xvf3800Control::kCmdVersion, version,
                                 sizeof(version));
            if (!back) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        if (!back) {
            ESP_LOGW(kTag, "XVF3800 reboot: XMOS does not answer after the reboot");
            return;
        }
        ESP_LOGI(kTag, "XVF3800 reboot: XMOS back (v%u.%u.%u), writing the overrides again",
                 version[0], version[1], version[2]);
        ApplyOverrides();
    }
#endif

    Xvf3800Control control_;
    uint8_t doa_cmd_ = 0;
    int64_t speaking_since_us_ = 0;
    uint64_t history_ = 0;  // one bit per poll, 1 = speech energy above threshold
    uint64_t strong_history_ = 0;  // one bit per poll, 1 = input rms above STRONG_RMS
    uint64_t voiced_history_ = 0;  // one bit per poll, 1 = a voiced speech poll
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
    int16_t recent_[kInputRingSamples];  // the input since the previous poll, see Run()
    int recent_count_ = 0;
    float frame_[kVoicingFrame];
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK
    bool ducked_ = false;
    uint64_t duck_history_ = 0;    // one bit per poll over the fast path's window, 1 = speech poll
    int duck_speech_polls_ = CONFIG_RESPEAKER_XVF3800_BARGE_IN_DUCK_SPEECH_POLLS;  // 0 = loud polls only
    int burst_polls_ = 0;          // speech polls in the current burst
    int64_t last_speech_us_ = 0;   // time of the burst's latest speech poll
    int64_t duck_since_us_ = 0;
#endif
#if CONFIG_RESPEAKER_XVF3800_ECHO_GUARD
    int reply_loud_polls_ = 0;  // polls above ECHO_GUARD_RMS in the current reply's start
    int echo_replies_ = 0;      // consecutive replies that started that loud
    int64_t last_reboot_us_ = -kEchoGuardCooldownUs;
#endif
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
    static constexpr int64_t kAecWatchQuietUs = 5000000;
    AecWatch aec_watch_{AecWatch::Config{
        strtof(CONFIG_RESPEAKER_XVF3800_AEC_WATCH_RATIO, nullptr), CONFIG_RESPEAKER_XVF3800_AEC_WATCH_WINDOWS,
        CONFIG_RESPEAKER_XVF3800_AEC_WATCH_WINDOWS / 2, CONFIG_RESPEAKER_XVF3800_AEC_WATCH_VERIFY_S,
        CONFIG_RESPEAKER_XVF3800_AEC_WATCH_MAX_REBOOTS}};
    int64_t aec_last_second_us_ = 0;
    int64_t aec_last_speaking_us_ = 0;
    bool aec_told_this_session_ = false;
#endif
    int64_t last_report_us_ = 0;
    bool armed_ = true;
};
#endif  // CONFIG_RESPEAKER_XVF3800_BARGE_IN || CONFIG_RESPEAKER_XVF3800_CONTROL_DIAGNOSTICS

class XiaoEsp32S3ReSpeakerXvf3800Board : public WifiBoard {
private:
    Button boot_button_{kBootButtonPin};

    class AudioCodec final : public NoAudioCodec {
    private:
        std::vector<int32_t> rx_buffer_;
        std::vector<int32_t> tx_buffer_;
        int32_t duck_applied_q16_ = 65536;  // g_output_duck_q16 as of the end of the last write
        int64_t last_rx_error_us_ = -5000000;
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
        int32_t previous_sample_ = 0;  // last input sample of the previous read, for InputShape
#endif
#if CONFIG_RESPEAKER_AUDIO_DIAGNOSTICS
        int64_t last_rx_report_us_ = 0;
        uint32_t rx_frames_ = 0;
        uint32_t rx_nonzero_ = 0;
        uint32_t rx_rails_ = 0;
        uint32_t rx_timeouts_ = 0;
        uint32_t rx_left_peak_ = 0;
        uint32_t rx_right_peak_ = 0;
        uint64_t rx_square_sum_ = 0;

        void ReportRx(const std::vector<int32_t>& raw, size_t frames) {
            const int64_t now = esp_timer_get_time();
            if (last_rx_report_us_ != 0 && now - last_rx_report_us_ < 5000000) {
                return;
            }
            const double rms =
                rx_frames_ == 0 ? 0.0 : std::sqrt(static_cast<double>(rx_square_sum_) / rx_frames_);
            ESP_LOGI(kTag,
                     "RX: frames=%lu left_peak32=%lu right_peak32=%lu "
                     "left_nonzero=%lu rails16=%lu rms16=%.1f timeouts=%lu (left slot, shift=16)",
                     static_cast<unsigned long>(rx_frames_),
                     static_cast<unsigned long>(rx_left_peak_),
                     static_cast<unsigned long>(rx_right_peak_),
                     static_cast<unsigned long>(rx_nonzero_), static_cast<unsigned long>(rx_rails_),
                     rms, static_cast<unsigned long>(rx_timeouts_));
            if (frames >= 4) {
                ESP_LOGI(kTag, "RX raw L/R: %08lx/%08lx %08lx/%08lx %08lx/%08lx %08lx/%08lx",
                         static_cast<unsigned long>(static_cast<uint32_t>(raw[0])),
                         static_cast<unsigned long>(static_cast<uint32_t>(raw[1])),
                         static_cast<unsigned long>(static_cast<uint32_t>(raw[2])),
                         static_cast<unsigned long>(static_cast<uint32_t>(raw[3])),
                         static_cast<unsigned long>(static_cast<uint32_t>(raw[4])),
                         static_cast<unsigned long>(static_cast<uint32_t>(raw[5])),
                         static_cast<unsigned long>(static_cast<uint32_t>(raw[6])),
                         static_cast<unsigned long>(static_cast<uint32_t>(raw[7])));
            }
            last_rx_report_us_ = now;
            rx_frames_ = rx_nonzero_ = rx_rails_ = rx_timeouts_ = 0;
            rx_left_peak_ = rx_right_peak_ = 0;
            rx_square_sum_ = 0;
        }
#endif

    public:
        AudioCodec() {
            duplex_ = true;
            input_channels_ = 1;
            output_channels_ = 2;
            input_sample_rate_ = kAudioSampleRate;
            output_sample_rate_ = kAudioSampleRate;

            i2s_chan_config_t channel_config = {
                .id = XIAOZHI_I2S_PORT(0),
                .role = kI2sRole,
                .dma_desc_num = AUDIO_CODEC_DMA_DESC_NUM,
                .dma_frame_num = AUDIO_CODEC_DMA_FRAME_NUM,
                .auto_clear_after_cb = true,
                .auto_clear_before_cb = false,
                .intr_priority = 0,
            };
            ESP_ERROR_CHECK(i2s_new_channel(&channel_config, &tx_handle_, &rx_handle_));

            i2s_std_config_t standard_config = {
                .clk_cfg =
                    {
                        .sample_rate_hz = kAudioSampleRate,
                        .clk_src = I2S_CLK_SRC_DEFAULT,
#if SOC_I2S_HW_VERSION_2
                        .ext_clk_freq_hz = 0,
#endif
                        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
                    },
                .slot_cfg =
                    {
                        .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
                        .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                        .slot_mode = I2S_SLOT_MODE_STEREO,
                        .slot_mask = I2S_STD_SLOT_BOTH,
                        .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
                        .ws_pol = false,
                        .bit_shift = true,
#if SOC_I2S_HW_VERSION_2
                        .left_align = true,
                        .big_endian = false,
                        .bit_order_lsb = false,
#endif
                    },
                .gpio_cfg =
                    {
                        .mclk = I2S_GPIO_UNUSED,
                        .bclk = kI2sBclkPin,
                        .ws = kI2sWsPin,
                        .dout = kI2sDoutPin,
                        .din = kI2sDinPin,
                        .invert_flags =
                            {
                                .mclk_inv = false,
                                .bclk_inv = false,
                                .ws_inv = false,
                            },
                    },
            };
            ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle_, &standard_config));
            ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle_, &standard_config));
            ESP_LOGI(kTag, "I2S: %s, %d Hz, 32-bit stereo bus; mono left-slot input", kI2sRoleName,
                     kAudioSampleRate);
        }

        bool InputData(std::vector<int16_t>& data) override {
            const int samples = Read(data.data(), data.size());
            data.resize(samples);
            return samples > 0;
        }

    protected:
        int Read(int16_t* dest, int samples) override {
            rx_buffer_.resize(static_cast<size_t>(samples) * 2);
            size_t bytes_read = 0;
            const esp_err_t err =
                i2s_channel_read(rx_handle_, rx_buffer_.data(), rx_buffer_.size() * sizeof(int32_t),
                                 &bytes_read, 200);
            if (err == ESP_ERR_TIMEOUT) {
#if CONFIG_RESPEAKER_AUDIO_DIAGNOSTICS
                ++rx_timeouts_;
#endif
                const int64_t now = esp_timer_get_time();
                if (now - last_rx_error_us_ >= 5000000) {
                    ESP_LOGW(kTag, "I2S read timed out; check XMOS power and clock configuration");
                    last_rx_error_us_ = now;
                }
            } else if (err != ESP_OK) {
                ESP_LOGE(kTag, "I2S read failed: %s", esp_err_to_name(err));
                return 0;
            }
            if (bytes_read % (2 * sizeof(int32_t)) != 0) {
                ESP_LOGE(kTag, "Incomplete I2S frame: %u bytes", static_cast<unsigned>(bytes_read));
                return 0;
            }

            const size_t frames = bytes_read / (2 * sizeof(int32_t));
            uint64_t square_sum = 0;
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
            uint64_t right_square_sum = 0;
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
            uint64_t diff_square_sum = 0;
            uint32_t peak = 0;
            uint32_t crossings = 0;
            int32_t previous = previous_sample_;
#endif
            for (size_t i = 0; i < frames; ++i) {
                const int32_t left = rx_buffer_[i * 2];
                const int32_t value = left >> 16;
                dest[i] = static_cast<int16_t>(value);
                square_sum += static_cast<uint64_t>(static_cast<int64_t>(value) * value);
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
                const int32_t diff = value - previous;
                diff_square_sum += static_cast<uint64_t>(static_cast<int64_t>(diff) * diff);
                peak = std::max(peak, static_cast<uint32_t>(value < 0 ? -value : value));
                crossings += (value < 0) != (previous < 0);
                previous = value;
#endif
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
                const int32_t right16 = rx_buffer_[i * 2 + 1] >> 16;
                right_square_sum += static_cast<uint64_t>(static_cast<int64_t>(right16) * right16);
#endif
#if CONFIG_RESPEAKER_AUDIO_DIAGNOSTICS
                const int32_t right = rx_buffer_[i * 2 + 1];
                const uint32_t left_peak =
                    static_cast<uint32_t>(left < 0 ? -static_cast<int64_t>(left) : left);
                const uint32_t right_peak =
                    static_cast<uint32_t>(right < 0 ? -static_cast<int64_t>(right) : right);
                rx_left_peak_ = std::max(rx_left_peak_, left_peak);
                rx_right_peak_ = std::max(rx_right_peak_, right_peak);
                rx_nonzero_ += left != 0;

                rx_rails_ += value == INT16_MIN || value == INT16_MAX;
                rx_square_sum_ += static_cast<uint64_t>(static_cast<int64_t>(value) * value);
#endif
            }
            if (frames > 0) {
                const uint32_t rms = static_cast<uint32_t>(std::sqrt(static_cast<double>(square_sum) / frames));
                g_input_rms.store(rms, std::memory_order_relaxed);
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN
                previous_sample_ = previous;
                portENTER_CRITICAL(&g_input_shape_lock);
                InputShape& shape = g_input_shape;
                ++shape.chunks;
                shape.samples += static_cast<uint32_t>(frames);
                shape.loud_chunks += rms >= static_cast<uint32_t>(CONFIG_RESPEAKER_XVF3800_BARGE_IN_MIN_RMS);
                shape.max_chunk_rms = std::max(shape.max_chunk_rms, rms);
                shape.peak = std::max(shape.peak, peak);
                shape.crossings += crossings;
                shape.square_sum += square_sum;
                shape.diff_square_sum += diff_square_sum;
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
                g_aec_watch_counts.rx_frames += static_cast<uint32_t>(frames);
                g_aec_watch_counts.left_square += square_sum;
                g_aec_watch_counts.right_square += right_square_sum;
#endif
                for (size_t done = 0; done < frames;) {
                    const size_t count =
                        std::min(frames - done, static_cast<size_t>(kInputRingSamples - g_input_ring_pos));
                    memcpy(g_input_ring + g_input_ring_pos, dest + done, count * sizeof(int16_t));
                    g_input_ring_pos = (g_input_ring_pos + static_cast<int>(count)) % kInputRingSamples;
                    done += count;
                }
                portEXIT_CRITICAL(&g_input_shape_lock);
#endif
            }
#if CONFIG_RESPEAKER_AUDIO_DIAGNOSTICS
            rx_frames_ += static_cast<uint32_t>(frames);
            ReportRx(rx_buffer_, frames);
#endif
            return static_cast<int>(frames);
        }

        int Write(const int16_t* data, int samples) override {
            std::lock_guard<std::mutex> lock(data_if_mutex_);
            tx_buffer_.resize(static_cast<size_t>(samples) * 2);
            const double volume_ratio = static_cast<double>(output_volume_) / 100.0;
            const int32_t volume_factor = static_cast<int32_t>(volume_ratio * volume_ratio * 65536);
            // The XVF3800 plays these samples and uses them as its echo reference, so a duck from
            // the barge-in monitor is ramped over one DMA chunk instead of stepped.
            const int32_t duck = g_output_duck_q16.load(std::memory_order_relaxed);
            const int ramp = duck == duck_applied_q16_ ? 0 : std::min(samples, AUDIO_CODEC_DMA_FRAME_NUM);
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
            uint64_t tx_square_sum = 0;
#endif

            for (int i = 0; i < samples; ++i) {
                const int64_t gain =
                    i < ramp ? duck_applied_q16_ + static_cast<int64_t>(duck - duck_applied_q16_) * (i + 1) / ramp
                             : duck;
                const int64_t scaled = static_cast<int64_t>(data[i]) * volume_factor * gain / 65536;
                const int32_t sample =
                    static_cast<int32_t>(std::clamp<int64_t>(scaled, INT32_MIN, INT32_MAX));
                tx_buffer_[static_cast<size_t>(i) * 2] = sample;
                tx_buffer_[static_cast<size_t>(i) * 2 + 1] = sample;
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
                const int32_t tx16 = sample >> 16;
                tx_square_sum += static_cast<uint64_t>(static_cast<int64_t>(tx16) * tx16);
#endif
            }
            duck_applied_q16_ = duck;
#if CONFIG_RESPEAKER_XVF3800_AEC_WATCH
            portENTER_CRITICAL(&g_input_shape_lock);
            g_aec_watch_counts.tx_frames += static_cast<uint32_t>(samples);
            g_aec_watch_counts.tx_square += tx_square_sum;
            portEXIT_CRITICAL(&g_input_shape_lock);
#endif

            constexpr size_t kFrameBytes = 2 * sizeof(int32_t);
            constexpr size_t kChunkBytes = AUDIO_CODEC_DMA_FRAME_NUM * kFrameBytes;
            constexpr int64_t kWriteBudgetUs = 500000;
            const int64_t deadline = esp_timer_get_time() + kWriteBudgetUs;
            const auto* bytes = reinterpret_cast<const uint8_t*>(tx_buffer_.data());
            const size_t total_bytes = tx_buffer_.size() * sizeof(int32_t);
            size_t written = 0;
            while (written < total_bytes) {
                const int64_t remaining_us = deadline - esp_timer_get_time();
                if (remaining_us <= 0) {
                    ESP_LOGE(kTag, "I2S write budget exceeded: %u/%u bytes; check XMOS clocks",
                             static_cast<unsigned>(written), static_cast<unsigned>(total_bytes));
                    break;
                }
                const uint32_t timeout_ms =
                    static_cast<uint32_t>(std::min<int64_t>(100, (remaining_us + 999) / 1000));
                const size_t chunk_bytes = std::min(kChunkBytes, total_bytes - written);
                size_t bytes_written = 0;
                const esp_err_t err = i2s_channel_write(tx_handle_, bytes + written, chunk_bytes,
                                                        &bytes_written, timeout_ms);
                written += bytes_written;
                if (err != ESP_OK || bytes_written == 0) {
                    ESP_LOGE(kTag, "I2S write stopped: %s, %u/%u bytes; remaining audio dropped",
                             esp_err_to_name(err), static_cast<unsigned>(written),
                             static_cast<unsigned>(total_bytes));
                    break;
                }
            }
            return static_cast<int>(written / kFrameBytes);
        }
    };

public:
    XiaoEsp32S3ReSpeakerXvf3800Board() {
        ProbeXvf3800();
#if CONFIG_RESPEAKER_I2S_CLOCK_PROBE_ONLY
        ProbeExternalI2sClock();
#endif
#if CONFIG_RESPEAKER_XVF3800_BARGE_IN || CONFIG_RESPEAKER_XVF3800_CONTROL_DIAGNOSTICS
        static Xvf3800Monitor xvf3800_monitor;
        xvf3800_monitor.Start();
#endif

        boot_button_.OnClick([this]() {
            ESP_LOGI(kTag, "XIAO BOOT clicked");
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
    }

    AudioCodec* GetAudioCodec() override {
        static AudioCodec codec;
        return &codec;
    }

    Led* GetLed() override {
        static GpioLed led(kLedPin, 1);
        return &led;
    }
};

}  // namespace

DECLARE_BOARD(XiaoEsp32S3ReSpeakerXvf3800Board);
