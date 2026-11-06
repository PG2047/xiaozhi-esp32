// Decides from the XVF3800's right channel whether its echo canceller works, without ESP-IDF
// so that it can be tested on a host (deploy/firmware/tests in xiaozhi-brains).
//
// The right channel carries the signal after the canceller and before the post-processing. On
// 11-04 the left channel looked quiet while the right one kept hitting full scale: the post-
// processing was suppressing the echo and the talker with it. The left channel cannot show this,
// the ratio of the right channel to what was sent to the XMOS can. It stayed at 0.09..0.16 over a
// minute of playback in healthy states and reached 0.35 while knocking next to the board.
#pragma once

#include <algorithm>
#include <array>
#include <cstdint>

class AecWatch {
public:
    struct Config {
        float ratio;         // a median right / playback at or above this is a broken canceller
        int windows;         // playback seconds in the median
        int min_windows;     // after a reboot, recovery can be declared from this many
        int verify_windows;  // after a reboot, still broken after this many means it did not help
        int max_reboots;     // in any 24 hours; 0 = only report
    };

    enum class Event { kNone, kBroken, kRecovered, kRebootFailed, kNeedsPowerCycle };

    static constexpr int kMaxWindows = 120;
    static constexpr int kMaxReboots = 8;
    static constexpr uint32_t kMinTxRms = 100;      // below this there is nothing to cancel
    static constexpr uint32_t kMaxLeftRms = 500;    // above this somebody is talking or knocking
    static constexpr int64_t kDaySeconds = 86400;

    explicit AecWatch(const Config& config) : config_(config) {
        config_.windows = std::clamp(config_.windows, 1, kMaxWindows);
        config_.min_windows = std::clamp(config_.min_windows, 1, config_.windows);
        config_.max_reboots = std::clamp(config_.max_reboots, 0, kMaxReboots);
    }

    // One second of measurements. Only a second of real playback without near-end sound counts.
    Event AddSecond(uint32_t left_rms, uint32_t right_rms, uint32_t tx_rms, bool full_second, int64_t now_s) {
        if (!full_second || tx_rms < kMinTxRms || left_rms >= kMaxLeftRms) {
            return Event::kNone;
        }
        ratios_[next_] = static_cast<float>(right_rms) / static_cast<float>(tx_rms);
        next_ = (next_ + 1) % config_.windows;
        count_ = std::min(count_ + 1, config_.windows);
        ++seen_;
        ++since_reboot_;
        const bool full = count_ == config_.windows;
        const bool bad = Median() >= config_.ratio;
        switch (state_) {
            case State::kHealthy:
                return full && bad ? BrokenOrGiveUp(now_s, Event::kBroken) : Event::kNone;
            case State::kVerifying:
                if (count_ >= config_.min_windows && !bad) {
                    state_ = State::kHealthy;
                    return Event::kRecovered;
                }
                if (since_reboot_ >= config_.verify_windows) {
                    const Event event = BrokenOrGiveUp(now_s, Event::kRebootFailed);
                    return event;
                }
                return Event::kNone;
            case State::kWaitingToReboot:
                if (full && !bad) {  // came back on its own before the device fell quiet
                    state_ = State::kHealthy;
                    return Event::kRecovered;
                }
                return Event::kNone;
            case State::kNeedsPowerCycle:
                if (full && !bad) {
                    state_ = State::kHealthy;
                    return Event::kRecovered;
                }
                if (full && CanReboot(now_s)) {  // a day has passed since the first attempts
                    state_ = State::kWaitingToReboot;
                    return Event::kBroken;
                }
                return Event::kNone;
        }
        return Event::kNone;
    }

    // Any reboot of the XMOS, also the echo guard's: the old measurements no longer apply.
    void OnReboot(int64_t now_s) {
        reboot_times_[reboot_next_] = now_s;
        reboot_next_ = (reboot_next_ + 1) % kMaxReboots;
        reboot_count_ = std::min(reboot_count_ + 1, kMaxReboots);
        count_ = 0;
        next_ = 0;
        since_reboot_ = 0;
        state_ = State::kVerifying;
    }

    bool reboot_wanted() const { return state_ == State::kWaitingToReboot; }
    bool needs_power_cycle() const { return state_ == State::kNeedsPowerCycle; }
    uint32_t seconds_seen() const { return seen_; }
    int count() const { return count_; }

    int RebootsInLastDay(int64_t now_s) const {
        int reboots = 0;
        for (int i = 0; i < reboot_count_; ++i) {
            reboots += now_s - reboot_times_[i] < kDaySeconds;
        }
        return reboots;
    }

    float Median() const {
        if (count_ == 0) {
            return 0.0f;
        }
        std::array<float, kMaxWindows> sorted;
        std::copy(ratios_.begin(), ratios_.begin() + count_, sorted.begin());
        std::nth_element(sorted.begin(), sorted.begin() + count_ / 2, sorted.begin() + count_);
        return sorted[count_ / 2];
    }

private:
    enum class State { kHealthy, kWaitingToReboot, kVerifying, kNeedsPowerCycle };

    bool CanReboot(int64_t now_s) const {
        return config_.max_reboots > 0 && RebootsInLastDay(now_s) < config_.max_reboots;
    }

    Event BrokenOrGiveUp(int64_t now_s, Event when_rebooting) {
        if (CanReboot(now_s)) {
            state_ = State::kWaitingToReboot;
            return when_rebooting;
        }
        state_ = State::kNeedsPowerCycle;
        return Event::kNeedsPowerCycle;
    }

    Config config_;
    State state_ = State::kHealthy;
    std::array<float, kMaxWindows> ratios_{};
    int next_ = 0;
    int count_ = 0;
    int since_reboot_ = 0;
    uint32_t seen_ = 0;
    std::array<int64_t, kMaxReboots> reboot_times_{};
    int reboot_next_ = 0;
    int reboot_count_ = 0;
};
