/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WIRELINK_DIAGNOSTICS_HOST_PROFILE_HPP
#define WIRELINK_DIAGNOSTICS_HOST_PROFILE_HPP

// Optional desktop diagnostics, never used by the C core. Enable consistently
// through CMake's WIRELINK_HOST_PROFILING / WIRELINK_HOST_LOCK_PROFILING options.
// Both OFF compiles out all sampling. Lock-only mode avoids broad CPU probes.
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <thread>
#if (defined(WIRELINK_HOST_PROFILING) || defined(WIRELINK_HOST_LOCK_PROFILING)) && defined(__linux__)
#include <time.h>
#endif

namespace wirelink::diagnostics {
enum class Stage : unsigned {
  usb_rx, usb_rx_gap, usb_tx, owner, pump, application, command,
  command_lock, wake, wait, status_callback, status_wait, rx_to_owner,
  notify_to_owner, latest_submit_wait, latest_submit_hold, latest_take_wait,
  latest_take_hold, latest_finish_wait, latest_finish_hold, rpc_admit_wait,
  rpc_admit_hold, rpc_collect_wait, rpc_collect_hold, rpc_finish_wait,
  rpc_finish_hold, count
};

#if defined(WIRELINK_HOST_PROFILING) || defined(WIRELINK_HOST_LOCK_PROFILING)
inline constexpr std::array names{
  "usb_rx", "usb_rx_gap", "usb_tx", "owner", "pump", "application",
  "command", "command_lock", "wake", "wait", "status_callback", "status_wait",
  "rx_to_owner", "notify_to_owner", "latest_submit_wait", "latest_submit_hold",
  "latest_take_wait", "latest_take_hold", "latest_finish_wait", "latest_finish_hold",
  "rpc_admit_wait", "rpc_admit_hold", "rpc_collect_wait", "rpc_collect_hold",
  "rpc_finish_wait", "rpc_finish_hold"
};
static_assert(names.size() == static_cast<unsigned>(Stage::count));
struct Totals {
  std::atomic<std::uint64_t> count{}, wall_ns{}, cpu_ns{}, max_ns{}, slow{};
};
struct Sample {
  Stage stage{};
  std::uint64_t begin_ns{}, wall_ns{}, cpu_ns{}, thread{};
};
inline std::array<Totals, static_cast<unsigned>(Stage::count)> totals{};
inline std::array<Sample, 2048> slow_samples{};
inline std::atomic<std::uint64_t> slow_count{};

inline std::uint64_t timestamp() noexcept {
  return static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count());
}
inline std::uint64_t thread_cpu() noexcept {
#if defined(__linux__)
  timespec value{};
  if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) == 0)
    return static_cast<std::uint64_t>(value.tv_sec) * 1000000000U + value.tv_nsec;
#endif
  return 0; // Unavailable, not an estimate of CPU time.
}
inline void record(Stage stage, std::uint64_t begin, std::uint64_t end,
                   std::uint64_t cpu = 0) noexcept {
  if (begin == 0 || end < begin) return;
  auto& total = totals[static_cast<unsigned>(stage)];
  const auto wall = end - begin;
  total.count.fetch_add(1, std::memory_order_relaxed);
  total.wall_ns.fetch_add(wall, std::memory_order_relaxed);
  total.cpu_ns.fetch_add(cpu, std::memory_order_relaxed);
  auto maximum = total.max_ns.load(std::memory_order_relaxed);
  while (maximum < wall && !total.max_ns.compare_exchange_weak(
      maximum, wall, std::memory_order_relaxed)) {}
  if (wall < 4000000U) return;
  total.slow.fetch_add(1, std::memory_order_relaxed);
  const auto index = slow_count.fetch_add(1, std::memory_order_relaxed);
  if (index < slow_samples.size()) {
    slow_samples[index] = {stage, begin, wall, cpu,
      static_cast<std::uint64_t>(std::hash<std::thread::id>{}(std::this_thread::get_id()))};
  }
}
// Process-wide; call only with ALL instrumented workers quiesced/joined.
// Writers reserve distinct slots, never overwrite, allocate, lock, or print.
inline void reset() noexcept {
  for (auto& value : totals) {
    value.count = 0; value.wall_ns = 0; value.cpu_ns = 0;
    value.max_ns = 0; value.slow = 0;
  }
  slow_count = 0;
}
inline void dump(std::FILE* output, unsigned cycle) noexcept {
#if defined(__linux__)
  std::fprintf(output, "wirelink_host_profile_v1,cycle=%u,cpu_clock=thread,unit=ns\n", cycle);
#else
  std::fprintf(output, "wirelink_host_profile_v1,cycle=%u,cpu_clock=unavailable,unit=ns\n", cycle);
#endif
  for (unsigned i = 0; i < names.size(); ++i) {
    const auto& value = totals[i];
    std::fprintf(output, "wirelink_host_profile_v1,stage=%s,cycle=%u,count=%llu,wall_ns=%llu,cpu_ns=%llu,max_ns=%llu,slow=%llu\n",
      names[i], cycle, static_cast<unsigned long long>(value.count.load()),
      static_cast<unsigned long long>(value.wall_ns.load()),
      static_cast<unsigned long long>(value.cpu_ns.load()),
      static_cast<unsigned long long>(value.max_ns.load()),
      static_cast<unsigned long long>(value.slow.load()));
  }
  const auto count = slow_count.load();
  for (std::uint64_t i = 0; i < count && i < slow_samples.size(); ++i) {
    const auto& value = slow_samples[i];
    std::fprintf(output, "wirelink_host_profile_v1,slow=%s,cycle=%u,begin_ns=%llu,wall_ns=%llu,cpu_ns=%llu,thread=%llu\n",
      names[static_cast<unsigned>(value.stage)], cycle,
      static_cast<unsigned long long>(value.begin_ns),
      static_cast<unsigned long long>(value.wall_ns),
      static_cast<unsigned long long>(value.cpu_ns),
      static_cast<unsigned long long>(value.thread));
  }
  std::fprintf(output, "wirelink_host_profile_v1,cycle=%u,slow_dropped=%llu\n", cycle,
    static_cast<unsigned long long>(count > slow_samples.size() ? count - slow_samples.size() : 0));
}
#else
inline std::uint64_t timestamp() noexcept { return 0; }
inline std::uint64_t thread_cpu() noexcept { return 0; }
inline void record(Stage, std::uint64_t, std::uint64_t, std::uint64_t = 0) noexcept {}
inline void reset() noexcept {}
inline void dump(std::FILE*, unsigned) noexcept {}
#endif

class Scope {
public:
  explicit Scope(Stage stage) noexcept
      : stage_(stage)
#if defined(WIRELINK_HOST_PROFILING)
      , begin_(timestamp()), cpu_(thread_cpu())
#else
      , begin_(0), cpu_(0)
#endif
      {}
  ~Scope() { finish(); }
  Scope(const Scope&) = delete;
  Scope& operator=(const Scope&) = delete;
  void finish() noexcept {
    if (begin_ == 0) return;
    const auto end_cpu = thread_cpu();
    record(stage_, begin_, timestamp(), end_cpu >= cpu_ ? end_cpu - cpu_ : 0);
    begin_ = 0;
  }
private:
  Stage stage_;
  std::uint64_t begin_, cpu_;
};

// Acquisition/critical-section WALL time, not thread CPU. Export accounting
// happens after unlock so shared diagnostic atomics do not extend the hold.
// OFF reduces to an ordinary mutex lock/unlock with no clock reads.
class MutexScope {
public:
  MutexScope(std::mutex& mutex, Stage wait, Stage hold)
      : mutex_(mutex), wait_(wait), hold_(hold), begin_(timestamp()) {
    mutex_.lock();
    acquired_ = timestamp();
  }
  ~MutexScope() {
    const auto end = timestamp();
    mutex_.unlock();
    record(wait_, begin_, acquired_);
    record(hold_, acquired_, end);
  }
  MutexScope(const MutexScope&) = delete;
  MutexScope& operator=(const MutexScope&) = delete;
private:
  std::mutex& mutex_;
  Stage wait_, hold_;
  std::uint64_t begin_, acquired_;
};

// Approximate first notification-to-next-consumer-pass latency. This tracks
// wake batches, NOT individual frames or their decoding/callback lifetimes.
class PendingTimestamp {
public:
  void notify() noexcept {
#if defined(WIRELINK_HOST_PROFILING) || defined(WIRELINK_HOST_LOCK_PROFILING)
    std::uint64_t empty{};
    (void)begin_.compare_exchange_strong(empty, timestamp(), std::memory_order_relaxed);
#endif
  }
  void consume(Stage stage) noexcept {
#if defined(WIRELINK_HOST_PROFILING) || defined(WIRELINK_HOST_LOCK_PROFILING)
    const auto begin = begin_.exchange(0, std::memory_order_relaxed);
    if (begin != 0) record(stage, begin, timestamp());
#else
    (void)stage;
#endif
  }
private:
  // Keep layout identical in profiled/unprofiled consumers.
  std::atomic<std::uint64_t> begin_{};
};
} // namespace wirelink::diagnostics
#endif
