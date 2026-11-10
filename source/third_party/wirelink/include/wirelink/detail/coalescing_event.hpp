/* SPDX-License-Identifier: Apache-2.0 */

#ifndef WIRELINK_DETAIL_COALESCING_EVENT_HPP
#define WIRELINK_DETAIL_COALESCING_EVENT_HPP

#include <atomic>
#include <chrono>
#include <semaphore>

namespace wirelink::detail {
// MPSC notification, single waiter. Events mean "recheck work", not a count
// of jobs. At most one permit exists, including when there is no waiter.
// Queue contents/ownership remain the caller's responsibility.
class CoalescingEvent {
public:
  void notify() noexcept {
    if (!pending_.exchange(true, std::memory_order_acq_rel)) ready_.release();
  }
  bool try_wait() noexcept {
    if (!ready_.try_acquire()) return false;
    consumed();
    return true;
  }
  void wait() noexcept {
    ready_.acquire();
    consumed();
  }
  template<class Rep, class Period>
  bool wait_for(const std::chrono::duration<Rep, Period>& timeout) {
    if (!ready_.try_acquire_for(timeout)) return false;
    consumed();
    return true;
  }
private:
  void consumed() noexcept {
    // Acquire publications from producers coalesced after the permit release.
    // Clear only AFTER acquiring the permit: otherwise another producer could
    // release twice into a binary semaphore. Arrivals overlapping this clear
    // either belong to the work about to be checked, or leave a new permit.
    (void)pending_.exchange(false, std::memory_order_acq_rel);
  }
  std::atomic<bool> pending_{false};
  std::binary_semaphore ready_{0};
};
} // namespace wirelink::detail
#endif
