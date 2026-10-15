#ifndef RUN_CTRL_H
#define RUN_CTRL_H

#include <stdbool.h>
#include <stdint.h>

#include "sys_sts.h"

// Event-driven run-controller for the prebuffered DMA model.
//
// In the prebuffered model software plays no part during a run, so the run loop is just an
// event wait: a single poll() over the hw_manager interrupt (/dev/hw_manager_irq -- the
// folded MCDMA/hw_manager doorbell) plus a local-abort eventfd, so a hardware fault and a
// local abort (SIGINT, or a software-detected error) unblock the same wait. This replaces
// the observe-only sys_sts monitor, which saw faults but drove nothing.
//
// The controller reports events; it does not decide when a run is finished. Normal end is a
// software fact (all expected capture words read back), so the caller loops run_ctrl_wait,
// draining captures and checking its own completion each time a doorbell or the backstop
// timeout wakes it, and stops on a fault or abort.

typedef enum {
  RUN_CTRL_EVENT = 0,  // a hw_manager doorbell fired; status is benign (still running)
  RUN_CTRL_FAULT,      // hw_manager halted with a fault; see last_status
  RUN_CTRL_ABORT,      // local abort (SIGINT or software request) via the eventfd
  RUN_CTRL_TIMEOUT,    // the backstop timeout elapsed with no event
  RUN_CTRL_ERROR       // controller-level failure (poll/read)
} run_ctrl_event_t;

struct run_ctrl_t {
  int irq_fd;             // /dev/hw_manager_irq (pl-irq-shim), armed and re-armed each wake
  int abort_fd;           // eventfd signalled by run_ctrl_request_abort (async-signal-safe)
  struct sys_sts_t *sys_sts;  // borrowed; used to read the sticky status word on a wake
  uint32_t last_status;   // hardware status word read at the most recent hardware wake
  bool ok;                // true only if both fds opened and the interrupt armed
};

// Open /dev/hw_manager_irq and an abort eventfd, and arm the interrupt. On failure prints a
// message and returns a struct with ok == false. sys_sts is borrowed, not owned.
struct run_ctrl_t run_ctrl_create(struct sys_sts_t *sys_sts, bool verbose);

// Close the fds. Safe on an ok == false controller.
void run_ctrl_destroy(struct run_ctrl_t *rc);

// Block until a hardware doorbell, a local abort, or timeout_ms (negative to block forever).
// On a hardware wake the sticky status word is read into last_status, the interrupt is
// re-armed, and the result is RUN_CTRL_FAULT if the system has left the running state with a
// fault, else RUN_CTRL_EVENT. On the abort eventfd the result is RUN_CTRL_ABORT.
run_ctrl_event_t run_ctrl_wait(struct run_ctrl_t *rc, int timeout_ms);

// Signal a local abort. Writing to an eventfd is async-signal-safe, so this may be called
// directly from a signal handler.
void run_ctrl_request_abort(struct run_ctrl_t *rc);

// True if the status word means the system has halted on a fault (left the running state, or
// carries a non-OK status code) -- the condition that maps a hardware wake to RUN_CTRL_FAULT.
bool run_ctrl_status_is_fault(uint32_t hw_status);

#endif // RUN_CTRL_H
