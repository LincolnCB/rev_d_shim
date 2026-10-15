// Event-driven run-controller -- see run_ctrl.h.
//
// One poll() blocks on the hw_manager interrupt (the folded MCDMA/hw_manager doorbell,
// delivered non-root via /dev/hw_manager_irq) and a local-abort eventfd, so a hardware fault
// and a SIGINT/software abort unblock the same wait. The sticky status word is the source of
// truth on each hardware wake; the interrupt is re-armed by writing 1 to the device (the
// pl-irq re-arm), so there is no spin-polling of the status register.

#define _POSIX_C_SOURCE 200809L

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include "run_ctrl.h"

#define HW_MANAGER_IRQ_DEV "/dev/hw_manager_irq"  // pl-irq-shim node, matches sys_sts.c

// Arm (or re-arm) the interrupt: writing 1 re-enables delivery after a wake.
static void run_ctrl_arm(int fd) {
  uint32_t one = 1;
  if (write(fd, &one, sizeof(one)) < 0) {
    // Non-fatal: a failed re-arm only means the next doorbell is missed, and the backstop
    // timeout still advances the caller's progress check.
    perror("run_ctrl: failed to arm hw_manager interrupt");
  }
}

struct run_ctrl_t run_ctrl_create(struct sys_sts_t *sys_sts, bool verbose) {
  struct run_ctrl_t rc;
  memset(&rc, 0, sizeof(rc));
  rc.irq_fd = -1;
  rc.abort_fd = -1;
  rc.sys_sts = sys_sts;
  rc.ok = false;

  rc.irq_fd = open(HW_MANAGER_IRQ_DEV, O_RDWR);
  if (rc.irq_fd < 0) {
    fprintf(stderr, "run_ctrl: open %s: %s\n", HW_MANAGER_IRQ_DEV, strerror(errno));
    return rc;
  }
  // EFD_NONBLOCK so a drain read never blocks; the counter semantics coalesce repeated
  // aborts into one pending event, which is exactly what we want.
  rc.abort_fd = eventfd(0, EFD_NONBLOCK);
  if (rc.abort_fd < 0) {
    fprintf(stderr, "run_ctrl: eventfd: %s\n", strerror(errno));
    close(rc.irq_fd);
    rc.irq_fd = -1;
    return rc;
  }

  run_ctrl_arm(rc.irq_fd);
  rc.ok = true;
  if (verbose) printf("run_ctrl: armed on %s with a local-abort eventfd.\n", HW_MANAGER_IRQ_DEV);
  return rc;
}

void run_ctrl_destroy(struct run_ctrl_t *rc) {
  if (!rc) return;
  if (rc->irq_fd >= 0) close(rc->irq_fd);
  if (rc->abort_fd >= 0) close(rc->abort_fd);
  rc->irq_fd = -1;
  rc->abort_fd = -1;
  rc->ok = false;
}

bool run_ctrl_status_is_fault(uint32_t hw_status) {
  uint32_t state = HW_STS_STATE(hw_status);
  uint32_t code  = HW_STS_CODE(hw_status);
  // A fault drives hw_manager out of S_RUNNING into S_HALTING/S_HALTED carrying its code.
  // Any state that is not one of the normal pre-run / running states, or a code that is not
  // OK/EMPTY, is treated as a stop condition.
  if (state == S_HALTING || state == S_HALTED) return true;
  if (code != STS_OK && code != STS_EMPTY) return true;
  return false;
}

run_ctrl_event_t run_ctrl_wait(struct run_ctrl_t *rc, int timeout_ms) {
  if (!rc || !rc->ok) return RUN_CTRL_ERROR;

  struct pollfd fds[2];
  fds[0].fd = rc->irq_fd;   fds[0].events = POLLIN;  fds[0].revents = 0;
  fds[1].fd = rc->abort_fd; fds[1].events = POLLIN;  fds[1].revents = 0;

  int n = poll(fds, 2, timeout_ms);
  if (n < 0) {
    if (errno == EINTR) return RUN_CTRL_TIMEOUT;  // a caught signal; let the caller re-check
    perror("run_ctrl: poll");
    return RUN_CTRL_ERROR;
  }
  if (n == 0) return RUN_CTRL_TIMEOUT;

  // Local abort takes priority: drain the counter and report it.
  if (fds[1].revents & POLLIN) {
    uint64_t v;
    (void)!read(rc->abort_fd, &v, sizeof(v));
    return RUN_CTRL_ABORT;
  }

  if (fds[0].revents & POLLIN) {
    uint32_t count;
    ssize_t r = read(rc->irq_fd, &count, sizeof(count));
    if (r != (ssize_t)sizeof(count)) {
      perror("run_ctrl: read hw_manager interrupt");
      return RUN_CTRL_ERROR;
    }
    rc->last_status = sys_sts_get_hw_status(rc->sys_sts, false);
    run_ctrl_arm(rc->irq_fd);  // re-arm for the next doorbell
    return run_ctrl_status_is_fault(rc->last_status) ? RUN_CTRL_FAULT : RUN_CTRL_EVENT;
  }

  return RUN_CTRL_TIMEOUT;
}

void run_ctrl_request_abort(struct run_ctrl_t *rc) {
  if (!rc || rc->abort_fd < 0) return;
  uint64_t one = 1;
  (void)!write(rc->abort_fd, &one, sizeof(one));  // async-signal-safe
}
