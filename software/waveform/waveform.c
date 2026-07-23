//  waveform.c
//
//  Usage:
//    waveform <file.csv> [--adc <path> | -a <path>] [--lockout <float> | -l <float>]
//    [--clk_MHz <float> | -c <float>] [--iters <int> | -i <int>]
//
//  <file.csv> is a required positional argument.
//  All flags are optional and have defaults:
//    --adc, -a      (string)  default: none (unset)
//    --lockout, -l  (double)  default: 10.0
//    --clk_MHz, -c  (double)  default: 30.0
//    --iters, -i    (int)     default: 1

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>

#include "waveform_file_handling.h"
#include "waveform_hw.h"

typedef struct {
  const char *input_file;  // required positional argument
  const char *adc_file;    // --adc, default: NULL (none)
  double lockout;          // --lockout, default: 10.0
  double clk_MHz;          // --clk_MHz, default: 30.0
  int iters;               // --iters, default: 1
} config_t;

// Set from the signal handler (async-signal-safe) and polled by the main
// monitor loop, which performs the actual stop-request + hardware power-off
// from normal thread context. A volatile sig_atomic_t is the only state the
// handler may safely touch.
static volatile sig_atomic_t g_stop_requested = 0;

static void print_usage(const char *prog) {
  fprintf(stderr,
    "Usage: %s <file.csv> [OPTIONS]\n"
    "\n"
    "Required:\n"
    "  <file.csv>            Input file path\n"
    "\n"
    "Options:\n"
    "  -a, --adc <path>          ADC file path (default: none)\n"
    "  -l, --lockout <float>     Lockout value in ms (default: 10.0)\n"
    "  -c, --clk_MHz <float>     Clock value (MHz, default: 30.0)\n"
    "  -i, --iters <int>         Number of iterations (default: 1)\n"
    "  -h, --help                Show this help message\n",
    prog);
}

// Parse a double from a string, exiting with an error on failure.
static double parse_double_arg(const char *flag, const char *val) {
  char *end;
  errno = 0;
  double d = strtod(val, &end);
  if (errno != 0 || end == val || *end != '\0') {
    fprintf(stderr, "Error: invalid value for %s: '%s'\n", flag, val);
    exit(EXIT_FAILURE);
  }
  return d;
}

// Parse an int from a string, exiting with an error on failure.
static int parse_int_arg(const char *flag, const char *val) {
  char *end;
  errno = 0;
  long l = strtol(val, &end, 10);
  if (errno != 0 || end == val || *end != '\0') {
    fprintf(stderr, "Error: invalid value for %s: '%s'\n", flag, val);
    exit(EXIT_FAILURE);
  }
  return (int)l;
}

// --- Signal handling ---------------------------------------------------

// Handler for SIGINT/SIGTERM: only record that a stop was requested. Everything
// that is unsafe to do from signal context -- locking thread mutexes, powering
// off the hardware, printing -- is done by the main thread once it observes
// this flag.
static void handle_stop_signal(int signum) {
  (void)signum;
  g_stop_requested = 1;
}

int main(int argc, char *argv[]) {
  config_t cfg = {
    .input_file = NULL,
    .adc_file   = NULL,
    .lockout    = 10.0,
    .clk_MHz    = 30.0,
    .iters      = 1
  };

  static struct option long_options[] = {
    {"adc",     required_argument, 0, 'a'},
    {"lockout", required_argument, 0, 'l'},
    {"clk_MHz", required_argument, 0, 'c'},
    {"iters",   required_argument, 0, 'i'},
    {"help",    no_argument,       0, 'h'},
    {0, 0, 0, 0}
  };

  int opt;
  int option_index = 0;

  // Default (permute) mode: getopt_long reorders argv so all flags are
  //  handled here, leaving any positional arguments at the end for us
  //  to read via optind below.
  while ((opt = getopt_long(argc, argv, "ha:l:c:i:", long_options, &option_index)) != -1) {
    switch (opt) {
      case 'a':
      cfg.adc_file = optarg;
      break;
      case 'l':
      cfg.lockout = parse_double_arg("--lockout", optarg);
      break;
      case 'c':
      cfg.clk_MHz = parse_double_arg("--clk_MHz", optarg);
      break;
      case 'i':
      cfg.iters = parse_int_arg("--iters", optarg);
      break;
      case 'h':
      print_usage(argv[0]);
      return EXIT_SUCCESS;
      default:
      print_usage(argv[0]);
      return EXIT_FAILURE;
    }
  }

  // After flag parsing, optind points at the first non-flag argument.
  if (optind >= argc) {
    fprintf(stderr, "Error: missing required <file.csv> argument\n");
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }
  cfg.input_file = argv[optind];

  if (optind + 1 < argc) {
    fprintf(stderr, "Error: unexpected extra argument '%s'\n", argv[optind + 1]);
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  if (cfg.iters < 1) {
    fprintf(stderr, "Error: --iters must be >= 1 (got %d)\n", cfg.iters);
    return EXIT_FAILURE;
  }

  printf("input_file = %s\n", cfg.input_file);
  printf("adc_file   = %s\n", cfg.adc_file ? cfg.adc_file : "(none)");
  printf("lockout    = %g\n", cfg.lockout);
  printf("clk_MHz    = %g\n", cfg.clk_MHz);
  printf("iters      = %d\n", cfg.iters);

  // --- Validate the input (DAC) file ------------------------------
  waveform_file_info_t input_info;
  if (validate_input_file(cfg.input_file, &input_info) != 0) {
    return EXIT_FAILURE;
  }
  printf("Input file OK: %d channel(s), %ld row(s), %ld trigger point(s), current range [%g, %g] A\n",
         input_info.num_channels, input_info.num_rows, input_info.num_trigs,
         input_info.min_current, input_info.max_current);
  input_info.iters = cfg.iters;

  // --- Initialize hardware pointers and check that enough boards/FIFOs are present
  hw_t hw = hw_init(input_info.num_channels, cfg.clk_MHz, false);

  // --- Validate the ADC file, if provided -------------------------
  adc_file_info_t adc_info;
  bool has_adc_info = false;
  if (cfg.adc_file != NULL) {
    if (validate_adc_file(cfg.adc_file, input_info.num_trigs, &adc_info) != 0) {
      return EXIT_FAILURE;
    }
    has_adc_info = true;
    adc_info.iters = cfg.iters;
    printf("ADC file OK: %ld trigger point(s) match the input file\n", adc_info.num_trigs);
  }

  // --- Catch SIGINT/SIGTERM so we can power off the hardware before exiting -
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = handle_stop_signal;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);

  // --- Bring up the hardware ---------------------------------------
  if (hw_power_on(&hw) != 0) {
    fprintf(stderr, "Error: failed to power on hardware\n");
    return EXIT_FAILURE;
  }

  if (!hw_running(&hw)) {
    fprintf(stderr, "Error: hardware is not running\n");
    hw_power_off(&hw);
    return EXIT_FAILURE;
  }


  if (hw_set_trigger_lockout(&hw, cfg.lockout) != 0) {
    fprintf(stderr, "Error: failed to set trigger lockout to %g ns\n", cfg.lockout);
    hw_power_off(&hw);
    return EXIT_FAILURE;
  }

  // --- Validate timing using the min dt values gathered earlier ----
  if (!hw_dac_timing_valid(&hw, input_info.min_dt)) {
    fprintf(stderr, "Error: DAC timing is invalid for min dt = %g\n", input_info.min_dt);
    hw_power_off(&hw);
    return EXIT_FAILURE;
  }
  if (has_adc_info && !hw_adc_timing_valid(&hw, adc_info.min_dt)) {
    fprintf(stderr, "Error: ADC timing is invalid for min dt = %g\n", adc_info.min_dt);
    hw_power_off(&hw);
    return EXIT_FAILURE;
  }

  // --- Start the stream threads ------------------------------------
  pthread_t dac_tid, adc_tid, adc_data_tid, trigger_tid;
  trigger_file_info_t trigger_arg;
  adc_data_file_info_t adc_data_info;
  bool has_adc_thread = false;
  bool has_adc_data_thread = false;

  trigger_file_info_init(&trigger_arg, "Trigger", input_info.num_trigs, cfg.iters);

  input_info.hw = &hw;
  trigger_arg.hw = &hw;
  if (has_adc_info) {
    adc_info.hw = &hw;
    // The ADC data stream drains one sample per ADC read command; its path is
    // wired up later, so it just drains the buffer for now.
    adc_data_file_info_init(&adc_data_info, adc_info.num_rows, cfg.iters);
    adc_data_info.hw = &hw;
  }

  // Block SIGINT/SIGTERM while spawning the stream threads so they inherit a
  // blocked mask and never take the signal themselves. It is unblocked again
  // below, once the threads are running, so a stop signal is delivered only to
  // the main thread -- keeping the handler off the worker threads and their
  // mutexes.
  sigset_t stop_signals;
  sigemptyset(&stop_signals);
  sigaddset(&stop_signals, SIGINT);
  sigaddset(&stop_signals, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &stop_signals, NULL);

  if (pthread_create(&dac_tid, NULL, dac_stream_thread, &input_info) != 0) {
    fprintf(stderr, "Error: failed to start DAC stream thread\n");
    waveform_file_info_destroy(&input_info);
    if (has_adc_info) {
      adc_file_info_destroy(&adc_info);
      adc_data_file_info_destroy(&adc_data_info);
    }
    trigger_file_info_destroy(&trigger_arg);
    hw_power_off(&hw);
    return EXIT_FAILURE;
  }

  if (has_adc_info) {
    if (pthread_create(&adc_tid, NULL, adc_stream_thread, &adc_info) != 0) {
      fprintf(stderr, "Error: failed to start ADC stream thread\n");
      waveform_file_info_request_stop(&input_info);
      waveform_file_info_destroy(&input_info);
      if (has_adc_info) {
        adc_file_info_destroy(&adc_info);
        adc_data_file_info_destroy(&adc_data_info);
      }
      trigger_file_info_destroy(&trigger_arg);
      hw_power_off(&hw);
      return EXIT_FAILURE;
    }
    has_adc_thread = true;
  }

  if (has_adc_info) {
    if (pthread_create(&adc_data_tid, NULL, adc_data_stream_thread, &adc_data_info) != 0) {
      fprintf(stderr, "Error: failed to start ADC data stream thread\n");
      waveform_file_info_request_stop(&input_info);
      if (has_adc_thread) {
        adc_file_info_request_stop(&adc_info);
      }
      waveform_file_info_destroy(&input_info);
      adc_file_info_destroy(&adc_info);
      adc_data_file_info_destroy(&adc_data_info);
      trigger_file_info_destroy(&trigger_arg);
      hw_power_off(&hw);
      return EXIT_FAILURE;
    }
    has_adc_data_thread = true;
  }

  if (pthread_create(&trigger_tid, NULL, trigger_stream_thread, &trigger_arg) != 0) {
    fprintf(stderr, "Error: failed to start trigger stream thread\n");
    waveform_file_info_request_stop(&input_info);
    if (has_adc_thread) {
      adc_file_info_request_stop(&adc_info);
    }
    if (has_adc_data_thread) {
      adc_data_file_info_request_stop(&adc_data_info);
    }
    waveform_file_info_destroy(&input_info);
    if (has_adc_info) {
      adc_file_info_destroy(&adc_info);
      adc_data_file_info_destroy(&adc_data_info);
    }
    trigger_file_info_destroy(&trigger_arg);
    hw_power_off(&hw);
    return EXIT_FAILURE;
  }

  // The stream threads are running; unblock SIGINT/SIGTERM so a stop signal is
  // delivered to (and handled only by) the main thread from here on.
  pthread_sigmask(SIG_UNBLOCK, &stop_signals, NULL);

  // --- Start the expected number of triggers ------------------------
  long expected_triggers = (long)cfg.iters * input_info.num_trigs;
  hw_start_triggers(&hw, expected_triggers);

  // --- Monitor progress ----------------------------------------------
  // Poll the hardware trigger counter while the stream threads are running.
  uint32_t last_trigger_count = 0;
  printf("[HW] polling hardware trigger count\n");

  // Let each stream thread run to completion on its own; we just poll their
  // state here. (A stop can still be requested externally, e.g. from the
  // signal handler.)
  bool dac_done = false;
  bool adc_done = false;
  bool adc_data_done = false;
  bool trigger_done = false;
  bool any_stopped_early = false;
  bool stop_all_requested = false;
  while (!dac_done || (has_adc_thread && !adc_done) ||
         (has_adc_data_thread && !adc_data_done) || !trigger_done) {
    // A stop signal (CTRL+C / SIGTERM): stop polling and fall through to the
    // join + power-off below.
    if (g_stop_requested) {
      break;
    }
    if (!dac_done && waveform_file_info_is_finished(&input_info)) {
      dac_done = true;
      if (waveform_file_info_stopped_early(&input_info)) {
        any_stopped_early = true;
      }
    }
    if (has_adc_thread && !adc_done && adc_file_info_is_finished(&adc_info)) {
      adc_done = true;
      if (adc_file_info_stopped_early(&adc_info)) {
        any_stopped_early = true;
      }
    }
    if (has_adc_data_thread && !adc_data_done && adc_data_file_info_is_finished(&adc_data_info)) {
      adc_data_done = true;
      if (adc_data_file_info_stopped_early(&adc_data_info)) {
        any_stopped_early = true;
      }
    }
    if (!trigger_done && trigger_file_info_is_finished(&trigger_arg)) {
      trigger_done = true;
      if (trigger_file_info_stopped_early(&trigger_arg)) {
        any_stopped_early = true;
      }
    }

    // If a thread ended early on its own (a hardware error -- a stop signal is
    // handled by the g_stop_requested break above), ask the rest to stop too so
    // they don't keep streaming into a system that is about to be powered off.
    if (any_stopped_early && !stop_all_requested) {
      fprintf(stderr, "Error: a stream thread stopped early; stopping the others\n");
      waveform_file_info_request_stop(&input_info);
      if (has_adc_thread) {
        adc_file_info_request_stop(&adc_info);
      }
      if (has_adc_data_thread) {
        adc_data_file_info_request_stop(&adc_data_info);
      }
      trigger_file_info_request_stop(&trigger_arg);
      stop_all_requested = true;
    }

    uint32_t current_trigger_count = hw_get_trigger_count(&hw);
    if (current_trigger_count != last_trigger_count) {
      // Print the current trigger count out of the total expected triggers
      // as well as which iteration and trigger within that iteration we are on.
      uint32_t iteration = current_trigger_count / input_info.num_trigs;
      uint32_t trigger_in_iteration = current_trigger_count % input_info.num_trigs;
      printf("[HW] trigger count: %u / %ld (iteration %u, trigger %u / %ld)\n",
             current_trigger_count, expected_triggers, iteration + 1, trigger_in_iteration + 1, input_info.num_trigs);
      fflush(stdout);
      last_trigger_count = current_trigger_count;
    }

    if (!dac_done || (has_adc_thread && !adc_done) ||
        (has_adc_data_thread && !adc_data_done) || !trigger_done) {
      usleep(500000); // Sleep for 500 ms before polling again
    }
  }

  // If we left the loop because of CTRL+C, threads may still be running; ask
  // them all to stop so the joins below return promptly.
  if (g_stop_requested) {
    waveform_file_info_request_stop(&input_info);
    if (has_adc_thread) {
      adc_file_info_request_stop(&adc_info);
    }
    if (has_adc_data_thread) {
      adc_data_file_info_request_stop(&adc_data_info);
    }
    trigger_file_info_request_stop(&trigger_arg);
  }

  // --- Join stream threads now that each has reported finished ------
  pthread_join(dac_tid, NULL);
  if (has_adc_thread) {
    pthread_join(adc_tid, NULL);
  }
  if (has_adc_data_thread) {
    pthread_join(adc_data_tid, NULL);
  }
  pthread_join(trigger_tid, NULL);

  // --- Power off hardware and exit -----------------------------------
  waveform_file_info_destroy(&input_info);
  if (has_adc_info) {
    adc_file_info_destroy(&adc_info);
    adc_data_file_info_destroy(&adc_data_info);
  }
  trigger_file_info_destroy(&trigger_arg);
  hw_power_off(&hw);

  if (g_stop_requested) {
    fprintf(stderr, "Interrupted by signal; hardware powered off\n");
    return EXIT_FAILURE;
  }
  if (any_stopped_early) {
    fprintf(stderr, "Error: one or more stream threads stopped early\n");
    return EXIT_FAILURE;
  }
  printf("Done.\n");

  return EXIT_SUCCESS;
}
