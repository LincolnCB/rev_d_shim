//  waveform.c
//
//  Usage:
//    waveform <file.csv> [--adc <path> | -a <path>] [--lockout <float> | -l <float>]
//    [--clk_MHz <float> | -c <float>] [--iters <int> | -i <int>] [--help | -h]
//
//  <file.csv> is a required positional argument.
//  All flags are optional and have defaults:
//    --adc, -a      (string)  default: none (unset)
//    --lockout, -l  (double)  default: 10.0
//    --clk_MHz, -c  (double)  default: 30.0
//    --iters, -i    (int)     default: 1
//    --help, -h     

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <pthread.h>
#include <stdbool.h>
#include <unistd.h>

#include "waveform_file_handling.h"
#include "waveform_hw.h"
#include "dma_ctrl.h"
#include "run_ctrl.h"

typedef struct {
  const char *input_file;  // required positional argument
  const char *adc_file;    // --adc, default: NULL (none)
  double lockout;          // --lockout, default: 10.0
  double clk_MHz;          // --clk_MHz, default: 30.0
  int iters;               // --iters, default: 1
  bool use_dma;            // --dma, default: false (prebuffered MCDMA datapath)
} config_t;

// Set from the signal handler (async-signal-safe) and polled by the main
// monitor loop, which performs the actual stop-request + hardware power-off
// from normal thread context. A volatile sig_atomic_t is the only state the
// handler may safely touch.
static volatile sig_atomic_t g_stop_requested = 0;

// Borrowed run-controller pointer so the signal handler can wake its poll() wait with an
// async-signal-safe eventfd write. NULL outside a DMA run.
static struct run_ctrl_t *g_run_ctrl = NULL;

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
    "  -d, --dma                 Use the prebuffered MCDMA datapath on all active boards\n"
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

// Copy src into dst, dropping the trailing filename extension if one exists
// (the last '.' in the final path component). "test.csv" -> "test",
// "archive.tar.gz" -> "archive.tar", "noext" -> "noext". dst must hold at
// least PATH_MAX bytes.
static void strip_extension(char *dst, size_t dst_size, const char *src) {
  snprintf(dst, dst_size, "%s", src);
  char *slash = strrchr(dst, '/');
  char *dot = strrchr(dst, '.');
  // Only trim when the dot belongs to the final path component and isn't a
  // leading dot (e.g. a dotfile like ".config" has no extension to drop).
  if (dot != NULL && (slash == NULL || dot > slash + 1) &&
      (slash == NULL || dot != slash + 1)) {
    *dot = '\0';
  }
}

// --- Signal handling ---------------------------------------------------

// Handler for SIGINT/SIGTERM: only record that a stop was requested. Everything
// that is unsafe to do from signal context -- locking thread mutexes, powering
// off the hardware, printing -- is done by the main thread once it observes
// this flag.
static void handle_stop_signal(int signum) {
  (void)signum;
  g_stop_requested = 1;
  // Wake a DMA run's poll() wait immediately (eventfd write is async-signal-safe).
  if (g_run_ctrl != NULL) run_ctrl_request_abort(g_run_ctrl);
}

// Print a one-line notice (flushed immediately so it doesn't interleave with
// other output) when a stream thread finishes, noting whether it ran to
// completion or stopped early. Called from the main monitor thread so the
// prints stay serialized.
static void print_stream_finished(const char *name, bool stopped_early) {
  if (stopped_early) {
    printf("[%s] stream stopped early\n", name);
  } else {
    printf("[%s] stream finished\n", name);
  }
  fflush(stdout);
}

// Start both ADC streams -- the command stream and the data (output) stream --
// together, since the data stream only runs while the command stream does. On
// success both threads are running and *cmd_tid / *data_tid are set, and it
// returns 0. On failure it prints an error, makes sure no ADC thread is left
// running (joining the command thread if the data thread failed to start), and
// returns -1 so the caller can clean up the rest.
static int start_adc_streams(adc_cmd_file_info_t *cmd_info, adc_data_file_info_t *data_info,
                             pthread_t *cmd_tid, pthread_t *data_tid) {
  if (pthread_create(cmd_tid, NULL, adc_cmd_stream_thread, cmd_info) != 0) {
    fprintf(stderr, "Error: failed to start ADC command stream thread\n");
    return -1;
  }
  if (pthread_create(data_tid, NULL, adc_data_stream_thread, data_info) != 0) {
    fprintf(stderr, "Error: failed to start ADC data stream thread\n");
    adc_cmd_file_info_request_stop(cmd_info);
    pthread_join(*cmd_tid, NULL);
    return -1;
  }
  return 0;
}

// --- DMA datapath run --------------------------------------------------
//
// The prebuffered MCDMA run. The DAC command stream is synthesized per board and prebuffered
// into DDR, so MM2S fills each board's DAC FIFO (deep enough for the whole sequence) and
// stalls behind the leading trigger-wait. The ADC command lane stays PIO (its stream thread
// is unchanged), but the ADC data lane is captured via S2MM into DDR, not the PIO drain. The
// trigger lane stays PIO. Software does nothing during the run except drain the capture and
// wait on the run-controller for a fault or local abort.
//
// Returns 0 on a clean finish, negative on a fault, abort, or setup error. The caller owns
// powering the hardware off.
static int run_dma_path(hw_t *hw, config_t *cfg, waveform_file_info_t *input_info,
                        adc_cmd_file_info_t *adc_info, bool has_adc,
                        const char *input_stem, long expected_triggers) {
  bool verbose = hw->verbose;
  uint32_t board_count = hw->board_count;

  if (!hw_running(hw)) {
    fprintf(stderr, "Error: [DMA] hardware is not running\n");
    return -1;
  }

  // Synthesize the per-board DAC command streams (byte-identical to the PIO feed).
  dac_word_buf_t dac_bufs[HW_MAX_CHANNELS / 8];
  if (waveform_build_dac_dma(input_info, dac_bufs) != 0) {
    fprintf(stderr, "Error: [DMA] failed to synthesize DAC command streams\n");
    return -1;
  }
  printf("[DMA] synthesized %zu DAC command words per board for %u board(s)\n",
         dac_bufs[0].count, board_count);

  // Capture length: four 32-bit words per 8-channel ADC read, one read per ADC file row,
  // replayed `iters` times. Zero when there is no ADC file (DAC-only run, no S2MM capture).
  uint32_t cap_words = 0;
  if (has_adc) {
    cap_words = (uint32_t)(4L * adc_info->num_rows * (long)cfg->iters);
    if (cap_words == 0) {
      fprintf(stderr, "Error: [DMA] ADC capture resolved to zero words\n");
      for (uint32_t b = 0; b < board_count; b++) dac_word_buf_free(&dac_bufs[b]);
      return -1;
    }
  }

  // Map the MCDMA mover and build the run-controller.
  struct dma_ctrl_t dma = create_dma_ctrl(verbose);
  if (!dma.ok) {
    fprintf(stderr, "Error: [DMA] MCDMA/u-dma-buf unavailable (is the bitstream DMA-enabled and u-dma-buf loaded?)\n");
    for (uint32_t b = 0; b < board_count; b++) dac_word_buf_free(&dac_bufs[b]);
    return -1;
  }
  struct run_ctrl_t rc = run_ctrl_create(&hw->sys_sts, verbose);
  if (!rc.ok) {
    fprintf(stderr, "Error: [DMA] run-controller unavailable (/dev/hw_manager_irq missing?)\n");
    destroy_dma_ctrl(&dma);
    for (uint32_t b = 0; b < board_count; b++) dac_word_buf_free(&dac_bufs[b]);
    return -1;
  }
  g_run_ctrl = &rc;

  // Arm every active board: prebuffer its DAC stream and (when capturing) its S2MM ring.
  int rc_arm = 0;
  if (dma_wave_begin(&dma, verbose) != 0) {
    rc_arm = -1;
  } else {
    for (uint32_t b = 0; b < board_count && rc_arm == 0; b++) {
      if (dma_wave_add(&dma, (int)b, dac_bufs[b].words, (uint32_t)dac_bufs[b].count,
                       cap_words, verbose) != 0) {
        rc_arm = -1;
      }
    }
  }
  for (uint32_t b = 0; b < board_count; b++) dac_word_buf_free(&dac_bufs[b]);
  if (rc_arm != 0) {
    fprintf(stderr, "Error: [DMA] failed to arm the run (see the message above)\n");
    dma_wave_halt(&dma, verbose);
    g_run_ctrl = NULL;
    run_ctrl_destroy(&rc);
    destroy_dma_ctrl(&dma);
    return -1;
  }

  // Open per-board ADC capture files and per-board sample-staging state.
  FILE *adc_csv[HW_MAX_CHANNELS / 8] = {0};
  int16_t sample_ch[HW_MAX_CHANNELS / 8][8];
  uint32_t filled[HW_MAX_CHANNELS / 8] = {0};
  uint32_t sample_index[HW_MAX_CHANNELS / 8] = {0};
  if (has_adc) {
    for (uint32_t b = 0; b < board_count; b++) {
      char path[PATH_MAX];
      snprintf(path, sizeof(path), "%s.adc_out_A.board%u.csv", input_stem, b);
      adc_csv[b] = fopen(path, "w");
      if (adc_csv[b] == NULL) {
        fprintf(stderr, "Error: [DMA] could not open '%s': %s\n", path, strerror(errno));
        for (uint32_t k = 0; k < b; k++) if (adc_csv[k]) fclose(adc_csv[k]);
        dma_wave_halt(&dma, verbose);
        g_run_ctrl = NULL;
        run_ctrl_destroy(&rc);
        destroy_dma_ctrl(&dma);
        return -1;
      }
      fprintf(adc_csv[b], "# DMA ADC capture (amps), board %u. ch0..ch7 per 8-channel read.\n", b);
      fprintf(adc_csv[b], "# sample,ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7\n");
    }
  }

  // Start the PIO command/trigger streams. Block the stop signals while spawning so they are
  // delivered only to the main thread.
  sigset_t stop_signals, old_mask;
  sigemptyset(&stop_signals);
  sigaddset(&stop_signals, SIGINT);
  sigaddset(&stop_signals, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &stop_signals, &old_mask);

  pthread_t adc_tid = 0, trig_tid = 0;
  bool have_adc_thread = false, have_trig_thread = false;

  trigger_file_info_t trig_info;
  char trig_path[PATH_MAX];
  trigger_file_info_init(&trig_info, "Trigger", input_info->num_trigs, cfg->iters);
  trig_info.hw = hw;
  snprintf(trig_path, sizeof(trig_path), "%s.trig_t_sec.csv", input_stem);
  trig_info.path = trig_path;

  int setup_rc = 0;
  if (has_adc) {
    adc_info->hw = hw;
    if (pthread_create(&adc_tid, NULL, adc_cmd_stream_thread, adc_info) != 0) {
      fprintf(stderr, "Error: [DMA] failed to start ADC command stream thread\n");
      setup_rc = -1;
    } else {
      have_adc_thread = true;
    }
  }
  if (setup_rc == 0) {
    if (pthread_create(&trig_tid, NULL, trigger_stream_thread, &trig_info) != 0) {
      fprintf(stderr, "Error: [DMA] failed to start trigger stream thread\n");
      setup_rc = -1;
    } else {
      have_trig_thread = true;
    }
  }

  pthread_sigmask(SIG_SETMASK, &old_mask, NULL);

  int result = 0;
  if (setup_rc != 0) {
    result = -1;
  } else {
    // Release the triggers: the DAC/ADC run from here, driven entirely by the prebuffered
    // stream plus the PIO command lanes.
    hw_start_triggers(hw, (uint32_t)expected_triggers);
    printf("[DMA] triggers released; waiting for the run to complete.\n");

    uint32_t last_trig = 0;
    bool done = false;
    while (!done) {
      run_ctrl_event_t ev = run_ctrl_wait(&rc, 200);
      if (ev == RUN_CTRL_FAULT) {
        fprintf(stderr, "[DMA] hardware fault during run (status 0x%08X):\n", rc.last_status);
        print_hw_status(rc.last_status, verbose);
        result = -1;
        break;
      }
      if (ev == RUN_CTRL_ABORT || g_stop_requested) {
        fprintf(stderr, "[DMA] run aborted (local request).\n");
        result = -1;
        break;
      }
      if (ev == RUN_CTRL_ERROR) {
        fprintf(stderr, "[DMA] run-controller error.\n");
        result = -1;
        break;
      }

      // Drain each board's capture into its CSV (on both EVENT and TIMEOUT wakes).
      if (has_adc) {
        uint32_t buf[1024];
        for (uint32_t b = 0; b < board_count; b++) {
          int got;
          while ((got = dma_wave_read_board(&dma, (int)b, buf,
                                            (uint32_t)(sizeof buf / sizeof buf[0]))) > 0) {
            for (int i = 0; i < got; i++) {
              uint32_t word = buf[i];
              int16_t lo = (int16_t)(word & 0xFFFF);
              int16_t hi = (int16_t)((word >> 16) & 0xFFFF);
              sample_ch[b][filled[b] * 2u]      = lo;
              sample_ch[b][filled[b] * 2u + 1u] = hi;
              filled[b]++;
              if (filled[b] == 4u) {
                fprintf(adc_csv[b], "%u", sample_index[b]);
                for (int ch = 0; ch < 8; ch++)
                  fprintf(adc_csv[b], ",%.4f", HW_MAX_ABS_AMPS * (double)sample_ch[b][ch] / 32767.0);
                fprintf(adc_csv[b], "\n");
                sample_index[b]++;
                filled[b] = 0;
              }
            }
          }
          fflush(adc_csv[b]);
        }
      }

      // Progress line on each new trigger.
      uint32_t tcount = hw_get_trigger_count(hw);
      if (tcount != last_trig) {
        printf("[DMA] trigger count: %u / %ld\n", tcount, expected_triggers);
        fflush(stdout);
        last_trig = tcount;
      }

      // Completion: with ADC, when every board has captured its full count; DAC-only, when
      // all triggers have fired and every DAC command FIFO has drained.
      if (has_adc) {
        done = true;
        for (uint32_t b = 0; b < board_count; b++)
          if (dma_wave_read_total_board(&dma, (int)b) < cap_words) { done = false; break; }
      } else {
        done = (tcount >= (uint32_t)expected_triggers);
        if (done) {
          for (uint32_t b = 0; b < board_count; b++)
            if (!FIFO_STS_EMPTY(sys_sts_get_dac_cmd_fifo_status(&hw->sys_sts, (uint8_t)b, false))) {
              done = false; break;
            }
        }
      }
    }
  }

  // Stop and join the PIO threads.
  if (have_adc_thread) adc_cmd_file_info_request_stop(adc_info);
  if (have_trig_thread) trigger_file_info_request_stop(&trig_info);
  if (have_adc_thread) pthread_join(adc_tid, NULL);
  if (have_trig_thread) pthread_join(trig_tid, NULL);
  trigger_file_info_destroy(&trig_info);

  if (has_adc) {
    for (uint32_t b = 0; b < board_count; b++) {
      if (adc_csv[b] == NULL) continue;
      fclose(adc_csv[b]);
      printf("[DMA] board %u: wrote %u ADC sample(s) to %s.adc_out_A.board%u.csv\n",
             b, sample_index[b], input_stem, b);
    }
  }

  // Coordinated shutdown (Stage 4): halt the MCDMA channels first, then clear the DMA-driven
  // FIFOs -- so the engine is never mid-transfer when the FIFOs reset, any DAC commands left
  // stranded in a paused FIFO are dropped, and a latched bridge fault clears before the next
  // run. The descriptor rings are reinitialized by the next run's dma_wave_begin.
  dma_wave_halt(&dma, verbose);
  hw_reset_dma_buffers(hw);
  g_run_ctrl = NULL;
  run_ctrl_destroy(&rc);
  destroy_dma_ctrl(&dma);
  return result;
}

int main(int argc, char *argv[]) {
  config_t cfg = {
    .input_file = NULL,
    .adc_file   = NULL,
    .lockout    = 10.0,
    .clk_MHz    = 30.0,
    .iters      = 1,
    .use_dma    = false
  };

  static struct option long_options[] = {
    {"adc",     required_argument, 0, 'a'},
    {"lockout", required_argument, 0, 'l'},
    {"clk_MHz", required_argument, 0, 'c'},
    {"iters",   required_argument, 0, 'i'},
    {"dma",     no_argument,       0, 'd'},
    {"help",    no_argument,       0, 'h'},
    {0, 0, 0, 0}
  };

  int opt;
  int option_index = 0;

  // Default (permute) mode: getopt_long reorders argv so all flags are
  //  handled here, leaving any positional arguments at the end for us
  //  to read via optind below.
  while ((opt = getopt_long(argc, argv, "ha:l:c:i:d", long_options, &option_index)) != -1) {
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
      case 'd':
      cfg.use_dma = true;
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
  printf("datapath   = %s\n", cfg.use_dma ? "DMA (prebuffered MCDMA)" : "PIO");

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
  adc_cmd_file_info_t adc_info;
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

  // --- Select the datapath before power-on -------------------------
  // datapath_mode is locked by ctrl_en, so it must be set while the system is off. In DMA
  // mode every active board uses the prebuffered MCDMA path (mask covers boards 0..N-1).
  if (cfg.use_dma) {
    uint8_t dma_mask = (uint8_t)((hw.board_count >= 8) ? 0xFFu : ((1u << hw.board_count) - 1u));
    sys_ctrl_set_datapath_mode(&hw.sys_ctrl, dma_mask, hw.verbose);
    printf("[HW] datapath_mode set to DMA for %u board(s) (mask 0x%02X)\n", hw.board_count, dma_mask);
  }

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

  // --- DMA datapath: prebuffer the DAC stream and run via MCDMA ----
  if (cfg.use_dma) {
    char dma_stem[PATH_MAX];
    strip_extension(dma_stem, sizeof(dma_stem), cfg.input_file);
    long expected = (long)cfg.iters * input_info.num_trigs;
    int dma_rc = run_dma_path(&hw, &cfg, &input_info, has_adc_info ? &adc_info : NULL,
                              has_adc_info, dma_stem, expected);
    waveform_file_info_destroy(&input_info);
    if (has_adc_info) adc_cmd_file_info_destroy(&adc_info);
    hw_power_off(&hw);
    if (g_stop_requested) {
      fprintf(stderr, "Interrupted by signal; hardware powered off\n");
      return EXIT_FAILURE;
    }
    if (dma_rc != 0) {
      fprintf(stderr, "Error: DMA run did not complete cleanly\n");
      return EXIT_FAILURE;
    }
    printf("Done.\n");
    return EXIT_SUCCESS;
  }

  // --- Start the stream threads ------------------------------------
  pthread_t dac_tid, adc_tid, adc_data_tid, trigger_tid;
  trigger_file_info_t trigger_arg;
  adc_data_file_info_t adc_data_info;
  bool has_adc_thread = false;
  bool has_adc_data_thread = false;

  // Output CSV paths, derived from the input file name with its extension
  // trimmed (so "test.csv" yields "test.trig_t_sec.csv", not
  // "test.csv.trig_t_sec.csv").
  char input_stem[PATH_MAX];
  char adc_out_path[PATH_MAX];
  char trig_out_path[PATH_MAX];
  strip_extension(input_stem, sizeof(input_stem), cfg.input_file);

  trigger_file_info_init(&trigger_arg, "Trigger", input_info.num_trigs, cfg.iters);

  input_info.hw = &hw;
  trigger_arg.hw = &hw;
  // The trigger data stream writes one trigger time (in seconds) per line.
  snprintf(trig_out_path, sizeof(trig_out_path), "%s.trig_t_sec.csv", input_stem);
  trigger_arg.path = trig_out_path;
  if (has_adc_info) {
    adc_info.hw = &hw;
    // The ADC data stream drains one sample per ADC read command and writes the
    // active-channel amps (comma-separated) as one line per sample.
    adc_data_file_info_init(&adc_data_info, adc_info.num_rows, cfg.iters);
    adc_data_info.hw = &hw;
    snprintf(adc_out_path, sizeof(adc_out_path), "%s.adc_out_A.csv", input_stem);
    adc_data_info.path = adc_out_path;
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
      adc_cmd_file_info_destroy(&adc_info);
      adc_data_file_info_destroy(&adc_data_info);
    }
    trigger_file_info_destroy(&trigger_arg);
    hw_power_off(&hw);
    return EXIT_FAILURE;
  }

  if (has_adc_info) {
    // The ADC command stream and its data (output) stream are launched together.
    if (start_adc_streams(&adc_info, &adc_data_info, &adc_tid, &adc_data_tid) != 0) {
      waveform_file_info_request_stop(&input_info);
      waveform_file_info_destroy(&input_info);
      adc_cmd_file_info_destroy(&adc_info);
      adc_data_file_info_destroy(&adc_data_info);
      trigger_file_info_destroy(&trigger_arg);
      hw_power_off(&hw);
      return EXIT_FAILURE;
    }
    has_adc_thread = true;
    has_adc_data_thread = true;
  }

  if (pthread_create(&trigger_tid, NULL, trigger_stream_thread, &trigger_arg) != 0) {
    fprintf(stderr, "Error: failed to start trigger stream thread\n");
    waveform_file_info_request_stop(&input_info);
    if (has_adc_thread) {
      adc_cmd_file_info_request_stop(&adc_info);
    }
    if (has_adc_data_thread) {
      adc_data_file_info_request_stop(&adc_data_info);
    }
    waveform_file_info_destroy(&input_info);
    if (has_adc_info) {
      adc_cmd_file_info_destroy(&adc_info);
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
      bool early = waveform_file_info_stopped_early(&input_info);
      if (early) {
        any_stopped_early = true;
      }
      print_stream_finished("DAC", early);
    }
    if (has_adc_thread && !adc_done && adc_cmd_file_info_is_finished(&adc_info)) {
      adc_done = true;
      bool early = adc_cmd_file_info_stopped_early(&adc_info);
      if (early) {
        any_stopped_early = true;
      }
      print_stream_finished("ADC", early);
    }
    if (has_adc_data_thread && !adc_data_done && adc_data_file_info_is_finished(&adc_data_info)) {
      adc_data_done = true;
      bool early = adc_data_file_info_stopped_early(&adc_data_info);
      if (early) {
        any_stopped_early = true;
      }
      print_stream_finished("ADC data", early);
    }
    if (!trigger_done && trigger_file_info_is_finished(&trigger_arg)) {
      trigger_done = true;
      bool early = trigger_file_info_stopped_early(&trigger_arg);
      if (early) {
        any_stopped_early = true;
      }
      print_stream_finished("Trigger", early);
    }

    // If a thread ended early on its own (a hardware error -- a stop signal is
    // handled by the g_stop_requested break above), ask the rest to stop too so
    // they don't keep streaming into a system that is about to be powered off.
    if (any_stopped_early && !stop_all_requested) {
      fprintf(stderr, "Error: a stream thread stopped early; stopping the others\n");
      waveform_file_info_request_stop(&input_info);
      if (has_adc_thread) {
        adc_cmd_file_info_request_stop(&adc_info);
      }
      if (has_adc_data_thread) {
        adc_data_file_info_request_stop(&adc_data_info);
      }
      trigger_file_info_request_stop(&trigger_arg);
      stop_all_requested = true;
    }

    uint32_t current_trigger_count = hw_get_trigger_count(&hw);
    if (current_trigger_count != last_trigger_count) {
      // Report progress against the just-counted trigger. current_trigger_count
      // is the number of completed triggers (>= 1 here), so its 0-based index
      // (count - 1) maps to the iteration and the trigger within that iteration
      // that just fired -- avoiding rolling over to the next iteration when a
      // full iteration's worth of triggers completes.
      uint32_t completed = current_trigger_count - 1;
      uint32_t iteration = completed / input_info.num_trigs;
      uint32_t trigger_in_iteration = completed % input_info.num_trigs;
      printf("[HW] trigger count: %u / %ld (iteration %u / %d, trigger %u / %ld)\n",
             current_trigger_count, expected_triggers,
             iteration + 1, cfg.iters, trigger_in_iteration + 1, input_info.num_trigs);
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
      adc_cmd_file_info_request_stop(&adc_info);
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
    adc_cmd_file_info_destroy(&adc_info);
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
