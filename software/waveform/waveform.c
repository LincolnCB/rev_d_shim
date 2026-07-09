/*
 * waveform.c
 *
 * Usage:
 *   waveform <file.csv> [--adc <path>] [--lockout <float>] [--clk <float>] [--iters <int>]
 *
 * <file.csv> is a required positional argument.
 * All flags are optional and have defaults:
 *   --adc      (string)  default: none (unset)
 *   --lockout  (double)  default: 10.0
 *   --clk      (double)  default: 30.0
 *   --iters    (int)     default: 1
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <stdbool.h>

#include "waveform_file_handling.h"

typedef struct {
  const char *input_file;  /* required positional argument */
  const char *adc_file;    /* --adc, default: NULL (none) */
  double lockout;          /* --lockout, default: 10.0 */
  double clk;              /* --clk, default: 30.0 */
  int iters;               /* --iters, default: 1 */
} config_t;

static void print_usage(const char *prog) {
  fprintf(stderr,
    "Usage: %s <file.csv> [OPTIONS]\n"
    "\n"
    "Required:\n"
    "  <file.csv>            Input file path\n"
    "\n"
    "Options:\n"
    "  --adc <path>          ADC file path (default: none)\n"
    "  --lockout <float>     Lockout value (default: 10.0)\n"
    "  --clk <float>         Clock value (default: 30.0)\n"
    "  --iters <int>         Number of iterations (default: 1)\n"
    "  -h, --help            Show this help message\n",
    prog);
}

/* Parse a double from a string, exiting with an error on failure. */
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

/* Parse an int from a string, exiting with an error on failure. */
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

/* --- Hardware placeholder functions ---------------------------------
 * Stand-ins for the real hardware API. Replace each body with the
 * appropriate call(s) into your hardware library.
 * --------------------------------------------------------------------- */

static void hw_set_spi_clock(double clk_mhz) {
  printf("[HW] TODO: set SPI clock to %g MHz\n", clk_mhz);
}

static int hw_power_on(void) {
  printf("[HW] TODO: power on hardware\n");
  return 0; /* 0 = success */
}

static int hw_check_status(void) {
  printf("[HW] TODO: check hardware status\n");
  return 0; /* 0 = running */
}

static void hw_set_trigger_lockout(double lockout_ns) {
  printf("[HW] TODO: set trigger lockout to %g ns\n", lockout_ns);
}

static void hw_validate_timing(double dac_min_dt, double adc_min_dt) {
  if (adc_min_dt >= 0.0) {
    printf("[HW] TODO: validate timing against DAC min dt = %g s and ADC min dt = %g s\n",
           dac_min_dt, adc_min_dt);
  } else {
    printf("[HW] TODO: validate timing against DAC min dt = %g s (no ADC file)\n", dac_min_dt);
  }
}

static int hw_check_boards_available(int num_channels) {
  printf("[HW] TODO: check enough boards/FIFOs are present for %d channel(s)\n", num_channels);
  return 0; /* 0 = success */
}

static void hw_start_triggers(long expected_triggers) {
  printf("[HW] TODO: start %ld expected trigger(s)\n", expected_triggers);
}

static void hw_power_off(void) {
  printf("[HW] TODO: power off hardware\n");
}

/* --- SIGINT handling --------------------------------------------------- */

static volatile sig_atomic_t g_interrupted = 0;

static void handle_sigint(int signum) {
  (void)signum;
  g_interrupted = 1;
}

int main(int argc, char *argv[]) {
  config_t cfg = {
    .input_file = NULL,
    .adc_file   = NULL,
    .lockout    = 10.0,
    .clk        = 30.0,
    .iters      = 1
  };

  static struct option long_options[] = {
    {"adc",     required_argument, 0, 'a'},
    {"lockout", required_argument, 0, 'l'},
    {"clk",     required_argument, 0, 'c'},
    {"iters",   required_argument, 0, 'i'},
    {"help",    no_argument,       0, 'h'},
    {0, 0, 0, 0}
  };

  int opt;
  int option_index = 0;

  /* Default (permute) mode: getopt_long reorders argv so all flags are
  * handled here, leaving any positional arguments at the end for us
  * to read via optind below. */
  while ((opt = getopt_long(argc, argv, "h", long_options, &option_index)) != -1) {
    switch (opt) {
      case 'a':
      cfg.adc_file = optarg;
      break;
      case 'l':
      cfg.lockout = parse_double_arg("--lockout", optarg);
      break;
      case 'c':
      cfg.clk = parse_double_arg("--clk", optarg);
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

  /* After flag parsing, optind points at the first non-flag argument. */
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

  printf("input_file = %s\n", cfg.input_file);
  printf("adc_file   = %s\n", cfg.adc_file ? cfg.adc_file : "(none)");
  printf("lockout    = %g\n", cfg.lockout);
  printf("clk        = %g\n", cfg.clk);
  printf("iters      = %d\n", cfg.iters);

  /* --- Validate the input (DAC) file ------------------------------ */
  waveform_file_info_t input_info;
  if (validate_input_file(cfg.input_file, &input_info) != 0) {
    return EXIT_FAILURE;
  }
  printf("Input file OK: %d channel(s), %ld row(s), %ld trigger point(s), current range [%g, %g] A\n",
         input_info.num_channels, input_info.num_rows, input_info.num_trigs,
         input_info.min_current, input_info.max_current);

  /* --- Make sure enough boards (FIFOs) are present for the channels - */
  if (hw_check_boards_available(input_info.num_channels) != 0) {
    fprintf(stderr, "Error: insufficient boards available for %d channel(s)\n",
            input_info.num_channels);
    return EXIT_FAILURE;
  }

  /* --- Validate the ADC file, if provided ------------------------- */
  adc_file_info_t adc_info;
  bool has_adc_info = false;
  if (cfg.adc_file != NULL) {
    if (validate_adc_file(cfg.adc_file, input_info.num_trigs, &adc_info) != 0) {
      return EXIT_FAILURE;
    }
    has_adc_info = true;
    printf("ADC file OK: %ld trigger point(s) match the input file\n", adc_info.num_trigs);
  }

  /* --- Catch SIGINT so we can power off the hardware before exiting - */
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = handle_sigint;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGINT, &sa, NULL);

  /* --- Bring up the hardware --------------------------------------- */
  hw_set_spi_clock(cfg.clk);

  if (hw_power_on() != 0) {
    fprintf(stderr, "Error: failed to power on hardware\n");
    return EXIT_FAILURE;
  }

  if (hw_check_status() != 0) {
    fprintf(stderr, "Error: hardware is not running\n");
    hw_power_off();
    return EXIT_FAILURE;
  }

  hw_set_trigger_lockout(cfg.lockout);

  /* --- Validate timing using the min dt values gathered earlier ---- */
  hw_validate_timing(input_info.min_dt, has_adc_info ? adc_info.min_dt : -1.0);

  /* --- Start the stream threads ------------------------------------ */
  pthread_t dac_tid, adc_tid, trigger_tid;
  stream_thread_arg_t dac_arg     = { "DAC",     cfg.input_file };
  stream_thread_arg_t adc_arg     = { "ADC",     cfg.adc_file };
  stream_thread_arg_t trigger_arg = { "Trigger", NULL };
  bool has_adc_thread = false;

  if (pthread_create(&dac_tid, NULL, dac_stream_thread, &dac_arg) != 0) {
    fprintf(stderr, "Error: failed to start DAC stream thread\n");
    hw_power_off();
    return EXIT_FAILURE;
  }

  if (cfg.adc_file != NULL) {
    if (pthread_create(&adc_tid, NULL, adc_stream_thread, &adc_arg) != 0) {
      fprintf(stderr, "Error: failed to start ADC stream thread\n");
      pthread_join(dac_tid, NULL);
      hw_power_off();
      return EXIT_FAILURE;
    }
    has_adc_thread = true;
  }

  if (pthread_create(&trigger_tid, NULL, trigger_stream_thread, &trigger_arg) != 0) {
    fprintf(stderr, "Error: failed to start trigger stream thread\n");
    pthread_join(dac_tid, NULL);
    if (has_adc_thread) {
      pthread_join(adc_tid, NULL);
    }
    hw_power_off();
    return EXIT_FAILURE;
  }

  /* --- Start the expected number of triggers ------------------------ */
  long expected_triggers = (long)cfg.iters * input_info.num_trigs;
  hw_start_triggers(expected_triggers);

  /* --- Monitor progress ----------------------------------------------
   * TODO: replace with real hardware polling every t ms. Once the
   * hardware API can report a live trigger count, poll it here and
   * print progress (e.g. "trigger N of TOTAL, iteration M") whenever
   * the count changes, instead of just waiting on the threads below. */
  printf("[HW] TODO: poll hardware every t ms and report trigger count progress\n");

  /* --- Wait for all stream threads to finish -------------------------
   * Note: pthread_join() is not interrupted by SIGINT, so if the person
   * hits Ctrl+C, g_interrupted is set but we still wait here for the
   * (short, fixed-length) stream threads to finish before checking it
   * and powering off. For longer-running streams, replace this with a
   * shared stop flag that each thread checks between chunks of work. */
  pthread_join(dac_tid, NULL);
  if (has_adc_thread) {
    pthread_join(adc_tid, NULL);
  }
  pthread_join(trigger_tid, NULL);

  if (g_interrupted) {
    printf("\nInterrupted, powering off hardware...\n");
    hw_power_off();
    return EXIT_FAILURE;
  }

  /* --- Power off hardware and exit ----------------------------------- */
  hw_power_off();
  printf("Done.\n");

  return EXIT_SUCCESS;
}
