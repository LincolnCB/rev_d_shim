#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <pthread.h>

#include "dma_waveform_commands.h"
#include "command_helper.h"
#include "system_commands.h"   // safe_buffer_reset
#include "dma_ctrl.h"
#include "dac_ctrl.h"
#include "adc_ctrl.h"
#include "sys_ctrl.h"
#include "sys_sts.h"
#include "map_memory.h"        // board_count

// Defaults. The carrier and envelope defaults are periods expressed in samples (of the
// matched DAC/ADC cadence), so the default frequencies track whatever the max sample rate
// works out to at the current SPI clock.
#define WAVE_DEF_BUF_KB       128.0
#define WAVE_DEF_AMP_A        0.2
#define WAVE_DEF_EXTRA_MS     1.0
#define WAVE_DEF_OUTFILE      "~/dma_wave"
#define WAVE_CARRIER_SAMPLES  100.0    // default carrier period = 100 samples
#define WAVE_ENV_SAMPLES      2000.0   // default envelope period = 2000 samples (20x carrier)

// One background collector at a time. shim-test is a single long-lived process, so a
// file-static holds the run's thread, geometry, and control flags. The collector drains the
// S2MM capture into the CSV as words arrive; the main thread reads these fields for status
// and sets `stop` to end a run early.
static struct {
  pthread_t thread;
  bool started;            // a thread exists and is joinable (running or finished)
  volatile bool running;   // collector loop is active
  volatile bool stop;      // request the collector to end early
  int board;
  uint32_t expected_words; // 4 * n_reads (exact)
  uint32_t clk_hz;
  uint32_t common_delay;   // matched DAC/ADC cadence in SPI cycles
  uint32_t n_dac;
  char adc_path[600];
} g_wave;

// ------------------------------------------------------------- small helpers --

static int wave_system_running(command_context_t* ctx) {
  uint32_t state = HW_STS_STATE(sys_sts_get_hw_status(ctx->sys_sts, *(ctx->verbose)));
  if (state == S_HALTED) {
    printf("Error: Hardware manager is halted. Restart with: off / ctrl_on / pow_on\n");
    return -1;
  }
  if (state != S_RUNNING) {
    printf("Error: Hardware manager is not running (state %u). Use 'ctrl_on' and 'pow_on' first.\n", state);
    return -1;
  }
  return 0;
}

static bool read_line_stdin(char* buf, size_t n) {
  if (fgets(buf, (int)n, stdin) == NULL) return false;
  size_t len = strlen(buf);
  if (len && buf[len - 1] == '\n') buf[len - 1] = '\0';
  return true;
}

// Prompt for a double, showing the default; empty or "." keeps the default.
static double prompt_double(const char* label, double defv) {
  char buf[128];
  printf("%s [%.6g]: ", label, defv);
  fflush(stdout);
  if (!read_line_stdin(buf, sizeof buf)) return defv;
  if (buf[0] == '\0' || (buf[0] == '.' && buf[1] == '\0')) return defv;
  char* end; errno = 0;
  double v = strtod(buf, &end);
  if (errno != 0 || end == buf) { printf("  (unparsed, using %.6g)\n", defv); return defv; }
  return v;
}

// Prompt for an int, showing the default; empty or "." keeps the default.
static int prompt_int(const char* label, int defv) {
  char buf[128];
  printf("%s [%d]: ", label, defv);
  fflush(stdout);
  if (!read_line_stdin(buf, sizeof buf)) return defv;
  if (buf[0] == '\0' || (buf[0] == '.' && buf[1] == '\0')) return defv;
  char* end; errno = 0;
  long v = strtol(buf, &end, 0);
  if (errno != 0 || end == buf) { printf("  (unparsed, using %d)\n", defv); return defv; }
  return (int)v;
}

// Prompt for a string, showing the default; empty or "." keeps the default.
static void prompt_str(const char* label, const char* defv, char* out, size_t n) {
  char buf[512];
  printf("%s [%s]: ", label, defv);
  fflush(stdout);
  if (!read_line_stdin(buf, sizeof buf) || buf[0] == '\0' || (buf[0] == '.' && buf[1] == '\0')) {
    snprintf(out, n, "%s", defv);
    return;
  }
  snprintf(out, n, "%s", buf);
}

static int64_t wave_now_us(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * 1000000 + t.tv_nsec / 1000;
}

// Triangle wave with period 4 in its input: 0 at 0,2,4,...; +1 at 1,5,...; -1 at 3,-1,....
// A libm-free stand-in for a sine (this build links no math library). One full cycle spans
// 4 input units, so a phase in cycles maps in as 4*cycles.
static double tri_wave(double x) {
  double r = x - 4.0 * (double)((long long)(x / 4.0));
  if (r < 0.0) r += 4.0;
  if (r < 1.0) return r;          // rising 0 -> +1
  if (r < 3.0) return 2.0 - r;    // falling +1 -> -1
  return r - 4.0;                 // rising -1 -> 0
}

// ------------------------------------------------------- background collector --

// Drain the S2MM capture into the ADC CSV as words arrive, flushing every pass so the data
// survives a later crash. Each 8-channel sample is 4 words; word w packs ch(2w) in [15:0]
// and ch(2w+1) in [31:16]. Runs until the expected sample count is read, a stop is
// requested, or the hardware halts -- no timeout, so a run may wait for its trigger(s).
static void* wave_collector_thread(void* arg) {
  command_context_t* ctx = (command_context_t*)arg;
  bool verbose = *(ctx->verbose);
  double period_s = (g_wave.clk_hz > 0) ? (double)g_wave.common_delay / (double)g_wave.clk_hz : 0.0;

  FILE* f = fopen(g_wave.adc_path, "w");
  if (!f) {
    fprintf(stderr, "Collector: could not open '%s': %s\n", g_wave.adc_path, strerror(errno));
    g_wave.running = false;
    return NULL;
  }
  fprintf(f, "# DMA waveform ADC capture (amps). cadence %u cyc @ %u Hz\n", g_wave.common_delay, g_wave.clk_hz);
  fprintf(f, "# time_s,ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7\n");

  uint32_t buf[1024];
  int16_t sample_ch[8];
  uint32_t filled = 0;         // words staged toward the current 8-channel sample (0..3)
  uint32_t sample_index = 0;

  while (!g_wave.stop) {
    if (HW_STS_STATE(sys_sts_get_hw_status(ctx->sys_sts, false)) == S_HALTED) {
      fprintf(stderr, "Collector: hardware halted at %u samples; stopping.\n", sample_index);
      break;
    }
    int got = dma_wave_read(ctx->dma_ctrl, buf, (uint32_t)(sizeof buf / sizeof buf[0]));
    if (got <= 0) {
      if (dma_wave_read_total(ctx->dma_ctrl) >= g_wave.expected_words) break;
      nanosleep(&(struct timespec){ .tv_sec = 0, .tv_nsec = 5 * 1000 * 1000 }, NULL); // 5 ms
      continue;
    }
    for (int i = 0; i < got; i++) {
      uint32_t word = buf[i];
      sample_ch[filled * 2u]      = (int16_t)(word & 0xFFFF);
      sample_ch[filled * 2u + 1u] = (int16_t)((word >> 16) & 0xFFFF);
      filled++;
      if (filled == 4u) {
        fprintf(f, "%.6f", (double)sample_index * period_s);
        for (int ch = 0; ch < 8; ch++) fprintf(f, ",%.4f", dac_to_amps(sample_ch[ch]));
        fprintf(f, "\n");
        sample_index++;
        filled = 0;
      }
    }
    fflush(f);   // durability: each pass reaches the file
    if (dma_wave_read_total(ctx->dma_ctrl) >= g_wave.expected_words) break;
  }

  fclose(f);
  set_file_permissions(g_wave.adc_path, verbose);
  printf("Collector: wrote %u samples to %s.\n", sample_index, g_wave.adc_path);
  fflush(stdout);
  g_wave.running = false;
  return NULL;
}

// Join a finished collector so its handle is reusable for the next run.
static void wave_join_if_done(void) {
  if (g_wave.started && !g_wave.running) {
    pthread_join(g_wave.thread, NULL);
    g_wave.started = false;
  }
}

// ------------------------------------------------------------- load command --

int cmd_dma_waveform_test(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  bool verbose  = *(ctx->verbose);
  bool use_def  = has_flag(flags, flag_count, FLAG_DEF);
  bool no_reset = has_flag(flags, flag_count, FLAG_NO_RESET);

  if (!ctx->dma_ctrl->ok) {
    fprintf(stderr, "DMA is not available (/dev/mcdma or u-dma-buf missing).\n");
    return -1;
  }
  wave_join_if_done();
  if (g_wave.running) {
    fprintf(stderr, "A waveform run is already active (dma_waveform_status / dma_waveform_stop).\n");
    return -1;
  }
  if (wave_system_running(ctx) != 0) return -1;

  // Board (positional arg 0, else default 0, else prompt).
  int board;
  if (arg_count > 0) {
    board = validate_board_number(args[0]);
    if (board < 0) return -1;
  } else if (use_def) {
    board = 0;
  } else {
    board = prompt_int("Board", 0);
    if (board < 0 || board >= board_count()) {
      fprintf(stderr, "Board %d out of range (0..%d).\n", board, board_count() - 1);
      return -1;
    }
  }

  // The board must be in DMA mode, with its FIFOs present. The ADC command lane stays PIO.
  uint8_t mask = (uint8_t)(*(ctx->sys_ctrl->datapath_mode) & 0xFF);
  if (!((mask >> board) & 1u)) {
    fprintf(stderr, "Board %d is in PIO mode. Set 'dma_mode %d 1' while the system is off, then ctrl_on/pow_on.\n",
            board, board);
    return -1;
  }
  uint32_t dac_cmd_sts  = sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  uint32_t adc_cmd_sts  = sys_sts_get_adc_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  uint32_t adc_data_sts = sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false);
  if (!FIFO_PRESENT(dac_cmd_sts) || !FIFO_PRESENT(adc_cmd_sts) || !FIFO_PRESENT(adc_data_sts)) {
    fprintf(stderr, "Board %d FIFOs are not all present.\n", board);
    return -1;
  }

  // Matched cadence: run both chips at the larger of the two minimum delays, so the DAC and
  // ADC step together and the two CSVs line up index-for-index.
  uint32_t clk_hz  = sys_sts_get_clk_freq_hz(ctx->sys_sts, false);
  uint32_t dac_min = sys_sts_get_dac_min_delay_time(ctx->sys_sts, false);
  uint32_t adc_min = sys_sts_get_adc_min_delay_time(ctx->sys_sts, false);
  if (clk_hz == 0 || dac_min == 0 || adc_min == 0) {
    fprintf(stderr, "Could not read SPI clock / min-delay status (is the SPI clock set?).\n");
    return -1;
  }
  uint32_t common_delay = (dac_min > adc_min) ? dac_min : adc_min;
  double period_s = (double)common_delay / (double)clk_hz;
  double rate_hz  = 1.0 / period_s;
  double def_carrier = rate_hz / WAVE_CARRIER_SAMPLES;
  double def_env     = rate_hz / WAVE_ENV_SAMPLES;

  printf("Matched cadence: %u cycles at %.3f MHz = %.4f us/sample (%.3f kHz).\n",
         common_delay, clk_hz / 1e6, period_s * 1e6, rate_hz / 1e3);

  // Remaining parameters (positional args, else defaults, else prompts).
  double buf_KB, carrier_Hz, env_Hz, amp_A, extra_ms;
  char outfile[512];
  if (arg_count > 1)      buf_KB = atof(args[1]);
  else if (use_def)       buf_KB = WAVE_DEF_BUF_KB;
  else                    buf_KB = prompt_double("DAC byte length (KB)", WAVE_DEF_BUF_KB);

  if (arg_count > 2)      carrier_Hz = atof(args[2]);
  else if (use_def)       carrier_Hz = def_carrier;
  else                    carrier_Hz = prompt_double("Carrier frequency (Hz)", def_carrier);

  if (arg_count > 3)      env_Hz = atof(args[3]);
  else if (use_def)       env_Hz = def_env;
  else                    env_Hz = prompt_double("Envelope frequency (Hz)", def_env);

  if (arg_count > 4)      amp_A = atof(args[4]);
  else if (use_def)       amp_A = WAVE_DEF_AMP_A;
  else                    amp_A = prompt_double("Amplitude (A, ch4-7 at half)", WAVE_DEF_AMP_A);

  if (arg_count > 5)      extra_ms = atof(args[5]);
  else if (use_def)       extra_ms = WAVE_DEF_EXTRA_MS;
  else                    extra_ms = prompt_double("Extra ADC run time (ms)", WAVE_DEF_EXTRA_MS);

  if (arg_count > 6)      snprintf(outfile, sizeof outfile, "%s", args[6]);
  else if (use_def)       snprintf(outfile, sizeof outfile, "%s", WAVE_DEF_OUTFILE);
  else                    prompt_str("Output base path", WAVE_DEF_OUTFILE, outfile, sizeof outfile);

  if (buf_KB <= 0.0 || amp_A < 0.0 || extra_ms < 0.0 || carrier_Hz <= 0.0 || env_Hz <= 0.0) {
    fprintf(stderr, "Invalid parameter (byte length, amplitude, extra time, and frequencies must be positive).\n");
    return -1;
  }

  // Run sizing. Each DAC update is a 5-word DAC_WR (20 bytes); the ADC produces 4 words per
  // 8-channel read at the same cadence, plus extra reads to cover the settle tail.
  uint32_t n_dac = (uint32_t)((buf_KB * 1024.0) / 20.0);
  if (n_dac < 2) {
    fprintf(stderr, "Byte length too small: need at least 2 DAC updates (%.1f KB gives %u).\n", buf_KB, n_dac);
    return -1;
  }
  double extra_reads_f = (extra_ms * 1e-3) / period_s;
  uint32_t extra_reads = (uint32_t)extra_reads_f;
  if (extra_reads_f > (double)extra_reads) extra_reads++;   // round up to a whole read
  uint32_t n_reads  = n_dac + extra_reads;
  uint32_t cap_words = 4u * n_reads;   // exact: the ADC emits 4 words per 8-channel read

  double run_ms = (double)n_dac * period_s * 1e3;
  printf("Run: %u DAC updates (%.1f KB), %u ADC reads, ~%.1f ms; carrier %.2f Hz, envelope %.2f Hz, amp %.3f A.\n",
         n_dac, n_dac * 20.0 / 1024.0, n_reads, run_ms, carrier_Hz, env_Hz, amp_A);

  // The DAC honors a per-command delay above its own minimum only when do_dac_pre_delay is
  // set; if the matched cadence exceeds the DAC minimum, warn that it must be enabled.
  if (common_delay > dac_min && !(*(ctx->sys_ctrl->do_dac_pre_delay) & 1u)) {
    printf("Warning: matched cadence (%u) exceeds the DAC minimum (%u), but do_dac_pre_delay is off.\n"
           "  Enable it ('toggle_dac_pre_delay' while the system is off) or the DAC will run faster than the ADC.\n",
           common_delay, dac_min);
  }

  if (!no_reset) {
    safe_buffer_reset(ctx, verbose);
    usleep(10000);
  }

  // Synthesize the DAC command stream and write the intended waveform to <outfile>_dac.csv
  // in amps. The first update (t=0) is a trigger-wait so the run releases on the trigger;
  // the rest step at the matched cadence. The last update is all-zero and without CONTINUE,
  // so the DAC returns to idle at a clean zero rather than underflowing.
  char base[512];
  clean_and_expand_path(outfile, base, sizeof base);
  char dac_path[600], adc_path[600];
  snprintf(dac_path, sizeof dac_path, "%s_dac.csv", base);
  snprintf(adc_path, sizeof adc_path, "%s_adc.csv", base);

  uint32_t total_words = n_dac * 5u;
  uint32_t* dac = malloc((size_t)total_words * sizeof(uint32_t));
  if (!dac) { fprintf(stderr, "Out of memory building %u DAC words.\n", total_words); return -1; }

  FILE* fdac = fopen(dac_path, "w");
  if (!fdac) {
    fprintf(stderr, "Could not open '%s': %s\n", dac_path, strerror(errno));
    free(dac);
    return -1;
  }
  fprintf(fdac, "# DMA waveform DAC input (amps). carrier=%.4f Hz envelope=%.4f Hz amp=%.4f A cadence=%u cyc @ %u Hz\n",
          carrier_Hz, env_Hz, amp_A, common_delay, clk_hz);
  fprintf(fdac, "# time_s,ch0,ch1,ch2,ch3,ch4,ch5,ch6,ch7\n");

  for (uint32_t i = 0; i < n_dac; i++) {
    int16_t ch_vals[8];
    double t = (double)i * period_s;
    bool is_last = (i == n_dac - 1);
    if (is_last) {
      for (int ch = 0; ch < 8; ch++) ch_vals[ch] = 0;
    } else {
      double carrier = tri_wave(4.0 * carrier_Hz * t);
      for (int ch = 0; ch < 8; ch++) {
        double amp   = (ch < 4) ? amp_A : (amp_A * 0.5);
        double phi   = (double)(ch % 4) * 0.25;   // per-channel envelope phase, in cycles/4
        double env   = tri_wave(4.0 * env_Hz * t + phi);
        ch_vals[ch]  = amps_to_dac(amp * carrier * env);
      }
    }
    dac_continue_mode_t cont = is_last ? DAC_NO_CONTINUE : DAC_CONTINUE;
    // First update waits for the trigger (count 1); the rest use the matched time delay.
    // LDAC latches all 8 channels to the outputs together -- without it the writes never
    // reach the DAC output and no current flows.
    dac_wait_mode_t trig = (i == 0) ? DAC_TRIGGER_WAIT : DAC_DELAY_WAIT;
    uint32_t value       = (i == 0) ? 1u : common_delay;
    dac_encode_dac_wr(ch_vals, trig, cont, DAC_LDAC, value, &dac[i * 5u]);

    fprintf(fdac, "%.6f", t);
    for (int ch = 0; ch < 8; ch++) fprintf(fdac, ",%.4f", dac_to_amps(ch_vals[ch]));
    fprintf(fdac, "\n");
  }
  fclose(fdac);
  set_file_permissions(dac_path, verbose);

  // Prebuffer into DDR and arm both engines (no trigger, no wait); time the load.
  int64_t t0 = wave_now_us();
  int rc = dma_wave_arm(ctx->dma_ctrl, board, dac, total_words, cap_words, verbose);
  int64_t t1 = wave_now_us();
  free(dac);
  if (rc < 0) return -1;

  double load_ms = (double)(t1 - t0) / 1e3;
  double load_kb = (double)total_words * 4.0 / 1024.0;
  printf("Loaded %u DAC words (%.1f KB) in %.2f ms (%.1f MB/s); wrote %s.\n",
         total_words, load_kb, load_ms, (load_kb / 1024.0) / (load_ms / 1e3), dac_path);

  // PIO ADC command: wait for the same trigger, then read all 8 channels. The core executes
  // (1 + repeat_count) reads, so repeat = n_reads - 1 gives exactly n_reads reads = cap_words.
  adc_cmd_noop(ctx->adc_ctrl, (uint8_t)board, ADC_TRIGGER_WAIT, ADC_CONTINUE, 1, verbose);
  adc_cmd_adc_rd(ctx->adc_ctrl, (uint8_t)board, ADC_DELAY_WAIT, ADC_NO_CONTINUE, common_delay, n_reads - 1u, verbose);

  // Start the background collector: it drains the capture into <base>_adc.csv as the PL
  // fills it, until it has cap_words words (n_reads samples) or is stopped.
  g_wave.board          = board;
  g_wave.expected_words = cap_words;
  g_wave.clk_hz         = clk_hz;
  g_wave.common_delay   = common_delay;
  g_wave.n_dac          = n_dac;
  snprintf(g_wave.adc_path, sizeof g_wave.adc_path, "%s", adc_path);
  g_wave.stop    = false;
  g_wave.running = true;
  if (pthread_create(&g_wave.thread, NULL, wave_collector_thread, ctx) != 0) {
    fprintf(stderr, "Failed to start the collector thread: %s\n", strerror(errno));
    g_wave.running = false;
    dma_wave_disarm(ctx->dma_ctrl);
    return -1;
  }
  g_wave.started = true;

  printf("\nCollector running, expecting %u samples. Fire the trigger to start the run:\n"
         "  force_trig            (one or more; the collector waits with no timeout)\n"
         "  dma_waveform_status   (progress)   dma_waveform_stop (end early)\n"
         "Data streams to %s as it arrives.\n", n_reads, adc_path);
  return 0;
}

// ------------------------------------------------------- status / stop commands --

int cmd_dma_waveform_status(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  (void)args; (void)arg_count; (void)flags; (void)flag_count;
  wave_join_if_done();
  if (!g_wave.started && !g_wave.running && g_wave.expected_words == 0) {
    printf("No waveform run has been loaded.\n");
    return 0;
  }
  int board = g_wave.board;
  uint32_t read_words   = dma_wave_read_total(ctx->dma_ctrl);
  uint32_t avail        = dma_wave_avail(ctx->dma_ctrl);
  uint32_t captured     = read_words + avail;
  uint32_t dac_cmd_fill = FIFO_STS_WORD_COUNT(sys_sts_get_dac_cmd_fifo_status(ctx->sys_sts, (uint8_t)board, false));
  uint32_t dac_cmds     = sys_sts_get_dac_cmds_since_reset(ctx->sys_sts, (uint8_t)board, false);
  uint32_t adc_data_fill= FIFO_STS_WORD_COUNT(sys_sts_get_adc_data_fifo_status(ctx->sys_sts, (uint8_t)board, false));
  uint32_t adc_cmds     = sys_sts_get_adc_cmds_since_reset(ctx->sys_sts, (uint8_t)board, false);

  printf("Waveform run on board %d -- %s\n", board, g_wave.running ? "collecting" : "finished");
  printf("  DAC: added to DMA (PS) %u cmds | cmd FIFO fill %u words | executed (since reset) %u\n",
         g_wave.n_dac, dac_cmd_fill, dac_cmds);
  printf("  ADC: executed (since reset) %u | data FIFO fill %u words | added to DMA (PL) %u words\n",
         adc_cmds, adc_data_fill, captured);
  printf("  Capture: read %u / captured %u / expected %u samples -> %s\n",
         read_words / 4u, captured / 4u, g_wave.expected_words / 4u, g_wave.adc_path);
  return 0;
}

int cmd_dma_waveform_stop(const char** args, int arg_count, const command_flag_t* flags, int flag_count, command_context_t* ctx) {
  (void)args; (void)arg_count; (void)flags; (void)flag_count;
  if (!g_wave.started && !g_wave.running) {
    printf("No waveform run to stop.\n");
    return 0;
  }
  g_wave.stop = true;
  if (g_wave.started) {
    pthread_join(g_wave.thread, NULL);
    g_wave.started = false;
  }
  uint32_t read_words = dma_wave_read_total(ctx->dma_ctrl);
  dma_wave_disarm(ctx->dma_ctrl);
  printf("Stopped. %u samples written to %s.\n", read_words / 4u, g_wave.adc_path);
  return 0;
}
