#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "commands.h"
#include "input.h"

// Entry point for interactive static-shims command interface.
int main(int argc, char **argv) {
	if (argc != 2) {
		fprintf(stderr, "Usage: %s <channel_count>\n", argv[0]);
		return 1;
	}

	char *end = NULL;
	unsigned long parsed_count = strtoul(argv[1], &end, 10);
	if (end == argv[1] || *end != '\0') {
		fprintf(stderr, "Invalid channel_count '%s'. Expected a positive integer.\n", argv[1]);
		return 1;
	}

	if (parsed_count < 1 || parsed_count > 64) {
		fprintf(stderr, "Channel count '%lu' is out of supported range (1-64).\n", parsed_count);
		return 1;
	}

	shim_runtime_state_t state = commands_init_state((uint32_t)parsed_count);

	printf("Static shims command interface\n");
    printf("NOTE: THIS IS NOT ACTUALLY WORKING YET, DO NOT USE THIS FOR ANYTHING OTHER THAN DEVELOPMENT TESTING.\n");
	printf("Configured channels: %u\n", state.channel_count);
	printf("Type H for help. Type Q to quit.\n\n");

	char line[INPUT_LINE_MAX];
	char error_buf[128];

	state.running = true;

	while (state.running) {
		printf("shim> ");
		fflush(stdout);

		if (fgets(line, sizeof(line), stdin) == NULL) {
			printf("\nExiting (EOF).\n");
			break;
		}

		size_t len = strlen(line);
		if (len > 0U && line[len - 1U] == '\n') {
			line[len - 1U] = '\0';
		}

		parsed_command_t cmd;
		input_parse_result_t parse_result = input_parse_line(line, &cmd, &state, error_buf, sizeof(error_buf));
		if (parse_result == INPUT_PARSE_EMPTY) {
			continue;
		}
		if (parse_result == INPUT_PARSE_ERROR) {
			fprintf(stderr, "Parse error: %s\n", error_buf);
			continue;
		}

		(void)commands_execute(&cmd, &state);
	}

	return 0;
}
