#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <time.h>

int read_adc() {
    return rand() % 1024;
}

int main(int argc, char *argv[]) {
    if (argc != 4) {
        fprintf(stderr, "Usage: %s <int port> <string server_ip> <float delay_ms>\n", argv[0]);
        return 1;
    }

    int port = atoi(argv[1]);
    const char *server_ip = argv[2];
    double delay_ms = atof(argv[3]);

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, server_ip, &server_addr.sin_addr) <= 0) {
        perror("inet_pton");
        close(sock);
        return 1;
    }

    if (connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        perror("connect");
        close(sock);
        return 1;
    }

    printf("Connected to %s:%d\n", server_ip, port);
    srand(time(NULL));

    char message[64];
    while (1) {
        int adc_value = read_adc();
        snprintf(message, sizeof(message), "ADC: %d\n", adc_value);
        if (send(sock, message, strlen(message), 0) < 0) {
            perror("send");
            break;
        }
        usleep(delay_ms * 1000);
    }

    close(sock);
    return 0;
}
