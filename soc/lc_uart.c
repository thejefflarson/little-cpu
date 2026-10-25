// Root-owned front for ftread, the only UART command the passwordless board rule runs.
// It takes one number, milliseconds, and reads at the fixed baud for at most an hour.
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#ifndef REAL_FTREAD
#error "REAL_FTREAD must name the root-owned ftread by absolute path"
#endif

#define BAUD "115200"
#define MAX_MS 3600000UL

int main(int argc, char **argv) {
  char *end;
  if (argc != 2 || !*argv[1] || argv[1][0] < '0' || argv[1][0] > '9') {
    fprintf(stderr, "usage: lc-uart MILLISECONDS\n");
    return 2;
  }
  errno = 0;
  unsigned long ms = strtoul(argv[1], &end, 10);
  if (errno || *end || ms < 1 || ms > MAX_MS) {
    fprintf(stderr, "lc-uart: milliseconds must be 1 to %lu\n", MAX_MS);
    return 2;
  }
  char num[24];
  snprintf(num, sizeof num, "%lu", ms);
  char *args[] = {(char *)REAL_FTREAD, BAUD, num, NULL};
  static char *envp[] = {"PATH=/usr/bin:/bin", "LC_ALL=C", NULL};
  execve(REAL_FTREAD, args, envp);
  perror("lc-uart: exec ftread");
  return 2;
}
