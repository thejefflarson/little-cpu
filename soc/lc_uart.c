// Root-owned front for ftread: one number, milliseconds, at the fixed baud for at most an hour;
// -q before it reads with every FT232H pin released (ftread's quiet mode).
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef REAL_FTREAD
#error "REAL_FTREAD must name the root-owned ftread by absolute path"
#endif

#define BAUD "115200"
#define MAX_MS 3600000UL

int main(int argc, char **argv) {
  char *end;
  int quiet = argc == 3 && !strcmp(argv[1], "-q");
  const char *arg = quiet ? argv[2] : argc == 2 ? argv[1] : NULL;
  if (!arg || !*arg || arg[0] < '0' || arg[0] > '9') {
    fprintf(stderr, "usage: lc-uart [-q] MILLISECONDS\n");
    return 2;
  }
  errno = 0;
  unsigned long ms = strtoul(arg, &end, 10);
  if (errno || *end || ms < 1 || ms > MAX_MS) {
    fprintf(stderr, "lc-uart: milliseconds must be 1 to %lu\n", MAX_MS);
    return 2;
  }
  char num[24];
  snprintf(num, sizeof num, "%lu", ms);
  char *args[] = {(char *)REAL_FTREAD, BAUD, num, quiet ? "quiet" : NULL, NULL};
  static char *envp[] = {"PATH=/usr/bin:/bin", "LC_ALL=C", NULL};
  execve(REAL_FTREAD, args, envp);
  perror("lc-uart: exec ftread");
  return 2;
}
