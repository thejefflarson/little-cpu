// Root-owned front for iceprog, the only flashing command the passwordless board rule runs.
// It accepts a bitstream under a build/ directory, or -t alone, each behind an optional -d i:VID:PID.
#define _GNU_SOURCE
#include <ctype.h>
#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <limits.h>
#include <pwd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#ifndef REAL_ICEPROG
#error "REAL_ICEPROG must name the root-owned iceprog by absolute path"
#endif

#define MAX_BITSTREAM (4L << 20)

static void die(const char *why) {
  fprintf(stderr, "lc-flash: %s\n", why);
  exit(2);
}

static int device_ok(const char *d) {
  if ((d[0] != 'i' && d[0] != 's') || d[1] != ':' || !d[2] || strlen(d) > 40) return 0;
  for (const char *p = d + 2; *p; p++)
    if (!isalnum((unsigned char)*p) && *p != ':' && *p != '_') return 0;
  return 1;
}

// The user sudo was run by, or the caller itself when not root.
static void invoker(uid_t *uid, gid_t *gid) {
  if (getuid() != 0) {
    *uid = getuid();
    *gid = getgid();
    return;
  }
  const char *u = getenv("SUDO_UID"), *g = getenv("SUDO_GID");
  char *end;
  if (!u || !g || !*u || !*g) die("run through sudo, which names the invoking user");
  errno = 0;
  unsigned long uu = strtoul(u, &end, 10);
  if (errno || *end || uu == 0 || uu > 0x7fffffffUL) die("SUDO_UID is not a non-root user id");
  unsigned long gg = strtoul(g, &end, 10);
  if (errno || *end || gg > 0x7fffffffUL) die("SUDO_GID is not a group id");
  *uid = (uid_t)uu;
  *gid = (gid_t)gg;
}

static int canonical_path(int fd, char *out) {
#ifdef __APPLE__
  return fcntl(fd, F_GETPATH, out);
#else
  char link[64];
  snprintf(link, sizeof link, "/proc/self/fd/%d", fd);
  ssize_t n = readlink(link, out, PATH_MAX - 1);
  if (n < 0) return -1;
  out[n] = 0;
  return 0;
#endif
}

// True for a canonical path with a build directory above a non-empty remainder.
static int under_build(const char *p) {
  for (const char *s = strstr(p, "/build/"); s; s = strstr(s + 1, "/build/"))
    if (s[7]) return 1;
  return 0;
}

// Opens `path` with the invoking user's credentials, never root's. The descriptor is judged by what it
// is (its canonical path, type, owner, size), so a symlink or a rename after the open changes nothing.
static int open_as_user(const char *path, uid_t uid, gid_t gid) {
  int dropped = geteuid() == 0;
  if (dropped) {
    struct passwd *pw = getpwuid(uid);
    if (!pw || initgroups(pw->pw_name, gid) || setegid(gid) || seteuid(uid)) die("cannot take the invoking user's identity");
  }
  int fd = open(path, O_RDONLY | O_NOCTTY);
  int err = errno;
  if (dropped && (seteuid(0) || setegid(0))) die("cannot restore privilege");
  if (fd < 0) {
    fprintf(stderr, "lc-flash: cannot open %s as you: %s\n", path, strerror(err));
    exit(2);
  }
  char real[PATH_MAX];
  struct stat st;
  if (canonical_path(fd, real) || fstat(fd, &st)) die("cannot resolve the opened file");
  if (!S_ISREG(st.st_mode)) die("not a regular file");
  if (st.st_uid != uid) die("the file is not yours");
  if (st.st_size <= 0 || st.st_size > MAX_BITSTREAM) die("not the size of a bitstream");
  if (!under_build(real)) die("the file does not resolve to a path under a build/ directory");
  return fd;
}

int main(int argc, char **argv) {
  const char *dev = NULL;
  int i = 1, reboot = 0;
  if (i < argc && !strcmp(argv[i], "-d")) {
    if (i + 1 >= argc || !device_ok(argv[i + 1])) die("-d takes i:VID:PID or s:SERIAL and nothing else");
    dev = argv[i + 1];
    i += 2;
  }
  if (i < argc && !strcmp(argv[i], "-t")) {
    reboot = 1;
    i++;
  }
  if (reboot ? i != argc : i + 1 != argc) die("usage: lc-flash [-d i:VID:PID] BITSTREAM | lc-flash [-d i:VID:PID] -t");

  uid_t uid;
  gid_t gid;
  invoker(&uid, &gid);

  char fdpath[32];
  char *args[8];
  int n = 0;
  args[n++] = (char *)REAL_ICEPROG;
  if (dev) {
    args[n++] = "-d";
    args[n++] = (char *)dev;
  }
  if (reboot) {
    args[n++] = "-t";
  } else {
    int fd = open_as_user(argv[i], uid, gid);
    snprintf(fdpath, sizeof fdpath, "/dev/fd/%d", fd);
    args[n++] = fdpath;
  }
  args[n] = NULL;

  static char *envp[] = {"PATH=/usr/bin:/bin", "LC_ALL=C", NULL};
  execve(REAL_ICEPROG, args, envp);
  perror("lc-flash: exec iceprog");
  return 2;
}
