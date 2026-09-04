#include <errno.h> // For errno
#include <fcntl.h> // For open function
#include <stdbool.h> // For bool type
#include <stdint.h> // For uint32_t type
#include <stdio.h> // For printf and fprintf functions
#include <string.h> // For strerror function
#include <sys/mman.h> // For mmap function
#include <unistd.h> // For sysconf and close functions

// Map a pl-reg register window by its /dev node (see map_memory.h)
uint32_t *map_pl_reg(const char *dev_path, bool verbose) {

  if (verbose) printf("Mapping register window [%s]...\n", dev_path);

  // pl-reg-shim publishes each node mode 0666, so no root is needed
  int fd = open(dev_path, O_RDWR);
  if (fd < 0) {
    fprintf(stderr, "Failed to open %s: %s\n", dev_path, strerror(errno));
    return NULL;
  }

  // Each pl-reg node exposes exactly one window, mapped at offset 0. Every
  // register window in this design fits in a page, so one page always covers it.
  long page_size = sysconf(_SC_PAGESIZE);
  void *mapped = mmap(NULL, page_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (mapped == MAP_FAILED) {
    fprintf(stderr, "mmap of %s failed: %s\n", dev_path, strerror(errno));
    close(fd);
    return NULL;
  }

  // The mapping outlives the fd
  close(fd);

  if (verbose) printf("Register window %s mapped\n", dev_path);
  return (uint32_t *)mapped;
}
