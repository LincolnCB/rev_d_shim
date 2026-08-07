#include <fcntl.h>    // For open function
#include <stdint.h>   // For uint32_t type
#include <stdbool.h>  // For bool type
#include <stdio.h>    // For printf and perror functions
#include <stdlib.h>   // For NULL definition etc.
#include <sys/mman.h> // For mmap function
#include <unistd.h>   // For sysconf, close functions

// Map a named shim-regs device into userspace as a volatile 32-bit word array.
// Opens `dev_path` (e.g. "/dev/axi_sys_ctrl"), mmaps `wordcount` words at
// offset 0, and returns a pointer to the mapped region. No physical addresses
// appear in userspace; the shim-regs kernel module owns address translation.
// The file descriptor is closed immediately after mmap -- the mapping persists
// independently. Returns NULL on failure.
volatile uint32_t *map_32bit_memory(const char *dev_path, size_t wordcount, const char *name, bool verbose) {

  if (verbose) {
    printf("Mapping device [%s] (%s, %zu words)...\n", name, dev_path, wordcount);
  }

  // Open the shim-regs device node
  int fd = open(dev_path, O_RDWR);
  if (fd < 0) {
    perror("open");
    return NULL;
  }

  // Round the requested byte size up to a page boundary for mmap
  long page_size = sysconf(_SC_PAGESIZE);
  size_t map_size = ((wordcount * 4) + page_size - 1) / page_size * page_size;

  if (verbose) {
    printf("Mapping %zu bytes (%zu pages of %ld bytes)...\n",
           map_size, map_size / page_size, page_size);
  }

  // Map the device's register window at offset 0 directly into this process.
  // After mmap returns, every register access is a single load/store -- no
  // syscall overhead per access.
  volatile uint32_t *mapped = (volatile uint32_t *)mmap(
      NULL, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  // The fd is no longer needed once the mapping is established
  close(fd);

  if (mapped == MAP_FAILED) {
    perror("mmap");
    return NULL;
  }

  if (verbose) {
    printf("Device [%s] mapped (%zu bytes).\n", name, map_size);
  }

  return mapped;
}
