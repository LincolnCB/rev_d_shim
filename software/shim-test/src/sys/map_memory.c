#include <fcntl.h> // For open function
#include <inttypes.h> // For PRIx32 format specifier
#include <stdint.h> // For uint32_t type
#include <stdbool.h> // For bool type
#include <stdio.h> // For printf and perror functions
#include <stdlib.h> // For exit function and NULL definition etc.
#include <sys/mman.h> // For mmap function
#include <unistd.h> // For sysconf function

// Map a 32-bit memory region
uint32_t *map_32bit_memory(uint32_t base_addr, size_t wordcount, char *name, bool verbose) {

  if (verbose) {
    printf("Mapping memory region [%s] at base address 0x%" PRIx32 " with size %zu bytes...\n", name, base_addr, wordcount * 4);
  }

  // File descriptor for /dev/mem
  int dev_mem_fd;
  
  // Open /dev/mem to access physical memory
  if (verbose) printf("Opening /dev/mem...\n");
  if((dev_mem_fd = open("/dev/mem", O_RDWR)) < 0) {
    perror("open");
    return NULL;
  }

  // Calculate the page size and the number of pages needed
  long page_size = sysconf(_SC_PAGESIZE);
  size_t num_pages = ((wordcount * 4) + page_size - 1) / page_size; // Round up to the nearest page

  // Map the memory region
  if (verbose) printf("Mapping %zu pages of size %ld bytes...\n", num_pages, page_size);
  uint32_t *mapped_memory = (uint32_t *)mmap(NULL, num_pages * page_size, PROT_READ | PROT_WRITE, MAP_SHARED, dev_mem_fd, base_addr);
  
  // Check if the mapping was successful
  if (mapped_memory == MAP_FAILED) {
    perror("mmap");
    close(dev_mem_fd);
    return NULL;
  }

  if (verbose) {
    printf("Memory region %s mapped", name);
  }

  // Close the file descriptor for /dev/mem
  close(dev_mem_fd);

  // Return the pointer to the mapped memory region
  return mapped_memory;
}
