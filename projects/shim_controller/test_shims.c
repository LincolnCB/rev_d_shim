#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>

#define PI 3.14159265
// 4-bit command word used by DAC to write to channel register, but not update the output: 0b0000 for LTC2656, 0b0001 for AD5676
#define DAC_CMD 0b00010000

typedef union {
  int32_t le_value;
  unsigned char b[4];
} swappable_int32_t;

typedef struct {
  float gradient_sens_x; // [mT/m/A]
  float gradient_sens_y; // [mT/m/A]
  float gradient_sens_z; // [mT/m/A]
  float gradient_sens_z2; // [?/A]
} gradient_spec_t;

typedef struct {
  float gradient_x; // [A]
  float gradient_y; // [A]
  float gradient_z; // [A]
  float gradient_z2; // [A];
} gradient_offset_t;

typedef enum {
        GRAD_ZERO_DISABLED_OUTPUT = 0,
        GRAD_ZERO_ENABLED_OUTPUT,
        GRAD_OFFSET_ENABLED_OUTPUT
} gradient_state_t;

typedef enum {
        GRAD_AXIS_X = 0,
        GRAD_AXIS_Y,
        GRAD_AXIS_Z,
        GRAD_AXIS_Z2
} gradient_axis_t;

typedef struct {
  float val;
} angle_t;


// Function 1
/* generate a gradient waveform that just changes a state 

  events like this need a 30us gate time in the sequence

  Notes about the DAC control:
  In the present OCRA hardware configuration of the AD5781 DAC, the RBUF bit must always be set so
  that it can function. (HW config is as Figure 52 in the datasheet).

*/
void update_shim_waveform_state(volatile uint32_t *shim,gradient_state_t state, unsigned int mode)
{ 
  uint32_t i;
  int32_t ival;
  
  float fLSB = 10.0/((1<<15)-1);
  
  switch(state) {
  default:
  case GRAD_ZERO_DISABLED_OUTPUT:
    // set the DAC register to zero
    shim[0] = 0x00ffffff & (0 | 0x003f0000);
    break;
  case GRAD_ZERO_ENABLED_OUTPUT:
    shim[0] = 0x00ffffff & (0 | 0x003f0000);
    break;
  case GRAD_OFFSET_ENABLED_OUTPUT:
    ival = (int32_t)12000;
    shim[0] = 0x007f0000;
    shim[1] = 0x000f0000;
    shim[2] = 0x000f0000;
    shim[3] = 0x000f0000;
    break;
  }

  if (mode == 0) {
    for (int k=1; k<500; k++) {
      // DAC C => scope channel 1
      shim[4*k] = 0x00020000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/12.5)+16000.0) & 0x0000ffff);
      // DAC A => scope channel 3
      shim[4*k+1] = 0x00000000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/12.5+M_PI/2.0)+16000.0) & 0x0000ffff);
      // DAC E => scope channel 4
      shim[4*k+2] = 0x00040000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/12.5+M_PI)+16000.0) & 0x0000ffff);
      // DAC G => scope channel 2
      shim[4*k+3] = 0x00060000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/12.5+3.0*M_PI/2.0)+16000.0) & 0x0000ffff);
    }
  } else if (mode == 1) {
    for (int k=1; k<500; k++) {
      // DAC C => scope channel 1
      shim[4*k] = 0x00020000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/12.5)+16000.0) & 0x0000ffff);
      // DAC A => scope channel 3
      shim[4*k+1] = 0x00000000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/12.5+M_PI/2.0)+16000.0) & 0x0000ffff);
      // DAC E => scope channel 4
      shim[4*k+2] = 0x00040000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/18.5+M_PI)+16000.0) & 0x0000ffff);
      // DAC G => scope channel 2
      shim[4*k+3] = 0x00060000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/18.5+3.0*M_PI/2.0)+16000.0) & 0x0000ffff);
    }
  } else if (mode == 2) {
    for (int k=1; k<500; k++) {
      // DAC C => scope channel 1
      shim[4*k] = 0x00020000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/12.5)+16000.0) & 0x0000ffff);
      // DAC A => scope channel 3
      shim[4*k+1] = 0x00000000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/12.5+M_PI/2.0)+16000.0) & 0x0000ffff);
      // DAC E => scope channel 4
      shim[4*k+2] = 0x00040000 + ((uint32_t)(16000.0*(float)(k % 100)/100+16000.0) & 0x0000ffff);
      // DAC G => scope channel 2
      shim[4*k+3] = 0x00060000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/60+3.0*M_PI/2.0)+16000.0) & 0x0000ffff);
    }
  } else if (mode == 3) {
    for (int k=1; k<500; k++) {
      // DAC C => scope channel 1
      shim[4*k] = 0x00020000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/25.0)+16000.0) & 0x0000ffff);
      // DAC A => scope channel 3
      shim[4*k+1] = 0x00000000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/25.0+M_PI/2.0)+16000.0) & 0x0000ffff);
      // DAC E => scope channel 4
      shim[4*k+2] = 0x00040000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/25.0+M_PI)+16000.0) & 0x0000ffff);
      // DAC G => scope channel 2
      shim[4*k+3] = 0x00060000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/25.0+3.0*M_PI/2.0)+16000.0) & 0x0000ffff);
    }
  } else if (mode == 4) {
    for (int k=1; k<2000; k++) {
      // DAC A => scope channel 1
      //shim[4*k] = 0x00000000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/25.0)+16000.0) & 0x0000ffff);
      shim[k] = 0x00000000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/25.0)+16000.0) & 0x0000ffff);
      // DAC C => scope channel 3
      //shim[4*k+1] = 0x00020000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/25.0+M_PI/2.0)+16000.0) & 0x0000ffff);
      // DAC E => scope channel 4
      //shim[4*k+2] = 0x00040000 + ((uint32_t)(16000.0*(float)(k % 100)/100+16000.0) & 0x0000ffff);
      // DAC G => scope channel 2
      //shim[4*k+3] = 0x00060000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/60+3.0*M_PI/2.0)+16000.0) & 0x0000ffff);
    }
  } else if (mode == 5) {
    for (int k=1; k<2000; k++) {
      // DAC B 
      shim[4*k] = 0x00010000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/25.0)+16000.0) & 0x0000ffff);
      // DAC D
      shim[4*k+1] = 0x00030000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/25.0+M_PI/2.0)+16000.0) & 0x0000ffff);
      // DAC F
      shim[4*k+2] = 0x00050000 + ((uint32_t)(16000.0*(float)(k % 100)/100+16000.0) & 0x0000ffff);
      // DAC H
      shim[4*k+3] = 0x00070000 + ((uint32_t)(16000.0*sin((float)(k)*M_PI/60+3.0*M_PI/2.0)+16000.0) & 0x0000ffff);
    }
  }
  
}

// Function 2
void clear_shim_waveforms( volatile uint32_t *shim)
{
  for (int k=0; k<65536; k++) {
    shim[k] = 0x0;
  }        
}
 
void sigint_handler(int s){
  fprintf(stderr,"Caught SIGINT signal %d! Shutting down waveform trigger\n",s);
  int fd;
  if((fd = open("/dev/mem", O_RDWR)) < 0) {
    perror("open");
    exit(1);
  }
  volatile uint32_t *dac_ctrl = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40201000);
  volatile uint32_t *dac_enable = ((uint32_t *)(dac_ctrl+3));
  
  *dac_enable = 0x0;
  exit(1); 
}

int main(int argc, char *argv[])
{
  int fd;
  void *cfg;
  volatile uint32_t *slcr,*dac_ctrl,*dac_enable,*shim_memory,*dac_nsamples,*dac_board_offset,*dac_version,*dac_control_register,*dac_trigger_count,*trigger_ctrl,*tc_trigger_count,*trigger_lockout_ptr,*trigger_polarity,*dac_refresh_divider,*trigger_enable;
  unsigned int mode;
  
  if (argc != 3) {
    fprintf(stderr,"Usage: %s mode inputfile\n The mode is an integer, check the code whats valid.\n",argv[0]);
    return -1;
  }

  // not the right conversion tool
  //mode = atoi(argv[1]);
  char *filename = argv[2];
  char *linebuffer;
  unsigned int line_length;
  unsigned int line_counter = 0;
  int **waveform_buffer = NULL;

  linebuffer = (char *)malloc(2048);
  
  FILE *input_file = fopen(filename,"r");
  if(input_file != NULL) {
    do {
      line_length = 2048;
      ssize_t nchars = getline((char **)&linebuffer,&line_length,input_file);
      if(nchars <= 0)
        break;
      if(linebuffer[0] != '#')
        line_counter++;
    } while(1);
    
    fprintf(stdout,"%d waveform samples found !\n",line_counter);
    // check if we have enough memory
    if(line_counter * 32 > 65536) {
      fprintf(stderr,"Not enough block RAM in this FPGA for your file with this software ! Try staying below %d samples.\n", 65536/32);
      exit(-1);
    }
    
    // allocate memory
    waveform_buffer = (int **)malloc(32*sizeof(int *));
    if(waveform_buffer == NULL) {
      fprintf(stderr,"Error allocating waveform memory !\n");
      exit(-1);
    }
    
    for (int k=0; k<32; k++) {
      waveform_buffer[k] = (int *)malloc(line_counter*sizeof(int));
      if(waveform_buffer[k] == NULL) {
        fprintf(stderr,"Error allocating waveform memory !\n");
        exit(-1);
      }
    }
    
    fprintf(stdout,"|"); fflush(stdout);
    rewind(input_file);
    unsigned int lrcounter = 0;
    do {
      line_length = 2048;
      ssize_t nchars = getline((char **)&linebuffer,&line_length,input_file);
      if(nchars <= 0)
        break;
      if(linebuffer[0] == '#')
        continue;

      int val, offset;
      char *linebuffer_p = linebuffer;
      // found a valid line
      for(int k=0; k<32; k++) {
        if(sscanf(linebuffer_p," %d%n", &val, &offset) == 0) {
          fprintf(stderr,"some sort of parsing error !\n");
          fprintf(stderr,"original line: %s\n",linebuffer);
          fprintf(stderr,"line fragment %d parsed: %s\n",k,linebuffer_p);
          exit(-1);
        }
        linebuffer_p = linebuffer_p + offset;
        waveform_buffer[k][lrcounter] = val;
      }
      fprintf(stdout,"."); fflush(stdout);
      lrcounter++;
    } while(1);
    fprintf(stdout,"|"); fflush(stdout);
    
    fprintf(stdout,"\n");
    
    fclose(input_file);
  } else {
    fprintf(stderr,"Cannot open input file %s for reading !\n",filename);
    exit(-1);
  }

  sleep(1);
  
  fprintf(stdout,"Opening /dev/mem...\n"); fflush(stdout);
  if((fd = open("/dev/mem", O_RDWR)) < 0) {
    perror("open");
    return EXIT_FAILURE;
  }
  

  // // Install SIGINT handler
  // fprintf(stdout,"Installing SIGINT handler...\n"); fflush(stdout);
  // struct sigaction sigIntHandler;
  // sigIntHandler.sa_handler = sigint_handler;
  // sigemptyset(&sigIntHandler.sa_mask);
  // sigIntHandler.sa_flags = 0;
  
  // sigaction(SIGINT, &sigIntHandler, NULL);

  sleep(1);

  // set up shared memory (please refer to the memory offset table)
  fprintf(stdout,"Mapping FPGA memory...\n"); fflush(stdout);
  slcr = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0xF8000000);
  cfg = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40200000);
  dac_ctrl = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40201000);
  trigger_ctrl = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40202000);
  /*
    NOTE: The block RAM can only be addressed with 32 bit transactions, so gradient_memory needs to
    be of type uint32_t. The HDL would have to be changed to an 8-bit interface to support per
    byte transactions
  */

  sleep(1);
  
  // shim_memory is now a full 256k
  printf("Mapping shim memory...\n"); fflush(stdout);

  shim_memory = mmap(NULL, 64*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40000000);
  
  sleep(1);

  printf("Clearing shim memory...\n"); fflush(stdout);

  clear_shim_waveforms(shim_memory);
  
  printf("Setup standard memory maps !\n"); fflush(stdout);

  //dac_enable = ((uint32_t *)(cfg + 0));

  sleep(1);
  
  printf("Setting FPGA clock to 50 MHz !\n"); fflush(stdout);
  /* set FPGA clock to 50 MHz */
  slcr[2] = 0xDF0D;
  slcr[92] = (slcr[92] & ~0x03F03F30) | 0x00201400;  
  printf(".... Done !\n"); fflush(stdout);

  sleep(1);

  // Check version etc
  dac_nsamples = ((uint32_t *)(dac_ctrl+0));
  dac_board_offset = ((uint32_t *)(dac_ctrl+1));
  dac_control_register  = ((uint32_t *)(dac_ctrl+2));
  dac_enable = ((uint32_t *)(dac_ctrl+3));
  dac_refresh_divider = ((uint32_t *)(dac_ctrl+4));
  
  dac_version = ((uint32_t *)(dac_ctrl+10));
  dac_trigger_count = ((uint32_t *)(dac_ctrl+9));
  tc_trigger_count = ((uint32_t *)(trigger_ctrl+4));
  trigger_lockout_ptr = ((uint32_t *)(trigger_ctrl + 1));
  trigger_polarity = ((uint32_t *)(trigger_ctrl+2));
  trigger_enable = ((uint32_t *)(trigger_ctrl));
  float lt = atof(argv[1])/20e-6;

  *trigger_lockout_ptr = (unsigned int)(floor(lt));

  sleep(1);
  
  printf("Lockout = %d (%d) FPGA clockcycles\n",*trigger_lockout_ptr,(unsigned int)(floor(lt)));
  *trigger_polarity = 1;
  *trigger_enable = 1;
  
  printf("FPGA version = %08lX\n",*dac_version);

  if(*dac_version != 0xffff0005) {
    printf("This tool only supports FPGA software version 5 or newer!!\n");
    exit(0);
 
  }

  *dac_nsamples = line_counter*8;
  *dac_board_offset = line_counter*8; // the boards copy each other
  int dbo = *dac_board_offset;

  fprintf(stdout,"board offset %d words\n",dbo);
  
  // copy the data in
  for (int sample=0; sample<line_counter; sample++)  {
    for (int channel=0; channel<8; channel++) {
      // board zero
      shim_memory[sample*8+channel] = ((channel & 0xf) << 16) + (waveform_buffer[channel][sample] & 0xffff);
      // board one
      shim_memory[sample*8+channel+dbo] = ((channel & 0xf) << 16) + (waveform_buffer[8+channel][sample] & 0xffff);
      // board two
      shim_memory[sample*8+channel+2*dbo] = ((channel & 0xf) << 16) + (waveform_buffer[16+channel][sample] & 0xffff);
      // board three
      shim_memory[sample*8+channel+3*dbo] = ((channel & 0xf) << 16) + (waveform_buffer[24+channel][sample] & 0xffff);
    }
  }
  
  // set the DAC to external SPI clock, not fully working, so set it to 0x0 (enable is 0x1)
  *dac_control_register = 0x0;

  // set to 50 KHz
  // LCB -- set to 1000 to match Don's divider
  *dac_refresh_divider = 1000;
  
  //update_shim_waveform_state(shim_memory,GRAD_ZERO_ENABLED_OUTPUT,mode);
  *dac_enable = 0x1;
  /*
  usleep(100);
  *dac_enable = 0x0;
  */
  while(1) {
    printf(".... trigger count = %d (tc = %d)!\n",*dac_trigger_count,*tc_trigger_count); fflush(stdout);
    sleep(2);
    
  }
  
  sleep(5);
  *dac_enable = 0x0;
  return EXIT_SUCCESS;
} // End main
