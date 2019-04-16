/* \Brief: RPi expansion board sensor read 
 * \Note: This library might be in a state not allowing to set / collect
 * all the necessary values: please correct on your own
 */


/* Include the external communication definitions: symlink to another 
 * repo or make sure it is identical to the sender board library
 */

//#include "../../msc_thesis_board/inc/comm_defs.h"
#include "comm_defs.h"

//! \Brief: parses the programme options
void parse_opts(int argc, char * argv[]);

//! \Brief: prints the use options
void print_usage(void);

/* \Brief: sets the spi modes and speed
 * \Param: file descriptor of the device used
 */
void set_spi(int fd);

//! \Brief: aborts the process and displays the error
void pabort(const char *s);

/* \Brief: sends the data included in the tx buffer and returns 
 * the received value, single byte
 * 
 * \Param file descriptor of the SPI interface
 * \Param value to be transmitted to external board
 * \Param transmission delay in microseconds
 */
uint8_t send_receive_single(int fd, uint8_t tx, uint16_t delay);