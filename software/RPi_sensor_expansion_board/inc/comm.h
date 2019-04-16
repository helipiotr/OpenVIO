/* comm.h
 *  \Brief: Library supporting the external communication with the main unit
 *  \Note: This library manages the communication with the main unit,
 *  which allows to abstract its communication. It also configures the
 *  peripherals.
 */

#ifndef COMM_H_
#define COMM_H_

#include "stm32f4xx.h"
#include "comm_defs.h"


//! \Brief: Set up the communication with the main unit
s8 comm_setup(void);

//! \Brief: Get the required operation mode from the main unit
u8 comm_get_mode(void);

//! \Brief: Send an error message to the main unit
s8 comm_send_error(void);

//! \Brief: Set SPI mode to full duplex using two lines, as slave
void comm_set_SPI_2Lines_FullDuplex(void);

//! \Brief: Set SPI mode to just sending data using one line, as slave
void comm_set_SPI_1Line_Tx(void);

#endif /* COMM_H_ */
