#ifndef __MAIN_H__
#define __MAIN_H__

#define UART_Module		  0
#define GPIO_UART0_TX   16
#define CHECK_ERRORS(x)     		error_handler(x)
#define GET_ARRAY_SIZE( ARRAY )     ( ( sizeof( ARRAY ) ) / ( sizeof( ARRAY[0] ) ) )
	
#define FIRMWARE_NAND_CELL			(0x2000)
extern void inline error_handler(unsigned int  ui32ErrorStatus);

void BoardsInit(void);
void heyos_application_init(void);
#endif
