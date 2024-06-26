#ifndef _SGP30_H_
#define _SGP30_H_

#include <stdio.h>
#include <stdint.h>


#define SGP30_ADDR          0x58
#define	SGP30_ADDR_WRITE	(SGP30_ADDR << 1)         //0xB0
#define	SGP30_ADDR_READ		((SGP30_ADDR << 1)|0X01)   //0xB1


#define SGP30_CMD_INIT_AIR_QUALITY 0x2003

#define SGP30_CMD_MEASURE_AIR_QUALITY 0x2008

#define SGP30_CMD_GET_SERIAL_ID  0X3682


int sgp30_init(void);
int sgp30_read(uint16_t* CO2, uint16_t* TVOC);
int sgp30_get_serial_id(uint8_t id[6]);
int sgp30_soft_reset(void);

#endif /* _SGP30_H_ */
