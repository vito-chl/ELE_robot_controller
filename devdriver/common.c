#include "common.h"


/* CRC校验 */
uint16_t get_crc16(uint8_t* ptr, uint8_t len)
{
	uint8_t i;
	uint16_t crc = 0xFFFF;
	if(len == 0) len = 1;
	while(len--)
	{
		crc ^= *ptr;
		for(i=0; i<8; i++)
		{
			if(crc&1)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else 
			{
				crc >>= 1;
			}
		}
		ptr++;
	}
	return crc;
}