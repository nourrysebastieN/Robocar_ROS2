#include "wesc/wesc.h"

// CRC16-CCITT (XMODEM) : poly 0x1021, valeur initiale 0, sur la charge utile seulement
uint16_t wesc_crc16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0x0000;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;
		for (int j = 0; j < 8; j++)
			if (crc & 0x8000)
				crc = (crc << 1) ^ 0x1021;
			else
				crc <<= 1;
	}
	return crc;
}
