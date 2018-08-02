#ifndef _CRC8_H_
#define _CRC8_H_

#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif

#define CRC8_DEFAULTPOLY  7  /**< Default CRC polynomial = X8+X2+X1+1 */

class CRC8 {
public:
    CRC8(uint8_t polynomial = CRC8_DEFAULTPOLY);
    uint8_t  crc8(void);
    uint8_t  crc8(uint8_t data);
    void     crc8Start(uint8_t poly);
private:
    uint8_t  _crc;
    uint8_t  _poly;
};

#endif /* _CRC8_H_ */
