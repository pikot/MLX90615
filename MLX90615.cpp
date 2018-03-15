/*
  MLX90615.h - Library for reading MLX90615 sensor.
*/

#include "MLX90615.h"

//================================================================================
//=====================  MLX90615 Device class functions.  =======================
//================================================================================

/*
 * \brief				MLX90615 Device class constructor.
 * \param [in] i2caddr 	Device adress (default: 0x5B)
 */
MLX90615::MLX90615(uint8_t i2caddr)
{
	_addr = i2caddr;
	_ready = false;
}

/*
 * \brief	Initialize the device and the i2c interface.
 */
boolean MLX90615::begin(void)
{
	_rwError = _pec = _crc8 = 0;
	return _ready = true;
}

/*
 * \brief	Turn the sensor to sleep mode.
 * \remarks
 * \li 		Send command 0xC6 to enter sleep mode.
 * \li 		Keep SCL high during sleep mode
 */
boolean MLX90615::sleep(void)
{
	CRC8 crc(MLX90615_CRC8POLY);

    // Build the CRC-8 of all bytes to be sent.
    crc.crc8(_addr << 1);
    _crc8 = crc.crc8(MLX90615_SLEEP_MODE);

    Serial.print("PEC (Sleep Mode) = ");
    Serial.println(_crc8);

    // Send the slave address then the command.
	Wire.beginTransmission(_addr);
    Wire.write(MLX90615_SLEEP_MODE);

    // Then write the crc and set the r/w error status bits.
    Wire.write(_crc8);
    _rwError |= (1 << Wire.endTransmission(true)) >> 1;

    // Now we need to keep SCL high
    pinMode(A5, OUTPUT);
    pinMode(A4, OUTPUT);
    digitalWrite(A5, HIGH);
    digitalWrite(A4, LOW);

    // Clear r/w errors if using broadcast address.
    if(_addr == MLX90615_BROADCASTADDR) _rwError &= MLX90615_NORWERROR;

	return _sleep = true;
}

/*
 * \brief	Wakes up the sensor from sleep mode.
 * \remarks
 * \li 		Set SCL to LOW for at least t>8ms.
 * \li 		The data will be ready 0.3s after waking up.
 */
boolean MLX90615::wakeUp(void)
{
	pinMode(A5, OUTPUT);
	digitalWrite(A5, LOW);
	delay(10);
	Wire.begin();
	delay(400);
	MLX90615::begin();
	return _ready;
}

/**
 *  \brief             Return a temperature from the specified source in specified units.
 *  \remarks
 *  \li                Temperature is stored in ram as a 16 bit absolute value to a resolution of 0.02K
 *  \li                Linearized sensor die temperature is available as Ta (ambient).
 *  \li                One object temperatures is linearized to the range -38.2C...125C
 *  \param [in] tsrc   Internal temperature source to read, default:OBJECT TEMPERATURE.
 *  \param [in] tunit  Temperature units to convert raw data to, default deg Celsius.
 *  \return            Temperature.
 */
double MLX90615::readTemp(tempSrc_t tsrc, tempUnit_t tunit)
{
	double temp;

	_rwError=0;
	switch(tsrc)
	{
		case MLX90615_SRCA : temp = readRAM(MLX90615_T_AMBIENT); break;
		default: temp = readRAM(MLX90615_T_OBJECT);
	}
	temp *= 0.02;
	switch(tunit)
	{
		case MLX90615_TC : return kelvin2celcius(temp);
		case MLX90615_TF : return celcius2farenheit(kelvin2celcius(temp));
	}
	return temp;
}

/**
 *  \brief            Convert temperature in degrees K to degrees C.
 *  \param [in] degK  Temperature in degrees Kelvin.
 *  \return           Temperature in degrees Centigrade.
 */
double MLX90615::kelvin2celcius(double degK) {return degK - 273.15;}

/**
 *  \brief            Convert temperature in degrees C to degrees F.
 *  \param [in] degC  Temperature in degrees Centigrade.
 *  \return           Temperature in degrees Fahrenheit.
 */
double MLX90615::celcius2farenheit(double degC) {return (degC * 1.8) + 32.0;}

/*
 * \brief				Set the emissivity of the object
 * \remarks				The emissivity is stored as a 16 bit integer defined by the following:
 * \n<tt>				emissivity = dec2hex[round(16384*emiss)]<tt>
 * \param [in] emiss 	Physical emissivity value in range 0.1 ... 1.0, default 1.0
 */
void MLX90615::setEmissivity(float emiss)
{
	_rwError = 0;
	uint16_t e = int(emiss * 16384.0 + 0.5);
	if((emiss > 1.0) || (e < 1638)) _rwError |= MLX90615_INVALIDATA;
	else writeEEprom(MLX90615_EMISSIVITY, e);
	/*
	Serial.print("Emissivity");
	Serial.println(e);
	*/
}

/*
 * \brief				Get the emissivity of the object
 * \remarks				The emissivity is stored as a 16 bit integer defined by the following:
 * \n<tt>				emissivity = dec2hex[round(16384*emiss)]<tt>
 * \param [in] emiss 	Physical emissivity value in range 0.1 ... 1.0, default 1.0
 */
float MLX90615::getEmissivity(void)
{
	_rwError = 0;
	uint16_t emissivity = readEEprom(MLX90615_EMISSIVITY);
	if(_rwError) return (float)1.0;
	return (float)emissivity / 16384.0;
}

/*
 *  \brief            Set the coefficients of the IIR digital filter.
 *  \remarks          The IIR digital filter coefficients are set by the 14,13 and 12 bits of ConfigRegister
 *  \n                The value of the coefficients is set as follows:
 *  \n <tt> \verbatim
 csb = 0   seq = 0 0 0 	FORBIDDEN SEQUENCE !!!
       1         0 0 1		a1 = 1      a2 = 0 (IIR bypassed)
       2         0 1 0	 		 0.5         0.5
       3         0 1 1		 	 0.333(3)    0.666(6)
       4         1 0 0 		     0.25        0.75
       5         1 0 1	 		 0.2         0.8
       6         1 1 0	 		 0.166(6)    0.833(3)
       7         1 1 1	 		 0.14286     0.87514 \endverbatim </tt>
 *  \param [in] csb   See page 10 of datasheet. Range 1...7, default = 1 (IIR bypassed)
 */
void MLX90615::setIIR(uint8_t csb)
{
	_rwError = 0;

	// Ensure legal range by clearing all but the LS 3 bits.
    csb &= 7; // 7 = 0111b

    // Prevents forbidden sequence defining default value
    if (csb == 0) csb = 1;

	// Get the current value of ConfigRegister1
	uint16_t reg = readEEprom(MLX90615_CONFIG);

	// Clear bits 14:12, mask in the new value, then write it back.
	if(!_rwError) {
		reg &= 0x8fff;
		reg |= ((uint16_t)csb << 12);
		writeEEprom(MLX90615_CONFIG, reg);
	}
}

/*
 *  \brief            Get the coefficients of the IIR digital filter.
 *  \remarks          The IIR digital filter coefficients are set by the 14,13 and 12 bits of ConfigRegister
 *  \return 		  Filter coefficient table index. Range 1...7
 */
uint8_t MLX90615::getIIR(void)
{
	_rwError = 0;

	// Get the current value of ConfigRegister1
	uint8_t iir = (readEEprom(MLX90615_CONFIG) >> 12) & 7;

	if(_rwError) return 1;
	return iir;
}

/*
 *  \brief            	Set device SMBus adress.
 *  \remarks
 *	\li 				Must be only device on the bus.
 *	\li 				Must power cycle the device after changing adress.
 *  \param [in] addr	New device adress. Range 1...127 (0x01...0x7f)
 */

/*
void MLX90615::setAddr(uint8_t addr)
{
	_rwError = 0;

	// It isassumedwedonot know the existing slave adress sothe broadcast adress is used
	// First ensure the new adress is in legal range (1...127)
	if(addr &= 0x7f)
	{
		_addr = MLX90615_BROADCASTADDR;
		writeEEprom(MLX90615_ADDR, addr);

		// There will always be a r/w error using the broadcast address so we cannot respond
        // to r/w errors. We must just assume this worked.
        _addr = addr;
	}
	else _rwError |= MLX90615_INVALIDATA;
}*/

/*
 *  \brief            Return the device SMBus address.
 *  \remarks
 *  \li               Must be only device on the bus.
 *  \li               Sets the library to use the new found address.
 *  \return           Device address.
 */
/*
uint8_t MLX90615::getAddr(void) {
    uint8_t tempAddr = _addr;

    _rwError = 0;

    // It is assumed we do not know the existing slave address so the broadcast address is used.
    // This will throw a r/w error so errors will be ignored.
    _addr = MLX90615_BROADCASTADDR;

    // Reload program copy with the existing slave address.
    _addr = lowByte(readEEProm(MLX90615_ADDR));

    return _addr;
}*/

/*
 * \brief				Return a 16 bit value read from EEPROM.
 * \param [in] addr 	Register adress to read from.
 * \return 				Value read from EEPROM.
 */
uint16_t MLX90615::readEEprom(uint8_t addr)
{
	return read16(addr | MLX90615_EEPROMADDR);
}

/*
 * \brief			Write a 16 bit value to EEPROM after first clearing the memory.
 * \remarks
 * \li 				Erase and write time 5ms per manufacter specification.
 * \li 				Manufacter does not specify max or min erase/write times.
 * \param [in] reg 	Adress to write to.
 * \param [in] data Value to write.
 */
void MLX90615::writeEEprom(uint8_t reg, uint16_t data)
{
	uint16_t val;
	reg |= MLX90615_EEPROMADDR; // bitwise OR with register adress to acess EEProm


	// Reads current value and compares to the new one, and do nothing on a match or if there are read errors
	val = read16(reg);
	if ((val != data) && !_rwError)
	{
		// On any R/W errors it is assumed the memory is corrupted.
		// Clear the memory and wait Terase (per manufacturer's documentation).
		write16(reg, 0);
		delay(10);
		if(_rwError) _rwError |= MLX90615_EECORRUPT;

		// Write the data and wait Twrite (per manufacturer's documentation)
		// and set the R/W error status bits.
		write16(reg, data);
		delay(10);
		if(_rwError) _rwError |= MLX90615_EECORRUPT;
	}
}

/*
 * \brief				Return a 16 bit value read from RAM.
 * \param [in] addr 	Register adress to read from.
 * \return 				Value read from RAM.
 */
uint16_t MLX90615::readRAM(uint8_t addr)
{
	return read16(addr | MLX90615_RAMADDR);
}

/*
 *  \brief            Return a 16 bit value read from RAM or EEPROM.
 *  \param [in] cmd   Command to send (register to read from).
 *  \return           Value read from memory.
 */
uint16_t MLX90615::read16(uint8_t cmd)
{
	uint16_t val;
	CRC8 crc(MLX90615_CRC8POLY);

	// Send the slave adress then the command and set any error status bits return by write
	Wire.beginTransmission(_addr);
	Wire.write(cmd);
	_rwError |= (1 << Wire.endTransmission(false)) >> 1;

	/*
	Serial.print("reading error WireEnd");
	Serial.println(_rwError);
	*/

    // Experimentally determined delay to prevent read errors.
	delayMicroseconds(MLX90615_XDLY);

	// Resend slave adress the get the 3 returned bytes.
	Wire.requestFrom(_addr, (uint8_t)3);

	val = Wire.read();
	val |= Wire.read() << 8;

	// Read the PEC (CRC-8 of all bytes)
	_pec = Wire.read();

	// Clear r/w errors if using broadcast address.
    if(_addr == MLX90615_BROADCASTADDR) _rwError &= MLX90615_NORWERROR;

    // Build our own CRC-8 of all received bytes.
    crc.crc8(_addr << 1);
    crc.crc8(cmd);
    crc.crc8((_addr << 1) + 1);
    crc.crc8(lowByte(val));
    _crc8 = crc.crc8(highByte(val));

    // Set error status bit if CRC mismatch.
    if(_crc8 != _pec) _rwError |= MLX90615_RXCRC;

    /*
    Serial.print("reading error CRC8");
	Serial.println(_rwError);
	*/

    return val;
}

/*
 *  \brief            Write a 16 bit value to memory.
 *  \param [in] cmd   Command to send (register to write to).
 *  \param [in] data  Value to write.
 */
void MLX90615::write16(uint8_t cmd, uint16_t data) {
    CRC8 crc(MLX90615_CRC8POLY);

    // Build the CRC-8 of all bytes to be sent.
    crc.crc8(_addr << 1);
    crc.crc8(cmd);
    crc.crc8(lowByte(data));
    _crc8 = crc.crc8(highByte(data));

    // Send the slave address then the command.
    Wire.beginTransmission(_addr);
    Wire.write(cmd);

    /*
    Serial.println(data);
    Serial.println(lowByte(data));
    Serial.println(highByte(data));
    */

    // Write the data low byte first.
    Wire.write(lowByte(data));
    Wire.write(highByte(data));

    // Then write the crc and set the r/w error status bits.
    Wire.write(_pec = _crc8);
    _rwError |= (1 << Wire.endTransmission(true)) >> 1;

    /*
    Serial.print("writting error WireEnd");
	Serial.println(_rwError);
	*/

    // Clear r/w errors if using broadcast address.
    if(_addr == MLX90615_BROADCASTADDR) _rwError &= MLX90615_NORWERROR;
}
