// This is a library for the Maxim MAX31856 thermocouple IC
// http://datasheets.maximintegrated.com/en/ds/MAX31856.pdf
//
// Written by Peter Easton (www.whizoo.com)
// Modify for Photon firmware-based SPI by John Calcote
// Released under CC BY-SA 3.0 license
//
// Look for the MAX31856 breakout boards on www.whizoo.com and eBay (madeatrade)
// http://stores.ebay.com/madeatrade
//
// Library Implementation Details
// ==============================
// DRDY and FAULT lines are not used in this driver. DRDY is useful for low-power mode so samples are only taken when
// needed; this driver assumes power isn't an issue.  The FAULT line can be used to generate an interrupt in the host
// processor when a fault occurs.  This library reads the fault register every time a reading is taken, and will
// return a fault error if there is one.  The MAX31856 has sophisticated usage scenarios involving FAULT.  For
// example, low and high temperature limits can be set, and the FAULT line triggers when these temperatures are
// breached. This is beyond the scope of this sample library.  The assumption is that most applications will be
// polling for temperature readings - but it is good to know these features are supported by the hardware.
//
// The MAX31856 differs from earlier thermocouple IC's in that it has registers that must be configured before
// readings can be taken.  This makes it very flexible and powerful, but one concern is power loss to the IC.  The IC
// should be as close to the cold junction as possible, which might mean there is a cable connecting the breakout
// board to the host processor.  If this cable is disconnected and reconnected (MAX31856 loses power) then the
// registers must be reinitialized.  This library detects this condition and will automatically reconfigure the
// registers.  This simplifies the software running on the host.
//
// A lot of configuration options appear in the .H file.  Of particular note is the line frequency filtering, which
// defaults to 60Hz (USA and others).  If your line voltage is 50Hz you should set CR0_NOISE_FILTER_50HZ.
//
// This library handles the full range of temperatures, including negative temperatures.

#include "MAX31856TC.h"

// Define which pins are connected to the MAX31856.  The DRDY and FAULT outputs
// from the MAX31856 are not used in this library.
MAX31856TC::MAX31856TC(int cs)
{
    _cs = cs;

    // Initialize SPI firmware API
    // SPI mode 1 or 3 required - chip can detect polarity and
    // can work either way but SPI must read on the rising edge
    SPI.begin(_cs);
    SPI.setDataMode(SPI_MODE1);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockSpeed(10, MHZ);

    // Add input pull up to MISO to detect MAX fault
    pinMode(MISO, INPUT_PULLUP);

    // Manage cs line separately
    pinMode(_cs, OUTPUT);

    // Default cs pin state
    digitalWrite(_cs, HIGH);

    // Set up the shadow registers with the default values
    byte reg[NUM_REGISTERS] = {0x00,0x03,0xff,0x7f,0xc0,0x7f,0xff,0x80,0,0,0,0};
    for (int i = 0; i < NUM_REGISTERS; i++)
        _registers[i] = reg[i];
}


// Write the given data to the MAX31856 register
void MAX31856TC::writeRegister(byte registerNum, byte data)
{
    // Sanity check on the register number
    if (registerNum >= NUM_REGISTERS)
        return;

    // Select the MAX31856 chip
    digitalWrite(_cs, LOW);

    // Write the register number, with the MSB set to indicate a write
    SPI.transfer(WRITE_OPERATION(registerNum));

    // Write the register value
    SPI.transfer(data);

    // Deselect MAX31856 chip
    digitalWrite(_cs, HIGH);

    // Save the register value, in case the registers need to be restored
    _registers[registerNum] = data;
}


// Read the thermocouple temperature either in Degree Celsius or Fahrenheit. Internally,
// the conversion takes place in the background within 155 ms, or longer depending on the
// number of samples in each reading (see CR1).
// Returns the temperature, or an error (FAULT_OPEN, FAULT_VOLTAGE or NO_MAX31856)
double	MAX31856TC::readThermocouple(byte unit)
{
    double temperature;
    long data;

    // Select the MAX31856 chip
    digitalWrite(_cs, LOW);

    // Read data starting with register 0x0c
    SPI.transfer(READ_OPERATION(0x0c));

    // Read 4 registers
    data = readData();

    // Deselect MAX31856 chip
    digitalWrite(_cs, HIGH);

    // If there is no communication from the IC then data will be all 1's because
    // of the internal pullup on the data line (INPUT_PULLUP)
    if (data == -1)
        return NO_MAX31856;

    // If the value is zero then the temperature could be exactly 0.000 (rare), or
    // the IC's registers are uninitialized.
    if (data == 0 && verifyMAX31856() == NO_MAX31856)
        return NO_MAX31856;

    // Was there an error?
    if (data & SR_FAULT_OPEN)
        temperature = FAULT_OPEN;
    else if (data & SR_FAULT_UNDER_OVER_VOLTAGE)
        temperature = FAULT_VOLTAGE;
    else {
        // Strip the unused bits and the Fault Status Register
        data = data >> 13;

        // Negative temperatures have been automagically handled by the shift above :-)

        // Convert to Celsius
        temperature = (double)data * 0.0078125;
	
        // Convert to Fahrenheit if desired
        if (unit == FAHRENHEIT)
            temperature = temperature * 9.0 / 5.0 + 32;
    }

    // Return the temperature
    return temperature;
}


// Read the junction (IC) temperature either in Degree Celsius or Fahrenheit.
// This routine also makes sure that communication with the MAX31856 is working and
// will return NO_MAX31856 if not.
double	MAX31856TC::readJunction(byte unit)
{
    double temperature;
    long data, temperatureOffset;

    // Select the MAX31856 chip
    digitalWrite(_cs, LOW);

    // Read data starting with register 8
    SPI.transfer(READ_OPERATION(8));

    // Read 4 registers
    data = readData();

    // Deselect MAX31856 chip
    digitalWrite(_cs, HIGH);

    // If there is no communication from the IC then data will be all 1's because
    // of the internal pullup on the data line (INPUT_PULLUP)
    if (data == -1)
        return NO_MAX31856;

    // If the value is zero then the temperature could be exactly 0.000 (rare), or
    // the IC's registers are uninitialized.
    if (data == 0 && verifyMAX31856() == NO_MAX31856)
        return NO_MAX31856;

    // Register 9 is the temperature offset
    temperatureOffset = (data & 0x00FF0000) >> 16;

    // Is this a negative number?
    if (temperatureOffset & 0x80)
        temperatureOffset |= 0xFFFFFF00;

    // Strip registers 8 and 9, taking care of negative numbers
    if (data & 0x8000)
        data |= 0xFFFF0000;
    else
        data &= 0x0000FFFF;

    // Remove the 2 LSB's - they aren't used
    data = data >> 2;

    // Add the temperature offset to the temperature
    temperature = data + temperatureOffset;

    // Convert to Celsius
    temperature *= 0.015625;
	
    // Convert to Fahrenheit if desired
    if (unit == FAHRENHEIT)
        temperature = temperature * 9.0 / 5.0 + 32;

    // Return the temperature
    return temperature;
}


// When the MAX31856 is uninitialzed and either the junction or thermocouple temperature is read it will return 0.
// This is a valid temperature, but could indicate that the registers need to be initialized.
double MAX31856TC::verifyMAX31856()
{
    long data, reg;

    // Select the MAX31856 chip
    digitalWrite(_cs, LOW);

    // Read data starting with register 0
    SPI.transfer(READ_OPERATION(0));

    // Read 4 registers
    data = readData();

    // Deselect MAX31856 chip
    digitalWrite(_cs, HIGH);

    // If there is no communication from the IC then data will be all 1's because
    // of the internal pullup on the data line (INPUT_PULLUP)
    if (data == -1)
        return NO_MAX31856;

    // Are the registers set to their correct values?
    reg = ((long)_registers[0] << 24) + ((long)_registers[1] << 16) + ((long)_registers[2] << 8) + _registers[3];
    if (reg == data)
        return 0;

    // Communication to the IC is working, but the register values are not correct
    // Select the MAX31856 chip
    digitalWrite(_cs, LOW);

    // Start writing from register 0
    SPI.transfer(WRITE_OPERATION(0));

    // Write the register values
    for (int i=0; i< NUM_REGISTERS; i++)
        SPI.transfer(_registers[i]);

    // Deselect MAX31856 chip
    digitalWrite(_cs, HIGH);

    // For now, return an error but soon valid temperatures will be returned
    return NO_MAX31856;
}


// Read in 32 bits of data from MAX31856 chip. Minimum clock pulse width is 100 ns
// so no delay is required between signal toggles.
long MAX31856TC::readData()
{
    long data = 0;
    data |= (SPI.transfer(0) << 24) & 0xFF000000;
    data |= (SPI.transfer(0) << 16) & 0x00FF0000;
    data |= (SPI.transfer(0) <<  8) & 0x0000FF00;
    data |= (SPI.transfer(0)      ) & 0x000000FF;
    return data;
}

