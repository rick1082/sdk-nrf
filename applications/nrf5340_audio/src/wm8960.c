#include "wm8960.h"
#include <stdint.h>
#include <zephyr/drivers/i2c.h>

int wm8960_write_reg(const struct i2c_dt_spec *spec, uint8_t reg, uint16_t value)
{
	int ret;
    uint8_t send_data[3];
    send_data[0] = reg;
    send_data[1] = (reg << 1); 
    
    // Shift value so only 9th bit is still there, plug it into 0th bit of 
    // control_byte_1
    send_data[1] |= (value >> 8); 

    send_data[2] = (uint8_t)(value & 0xFF);
    printk("%X %X %X\n\r", send_data[0], send_data[1], send_data[2]);
	ret = i2c_write_dt(spec, send_data, 3);
	return ret;
}

int wm8960_write_reg_bit(const struct i2c_dt_spec *spec, uint8_t registerAddress, uint8_t bitNumber,
			 bool bitValue)
{
	int ret;
	uint16_t regvalue = 0;

	if (bitValue == 1) {
		regvalue |= (1 << bitNumber); // Set only the bit we want
	} else {
		regvalue &= ~(1 << bitNumber); // Clear only the bit we want
	}
	ret = wm8960_write_reg(spec, registerAddress, regvalue);
	return ret;
}

int wm8960_write_reg_multi_bit(const struct i2c_dt_spec *spec, uint8_t registerAddress,
			       uint8_t settingMsbNum, uint8_t settingLsbNum, uint8_t setting)
{
	int ret;
	uint8_t numOfBits = (settingMsbNum - settingLsbNum) + 1;

	// Get the local copy of the register
	uint16_t regvalue = 0;

	for (int i = 0; i < numOfBits; i++) {
		regvalue &= ~(1 << (settingLsbNum + i)); // Clear bits we care about
	}

	// Shift and set the bits from in incoming desired setting value
	regvalue |= (setting << settingLsbNum);
	ret = wm8960_write_reg(spec, registerAddress, regvalue);
	return ret;
}

int wm8960_reset(const struct i2c_dt_spec *spec)
{
  // Doesn't matter which bit we flip, writing anything will cause the reset
    return wm8960_write_reg_bit(spec, WM8960_REG_RESET, 7, 1);
}

int wm8960_enableVREF(const struct i2c_dt_spec *spec)
{
	return wm8960_write_reg_bit(spec, WM8960_REG_PWR_MGMT_1, 6, 1);
}