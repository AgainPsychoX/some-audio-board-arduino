#include <Arduino.h>
#include <Wire.h>

class TDA6610
{
protected:
	uint8_t trebleBass = 0b10001000;
	uint8_t address;

public:
	TDA6610(uint8_t address) : address(address) {}

	void initialize()
	{
		// Switch I
		Wire.beginTransmission(address);
		Wire.write(byte(0b111));
		Wire.write(0b11000100);
		Wire.endTransmission();

		// Switch II
		Wire.beginTransmission(address);
		Wire.write(byte(0b000));
		//Wire.write(0b11000010); // samo stereo (tak jak w innym kodzie)
		Wire.write(0b11110010); // stereo + quasi-stereo + basewitch expansion
		Wire.endTransmission();
	}

	/// Set loudspeaker volume left to specified value from 0 (mute) to 55 (max).
	void setLoudspeakerVolumeLeft(uint8_t value)
	{
		Wire.beginTransmission(address);
		Wire.write(byte(0b001));
		Wire.write(value + 0b111);
		Wire.endTransmission();
	}

	/// Set loudspeaker volume right to specified value from 0 (mute) to 55 (max).
	void setLoudspeakerVolumeRight(uint8_t value)
	{
		Wire.beginTransmission(address);
		Wire.write(byte(0b010));
		Wire.write(value + 0b111);
		Wire.endTransmission();
	}

	/// Set headphone volume to specified value from 0 (mute) to 31 (max).
	void setHeadphoneVolume(uint8_t value)
	{
		Wire.beginTransmission(address);
		Wire.write(byte(0b011));
		Wire.write(value);
		Wire.endTransmission();
	}

protected:
	/// Set both treble/bass register to custom (see datasheet).
	void setTrebleBass(uint8_t value)
	{
		Wire.beginTransmission(address);
		Wire.write(byte(0b101));
		Wire.write(value);
		Wire.endTransmission();
	}

public:
	/// Set treble modifier to value from 0 (min) to 15 (max).
	void setTreble(uint8_t value)
	{
		trebleBass &= 0b00001111;
		trebleBass |= value << 4;
		setTrebleBass(trebleBass);
	}

	/// Set bass modifier to value from 0 (min) to 15 (max).
	void setBass(uint8_t value)
	{
		trebleBass &= 0b11110000;
		trebleBass |= value;
		setTrebleBass(trebleBass);
	}

};
