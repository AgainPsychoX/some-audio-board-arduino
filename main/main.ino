  
// Ustawienia pinów
constexpr uint8_t pinAudioLeft  = A0;
constexpr uint8_t pinAudioRight = A1;
constexpr uint8_t pinIndexAudioSpectrum = 2; // as index, A2 -> 2
constexpr uint8_t pinButtonPrevious = 3;
constexpr uint8_t pinButtonNext     = 4;
constexpr uint8_t pinButtonEnter    = 2;

// Ustawienia VU metru
#define FULL_VOLUME 100                 // Maksymalna glosność (do skalowania)
#define T_REFRESH   25                  // Odświerzanie opadania (mniej = szybciej)
#define T_PEAKHOLD  (50 * T_REFRESH)    // Czas przed rozpoczęciem opadania

// Ustawienia analizatora spektrum
#define FHT_N 256 // Number of samples (256 is max supported number for ArduinoFHT library).
#define LOG_OUT 1 // Make FHT provide log output (for magnitude calculations).
constexpr uint16_t samples = FHT_N;
constexpr uint8_t bands = 20;
constexpr unsigned int samplingHighFrequency = 20000; // Hz
constexpr unsigned int samplingHighFrequencyPeriod = 1000000 / samplingHighFrequency + 1; // us
constexpr unsigned int samplingLowFrequency = 4000; // Hz
constexpr unsigned int samplingLowFrequencyPeriod = 1000000 / samplingLowFrequency; // us
// Calculator: https://docs.google.com/spreadsheets/d/1KevICgI320T1rRexd8FRuEP9zuVGCDk5/edit
const uint8_t PROGMEM lowFrequencyBinsRangesForBands[0] = {};
const uint8_t PROGMEM highFrequencyBinsRangesForBands[] = {
	2, 2, 2, 3, 3, 4, 4, 4, 4, 5, 5, 6, 7, 7, 8, 9, 9, 12, 12, 16, 17, 19, 20, 23, 24, 31, 32, 36, 37, 48, 49, 57, 58, 71, 72, 89, 90, 111, 111, 127
};
constexpr bool refineBandsExtrapolate = true;
constexpr uint8_t refineBandsExtrapolateFrom = 160;
constexpr uint8_t refineBandsNoiseCutoff = 40;

// Inne ustawienia
#define I2C_ADDRESS_LCD     0x27                // Adres I2C wyświetlacza 2x20.
#define I2C_ADDRESS_TBA6610 0x42                // Adres I2C układu TBA6610.
#define EEPROM_ADDRESS 0                        // Adres do przechowywania ustawień w EEPROM.
constexpr bool showVolumeAsPercent = false;     // Czy pokazywać głośność w procentach? (prawda - true, fałsz - false).
constexpr uint8_t volumeHearableThreshold = 20; // Minimalna słyszalna głośność (0-55). Przy ustaweniu poniżej tej wartości następuje wyciszenie.
constexpr uint8_t volumeBalanceMax = 15;
constexpr unsigned long int menuIdleExitTime = 4000; // ms

// Debugowanie
constexpr bool debugSpectrumAnalyzerLoopTime    = false;
constexpr bool debugDisableLowFrequency         = true;
constexpr bool debugDisableHighFrequency        = false;
constexpr bool debugHighFrequencySamplingTime   = false;
constexpr bool debugLowFrequencySamplingTime    = false;
constexpr bool debugBandsRanges                 = false;
constexpr bool debugBandsValues                 = false;
constexpr bool debugBlinkBuiltInLedWhenSampling = false;

////////////////////////////////////////////////////////////////////////////////

#include <float.h>
#include <FHT.h> 
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include "TDA6610.hpp"

unsigned long debugTimeStart;
unsigned long debugTimeEnd;

LiquidCrystal_I2C lcd(I2C_ADDRESS_LCD, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

TDA6610 tba6610(I2C_ADDRESS_TBA6610);

enum class Mode : uint8_t {
	VUMeter,
	SpectrumAnalyzer,
	Count
};

typedef struct settings_t {
	Mode mode;
	uint8_t pattern;
	uint8_t volume;
	int8_t volumeBalance;
	uint8_t bass;
	uint8_t treble;
} settings_t;

settings_t settings;

unsigned long lastUpdateTime = 0;

static_assert(
	bands == (sizeof(lowFrequencyBinsRangesForBands) + sizeof(highFrequencyBinsRangesForBands)) / 2, 
	"Number of bands is not equal to total bands by low/high frequency bins ranges to bands."
);

uint8_t bandsValues[bands];

////////////////////////////////////////////////////////////////////////////////

void setVolumeLeftRightFromVolumeAndBalance()
{
	const uint8_t balanceAbs = abs(settings.volumeBalance);
	const uint8_t louder = constrain(settings.volume + (balanceAbs + 1) / 2, 0, 55);
	const uint8_t quieter = constrain(settings.volume - balanceAbs / 2, 0, 55);

	if (settings.volumeBalance < 0) {
		tba6610.setLoudspeakerVolumeLeft(louder);
		tba6610.setLoudspeakerVolumeRight(quieter);
	}
	else {
		tba6610.setLoudspeakerVolumeLeft(quieter);
		tba6610.setLoudspeakerVolumeRight(louder);
	}
}

inline void lcd_print_spaces() {
    lcd.print(F("                    "));
}

const uint8_t PROGMEM lcdChars_VUMeter[] = {
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
	0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18,
	0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x1C,
	0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E, 0x1E,
	0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08,
	0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
	0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02, 0x02,
	0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
};

inline void setupLCDChars(const uint8_t* chars) {
	for (uint8_t j = 0; j < 8; j++) {
		lcd.createChar_P(j, chars + j * 8);
	}
}

const uint8_t PROGMEM lcdChars_spectrumAnalyzer_pattern_0[] = {
	0, 0, 0, 0, 0, 0, 0, 31,
	0, 0, 0, 0, 0, 0, 31, 31,
	0, 0, 0, 0, 0, 0, 31, 31,
	0, 0, 0, 0, 31, 0, 31, 31,
	0, 0, 0, 31, 31, 0, 31, 31,
	0, 0, 0, 31, 31, 0, 31, 31,
	0, 31, 0, 31, 31, 0, 31, 31,
	31, 31, 0, 31, 31, 0, 31, 31,
};
const uint8_t PROGMEM lcdChars_spectrumAnalyzer_pattern_1[] = {
	0, 0, 0, 0, 0, 0, 0, 27,
	0, 0, 0, 0, 0, 0, 27, 27,
	0, 0, 0, 0, 0, 0, 27, 27,
	0, 0, 0, 0, 27, 0, 27, 27,
	0, 0, 0, 27, 27, 0, 27, 27,
	0, 0, 0, 27, 27, 0, 27, 27,
	0, 27, 0, 27, 27, 0, 27, 27,
	27, 27, 0, 27, 27, 0, 27, 27,
};
const uint8_t PROGMEM lcdChars_spectrumAnalyzer_pattern_2[] = {
	0, 0, 0, 0, 0, 0, 0, 14,
	0, 0, 0, 0, 0, 0, 14, 14,
	0, 0, 0, 0, 0, 0, 14, 14,
	0, 0, 0, 0, 14, 0, 14, 14,
	0, 0, 0, 14, 14, 0, 14, 14,
	0, 0, 0, 14, 14, 0, 14, 14,
	0, 14, 0, 14, 14, 0, 14, 14,
	14, 14, 0, 14, 14, 0, 14, 14,
};
const uint8_t PROGMEM lcdChars_spectrumAnalyzer_pattern_3[] = {
	0, 0, 0, 0, 0, 0, 0, 31,
	0, 0, 0, 0, 0, 0, 31, 31,
	0, 0, 0, 0, 0, 31, 31, 31,
	0, 0, 0, 0, 31, 31, 31, 31,
	0, 0, 0, 31, 31, 31, 31, 31,
	0, 0, 31, 31, 31, 31, 31, 31,
	0, 31, 31, 31, 31, 31, 31, 31,
	31, 31, 31, 31, 31, 31, 31, 31,
};
const uint8_t PROGMEM lcdChars_spectrumAnalyzer_pattern_4[] = {
	0, 0, 0, 0, 0, 0, 0, 27,
	0, 0, 0, 0, 0, 0, 27, 27,
	0, 0, 0, 0, 0, 27, 27, 27,
	0, 0, 0, 0, 27, 27, 27, 27,
	0, 0, 0, 27, 27, 27, 27, 27,
	0, 0, 27, 27, 27, 27, 27, 27,
	0, 27, 27, 27, 27, 27, 27, 27,
	27, 27, 27, 27, 27, 27, 27, 27,
};
const uint8_t PROGMEM lcdChars_spectrumAnalyzer_pattern_5[] = {
	0, 0, 0, 0, 0, 0, 0, 14,
	0, 0, 0, 0, 0, 0, 14, 14,
	0, 0, 0, 0, 0, 14, 14, 14,
	0, 0, 0, 0, 14, 14, 14, 14,
	0, 0, 0, 14, 14, 14, 14, 14,
	0, 0, 14, 14, 14, 14, 14, 14,
	0, 14, 14, 14, 14, 14, 14, 14,
	14, 14, 14, 14, 14, 14, 14, 14,
};

void setupLCDCharsForSpectrumAnalyzer(uint8_t pattern) {
	const uint8_t* chars = 0;
	switch (pattern) {
		case 0: chars = lcdChars_spectrumAnalyzer_pattern_0; break;
		case 1: chars = lcdChars_spectrumAnalyzer_pattern_1; break;
		case 2: chars = lcdChars_spectrumAnalyzer_pattern_2; break;
		case 3: chars = lcdChars_spectrumAnalyzer_pattern_3; break;
		case 4: chars = lcdChars_spectrumAnalyzer_pattern_4; break;
		case 5: chars = lcdChars_spectrumAnalyzer_pattern_5; break;
	}
	if (chars) {
		setupLCDChars(chars);
	}
}

////////////////////////////////////////////////////////////////////////////////

void setup() {
	// Delay and serial for debugging
	delay(1111);
	Serial.begin(115200);
	Serial.println("start");
	pinMode(LED_BUILTIN, OUTPUT);

	// Prepare pins
	pinMode(pinButtonPrevious, INPUT_PULLUP);
	pinMode(pinButtonNext,     INPUT_PULLUP);
	pinMode(pinButtonEnter,    INPUT_PULLUP);

	// Prepare LCD
	lcd.begin(20, 2);
	lcd.clear();

	// Prepare settings
	if (digitalRead(pinButtonEnter) == LOW) {
		// Using default settings
		settings.mode = Mode::VUMeter;
		settings.pattern = 0;
		settings.volume = 55;
		settings.volumeBalance = 0;
		settings.bass = 0b1000;
		settings.treble = 0b1000;

		// Overwriting EEPROM with default settings
        EEPROM.put(EEPROM_ADDRESS, settings);
	}
	else {
		// Load settings from EEPROM
        EEPROM.get(EEPROM_ADDRESS, settings);
	}

	// Send TBA6610 settings on start
	tba6610.initialize();
	//tba6610.setHeadphoneVolume(31);
	setVolumeLeftRightFromVolumeAndBalance();
	tba6610.setBass(settings.bass);
	tba6610.setTreble(settings.treble);

	// Get starting LCD custom chats
	if (settings.mode == Mode::VUMeter) {
		setupLCDChars(lcdChars_VUMeter);
	}
	else {
		enterSpectrumAnalyzerMode();
	}
}

////////////////////////////////////////////////////////////////////////////////

const uint8_t PROGMEM VUMeter_fill_chars[6] = {0x20, 0x00, 0x01, 0x02, 0x03, 0xFF};
const uint8_t PROGMEM VUMeter_peak_chars[7] = {0x20, 0x00, 0x04, 0x05, 0x06, 0x07, 0x20};

int lmax[2];
int dly[2];

void writeVUMeterBar(int analogValue, int row) {
	const int inp = constrain(map(analogValue, 0, FULL_VOLUME, 0, 1024), 0, 1024);
	const int lev = map(sqrt(inp  * 16), 0, 128, 0, 100);

	for (int i = 1; i < 20; i++) {
		int f = constrain(lev - i * 5, 0, 5);
		int p = constrain(lmax[row] - i * 5, 0, 6);
		if (f) {
			lcd.write(pgm_read_byte(VUMeter_fill_chars + f));
		}
		else {
			lcd.write(pgm_read_byte(VUMeter_peak_chars + p));
		}
	}

	if (lev > lmax[row]) {
		lmax[row] = lev;
		dly[row]  = -(T_PEAKHOLD) / T_REFRESH;
	}
	else {
		if (dly[row] > 0) {
			lmax[row] -= dly[row];
		}

		if (lmax[row] < 0) {
			lmax[row] = 0;
		}
		else {
			dly[row]++;
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

uint8_t ADCSRA_backup;

void enterSpectrumAnalyzerMode()
{
	setupLCDCharsForSpectrumAnalyzer(settings.pattern);	

	// Setup analog to digital converter
	{
		// Backup ADCSRA
		ADCSRA_backup = ADCSRA;

		ADMUX = (
			0 << REFS1 | 1 << REFS0 | // Use internal voltage reference (5V)
			1 << ADLAR |              // Use left justified value
			pinIndexAudioSpectrum     // Index of ADC pin to be used
		);
		ADCSRB &= ~(1 << ADTS0 | 1 << ADTS1 | 1 << ADTS2); // Free running mode
		ADCSRA = (
			1 << ADEN  | // Enable ADC 
			1 << ADSC  | // Start conversion
			1 << ADATE | // Free running mode
			0 << ADIF  | // Clear ready flag
			0 << ADIE  | // Enable ADC ISR? No.
			1 << ADPS2 | 0 << ADPS1 | 0 << ADPS0 // Use 16 division factor prescaler
		);

		// Disable digital mode for used analog pin to reduce noise.
		DIDR0 |= 1 << pinIndexAudioSpectrum;

		// Collect dummy sample
		collectSampleForFHT();
	}
}

void exitSpectrumAnalyzerMode()
{
	// Reenable digital mode for used analog pin
	DIDR0 &= ~(1 << pinIndexAudioSpectrum);

	// Revert ADCSRA
	ADCSRA = ADCSRA_backup;
}

inline int collectSampleForFHT()
{
	int value;

	// Wait until value is ready
	while ((ADCSRA & (1 << ADIF)) == 0);
	// ADCSRA |= (1 << ADIF);

	// Proper order while reading ADCL & ADCH is required.
	const uint8_t low = ADCL;
	const uint8_t high = ADCH;
	value = (high << 8) | low;

	if (debugBlinkBuiltInLedWhenSampling) {
		PORTB ^= (1 << 5);
	}

	return value;
}

uint8_t condenseMaxValueForFHTBinsRange(const uint16_t first, const uint16_t last)
{
	uint8_t maxValue = 0;
	for (uint16_t i = first; i <= last; i++) {
		if (maxValue < fht_log_out[i]) {
			maxValue = fht_log_out[i];
		}
	}
	return maxValue;
};

inline void loopSpectrumAnalyzerMode()
{
	if (!debugDisableLowFrequency) {
		// Collect the low frequency samples
		{
			if (debugLowFrequencySamplingTime) {
				debugTimeStart = micros();
			}
			for (uint16_t i = 0; i < samples; i++) {
				fht_input[i] = collectSampleForFHT();

				// Delay a bit to match required sampling frequency
				_delay_us(samplingLowFrequencyPeriod - 3);
			}
			if (debugLowFrequencySamplingTime) {
				debugTimeEnd = micros();
				Serial.print(F("LFST: "));
				Serial.println(debugTimeEnd - debugTimeStart);
			}
		}

		// Compute FHT for low frequencies
		fht_window();
		fht_reorder();
		fht_run();
		fht_mag_log();

		// Process results for low frequencies into low band values to display
		{
			if (debugBandsRanges) {
				Serial.println(F("LBR: "));
			}
			constexpr uint16_t from = 0;
			constexpr uint16_t to = from + sizeof(highFrequencyBinsRangesForBands) / 2;
			const uint8_t* address = lowFrequencyBinsRangesForBands;
			for (uint16_t i = from; i < to; i++) {
				const uint8_t first = pgm_read_byte(address);
				const uint8_t last = pgm_read_byte(address + 1);
				address += 2;
				bandsValues[i] = condenseMaxValueForFHTBinsRange(first, last);
				if (debugBandsRanges) {
					Serial.print(i);
					Serial.print(": ");
					Serial.print(first);
					Serial.print('-');
					Serial.println(last);
				}
			}
		}
	}

	if (!debugDisableHighFrequency) {
		// Collect the high frequency samples
		{
			if (debugHighFrequencySamplingTime) {
				debugTimeStart = micros();
			}
			for (uint16_t i = 0; i < samples; i++) {
				fht_input[i] = collectSampleForFHT();

				// Delay a bit to match required sampling frequency
				_delay_us(samplingHighFrequencyPeriod - 3);
			}
			if (debugHighFrequencySamplingTime) {
				debugTimeEnd = micros();
				Serial.print(F("HFST: "));
				Serial.println(debugTimeEnd - debugTimeStart);
			}
		}

		// Compute FHT for high frequencies
		fht_window();
		fht_reorder();
		fht_run();
		fht_mag_log();

		// Process results for high frequencies into high band values to display
		{
			if (debugBandsRanges) {
				Serial.println(F("HBR: "));
			}
			constexpr uint16_t from = sizeof(lowFrequencyBinsRangesForBands) / 2;
			constexpr uint16_t to = from + sizeof(highFrequencyBinsRangesForBands) / 2;
			const uint8_t* address = highFrequencyBinsRangesForBands;
			for (uint16_t i = from; i < to; i++) {
				const uint8_t first = pgm_read_byte(address);
				const uint8_t last = pgm_read_byte(address + 1);
				address += 2;
				bandsValues[i] = condenseMaxValueForFHTBinsRange(first, last);
				if (debugBandsRanges) {
					Serial.print(i);
					Serial.print(": ");
					Serial.print(first);
					Serial.print('-');
					Serial.println(last);
				}
			}
		}
	}

	// Refine values
	{
		if (refineBandsNoiseCutoff != 0) {
			for (uint8_t x = 0; x < bands; x++) {
				if (bandsValues[x] <= refineBandsNoiseCutoff) {
					bandsValues[x] = 0;
				}
				else {
					bandsValues[x] -= refineBandsNoiseCutoff;
				}
			}
		}
		if (refineBandsExtrapolate) {
			for (uint8_t x = 0; x < bands; x++) {
				const float ratio = static_cast<float>(bandsValues[x]) / refineBandsExtrapolateFrom;
				constexpr uint8_t maxGain = 255 - refineBandsExtrapolateFrom;
				const uint16_t value = bandsValues[x] + maxGain * ratio * ratio;
				if (bandsValues[x] > 255) {
					bandsValues[x] = 255;
				}
				else {
					bandsValues[x] = value;
				}
			}
		}
	}

	// Display the band values
	if (debugBandsValues) {
		Serial.print(F("BV: "));
	}
	for (uint8_t x = 0; x < bands; x++) {  
		if (debugBandsValues) {
			Serial.print(bandsValues[x]);
			Serial.print(' ');
		}

		lcd.setCursor(x, 0);
		int level = constrain(map(bandsValues[x], 0, 255, 0, 15), 0, 15);

		if (level > 7) {
			lcd.write((uint8_t)level - 8);
			lcd.setCursor(x, 1);
			lcd.write(7);
		} 
		else {
			lcd.print(' ');
			lcd.setCursor(x, 1);	
			lcd.write((uint8_t)level);
		}
	}
	if (debugBandsValues) {
		Serial.println();
	}

	// Calculate uptime for debugging
	if (debugSpectrumAnalyzerLoopTime) {
		unsigned long int now = micros();
		Serial.print(F("SALT: "));
		Serial.print(now - lastUpdateTime);
		Serial.println(F("us"));
		lastUpdateTime = micros();
	}
}

////////////////////////////////////////////////////////////////////////////////

void loop() {
	switch (settings.mode) {
		////////////////////////////////////////////////////////////////////////////////
		case Mode::VUMeter: {
			unsigned long now = millis();
			if (millis() - lastUpdateTime > T_REFRESH) {
				lastUpdateTime = now;

				lcd.setCursor(0, 0);
				lcd.write('L');
				writeVUMeterBar(analogRead(pinAudioLeft), 0);

				lcd.setCursor(0, 1);
				lcd.write('R');
				writeVUMeterBar(analogRead(pinAudioRight), 1);
			}

			if (digitalRead(pinButtonPrevious) == LOW) {
				delay(333);
				settings.mode = static_cast<Mode>((static_cast<uint8_t>(settings.mode) + 1) % static_cast<uint8_t>(Mode::Count));
				settings.pattern = 0;
				enterSpectrumAnalyzerMode();
			}
			break;
		}
		////////////////////////////////////////////////////////////////////////////////
		case Mode::SpectrumAnalyzer: {
			loopSpectrumAnalyzerMode();

			// Handle switching to next mode
			if (digitalRead(pinButtonPrevious) == LOW) {
				delay(333);
				settings.pattern += 1;
				if (settings.pattern <= 5) {
					setupLCDCharsForSpectrumAnalyzer(settings.pattern);
				}
				else {
					exitSpectrumAnalyzerMode();
					settings.mode = static_cast<Mode>((static_cast<uint8_t>(settings.mode) + 1) % static_cast<uint8_t>(Mode::Count));
					settings.pattern = 0;
					setupLCDChars(lcdChars_VUMeter);
				}
			}
			break;
		}
		////////////////////////////////////////////////////////////////////////////////
		default:
			break;
	}

	// If enter pressed, enter menu
	if (digitalRead(pinButtonEnter) == LOW) {
		// Wait for button release
		while (digitalRead(pinButtonEnter) == LOW);

		// Enter menu
		menu();
	}
}

void menu()
{
	uint8_t menuIndex = 0;
	bool changingValue = false;
	bool inMenu = true;
	lastUpdateTime = millis();

	// Menu loop
	while (inMenu) {
		// Drawing menu
		lcd.setCursor(0, 0);
		switch (menuIndex) {
			case 0:
			case 1:
				//////////////////// 0 ////////////////////
				lcd.print(menuIndex == 0 ? '>' : ' ');
				lcd.print(F("  Glosnosc "));
				lcd.setCursor(12, 0);
				if (menuIndex != 0 || millis() % 800 > 200 || !changingValue) {
					if (showVolumeAsPercent) {
						lcd.print(round((float)(settings.volume - volumeHearableThreshold) / (55 - volumeHearableThreshold) * 100));
						lcd.print('%');
					}
					else {
						lcd.print(settings.volume - volumeHearableThreshold);
					}
				}
				lcd_print_spaces();
				//////////////////// 1 ////////////////////
				lcd.setCursor(0, 1);
				lcd.print(menuIndex == 1 ? '>' : ' ');
				lcd.print(F("  Balans LP "));
				lcd.setCursor(13, 1);
				if (menuIndex != 1 || millis() % 800 > 200 || !changingValue) {
					lcd.print(settings.volumeBalance);
				}
				lcd_print_spaces();
				break;
			case 2:
			case 3:
				//////////////////// 2 ////////////////////
				lcd.print(menuIndex == 2 ? '>' : ' ');
				lcd.print(F("  Bass    "));
				lcd.setCursor(11, 0);
				if (menuIndex != 2 || millis() % 800 > 200 || !changingValue) {
					lcd.print(settings.bass);
					// lcd.print(round((float)settings.bass / 15 * 50));
					// lcd.print('%');
				}
				lcd_print_spaces();
				//////////////////// 3 ////////////////////
				lcd.setCursor(0, 1);
				lcd.print(menuIndex == 3 ? '>' : ' ');
				lcd.print(F("  Sopran  "));
				lcd.setCursor(11, 1);
				if (menuIndex != 3 || millis() % 800 > 200 || !changingValue) {
					lcd.print(settings.treble);
					// lcd.print(round((float)settings.treble / 15 * 50));
					// lcd.print('%');
				}
				lcd_print_spaces();
				break;
		}

		// Handle buttons
		if (digitalRead(pinButtonPrevious) == LOW) {
			lastUpdateTime = millis();
			if (changingValue) {
				switch (menuIndex) {
					case 0: 
						if (settings.volume < 55) {
							settings.volume += 1;
							setVolumeLeftRightFromVolumeAndBalance();
						}
						break;
					case 1: 
						if (settings.volumeBalance < volumeBalanceMax) {
							settings.volumeBalance += 1;
							setVolumeLeftRightFromVolumeAndBalance();
						}
						break;
					case 2: 
						if (settings.bass < 15) {
							settings.bass += 1;
							tba6610.setBass(settings.bass);
						}
						break;
					case 3:
						if (settings.treble < 15) {
							settings.treble += 1;
							tba6610.setTreble(settings.treble);
						}
						break;
				}
				delay(100);
			}
			else {
				if (menuIndex < 3) {
					menuIndex += 1;
				}
				else {
					menuIndex = 0;
				}
				delay(333);
			}
		}
		else if (digitalRead(pinButtonNext) == LOW) {
			lastUpdateTime = millis();
			if (changingValue) {
				switch (menuIndex) {
					case 0: 
						if (settings.volume > volumeHearableThreshold) {
							settings.volume -= 1;
							setVolumeLeftRightFromVolumeAndBalance();
						}
						else {
							// Make sure its muted
							tba6610.setLoudspeakerVolumeLeft(0);
							tba6610.setLoudspeakerVolumeRight(0);
						}
						break;
					case 1: 
						if (settings.volumeBalance > -volumeBalanceMax) {
							settings.volumeBalance -= 1;
							setVolumeLeftRightFromVolumeAndBalance();
						}
						break;
					case 2: 
						if (settings.bass > 0) {
							settings.bass -= 1;
							tba6610.setBass(settings.bass);
						}
						break;
					case 3:
						if (settings.treble > 0) {
							settings.treble -= 1;
							tba6610.setTreble(settings.treble);
						}
						break;
				}
				delay(100);
			}
			else {
				if (menuIndex > 0) {
					menuIndex -= 1;
				}
				else {
					menuIndex = 3;
				}
				delay(333);
			}
		}
		else if (digitalRead(pinButtonEnter) == LOW) {
			lastUpdateTime = millis();
			changingValue = !changingValue;
			delay(333);
		}
		// Idle
		else {
			if (menuIdleExitTime != 0) {
				// Save and close if idle for sometime
				if (millis() - lastUpdateTime > menuIdleExitTime) {
					// Save settings
					EEPROM.put(EEPROM_ADDRESS, settings);

					// Exit menu loop
					inMenu = false;
				}
			}

			delay(50);
		}
	}
}
