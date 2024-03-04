#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#define TEMPERATURE_TABLE_BITS		10
#define TEMPERATURE_TABLE_ENTRIES	46
#define TEMPERATURE_TABLE_COUNT		1

#define TEMPERATURE_RAW				0
#define TEMERATURE_TEMP				1

const int16_t temperatureTables[TEMPERATURE_TABLE_ENTRIES][2][TEMPERATURE_TABLE_COUNT] PROGMEM = {
	{1, 938},
	{31, 314},
	{41, 290},
	{51, 272},
	{61, 258},
	{71, 247},
	{81, 237},
	{91, 229},
	{101, 221},
	{111, 215},
	{121, 209},
	{131, 204},
	{141, 199},
	{151, 195},
	{161, 190},
	{171, 187},
	{181, 183},
	{191, 179},
	{201, 176},
	{221, 170},
	{241, 165},
	{261, 160},
	{281, 155},
	{301, 150},
	{331, 144},
	{361, 139},
	{391, 133},
	{421, 128},
	{451, 123},
	{491, 117},
	{531, 111},
	{571, 105},
	{611, 100},
	{681, 90},
	{711, 85},
	{811, 69},
	{831, 65},
	{881, 55},
	{901, 51},
	{941, 39},
	{971, 28},
	{981, 23},
	{991, 17},
	{1001, 9},
	{1021, -27},
	{1023, -200}
};

#endif