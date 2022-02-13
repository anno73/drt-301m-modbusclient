
#include <modbus/modbus.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <time.h>
#include <string.h>
#include <math.h>

/*
DRT-301M Multi Tariff Energy Meter with MODBUS RTU

COM parameter Setting:
Default: 1200bps, 8bits, EVEN, 1Stop bit
2400, 4800, 9600, 19200 Optional

meter default / slave address ist: 01


ToDo:
	* prevent multiple instances from interfering
	* prefix output with timestamp of now
	* several commandline options
	* proper error handling 
	* proper error reporting

	* Verbose structure bit select
		2x MODBUS IO
		Commandline Option Parsing
		Intra Function logging

*/

modbus_t *ctx;

typedef struct {
	uint16_t regNr;
	uint16_t regLen;
	uint8_t regType;
	int regBase10;
	const char * unitStr;
	const char * descStr;
} regDef_s_t;

regDef_s_t regDef[] = {
	  { 0x0010,	2,	1,	 0,	"V",	"Voltage L1" }
	, { 0x0012,	2,	1,	 0,	"V",	"Voltage L2" }
	, { 0x0014,	2,	1,	 0,	"V",	"Voltage L3" }
	, { 0x004E,	2,	1,	 0,	"Hz",	"Frequency" }		// !
	, { 0x0050,	2,	2,	-2,	"A",	"Current L1" }
	, { 0x0052,	2,	2,	-2,	"A",	"Current L2" }
	, { 0x0054,	2,	2,	-2,	"A",	"Current L3" }
	, { 0x0056,	2,	2,	-2,	"A",	"Current N" }
	, { 0x0090,	2,	2,	-4,	"kW",	"Power L1" }
	, { 0x0092,	2,	2,	-4,	"kW",	"Power L2" }
	, { 0x0094,	2,	2,	-4,	"kW",	"Power L3" }
	, { 0x0096,	2,	2,	-4,	"kW",	"Power Total" }
	, { 0x00D0,	2,	2,	-4,	"kVA",	"Apparent Power L1" }
	, { 0x00D2,	2,	2,	-4,	"kVA",	"Apparent Power L2" }
	, { 0x00D4,	2,	2,	-4,	"kVA",	"Apparent Power L3" }
	, { 0x00D6,	2,	2,	-4,	"kVA",	"Apparent Power Total" }
	, { 0x0110,	2,	2,	-2,	"kvar",	"Reactive Power L1" }
	, { 0x0112,	2,	2,	-2,	"kvar",	"Reactive Power L2" }
	, { 0x0114,	2,	2,	-2,	"kvar",	"Reactive Power L3" }
	, { 0x0116,	2,	2,	-2,	"kvar",	"Reactive Power Total" }
	, { 0x0150,	2,	2,	-3,	"cos phi",	"Power Factor L1" }
	, { 0x0152,	2,	2,	-3,	"cos phi",	"Power Factor L2" }
	, { 0x0154,	2,	2,	-3,	"cos phi",	"Power Factor L3" }
	, { 0x0156,	2,	2,	-3,	"cos phi",	"Power Factor Total" }
	, { 0x0160,	2,	2,	-2,	"kWh",	"Import Energy" }
	, { 0x0166,	2,	2,	-2,	"kWh",	"Export Energy" }
	, { 0x07D0,	2,	2,	-2,	"kWh",	"Import Energy Rate 1" }
	, { 0x07D2,	2,	2,	-2,	"kWh",	"Import Energy Rate 2" }
	, { 0x07D4,	2,	2,	-2,	"kWh",	"Import Energy Rate 3" }
	, { 0x07D6,	2,	2,	-2,	"kWh",	"Import Energy Rate 4" }
	, { 0x08D0,	2,	2,	-2,	"kWh",	"Export Energy Rate 1" }
	, { 0x08D2,	2,	2,	-2,	"kWh",	"Export Energy Rate 2" }
	, { 0x08D4,	2,	2,	-2,	"kWh",	"Export Energy Rate 3" }
	, { 0x08D6,	2,	2,	-2,	"kWh",	"Export Energy Rate 4" }
	, { 0xF000,	4,	3,	 0,	"",		"Time/Date" }
    , { 0xF111,	10,	4,	-2,	"kWh",	"Last 1 month positive Energy" }
    , { 0xF121,	10,	4,	-2,	"kWh",	"Last 2 month positive Energy" }
    , { 0xF131,	10,	4,	-2,	"kWh",	"Last 3 month positive Energy" }
    , { 0xF141,	10,	4,	-2,	"kWh",	"Last 4 month positive Energy" }
    , { 0xF151,	10,	4,	-2,	"kWh",	"Last 5 month positive Energy" }
    , { 0xF161,	10,	4,	-2,	"kWh",	"Last 6 month positive Energy" }
    , { 0xF171,	10,	4,	-2,	"kWh",	"Last 7 month positive Energy" }
    , { 0xF181,	10,	4,	-2,	"kWh",	"Last 8 month positive Energy" }
    , { 0xF191,	10,	4,	-2,	"kWh",	"Last 9 month positive Energy" }
    , { 0xF1A1,	10,	4,	-2,	"kWh",	"Last 10 month positive Energy" }
    , { 0xF1B1,	10,	4,	-2,	"kWh",	"Last 11 month positive Energy" }
    , { 0xF1C1,	10,	4,	-2,	"kWh",	"Last 12 month positive Energy" }
    , { 0xF211,	10,	4,	-2,	"kWh",	"Last 1 month reverse Energy" }
    , { 0xF221,	10,	4,	-2,	"kWh",	"Last 2 month reverse Energy" }
    , { 0xF231,	10,	4,	-2,	"kWh",	"Last 3 month reverse Energy" }
    , { 0xF241,	10,	4,	-2,	"kWh",	"Last 4 month reverse Energy" }
    , { 0xF251,	10,	4,	-2,	"kWh",	"Last 5 month reverse Energy" }
    , { 0xF261,	10,	4,	-2,	"kWh",	"Last 6 month reverse Energy" }
    , { 0xF271,	10,	4,	-2,	"kWh",	"Last 7 month reverse Energy" }
    , { 0xF281,	10,	4,	-2,	"kWh",	"Last 8 month reverse Energy" }
    , { 0xF291,	10,	4,	-2,	"kWh",	"Last 9 month reverse Energy" }
    , { 0xF2A1,	10,	4,	-2,	"kWh",	"Last 10 month reverse Energy" }
    , { 0xF2B1,	10,	4,	-2,	"kWh",	"Last 11 month reverse Energy" }
    , { 0xF2C1,	10,	4,	-2,	"kWh",	"Last 12 month reverse Energy" }
    , { 0xF311,	10,	4,	-4,	"kW",	"Last 1 month positive max Demand" }
    , { 0xF321,	10,	4,	-4,	"kW",	"Last 2 month positive max Demand" }
    , { 0xF331,	10,	4,	-4,	"kW",	"Last 3 month positive max Demand" }
    , { 0xF341,	10,	4,	-4,	"kW",	"Last 4 month positive max Demand" }
    , { 0xF351,	10,	4,	-4,	"kW",	"Last 5 month positive max Demand" }
    , { 0xF361,	10,	4,	-4,	"kW",	"Last 6 month positive max Demand" }
    , { 0xF371,	10,	4,	-4,	"kW",	"Last 7 month positive max Demand" }
    , { 0xF381,	10,	4,	-4,	"kW",	"Last 8 month positive max Demand" }
    , { 0xF391,	10,	4,	-4,	"kW",	"Last 9 month positive max Demand" }
    , { 0xF3A1,	10,	4,	-4,	"kW",	"Last 10 month positive max Demand" }
    , { 0xF3B1,	10,	4,	-4,	"kW",	"Last 11 month positive max Demand" }
    , { 0xF3C1,	10,	4,	-4,	"kW",	"Last 12 month positive max Demand" }
    , { 0xF411,	10,	4,	-4,	"kW",	"Last 1 month reverse max Demand" }
    , { 0xF421,	10,	4,	-4,	"kW",	"Last 2 month reverse max Demand" }
    , { 0xF431,	10,	4,	-4,	"kW",	"Last 3 month reverse max Demand" }
    , { 0xF441,	10,	4,	-4,	"kW",	"Last 4 month reverse max Demand" }
    , { 0xF451,	10,	4,	-4,	"kW",	"Last 5 month reverse max Demand" }
    , { 0xF461,	10,	4,	-4,	"kW",	"Last 6 month reverse max Demand" }
    , { 0xF471,	10,	4,	-4,	"kW",	"Last 7 month reverse max Demand" }
    , { 0xF481,	10,	4,	-4,	"kW",	"Last 8 month reverse max Demand" }
    , { 0xF491,	10,	4,	-4,	"kW",	"Last 9 month reverse max Demand" }
    , { 0xF4A1,	10,	4,	-4,	"kW",	"Last 10 month reverse max Demand" }
    , { 0xF4B1,	10,	4,	-4,	"kW",	"Last 11 month reverse max Demand" }
    , { 0xF4C1,	10,	4,	-4,	"kW",	"Last 12 month reverse max Demand" }

	, { 0xF500,	 2,	5, 	 0,	"",		"Intervals & Times" }
	, { 0xF600,	 0,	6,	 0,	"", 	"!!! Meter Number" }	// ! Not working?
	, { 0xF700,	15,	7,	 0,	"",		"Tariff" }

	, { 0xF800,	 2,	1,	 0,	"Baud",	"!!!Baudrate" }	// ! Baudrate write only?
          
    , { 0xFA01,	10,	4,	-4,	"kW",	"Current month positive max Demand" }
    , { 0xFB01,	10,	4,	-4,	"kW",	"Current month reverse max Demand" }

	, { 0x0000,	0,	0,	 0,	"",		"" }
};

char optSetDate = 0;
char optCheckDate = 0;
int  optSetBaudrate = 0;
char optUnit = 0;
char optTitle = 0;
char optReport = 0;
char optHelp = 0;
char optVerbose = 0;
char *optSerialParms = NULL;
char *optSerialDevice = NULL;
unsigned int *optRegsToDump = NULL;
int countRegs = 0;
struct timeval *optByteTimeout = NULL;
struct timeval *optResponseTimeout = NULL;

struct timeval byteTimeout, responseTimeout;

char dateNow[] = "YYYY-MM-DD hh:mm:ss";

#define defaultSerialDevice		"/dev/ttyUSB0"
#define defaultSerialBaud		1200
#define defaultSerialDataBits	8
#define defaultSerialParity		'E'
#define defaultSerialStopBits	1

char *serialDevice;
int serialBaud;
int serialDataBits;
char serialParity;
int serialStopBits;

/**********************************************************************
**********************************************************************/
#if 0
uint32_t read32(uint16_t addr)
{
	uint16_t tab16[2];
	uint32_t q = 0;
	
	int rc = modbus_read_registers(ctx, addr, 2, tab16);
	
	if (rc == -1) 
	{
		fprintf(stderr, "MODBUS read registers failed for addr %04X: %s\n", addr, modbus_strerror(errno));
		return 0;
	}

	
	fprintf(stderr, "Read32[%04X]: %4X %4X %5d %5d %d\n", addr, tab16[0], tab16[1], tab16[0], tab16[1], (tab16[0] << 16) + tab16[1]);
	
	return q;
}	// read32
#endif

/**********************************************************************
**********************************************************************/
int setDate(modbus_t *_ctx, struct tm *_tm)
{
	struct tm *raw_tm;

	if (_tm)
	{	
		raw_tm = _tm;
	} 
	else
	{	// use now as time
		time_t raw_time; 
		
		time(&raw_time);
		
		raw_tm = localtime(&raw_time);
	}

	raw_tm->tm_year += 1900;
	raw_tm->tm_mon  +=    1;
	
	if (optVerbose > 3)
		printf("Set Time: %04d-%02d-%02d %02d:%02d:%02d\n", raw_tm->tm_year, raw_tm->tm_mon, raw_tm->tm_mday, raw_tm->tm_hour, raw_tm->tm_min, raw_tm->tm_sec);

//	return 0;

	// write to DRT
	// 0xF000: sec, min, hour, week, day, month, year, 20

	uint8_t raw_req[] = { 
		0x01,				// Device ID
		0x10, 				// Write multiple register
		0xF0, 0x00,			// Start address
		0x00, 0x04,			// Quantity of registers
		0x08,				// Byte count
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 
	};		// Date
	
	#define OFFSET	7
	#define INT2BCD(A) ( ((A) / 10 * 16) + ((A) % 10) )
	raw_req[OFFSET + 0] = INT2BCD(raw_tm->tm_sec);
	raw_req[OFFSET + 1] = INT2BCD(raw_tm->tm_min);
	raw_req[OFFSET + 2] = INT2BCD(raw_tm->tm_hour);
	raw_req[OFFSET + 3] = INT2BCD(raw_tm->tm_wday);
	raw_req[OFFSET + 4] = INT2BCD(raw_tm->tm_mday);
	raw_req[OFFSET + 5] = INT2BCD(raw_tm->tm_mon);
	raw_req[OFFSET + 6] = INT2BCD(raw_tm->tm_year - 2000);
	raw_req[OFFSET + 7] = INT2BCD(20);

	uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

	int req_length = modbus_send_raw_request(ctx, raw_req, sizeof(raw_req));
	int res_length = modbus_receive_confirmation(ctx, rsp);

	if (optVerbose > 2)
		printf("0x%04X REQ Length: %d RES Length: %d\n", 0xF000, req_length, res_length);

	if (res_length <= 0)
		return(-1);
		
	if (optVerbose > 2)
	{
		printf("0x%04X Response [", 0xF000);
		for (int i = 0; i < res_length; i++)
		{
			printf("%02X", rsp[i]);
			if (i < res_length - 1)
				printf(" ");
		}
		printf("]\n");
	}

	return 0;
}	// setDate

/**********************************************************************
**********************************************************************/
int dumpRegister(unsigned int reg)
{
	uint16_t dest[256];
	uint8_t *bp = (uint8_t *) dest;
	int rc;
	int i;

	for (i = 0; regDef[i].regNr && (regDef[i].regNr != reg); i++)
			if (optVerbose > 3)
				printf("Search Register Definition: %04X, %d, %d, %d\n", regDef[i].regNr, regDef[i].regLen, regDef[i].regType, regDef[i].regBase10)
	;
	
	if (! regDef[i].regNr)
	{
		printf("Undefined register %04X (%d)\n", reg, i);
		abort();
	}

	int reg_addr = regDef[i].regNr;
	int reg_size = regDef[i].regLen;
	int reg_type = regDef[i].regType;
	int reg_base = regDef[i].regBase10;

	if (optVerbose > 3)
		printf("Found Register Definition: %04X, %d, %d, %d\n", reg_addr, reg_size, reg_type, reg_base);

	if (optVerbose > 3)
		printf("To Dump %04X: reg_def[%d]=%04X, %d, %d, %d\n", reg, i, reg_addr, reg_size, reg_type, reg_base);

	rc = modbus_read_registers(ctx, reg_addr, reg_size, dest);
	if (rc == -1)
	{
		printf("Read register failed: %s\n", modbus_strerror(errno));
		abort();
	}

	unsigned int ul = 0;
	float f = 0;

	switch (reg_type)
	{
	case 0:
		printf("Register disabled\n");

		return(0);
		break;
	case 1:
		if (optVerbose > 3)
			printf("Unsigned Int Register:\n");
			
		ul = (dest[0] << 16) + dest[1];
		
		if (optTitle)
			printf("%s: ", regDef[i].descStr);

		printf("%u", ul);

		if (optUnit)
			printf("%s", regDef[i].unitStr);
		
		printf("\n");
		
		return(0);
		break;
	case 2:
		if (optVerbose > 3)
			printf("Float Register:\n");

		f = dest[0];
		f *= 256 * 256;
		f += dest[1];
//		f *= exp10(reg_base);
		f *= pow(10, reg_base);
		
		if (optTitle)
			printf("%s: ", regDef[i].descStr);
			
		printf("%.*f", abs(reg_base), f);
		
		if (optUnit)
			printf("%s", regDef[i].unitStr);
		
		printf("\n");
		return(0);
		break;
	case 3:
		if (optVerbose > 3)
			printf("Time Register:\n");

		printf("%02X%02X-%02X-%02X %02X:%02X:%02X %02X\n", bp[6], bp[7], bp[4], bp[5], bp[3], bp[0], bp[1], bp[2]);
		return(0);
		break;
	case 4:
		if (optVerbose > 3)
			printf("Rate Summary:\n");

		if (optTitle)
			printf("%s: ", regDef[i].descStr);

		for (int j = 0; j < 8; j += 2)
		{
			f = (dest[j] * 256 * 256 + dest[j + 1]) * pow(10, reg_base);
			printf("%.*f", abs(reg_base), f);
			if (optUnit)
				printf("%s ", regDef[i].unitStr);
		}

		printf("\n");
		return(0);
		break;
	default:
		printf("dumpRegister: Unimplemented Register Type: %d\n", reg_type);
		return(0);
		break;
	}
	
	return(0);
}	// dumpRegister

/**********************************************************************
**********************************************************************/
int setBaudrate(modbus_t * _ctx, int _baudrate)
{
	int value = 0;

	switch (_baudrate)
	{
	case 1200:
		value = 1;
		break;
	case 2400:
		value = 2;
		break;
	case 4800:
		value = 3;
		break;
	case 9600:
		value = 4;
		break;
	default:
		printf("Error setBaudrate: %i Baud not supported.\n", _baudrate);
		return -1;
	}
	
	//0xF800
	// 0x01, 0x10, 0xF8, 0x00, value

	uint8_t raw_req[] = { 
		0x01,				// Device ID
		0x10, 				// Write multiple register
		0xF8, 0x00,			// Start address
		0x00, 0x01,			// Quantity of registers
		0x02,				// Byte count
		0x00, 0x00
	};		// Date
	
	#define OFFSET	7
	raw_req[OFFSET + 0] = 0;
	raw_req[OFFSET + 1] = value;

	uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

	int req_length = modbus_send_raw_request(ctx, raw_req, sizeof(raw_req));
	int res_length = modbus_receive_confirmation(ctx, rsp);

	if (optVerbose > 2)
		printf("0x%04X REQ Length: %d RES Length: %d\n", 0xF000, req_length, res_length);

	if (res_length <= 0)
		return(-1);
		
	if (optVerbose > 2)
	{
		printf("0x%04X Response [", 0xF000);
		for (int i = 0; i < res_length; i++)
		{
			printf("%02X", rsp[i]);
			if (i < res_length - 1)
				printf(" ");
		}
		printf("]\n");
	}

	return 0;
}	// setBaudrate

/**********************************************************************
**********************************************************************/
void dumpRegDef()
{
	for (int i = 0; regDef[i].regNr; i++ )
	{
		printf("0x%04X %4d %4d %4d\t%s\t%s\n",
			regDef[i].regNr,
			regDef[i].regLen,
			regDef[i].regType,
			regDef[i].regBase10,
			regDef[i].unitStr,
			regDef[i].descStr
		);
	}
	exit(0);
}	// dumpRegDef

/**********************************************************************
**********************************************************************/
void usage(void)
{
	printf("usage: \n"
		"	-h			this help\n"
		"	-v [n] [-v ...]		verbose\n"
		"	* -i /dev/ttyUSB0	device\n"
		"	* -s 1200,8,E,1		serial parameter\n"
		"	* -sa nr		slave address\n"
		"	-u			units of measure\n"
		"	-t			title of register\n"
		"	-setDate		set date on energy meter\n"
		"	* -checkDate n		check date on energy meter and report if off more than n sec\n"
		"	-setBaudrate n		set baudrate to either 1200, 2400, 4800, 9600\n"
		"	-l			list register configuration\n"
		"	-r 0x1,0x2,0x3,...	dump register\n"
		"	-bt n			byte timeout [ms]\n"
		"	-rt n			response timeout [ms]\n"
		"	-R n			report n\n"
		"				1 Export Energy\n"
		"				2 Current Volt and Current\n"
		"				3 Power & cos phi\n"
		"				* 4 Monthly reports\n"
		"	For write operations:\n"
		"		Unlock meter - no lock symbol on LCD\n"
		"		Increase timeout values\n"
	);
	
	abort();

}	// help


/**********************************************************************
**********************************************************************/
int main(int argc, char *argv[])
{
	int rc;

	if (argc == 1)
		usage();

	for (int i = 1; i < argc; i++)
	{	// Process commandline parameters
		if (optVerbose > 3)
			printf("Process commandline arg: argc=%d argv[%d]=%s.\n", argc, i, argv[i]);
	
		if (strcmp(argv[i], "-v") == 0)
		{	// Verbose: -v [n]
			optVerbose ++;

			if (argc - i > 1)
			{	// check for optional verbosity level
				i++;
				char *cp = NULL;
				optVerbose = strtol(argv[i], &cp, 0);
				
				if (*cp != 0)
				{
					optVerbose = 0;
				}
			}

			if (! optVerbose)
			{
				optHelp++;
				i = argc;
				break;
			}			
		}

		else if (strcmp(argv[i], "-u") == 0)
			optUnit ++;

		else if (strcmp(argv[i], "-t") == 0)
			optTitle ++;

		else if (strcmp(argv[i], "-l") == 0)
			dumpRegDef();	// does not return

		else if (strcmp(argv[i], "-setDate") == 0)
			optSetDate ++;

		else if (strcmp(argv[i], "-setBaudrate") == 0)
		{
			if (argc - i > 1)
			{
				i++;
				optSetBaudrate = strtol(argv[i], NULL, 0);
			}
			
			if (! optSetBaudrate)
			{
				optHelp++;
				i = argc;
				break;
			}
		}
			
		else if (strcmp(argv[i], "-h") == 0)
			optHelp ++;

		
		else if (strcmp(argv[i], "-s") == 0)
		{	// Set serial parameters
			printf("-s not implemented yet\n"); abort();
			
			if (argc - i > 2)
			{
				optSerialParms = argv[i + 1];
				i++;
			}
			else
			{
				optHelp++;
				i = argc;
			}
		}
		
		else if (strcmp(argv[i], "-i") == 0)
		{	// Set serial device
			if (argc - i > 1)
			{
				i++;
				optSerialDevice = argv[i];
				
				if (optVerbose > 3)
					printf("optSerialDevice: %s\n", optSerialDevice);
			}
			else
				printf("optSerialDevice missing device string.\n");
			
			if (! optSerialDevice)
			{
				optHelp++;
				i = argc;
			}
		}
		
		else if (strcmp(argv[i], "-r") == 0)
		{	// Dump registers
			if (argc - i > 1)
			{
				#define DELIMITER ','
				
				i++;
				
				if (optVerbose > 3)
					printf("optRegsToDump: registers: %s\n", argv[i]);

//				countRegs = 0;
				
				char *cp = argv[i];
				
				cp = strtok(argv[i], ",");		// read man page for features and behaviour (",.." "...," ".,,,.")

				if (cp)		// if /,+/ is given, cp is NULL
				do {
				
					countRegs++;
					long int li = strtol(cp, NULL, 0);	// convert ASCII dec, hex, oct to number
				
					if (optVerbose > 3)
						printf("Token[%d]: %s %d %ld\n", countRegs, cp, strlen(cp), li);

					// extend reg array by one and add new value to end
					unsigned int *ui_p = realloc(optRegsToDump, countRegs * sizeof(*optRegsToDump));
					if (! ui_p)
					{
						printf("optRegsToDump realloc failed\n");
						abort();
					}
					optRegsToDump = ui_p;
					optRegsToDump[countRegs - 1] = (unsigned int) li;
				
				} while ((cp = strtok(NULL, ",")));

				// Dump reg array
				if (optVerbose > 3)
					for (int i = 0; i < countRegs; i++)
						printf("optRegsToDump[%02d] = %5d, %04X\n", i, optRegsToDump[i], optRegsToDump[i]);
				
				if (optVerbose > 3)
					printf("optRegsToDump: %d\n", countRegs);
			}
			else
				printf("optRegsToDump missing device string.\n");
			
			if (! optRegsToDump)
			{
				optHelp++;
				i = argc;
				break;
			}
		}	// -r
		
		else if (strcmp(argv[i], "-R") == 0)
		{	// Print predefined report
			if (argc - i > 1)
			{
				i++;
				optReport = strtol(argv[i], NULL, 0);
			}
			
			if (! optReport)
			{
				optHelp++;
				i = argc;
				break;
			}
		}

		else if (strcmp(argv[i], "-bt") == 0)
		{	// Print predefined report
			if (argc - i > 1)
			{
				i++;
				optByteTimeout = malloc(sizeof(*optByteTimeout));
				if (! optByteTimeout)
				{
					printf("ERROR malloc optByteTimeout failed\n");
					abort();
				}
				int millis = strtol(argv[i], NULL, 0);
				optByteTimeout->tv_sec = millis / 1000;
				optByteTimeout->tv_usec = (millis % 1000) * 1000;	// micro seconds
			}
			
			if (! optByteTimeout)
			{
				optHelp++;
				i = argc;
				break;
			}
		}

		else if (strcmp(argv[i], "-rt") == 0)
		{	// Print predefined report
			if (argc - i > 1)
			{
				i++;
				optResponseTimeout = malloc(sizeof(*optResponseTimeout));
				if (! optResponseTimeout)
				{
					printf("ERROR malloc optResponseTimeout failed\n");
					abort();
				}
				int millis = strtol(argv[i], NULL, 0);
				optResponseTimeout->tv_sec = millis / 1000;
				optResponseTimeout->tv_usec = (millis % 1000) * 1000;	// micro seconds
			}
			
			if (! optResponseTimeout)
			{
				optHelp++;
				i = argc;
				break;
			}
		}

		else
		{	// Unknown option
			printf("Unkown option: %s.\n", argv[i]);
			optHelp++;
			i = argc;
		}
	}	// Process command line arguments

	if (optHelp) usage();

	if (! optSerialParms)
	{	// Set serial default parameters
		serialDevice	= strdup(defaultSerialDevice);
		serialBaud		= defaultSerialBaud;
		serialDataBits	= defaultSerialDataBits;
		serialParity	= defaultSerialParity;
		serialStopBits	= defaultSerialStopBits;
		
		if (optVerbose > 2)
			printf("Serial: set default parameters: %s,%d,%d,%c,%d\n", 
				serialDevice, serialBaud, serialDataBits, serialParity, serialStopBits);
	}
	
	#if 1	// modbus related stuff
	// Create modbus rtu context
//	ctx = modbus_new_rtu("/dev/ttyUSB0", 1200, 'E', 8, 1);
	ctx = modbus_new_rtu(serialDevice, serialBaud, serialParity, serialDataBits, serialStopBits);
	if (ctx == NULL)
	{
		fprintf(stderr, "MODBUS new failed: %s\n", modbus_strerror(errno));
		exit(-1);
	}

	modbus_set_debug(ctx, FALSE);
	if (optVerbose > 1)
		modbus_set_debug(ctx, TRUE);
		
	rc = modbus_connect(ctx);
	if (rc == -1)
	{
		fprintf(stderr, "MODBUS connect failed: %s\n", modbus_strerror(errno));
		abort();
	}

	// Select slave to talk to
	rc = modbus_set_slave(ctx, 1);
	if (rc)
	{
		fprintf(stderr, "MODBUS set slave failed: %d, %s\n", rc, modbus_strerror(errno));
		exit(-1);
	}
	
	if (optByteTimeout)
	{
		// modbus_set_byte_timeout(ctx, optByteTimeout);	####
		modbus_set_byte_timeout(ctx, optByteTimeout->tv_sec, optByteTimeout->tv_usec);
	}

	if (optResponseTimeout)
	{
		// modbus_set_response_timeout(ctx, optResponseTimeout); ####
		modbus_set_response_timeout(ctx, optResponseTimeout->tv_sec, optResponseTimeout->tv_usec);
	}
	
	if (optVerbose > 2)
	{
		uint32_t sec, usec;
		// void modbus_set_byte_timeout(modbus_t *ctx, struct timeval *timeout);
		modbus_get_byte_timeout(ctx, &sec, &usec);
		byteTimeout.tv_sec = sec;
		byteTimeout.tv_usec = usec;
		// modbus_get_response_timeout(ctx, &responseTimeout);	####
		modbus_get_response_timeout(ctx, &sec, &usec);
		responseTimeout.tv_sec = sec;
		responseTimeout.tv_usec = usec;
		
		printf("MODBUS Byte Timeout: %lus %luus\n", byteTimeout.tv_sec, byteTimeout.tv_usec);
		printf("MODBUS Response Timeout: %lus %luus\n", responseTimeout.tv_sec, responseTimeout.tv_usec);
	}
	#endif
	
	if (optReport)
	{
		switch (optReport)
		{
		case 1:
			// kWh
			dumpRegister(0x0160);
			break;
		case 2:
			// Voltage
			dumpRegister(0x0010);
			dumpRegister(0x0012);
			dumpRegister(0x0014);
			// Current
			dumpRegister(0x0050);
			dumpRegister(0x0052);
			dumpRegister(0x0054);
			dumpRegister(0x0056);
			break;
		case 3:
			// Power
			dumpRegister(0x0090);
			dumpRegister(0x0092);
			dumpRegister(0x0094);
			dumpRegister(0x0096);
			// Apparent Power
			dumpRegister(0x00D0);
			dumpRegister(0x00D2);
			dumpRegister(0x00D4);
			dumpRegister(0x00D6);
			// Reactive Power
			dumpRegister(0x0110);
			dumpRegister(0x0112);
			dumpRegister(0x0114);
			dumpRegister(0x0116);
			// Power Factor
			dumpRegister(0x0150);
			dumpRegister(0x0152);
			dumpRegister(0x0154);
			dumpRegister(0x0156);
			break;
		case 4:
			dumpRegister(0xF111);
			break;
		default:
			break;
		}

		exit(0);
	}	// optReport
	
	if (optSetDate)
	{	// Set current date on device
		int rc = setDate(ctx, NULL);
		exit(rc);
	}

	if (optSetBaudrate)
	{	// Set new baudrate on device
		int rc = setBaudrate(ctx, optSetBaudrate);
		exit(rc);
	}
	
	if (optRegsToDump)
	{
		int rc = 0;
	
		for (int i = 0; i < countRegs; i++)
		{
			rc = dumpRegister(optRegsToDump[i]);
//			if (rc) break;
		}
		exit(rc);
	}	// optRegsToDump

	modbus_close(ctx);
	modbus_free(ctx);

	exit(0);
}	// main	

/**********************************************************************
	Some weird test stuff below this point only
**********************************************************************/
//uint8_t *tab_bytes;
//uint16_t tab_word[64];
//uint32_t tab_quad[64];
	
#if 0
	for (int i = 0; reg_def[i]; i += 4)
	{
		uint8_t hi = reg_def[i] >> 8;
		uint8_t lo = reg_def[i] & 0xFF;
		uint8_t len = reg_def[i + 1];
		
		printf("0x%04X Request [%02X:%02X:%4d]\n", reg_def[i], hi, lo, len);
	
		uint8_t raw_req[] = { 0x01, 0x03, 0x00, 0x00 , 0x00, 0x00};
	
		raw_req[2] = hi;
		raw_req[3] = lo;
		raw_req[5] = len;
	
		uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

		int req_length = modbus_send_raw_request(ctx, raw_req, sizeof(raw_req));
		int res_length = modbus_receive_confirmation(ctx, rsp);
	
		printf("0x%04X REQ Length: %d RES Length: %d\n", reg_def[i], req_length, res_length);

		if (res_length > 0)
		{
			printf("0x%04X Response [", reg_def[i]);
			for (int i = 0; i < res_length; i++)
			{
				printf("%02X", rsp[i]);
				if (i < res_length - 1)
					printf(" ");
			}
			printf("]\n");
		}
		else
			abort();
	}
#endif	

#if 0	
	rc = modbus_read_registers(ctx, 0x0010, 2, tab_word);
	if (rc == -1) 
	{
		fprintf(stderr, "MODBUS read registers failed: %s\n", modbus_strerror(errno));
		exit(-1);
	}
	
	printf("Read %d registers.\n", rc);
	
	for (uint8_t i = 0; i < rc; i++)
	{
		printf("%4d : %d\n", i, tab_word[i]);
	}
#endif

#if 0
	read32(0x0010);
	read32(0x0012);
	read32(0x0014);

	read32(0x0030);
	read32(0x0032);
	read32(0x0034);

	read32(0x004E);

	read32(0x0050);
	read32(0x0052);
	read32(0x0054);
	read32(0x0056);
	
	read32(0x0090);
	read32(0x0092);
	read32(0x0094);
	read32(0x0096);

	read32(0x0160);
	read32(0x0166);

	read32(0xF600);
#endif

#if 0	// read 0x0160, 0x0166
	read32(0x0160);
	read32(0x0166);
#endif

#if 0	// Raw request for Report Device ID
	uint8_t raw_req[] = { 0x01, 0x11 };
	uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];
	
	int req_length = modbus_send_raw_request(ctx, raw_req, 2);
	modbus_receive_confirmation(ctx, rsp);
#endif

#if 0	// Raw read request for ...
	uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];
//	uint8_t raw_req[] = { 0x00, 0x11 };		// ?! Broadcast Report Device ID
//	uint8_t raw_req[] = { 0x01, 0x11 };		// ?! Report Device ID
//		Device, Read, AddrHi, AddrLo, 
//	uint8_t raw_req[] = { 0x01, 0x03, 0x00, 0x10 , 0x00, 0x02};		// Voltage L1
//	uint8_t raw_req[] = { 0x01, 0x03, 0x00, 0x12 , 0x00, 0x02};		// Voltage L2
//	uint8_t raw_req[] = { 0x01, 0x03, 0x00, 0x13 , 0x00, 0x02};		// Voltage L3
//	uint8_t raw_req[] = { 0x01, 0x03, 0x00, 0x4E , 0x00, 0x02};		// ?! Frequency	-> E7??!!
//	uint8_t raw_req[] = { 0x01, 0x03, 0x01, 0x60 , 0x00, 0x02};		// Import Energy
//	uint8_t raw_req[] = { 0x01, 0x03, 0x01, 0x66 , 0x00, 0x02};		// Export Energy
	uint8_t raw_req[] = { 0x01, 0x03, 0xF0, 0x00 , 0x00, 0x04};		// Time
//	uint8_t raw_req[] = { 0x01, 0x03, 0xF1, 0x11 , 0x00, 10};		// Last 1 month positive Energy
//	uint8_t raw_req[] = { 0x01, 0x03, 0xF5, 0x00 , 0x00, 0x02};		// Demand interval, Slide time, Display time, Display interval
//	uint8_t raw_req[] = { 0x01, 0x03, 0xF6, 0x00 , 0x00, 0x06};		// ?! Meter number
//	uint8_t raw_req[] = { 0x01, 0x03, 0xF7, 0x00 , 0x00, 15};		// Tariff
//	uint8_t raw_req[] = { 0x01, 0x03, 0xFA, 0x01 , 0x00, 10};		// Current month positive max Demand
	
	int req_length = modbus_send_raw_request(ctx, raw_req, sizeof(raw_req));
	int res_length = modbus_receive_confirmation(ctx, rsp);
	
	printf("REQ Length: %d RES Length: %d\n", req_length, res_length);
	
	for (int i = 0; i < res_length; i++)
	{
		printf("%02X ", rsp[i]);
	}
	printf("\n");
#endif

#if 0	// write_register => 0x06
	rc = modbus_write_register(ctx, 0x1234, 0x5678);
	
	if (rc)
	{
		printf("MODBUS write error: %s\n", modbus_strerror(errno));
		abort();
	}
#endif

#if 0	// write_registers => 0x10
	uint16_t reg_val = 0x5678;
	
	rc = modbus_write_registers(ctx, 0x1234, 1, &reg_val);
	if (rc)
	{
		printf("MODBUS write error: %s\n", modbus_strerror(errno));
		abort();
	}
#endif

#if 0	// try to read all registers
	
	for (unsigned int i = 0; i < 0xFFF0; i++)
	{
		read32(i);
	}
#endif

#if 0 // search for valid length of a register response

	// search for valid length of a register response

	for (int i = 0; i <= 0x100; i++)
	{
	
		uint8_t hi = 0xF6;
		uint8_t lo = 0x00;
		uint8_t len = i;
	
		printf("Test 0xF600:%02X %3d\n", i, i);
	
		uint8_t raw_req[] = { 0x01, 0x03, 0x00, 0x00 , 0x00, 0x00};
	
		raw_req[2] = hi;
		raw_req[3] = lo;
		raw_req[5] = len;
	
		uint8_t rsp[MODBUS_TCP_MAX_ADU_LENGTH];

		int req_length = modbus_send_raw_request(ctx, raw_req, sizeof(raw_req));
		int res_length = modbus_receive_confirmation(ctx, rsp);
	
		printf("REQ Length: %d RES Length: %d\n", req_length, res_length);

		if (res_length > 0)
		{
			for (int i = 0; i < res_length; i++)
			{
				printf("%02X ", rsp[i]);
				abort();
			}
			printf("\n");
		}
	}

	abort();
#endif











