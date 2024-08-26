/******************************************************************************

GeoCache Hunt Project (GeoCache.cpp)
  
This is skeleton code is provided to the work to be done.  You are not 
required to follow this coding structure.  You are free to implement
your project however you wish.

Consider using sprintf(), strtok() and strtod() for message string
parsing and converting between floats and strings.

The GPS provides latitude and longitude in degrees minutes format (DDDMM.MMMM).
You will need to convert it to Decimal Degrees format (DDD.DDDD).

*******************************************************************************
Following is the GPS Shield "GPRMC" Message Structure.  This message is received
once a second.  You must figure out how to parse the message to obtain the
parameters required for the GeoCache project.  Additional information on the
GPS device and messaging can be found in the documents supplied in your resource
coordinates in the following GPRMC sample message, after convert to Decimal
Degrees (DDD.DDDDDD) as latitude(23.118757) longitude(120.274060).  By the way,
this coordinate is GlobalTop Technology in Tiawan, who designed and manufactured
the GPS Chip.

"$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C\r\n"

$GPRMC,         // GPRMC Message
064951.000,     // utc time hhmmss.sss
A,              // coordinate status A=data valid or V=data not valid
2307.1256,      // Latitude 2307.1256 (degrees minutes format dddmm.mmmm) range[0.90]
N,              // N/S Indicator N=north or S=south
12016.4438,     // Longitude 12016.4438 (degrees minutes format dddmm.mmmm) range[0.180]
E,              // E/W Indicator E=east or W=west
0.03,           // Speed over ground knots
165.48,         // Course over ground (decimal degrees format ddd.dd)
260406,         // date ddmmyy
3.05,           // Magnetic variation (decimal degrees format ddd.dd)
W,              // E=east or W=west
A               // Mode A=Autonomous D=differential E=Estimated
*2C             // checksum
\r\n            // return and newline

*******************************************************************************

Configuration settings.

These defines make it easy for you to enable/disable certain
code during the development and debugging cycle of this project.

The results below are calculated from above GPS GPRMC message
and the GEOLAT0/GEOLON0 tree as target.  Your results should be
nearly identical, if not exactly the same.

Results of converting GPS LAT or LON string to a float:
LAT_2307.1256 = 2307.125488
LON_12016.4438 = 12016.443359

Results of executing the following functions:
degMin2DecDeg() LAT_2307.1256_N = 23.118757 decimal degrees
degMin2DecDeg() LON_12016.4438_E = 120.274055 decimal degrees
calcDistance() to GEOLAT0/GEOLON0 target = 45335760 feet
calcBearing() to GEOLAT0/GEOLON0 target = 22.999652 degrees

Results for adjusting for relative bearing towards tree = 217.519650 degrees

******************************************************************************/

#include <SD.h>
#include "wiring_private.h"
#include <Adafruit_seesaw.h>
#include <Adafruit_SH110X.h>

// compile flags
#define GPS_ON  1	// GPS messages classroom testing=0, live outside=1
#define LOG_ON 1

// Feather PINS for peripherals
#define M0TX_PIN	10	// MO TX -> GPS RX
#define M0RX_PIN	11	// MO RX <- GPS TX
#define SDC_CS	4		  // Secure Digital Card SPI chip select
#define BAT_IN	A7		// Battery analog pin

// joy BITS for buttons
#define BUT_RT	(1<<6)
#define BUT_DN	(1<<7)
#define BUT_LF	(1<<9)
#define BUT_UP	(1<<10)
#define BUT_SL	(1<<14)
#define BUT_MSK (BUT_RT|BUT_DN|BUT_LF|BUT_UP|BUT_SL)

#define GPS_BUFSIZ	96	// max size of GPS char buffer

// OLED PINS (not used)
// #define BUT_A 	9
// #define BUT_B 	6
// #define BUT_C	5

// GPS control messages
#define PMTK_AWAKE "$PMTK010,002*2D"
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_Q_RELEASE "$PMTK605*31"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"
#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_CMD_HOT_START "$PMTK101*32"
#define PMTK_CMD_WARM_START "$PMTK102*31"
#define PMTK_CMD_COLD_START "$PMTK103*30"
#define PMTK_CMD_FULL_COLD_START "$PMTK104*37"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C"
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
#define PMTK_SET_NMEA_OUTPUT_RMC "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_GGA "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"

#define EARTH_RADIUS_FEET (3959.00 * 5280) // feet per mile

/****
Sample coordinates that can be used for testing.
****/

// FS3B-116 large tree outside front door
#define GEOLAT0 28.594532
#define GEOLON0 -81.304437

// grass between taco bell and office
#define GEOLAT1 28.596556
#define GEOLON1 -81.306430

// FS2 rear parking retention pond
#define GEOLAT2 28.595888
#define GEOLON2 -81.301271

// Front FS4 436 entrance
#define GEOLAT3 28.591209
#define GEOLON3 -81.306019

// waypoint structure
typedef struct
{
	float latitude;
	float longitude;
} WAYPOINT;

/**************************/
/**** GLOBAL VARIABLES ****/
/**************************/

bool recording = false;	// SDC is recording
bool acquired = false;	// GPS acquired position

uint8_t target = 0;		// GeoCache target number
float flat = 0.0;		// current GPS latitude position
float flon = 0.0;		// current GPS longitude position
float fcog = 0.0;		// current course over ground
float fbrg = 0.0;		// true north target bearing
float bearring = 0.0;		// relative bearing to target
float fdis = 0.0;		// current distance to target
unsigned long ledToggle = 0;   // Intialize  toggle led millis

/************************/
/**** GLOBAL OBJECTS ****/
/************************/
WAYPOINT waypoint[] = 
{
	GEOLAT0, GEOLON0,
	GEOLAT1, GEOLON1,
	GEOLAT2, GEOLON2,
	GEOLAT3, GEOLON3,
};

File MyMap;
Adafruit_seesaw joy;
Adafruit_SH1107 oled = Adafruit_SH1107(64, 128, &Wire);
Uart gps (&sercom1, M0RX_PIN, M0TX_PIN, SERCOM_RX_PAD_0, UART_TX_PAD_2);

// gps serial interrupt handler
void SERCOM1_Handler(void)
{
  gps.IrqHandler();
}

/**************************************************
Convert Degrees Minutes (DDMM.MMMM) into Decimal Degrees (DDD.DDDD)

float degMin2DecDeg(char *ccor, char *cind)

Input:
	ccor = char string pointer containing the GPRMC latitude or longitude DDDMM.MMMM coordinate
	cind = char string pointer containing the GPRMC latitude(N/S) or longitude (E/W) indicator

Return:
	Decimal degrees coordinate.

**************************************************/
float degMin2DecDeg(char *cind, char *ccor)
{
	   // Convert string into float
     float num = atof(ccor);

     // Take whole degree, store in float 
     int degreesVar = static_cast<int>(num/100);
    
     // Take minutes 
     float minutes = num - (degreesVar * 100);

     // Divide by 60
     // Add back to whole degree
     float degrees = (degreesVar + (minutes/60));

     // N/S E/W
     if (*cind == 'S' || *cind == 'W') 
     {
      degrees = -degrees;
     }
     
	
#if LOG_ON
	Serial.print("degMin2DecDeg() returned: ");
	Serial.println(degrees, 6);
#endif

	return(degrees);
}

/**************************************************
Calculate Great Circle Distance between to coordinates using
Haversine formula.

float calcDistance(float flat1, float flon1, float flat2, float flon2)

EARTH_RADIUS_FEET = 3959.00 radius miles * 5280 feet per mile

Input:
	flat1, flon1 = GPS latitude and longitude coordinate in decimal degrees
	flat2, flon2 = Target latitude and longitude coordinate in decimal degrees

Return:
	distance in feet (3959 earth radius in miles * 5280 feet per mile)
**************************************************/
float calcDistance(float flat1, float flon1, float flat2, float flon2)
{
	float distance = 0.0;

  // Convert the degrees to RADIANS 
  float latFirstPoint = radians(flat1);
  float latSecondPoint = radians(flat2);
  float lonFirstPoint = radians(flon1);
  float lonSecondPoint = radians(flon2);

  // Difference between 2 points 
  float diffLat = (latSecondPoint - latFirstPoint);
  float diffLon = (lonSecondPoint - lonFirstPoint);

  // Haversine formula 
  float a = sin(diffLat / 2) * sin(diffLat / 2) +
            cos(latFirstPoint) * cos(latSecondPoint) * sin(diffLon / 2) * sin(diffLon / 2);

  float c = 2 * atan2(sqrt(a), sqrt(1 - a));

  // Converting distance to feet 
  distance = EARTH_RADIUS_FEET * c;
	
#if LOG_ON
	Serial.print("calcDistance() returned: ");
	Serial.println(distance, 6);
#endif

	return(distance);
}

// Notes: http://www.movable-type.co.uk/scripts/latlong.html
// Notes: https://stackoverflow.com/questions/14920675/is-there-a-function-in-c-language-to-calculate-degrees-radians
// Notes Geek: https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
// Notes vid: https://www.youtube.com/watch?v=nsVsdHeTXIE

/******************************************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
	flat1, flon1 = gps latitude and longitude coordinate in decimal degrees
	flat2, flon2 = target latitude and longitude coordinate in decimal degrees

Return:
	angle in decimal degrees from magnetic north

NOTE: atan2() returns range of -pi/2 to +pi/2)

******************************************************************************/
/**************************************************
Calculate Great Circle Bearing between two coordinates

float calcBearing(float flat1, float flon1, float flat2, float flon2)

Input:
	flat1, flon1 = gps latitude and longitude coordinate in decimal degrees
	flat2, flon2 = target latitude and longitude coordinate in decimal degrees

Return:
	angle in decimal degrees from magnetic north
	
NOTE: atan2() returns range of -pi/2 to +pi/2)

**************************************************/
float calcBearing(float flat1, float flon1, float flat2, float flon2)
{
	float bearing = 0.0;
	
 // Convert the degrees to RADIANS 
  float latFirstPoint = radians(flat1);
  float latSecondPoint = radians(flat2);
  float lonFirstPoint = radians(flon1);
  float lonSecondPoint = radians(flon2);

  // Difference between 2 points in longitude
  float diffLon = (lonSecondPoint - lonFirstPoint);

  // Calc bearing formula 
  float y = sin(diffLon) * cos(latSecondPoint);
  float x = cos(latFirstPoint) * sin(latSecondPoint) - sin(latFirstPoint) * cos(latSecondPoint) * cos(diffLon);

  float radiansBearing = atan2(y, x);

  // Converting calculation to degrees 
  bearing = degrees(radiansBearing);


#if LOG_ON
	Serial.print("calcBearing() returned: ");
	Serial.println(bearing, 6);
#endif

	return(bearing);
}

// Notes: http://www.movable-type.co.uk/scripts/latlong.html

#if GPS_ON
/*
Get valid GPS message.

char* getGpsMessage(void)

Side affects:
Message is placed in local static char buffer.

Input:
none

Return:
char* = null char pointer if message not received
char* = pointer to static char buffer if message received

*/

char* getGpsMessage(void)
{
	bool rv = false;
	static uint8_t x = 0;
	static char cstr[GPS_BUFSIZ];

	// get nmea string
	while (gps.peek() != -1)
	{
		// reset or bound cstr
		if (x == 0) memset(cstr, 0, sizeof(cstr));
		else if (x >= (GPS_BUFSIZ - 1)) x = 0;

		// read next char
		cstr[x] = gps.read();

		// looking for "$GPRMC", toss out undesired messages
		if ((x >= 3) && (cstr[0] != '$') &&  (cstr[3] != 'R'))
		{
			x = 0;
			break;
		}

		// if end of message received (sequence is \r\n)
		if (cstr[x] == '\n')
		{
			// nul terminate char buffer (before \r\n)
			cstr[x - 1] = 0;

			// if checksum not found
			if (cstr[x - 4] != '*')
			{
				x = 0;
				break;
			}

			// convert hex checksum to binary
			uint8_t isum = strtol(&cstr[x - 3], NULL, 16);

			// reverse checksum
			for (uint8_t y = 1; y < (x - 4); y++) isum ^= cstr[y];

			// if invalid checksum
			if (isum != 0)
			{
				x = 0;
				break;
			}

			// else valid message
			rv = true;
			x = 0;
			break;
		}

		// increment buffer position
		else x++;

		// software serial must breath, else miss incoming characters
		delay(1);
	}

	if (rv) return(cstr);
	else return(nullptr);
}

#else
/*
Get simulated GPS message provided once a second.

This is the same message and coordinates as described at the top of this
file.

NOTE: DO NOT CHANGE THIS CODE !!!

char* getGpsMessage(void)

Side affects:
Message is place in local static char buffer

Input:
none

Return:
char* = null char pointer if message not received
char* = pointer to static char buffer if message received

*/
char* getGpsMessage(void)
{
	static char cstr[GPS_BUFSIZ];
	static uint32_t timestamp = 0;
	uint32_t timenow = millis();

	// provide message every second
	if (timestamp >= timenow) return(nullptr);

	String sstr = "$GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C";

	memcpy(cstr, sstr.c_str(), sstr.length());

	timestamp = timenow + 1000;

	return(cstr);
}

#endif

float getBatteryVoltage(void)
{
	float vbat = analogRead(BAT_IN);
	vbat *= 2;    // normalize - input is divided by 2 using restor divider.
	vbat *= 3.3;  // multiply by analog input reference voltage of 3.3v.
	vbat /= 1024; // convert to actual battery voltage (10 bit analog input)
	return(vbat);
}

void setup(void)
{
	// delay till terminal opened
	Serial.begin(115200);

  // wait upto 5 seconds to open serial terminal
  while(!Serial && (millis() < 5000))

	// initialize status LED=OFF
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);

  // Store current time for led toggle 
  ledToggle = millis();

	// initialize oled dispaly
	while (oled.begin(0x3C, true) == false)
	{
		Serial.println("oled.begin() failed");
		delay(1000);
	}

	/********************************
	oled is 128w x 64h
	Chars initialized to 6w x 8h
	Max 21 chars per line
	Max 8 lines per display
	********************************/
	oled.clearDisplay();
  oled.setTextSize(1);
	oled.setRotation(1);
	oled.setTextColor(SH110X_WHITE);
	oled.setCursor(0, 0);
	oled.println("Hello Kheera <3");
	oled.display();

	// initialize joy board
	while (joy.begin() == false)
	{
		Serial.println("joy.begin() failed");
		delay(1000);
	}

	// set joy pin modes/interrupts
	joy.pinModeBulk(BUT_MSK, INPUT_PULLUP);
	joy.setGPIOInterrupts(BUT_MSK, true);
  pinMode(5, INPUT_PULLUP);

  // TODO - initialize Secure Digital Card and open "MyMap.txt" file for writing
  if (SD.begin(4))
  {
    MyMap = SD.open("MyMap.txt", FILE_WRITE);
    Serial.println("MyMap() txt file created success");
  } 
   else
  {
    Serial.println("MyMap() txt file created fail");
  }
 


  // Notes: https://docs.arduino.cc/learn/programming/sd-guide/

#if GPS_ON
	// initilaze gps serial baud rate
	gps.begin(9600);

	// map rx/tx pins to sercom
	pinPeripheral(M0RX_PIN, PIO_SERCOM);
  pinPeripheral(M0TX_PIN, PIO_SERCOM);

	// initialize gps message type/rate
	gps.println(PMTK_SET_NMEA_UPDATE_1HZ);
	gps.println(PMTK_API_SET_FIX_CTL_1HZ);
	gps.println(PMTK_SET_NMEA_OUTPUT_RMC);
#endif

	Serial.println("setup() complete");
}



void loop(void)
{
	// get GPS message
	char* cstr = getGpsMessage();

	// if valid message
	if (cstr)
	{
		// print the GPRMC message
		Serial.println(cstr);	
     
		// TODO - Check button for incrementing target index 0..3
    uint32_t buttons = 0;
    static uint32_t _buttons = joy.digitalReadBulk(BUT_MSK);
    bool pressedButton = false; 

    // Read current state
    buttons = joy.digitalReadBulk(BUT_MSK);

    // check if state changed 
    if (_buttons != buttons)
    {
      // Is button currently pressed?
      if(!(buttons & BUT_RT && pressedButton == false))
      {
        Serial.println("Button Pressed!");

        // increment target index
        target++;
        
        // If larger than 3, reset to 0
        if(target > 3)
        {
          target = 0;
        }

        // Pressed 
        pressedButton = true; 
      }
    }
    else
    {
      pressedButton = false; 
    }

    // Update 
    _buttons = buttons;
   

		// TODO - Parse 5 parameters latitude, longitude, and hemisphere indicators, and course over ground from GPS message
    // Example: I will need 8 parameters for geocache
    char *P0 = strtok(cstr, ", ");
    Serial.print("GPRMC Message: "); 
    Serial.println(P0);

    char *P1 = strtok(NULL, ", ");
    // Not needed 

    char *P2 = strtok(NULL, ", ");
    Serial.print("Coordinate status: "); 
    Serial.println(P2);

    char *P3 = strtok(NULL, ", ");
    Serial.print("Latitude: "); 
    Serial.println(P3);
    

    char *P4 = strtok(NULL, ", ");
    Serial.print("N/S Indicator: ");
    
    
    char *P5 = strtok(NULL, ", ");
    Serial.print("Longitude: ");
    Serial.println(P5); 
    flon = atof(P5);

    char *P6 = strtok(NULL, ", ");
    Serial.print("E/W Indicator: ");
    Serial.println(P6);
    
    char *P7 = strtok(NULL, ", ");
    // Not needed 

    char *P8 = strtok(NULL, ", ");
    Serial.print("Course over ground: ");
    Serial.println(P8);
    

    
    // TODO - Call degMin2DecDeg() convert latitude deg/min to dec/deg
    float latitude = degMin2DecDeg(P4, P3);
		
		// TODO - Call degMin2DecDeg() convert longitude deg/min to dec/deg
    float longitude = degMin2DecDeg(P6, P5);

		// TODO - Call calcDistance() calculate distance to target
    fdis = calcDistance(latitude, longitude, waypoint[target].latitude, waypoint[target].longitude);
   
		// TODO - Call calcBearing() calculate bearing to target
    float bearingg = calcBearing(latitude, longitude, waypoint[target].latitude, waypoint[target].longitude);
		
		// TODO - Calculate relative bearing within range >= 0 and < 360
    float fcog = atof(P8);
    float frel = (bearingg - fcog);

    // Wrap around
    // If negative bearing is less than zero, then adding 360 will put it back into the 0 to 359 range
    if(frel < 0)
    {
      frel += 360;
    }
    // If bearing that is equal to or greater than 360 (>=360), then subtracting 360 will put it back into the 0 to 359 range
    else if(frel >= 360)
    {
      frel -= 360;
    }
		
  #if LOG_ON 
		Serial.print("Relative Bearing: ");
    Serial.println(frel);
  #endif
		
		//TODO write required data to SecureDigital then execute flush()
    if(MyMap)
    {
      // TODO: You must write a line once a second to the SD card containing the 
      // received GPS coordinates and calculated target distance in feet using the 
      // following format: "longitude,latitude,bearing.distance”.  ???
      MyMap.print(longitude, 6);
      MyMap.print(",");
      MyMap.print(latitude, 6);
      MyMap.print(",");
      MyMap.print(frel, 0);
      MyMap.print(".");
      MyMap.print(fdis, 0);
      MyMap.println();

      // Data written to file 
      MyMap.flush(); 
    }
    
      // wait one second 
      delay(1000);
		
	}
  // Notes: https://docs.arduino.cc/learn/programming/sd-guide/

  // Writing to OLED - target, distance, bearing, battery 
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setRotation(1);
	oled.setTextColor(SH110X_WHITE);
	oled.setCursor(0, 0);
  oled.print("Target: ");
  oled.println(target);
  oled.print("Distance: ");
  oled.println(fdis);
  oled.print("Bearing: "); 
  oled.println(bearring);
  oled.print("Battery: ");
  oled.println(getBatteryVoltage());
  oled.display();

  // TODO - toggle LED_BUILTIN once a second.
  if(millis() - ledToggle >= 1000)
  {
    // Update 
    ledToggle = millis();

    // Turn on / off
   digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); 
  }
}
