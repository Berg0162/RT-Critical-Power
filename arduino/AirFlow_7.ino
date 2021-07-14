/*******************************************************************************
  This is an application for the Adafruit Feather nRF52 (Bluefruit Bluetooth LE)
  --> Nordic Semiconductor SoftDevice installed: S140
 *******************************************************************************/

/*  This sketch heavily uses the BLEClientService and BLEClientCharacteristic of the
    Bluefruit library to implement a custom client (a.k.a. CENTRAL) that is used to listen
    and talk with a Gatt server on the indoor cycling trainer or power meter. In our case a
    TACX Indoor Bike Trainer of type NEO... a so called "smart trainer" that is capable
    of working with BLE and ANT+.

    In addition a HRM sensor strap that is capable of BLE communication with the Airflow

    Optional (but extremely usefull) is  a mobile phone can be (optionally) connected over BLE UART,
    (a) to set critical and persistent values that constrain high level functions,
    (b) to manually control air flow rate if one does not allow the algorithms to do the work
    In analogy to Zwift you need to download and install the AIRFLOW Companion App on your phone!

    Note: you need a TACX Trainer or Power meter (BLE capable), a HRM strap and Mobile Phone to exploit
    this sketch to its max.
    Stand alone (i.e. with no BLE connectable peripherals aside from a mobile phone) the sketch tests well
    functioning and can set "manually" the fans.....
*/

#include <bluefruit.h>
// Libraries for use of I2C devices (Oled and Temperature sensor)
#include <SPI.h>
#include <Wire.h>
// -------------------------------------------------------
#include <nrf_timer.h> // Native nRF52 timers library
#include <Timer.h>     // Heavy duty micro(!)seconds timer library based on nRF52 timers, needs to reside in the libraries folder
#include <math.h>      // We need the math library
// -------------------------------------------------------
// Necessary libraries for use of Oled display(s)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h> // needs to reside in the libraries folder

// Additional splash screen bitmap and icon(s) for SSD1306 Oled
#include <Adafruit_SSD1306_Icons.h> // Icons (SSD1306) designed for the application, needs to reside in the libraries folder

// Declarations for an SSD1306 128x64 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128            // SSD1306-OLED display width, in pixels
#define SCREEN_HEIGHT 64            // SSD1306-OLED display height, in pixels
#define OLED_RESET -1               // No reset pin on this OLED display
#define SSD1306_I2C_ADDRESS 0x3C    // I2C Address for SSD1306-OLED display
// Declare the display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ---------AIR TEMPERATURE AND HUMIDITY MEASUREMENT-----------
#include <Adafruit_SHT31.h> // SHT31 library, needs to reside in the libraries folder
// Create SENSIRON SHT31 Temp. and Humidity sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();
float Tair = 19.5; // Actual AIR Temp. measured
float RH = 0.40;   // Relative Humidity (##%) measured
bool IsTempSensor = false; // Test for is working
// ----------------------------------------------

#include <avr/dtostrf.h> // Library used for formatting floating point numbers

// ------------ W' Balance calculation -------------------
// Global variables related to Cycling Power and W-Prime
uint16_t TAWC_Mode = 1;  // Track Anaerobic Capacity Depletion Mode == TRUE -> APPLY and SHOW
uint16_t CP60 = 160;  // Your (estimate of) Critical Power, more or less the same as FTP
uint16_t eCP = CP60;  // Algorithmic estimate of Critical Power during intense workout
uint16_t w_prime_usr = 7500;          // Your (estimate of) W-prime or a base value
uint16_t ew_prime_mod = w_prime_usr;  // First order estimate of W-prime modified during intense workout
uint16_t ew_prime_test = w_prime_usr;   // 20-min-test algorithmic estimate (20 minute @ 5% above eCP) of W-prime for a given eCP! 
long int w_prime_balance = 0;  // Can be negative !!!
bool IsShowWprimeValuesDominant = false;
//-------------------------------------------------------

#define SIXTY_SECONDS_TIME 60000 // Time span in millis -> one minute
unsigned long OneMinuteInterval = 0; //

#define ADVERTISE_TIME 20 // Time span in seconds
unsigned long TimeCaptureMillis = 0;  // Soon to be instantiated
// Instantiate timing for Power calculation interval
#define SOME_SECONDS_TIME 5000 // Time span in millis -> 5 seconds
unsigned long SomeSecondsInterval = 0; // POWER Algorithm calculation interval

// Create a SoftwareTimer that will drive our OLED display sequence.
SoftwareTimer RoundRobin; // Timer for OLED display show time
#define SCHEDULED_TIME 10000 // Time span to show an OLED Display screen in millis
int Scheduled = -1; // Counter for current OLED display screen, start to show SETTINGS Oled Display first !!
#define MAX_SCHEDULED 11 // Number of Oled Display screens to show sequentially -> Settings Screen (-1) does not count!

// Display Settings Sequence Show ("1") NoShow ("0")
char DisplaySettings[MAX_SCHEDULED + 1] = "00000000000"; // MAX_SCHEDULED display screen sequence --> all OFF
// -------------------------------------------------------

// LittleFS for internal storage of persistent data on the Feather nRF52
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
// Managing persistence of some data with LittleFile system
// PeRSistent Data  --> PRS data
#define PRS_FILENAME "/prsdata.txt"
File file(InternalFS);
#define RECORD_LENGTH 64 // File will contain only some records of this size
// LittleFS--------------------------------------------------------------


/* Cycling Power Service declarations
   CP Service: 0x1818
   CP Characteristic: 0x2A63 (Measurement)
   CP Characteristic: 0x2A65 (Feature)
   CP Characteristic: 0x2A5D (Location)
*/
BLEClientService        cps(UUID16_SVC_CYCLING_POWER);
BLEClientCharacteristic cpmc(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
// BLEClientCharacteristic cpfc(UUID16_CHR_CYCLING_POWER_FEATURE);
BLEClientCharacteristic cplc(UUID16_CHR_SENSOR_LOCATION);

/*
    Declare the BLE DIS Service and Characterics Uuid's
*/
#define SERVICE_DEVICE_INFORMATION                  0x180A
#define CHARACTERISTIC_MANUFACTURER_NAME_STRING     0x2A29
#define CHARACTERISTIC_MODEL_NUMBER_STRING          0x2A24

/* Declare additional services and characteristics for Device Information Service DIS
*/
BLEClientService        diss(SERVICE_DEVICE_INFORMATION);
BLEClientCharacteristic disma(CHARACTERISTIC_MANUFACTURER_NAME_STRING);
BLEClientCharacteristic dismo(CHARACTERISTIC_MODEL_NUMBER_STRING);
// ---------------------------------------------------------

/* HRM Service Definitions
   Heart Rate Monitor Service:  0x180D
   Heart Rate Measurement Char: 0x2A37 (Mandatory)
   Body Sensor Location Char:   0x2A38 (Optional)
*/
BLEClientService        hrms(UUID16_SVC_HEART_RATE);
BLEClientCharacteristic hrmc(UUID16_CHR_HEART_RATE_MEASUREMENT);
BLEClientCharacteristic bslc(UUID16_CHR_BODY_SENSOR_LOCATION);
// --------------------------------------------------------------

// Declare bleuart for UART communication over BLE with mobile Phone
BLEUart bleuart;
bool ConnectedToMobilePhone = false;
// -------------------------------------------------------------


// Instantiate Space for a Simple Moving Average Filter for Cycling Power values
const uint8_t CPS_Length = 30;  // 60 Readings equals about 60 seconds
uint16_t *CPS_history = (uint16_t *)calloc(CPS_Length, sizeof(uint16_t));
// Global CPS variable definitions
bool ConnectedToCPS = false;
uint16_t CPS_conn_handle = 0;
uint16_t CPS_cur = 0;
uint16_t CPS_average = 0;
char CPSdeviceName[32 + 1] = {0}; // CPS/DIS device name
char CPSModelNum[32 + 1] = {0}; // DIS Model
const char* CPSSensorLoc_str[] = { "Other", "Top of shoe", "In shoe", "Hip", "Front wheel", "Left crank", "Right crank",
                                   "Left pedal", "Right pedal", "Front hub", "Rear dropout", "Chainstay", "Rear wheel", "Rear hub", "Chest", "Spider", "Chain ring"
                                 };
//--------------------------------------------------------------------

// Feather nRF52840 I/O Pin declarations for connection to the ROBOTDYN AC DIMMER boards
#define PWM_PIN_U_FAN  (12U) // Upper Fan 
#define ISR_PIN        (11U) // AC Cycle Zero Cross detection pin (for both FANS)
#define PWM_PIN_L_FAN  (10U) // Lower FAN

// Values are in microseconds and a half 50Hz cycle is 10.000us = 10 ms !
#define MAX_TIME_OFF_UPPER (6950U) // Maximal time of half an AC Cycle (10.000us) to be CUT-OFF in microseconds!
#define MAX_TIME_OFF_LOWER (6920U) // Maximal time of half an AC Cycle (10.000us) to be CUT-OFF in microseconds!
#define MIN_TIME_OFF (2000U)   // Minimal time to process for full power --> NO (!) CUT-OFF situation in microseconds!
uint16_t UpperThreshold = 100; // Setting of Upper comfortable Fan intensity --> range = 80 - 100%
uint16_t LowerThreshold = 0;   // Setting of Lower comfortable Fan intensity --> range = 0 - 20%
int16_t FanBalance = 0;        // Balance Upper Lower fan 0-20% // Short Int Value: -32000 to +32000 --> Balance < 0 means Upper Fan
bool AreFansWorking = false;

// Mode of Operation
#define MANUAL       (0U) // Process Manual Operation
#define HEATBALANCE  (1U) // Air Flow (OperationMode) as a function of the Heat Balance Equation
const bool TWEAK = true; // Allow some tweaking when DeltaStoredHeat<0 --> Heat Loss
const char* OpMode_str[] = { " MAN - ", "HBAL - ", " None "};
uint16_t OpMode = MANUAL; // See above for possible values

// Variables that account for BIKE POSITION
#define UP   (0U) // Upright Bike Position
#define DP   (1U) // Dropped Bike Position (straight arms!)
#define TTP  (2U) // Time-Trial Bike Position
const char* BikePos_str[] = { "UP", "DP", "TTP" };
uint16_t BikePos = UP; // Default set to UP

// Airflow speed of the impellor of the fan is linear proportional to the rotation frequency
float Vair = 0.0; // Calculated requested AirFlow as a result of the Heat Balance equation
// According to the First Fan Affinity Law: Volumetric air flow is proportional to RPM (Impellor Rotations Per Minute)
// --> Higher impellor frequency results in proportional higher air flow speed
// Just ONE multiplying factor (FUP) is therefore appropriate for the whole range of fan operation!

// --> Measurement with a handheld anemometer showed that the applied fan generates, at max capacity, an airflow velocity of about 30 km/hour !!
// NOTICE: This is fully dependent of the Fan's mechanical properties, size, power, manufacturer, etcetera and shall be different in your case!
//
// The fans are set from 0% - 100% duty cycle (no flow - max flow). To map the fan percentage to the requested Heat Balanced Airflow follow this rule:
// At a certain set of situational values (RH, air temperature, Gross Efficiency, cyling power induced, etcetera) the heat balance equation results in 
// a requested Airflow of 8.32 m/s (3.6 * 8.32 = 30 km/h) and this should equal the very same Fan Airflow velocity, in our case at max fan capacity (100%).
//
// ---> When a requested airflow of 8.32 m/s is reached the fan(s) should operate close to 100% capacity --> therefore FUP should be 100/8.32 = 12 !!
// (FUP * Requested Airflow) == (12 (No Dimension) * 8.32 (m/s)) = 100% of the max Fan capacity (resulting in 30 km/h air velocity)
const float FUP = 12.0;  // The (constant) multiplying factor
// The AirFlowToFanPercFactor Conversion Factor can be derived taking into account the compensation for smaller sized frontal areas of the rider in
// the different bike positions with their appropriately increased factors!
const float AirFlowToFanPercFactor[] = {FUP,        // Upright Bike Position
                                       (FUP + 0.4), // Dropped Bike Position (straight arms!)
                                       (FUP + 0.8)};// Time-Trial Bike Position
#define MPS (1U) // Show Vair in meters per second m/s
#define KPH (2U) // Show Vair in kilometers per hour km/h
#define MPH (3U) // Show Vair in Miles per hour mph
const uint8_t AirSpeedUnit = KPH;

//----------Heart Rate Measurement----------------------
// Instanstiate Space for a Simple Moving Average Filter for HBM values
const uint8_t HBM_Length = 30;  // Average over about 60 seconds
uint16_t *HBM_history = (uint16_t *)calloc(HBM_Length, sizeof(uint16_t));
uint16_t HBM_average = 0;

bool IsConnectedToHRM = false;
uint16_t HRM_conn_handle = 0;
char HRMdeviceName[32 + 1] = {0};
// Body sensor location value is 8 bit
const char* HRMSensorLoc_str[] = { "Other", "Chest", "Wrist", "Finger", "Hand", "Ear Lobe", "Foot" };
// ------------------------------------------------------

// Core Temperature and heat algorithm(s) Global variables --------
const double Tcore_start = 37.0; // Starting core body temperature
double Tcore_cur = (Tcore_start - 0.005); // EstimateTc result Variable, set 0.005 Celsius below start value
double v_cur = 0.0;  // EstimateTc result Variable
double v_prev = 0.0; // Starting variance
double Tcore_prev = Tcore_cur;
double DeltaStoredHeat = 0.0; // HeatChange since last measurement....
double StoredHeat = 0.0; // Internally Stored Body Heat, aka "S" in Heat Balance equations

const char* State_str[] = { "Balance", "Heating", "Cooling"};
uint8_t HeatState = 0;

double HeatProduced = 0.0; // Net Metabolic Heat Produced during cycling equation H = (M - W)
double SumHeatProduced = 0.0; // Sum of all HeatProduced during workout
double SumEnergyProduced = 0.0;
double MetEnergy = 0.0;
float Tbody_cur = 0.0, Tbody_prev = 0.0; // Mean whole BODY temperature
float Tskin = 0.0; // Mean SKIN Temperature for the whole body
float Ereq = 0.0; // Evaporative Heat Exchange Requested
float Convection = 0.0; // Heat Exchange by Convection
double Radiation = 0.0; // Heat Exchange by Radiation
double DeltaTc = 0.0; // Absolute Tcore change since...
float SwRate; // Sweat Rate per hour !
const float HC_Interval = 60.0; // Heat Change/Storage Sample Rate Interval: one sample per Minute --> 60 seconds
const float Cf = 1.0;     // 0.92 Clothing factor according to Brotherhood: the E, C and R equations are for nude bodies.....
const float Hc = 8.5 * Cf; // Convective Heat Transfer Coefficient; Kerslake (1993): 8.3, 8.6 others and CFD study (Defraeye et al.2011): 8.5 !!!
const float He = 16.5 * Hc; // Evaporative Heat Transfer Coefficient; SportScience paper multiplier: 16.5 and according to Brotherhood: 15.5 ??
const float Hr = 4.7 * Cf; // Radiative Heat Transfer Coefficient; Brotherhood: 5.5*1.82*0.92 = 9.1;  Saunders(2005): 4.7(Kenney 1998)
uint8_t GE = 19;          // Gross Efficiency or Cycling Efficiency range from 0.19 (Untrained Recreational) -> 0.28 (Elite) GE is in percentage: between 0-99

// Global variables and default (!) values related to the individual
uint16_t BodyWeight = 78; // Average normal bodyweight in kg
uint16_t BodyHeight = 178; // Body height in cm
float Ad = 1.85; // Body Surface Area according to Dubois
float CHeatCapacity = 132000.0; // HeatCapacity Constant 3747 (Joules/kg/C) Bw in Kg Ad in m2
// --------------------------------------------------------------------------------

// NOTICE type (int) for period !!
int ActualLowerFanPeriod = MAX_TIME_OFF_LOWER; // microseconds time (OFF) --> cut out of half an AC cycle;
uint16_t ActualLowerFanPerc = 0; // LFan Actual Operation Mode setting in Percentage 0-100%
// NOTICE type (int) for period !!
int ActualUpperFanPeriod = MAX_TIME_OFF_UPPER; // microseconds time (OFF) --> cut out of half an AC cycle;
uint16_t ActualUpperFanPerc = 0; // UFan Actual Operation Mode setting in Percentage 0-100%
//-------------------------------------------------------

// Declaration of Function Prototypes -----------------------------------------------
bool getPersistentData(void);
bool setDataToPersistence(void);
void prph_connect_callback(uint16_t conn_handle);
void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason);
void prph_bleuart_rx_callback(uint16_t conn_handle);
void scan_stop_callback(void);
void adv_stop_callback(void);
void scan_callback(ble_gap_evt_adv_report_t* report);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);

void zero_Cross_ISR(void);
void UFanDutyCycleEnds(void);
void LFanDutyCycleEnds(void);

bool SetCurrentTemperatureAndHumidity(void);
float DuboisBodyArea(float BW, float BH);
float HeatCapacity(float BW, float Area);
float EstimateSkinTemp(float AirTemp);
double Pantoine(double T);
double PercentageofChange(double Initial, double Final);
float HeatChange(float Tc, float Tp, float Tdiff);
void EstimateTcore(double TCprev, double Vprev, double HRcurrent);
uint8_t HeatBalanceState(int current_value);

void AdjustForFanBalance(void);
void SetFanThresholds(void);
void SetBothFanPeriods(void);
void HeatBalanceAlgorithm(void);

// Oled Display related Functions
void CallOledDisplaySchedular(void);
void DisplaySchedular_callback(TimerHandle_t _handle);
void ShowIconsOnTopBar(uint8_t Mode);
void ProgressBar(unsigned long StartTime, unsigned long TimeSpan);
void ShowOnOledLarge(char *Line1, char *Line2, uint16_t Pause);
void ShowRequestedAirflowValueOnOled(void);
void ShowEnergyProductionValuesOnOled(void);
void ShowHeatProductionValuesOnOled(void);
void ShowFanSettingsOnOled(void);
void ShowStoredHeatValuesOnOled(void);
void ShowTcoreValuesOnOled(void);
void ShowTbody_TskinValuesOnOled(void);
void ShowEreq_SwrateValuesOnOled(void);
void ShowRad_ConvValuesOnOled(void);
void ShowAirTemp_HumidityValuesOnOled(void);
void ShowPwr_HbmValuesOnOled(void);
void ShowWorkValueOnOled(void);
//void ShowWprimeValuesOnOled(uint8_t Mode);
void ShowWprimeValuesOnOled(void);
void ShowSettingsOnOled(void);

// Exponential and Simple Moving Average filter definitions
#ifndef EMA_ALPHA
#define EMA_ALPHA 10 //This is in percentage. Should be between 0-99
#endif
int EMA_HeatBalanceFilter(int current_value); // EMA filter function for energy state determination
int EMA_MetEnergyFilter(int current_value);   // EMA filter function for Metabolic Energy determination
long sma_filter(uint16_t current_value, uint16_t history_SMA[], uint8_t SMA_Length);
// -----------------------------------------------------------------------------------


// ------------------------   W'Balance Functions  -----------------------------------
uint16_t CalculateAveragePowerBelowCP(uint16_t iPower, uint16_t iCP);
void CalculateAveragePowerAboveCP(uint16_t iPower, uint16_t &iavPwr, unsigned long &iCpACp);
double tau_w_prime_balance(uint16_t iPower, uint16_t iCP);
void w_prime_balance_waterworth(uint16_t iPower, uint16_t iCP, uint16_t iw_prime);
void ConstrainW_PrimeValue(uint16_t &iCP, uint16_t &iw_prime);
uint16_t GetCPfromTwoParameterAlgorithm(uint16_t iav_Power, unsigned long iT_lim, uint16_t iw_prime);
uint16_t GetWPrimefromTwoParameterAlgorithm(uint16_t iav_Power, double iT_lim, uint16_t iCP);
// ------------------------   W'Balance Functions  ------------------------------------


// -------------------------------------------------------------------------------------------
// COMPILER DIRECTIVE to allow/suppress SERIAL.PRINT messages that help debugging...
// Uncomment to activate
//#define DEBUGAIR
// --------------------------------------------------------------------------------------------

void setup()
{
#ifdef DEBUGAIR
  Serial.begin(115200);
  while ( !Serial ) {
    delay(10); // for Feather nRF52 with native usb
  }
  Serial.print("Running... \n");
#endif

  // Startup Temperature and Humidity sensor
  if (! sht31.begin(0x44)) {   // Set to 0x45 for alternate i2c addr
#ifdef DEBUGAIR
    Serial.println("Couldn't find SHT31!");
#endif
  } else {
#ifdef DEBUGAIR
    Serial.println("Found SHT31!");
#endif
  }
  IsTempSensor = SetCurrentTemperatureAndHumidity();
  if (!IsTempSensor ) {
#ifdef DEBUGAIR
    Serial.println("Couldn't find SHT31!/n");
#endif
  }

  // LittleFS start the Littlefilesystem lib and see if we have persistent data ----
  InternalFS.begin();
  // Now get (or set first time only) the persistent values of relevant and crucial variables
  // whith the Companion App the user can set these on the fly!
  if (!getPersistentData()) { // Get record if it fails (file does not exist) setup filesystem and create PRSDATA File
#ifdef DEBUGAIR
    Serial.println(F("Failed --> Format and Create new PRSDATA file plus write record with default values!"));
#endif
    InternalFS.format();  // Wipe out all files and clear the memory
    setDataToPersistence();  // create new File and record with the default values !!
  }
  //Little FS ------------------------------------------------------------------------

#ifdef DEBUGAIR
  Serial.printf("Default --> Ad: %5.3f BW: %d BH: %d ", Ad, BodyWeight, BodyHeight);
  Serial.printf("CHeatCapacity: %f \n", CHeatCapacity);
#endif

  // Update global variable values
  Ad = DuboisBodyArea((float)BodyWeight, (float)BodyHeight);
  CHeatCapacity = HeatCapacity((float)BodyWeight, Ad);
#ifdef DEBUGAIR
  Serial.printf("Ad: %5.3f BW: %d BH: %d ", Ad, BodyWeight, BodyHeight);
  Serial.printf("CHeatCapacity: %f \n", CHeatCapacity);
#endif

  // Start the show for the SSD1306 Oled display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS)) {
#ifdef DEBUGAIR
    Serial.println(F("SSD1306 allocation failed!"));
#endif
  }
  else {
#ifdef DEBUGAIR
    Serial.println(F("SSD1306 is running..."));
#endif
    // Load Oled with initial display buffer contents on the screen,
    // the SSD1306 library initializes with a Adafruit splash screen,
    // (respect or edit the splash.h in the library).
    display.display(); // Acknowledge Adafruit rights, license and efforts
    delay(500); // show some time
  }
  // Ready to show our own SIMCLINE splash screen
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  display.drawBitmap(24, 0, Air_Flow_bw_80_64, 80, 64, 1);
  display.display();
  delay(2000); // Take somewhat more time.....

  //Show Name and SW version on Oled
  ShowOnOledLarge("AIRFLOW", "0.3", 500);

  // Setup the different PINS for controlling the ROBOTDYN AC DIMMER boards c.q. the FANS !!
  pinMode(ISR_PIN, INPUT_PULLDOWN);  // Set Zero Cross Interrupt pin
  pinMode(PWM_PIN_U_FAN, OUTPUT);
  digitalWrite(PWM_PIN_U_FAN, LOW); // Start at OFF state = LOW
  pinMode(PWM_PIN_L_FAN, OUTPUT);
  digitalWrite(PWM_PIN_L_FAN, LOW); // Start at OFF state = LOW

  //---------------------------------------------------------------------------------------------
  //Enable and setup connections over BLE, first with Tacx Neo and then try to connect smartphone
  //---------------------------------------------------------------------------------------------
#ifdef DEBUGAIR
  Serial.println(F("Bluefruit-nRF52 Central Airflow v 0.0"));
  Serial.println(F("----------------------------------\n"));
#endif
  // CPS/Trainer connection is of type Central BUT for Smartphone connection add 1 Peripheral !!!!
  // Initialize Bluefruit with maximum connections as Peripheral = 1, Central = 1
  // SRAM usage required by SoftDevice will increase dramatically with number of connections
  Bluefruit.begin(1, 2); // HRM and CPS are (2) Central BLE devices; Smartphone is (1) BLE Peripheral device
  Bluefruit.setTxPower(4); // Check bluefruit.h for supported values
  Bluefruit.setName("Bluefruit-nRF52840");

  // Set the LED interval for blinky pattern on BLUE LED during Advertising
  Bluefruit.setConnLedInterval(250);

  // Declare Callbacks for Peripheral (smartphone connection)
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  // Callbacks for Central (HRM & CPS/Trainer connection)
  Bluefruit.Central.setDisconnectCallback(disconnect_callback);
  Bluefruit.Central.setConnectCallback(connect_callback);
  Bluefruit.Scanner.setRxCallback(scan_callback);
  Bluefruit.Scanner.setStopCallback(scan_stop_callback);

  // Initialize CPS client
  cps.begin();
  // Initialize client characteristics of CPS.
  // Note: Client Char will be added to the last service that is begin()ed.
  cplc.begin();
  // set up callback for receiving measurement
  cpmc.setNotifyCallback(cpmc_notify_callback);
  cpmc.begin();

  // Initialize DISS client.
  diss.begin();
  // Initialize some characteristics of the Device Information Service.
  disma.begin();
  dismo.begin();

  // Initialize HRM strap client services and characteristics
  hrms.begin();
  // Initialize client characteristics of HRM.
  // Note: Client Char will be added to the last service that is begin()ed.
  bslc.begin();
  // set up callback for receiving a HRM measurement
  hrmc.setNotifyCallback(hrmc_notify_callback);
  hrmc.begin();

  // ---------------  All initialized --> Start the actual scanning   -------------
#ifdef DEBUGAIR
  Serial.println(F("Scanning for BLE devices ..."));
#endif
  /* Setup Central Scanning for an advertising CPS/trainer/HRM...
     - Enable auto scan if disconnected
     - Interval = 100 ms, window = 80 ms
     - Filter only to accept CPS service
     - We use active scan
  */
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);  // ...
  Bluefruit.Scanner.filterRssi(-70);      // original value of -80 , we want to scan only nearby peripherals !!
  // First scan for HRM strap
  Bluefruit.Scanner.filterUuid(UUID16_SVC_HEART_RATE); // Use filterUuid for HRM strap
  // Show Scanning message on the Oled
  ShowOnOledLarge("Scanning", "HRM strap", 1000);
  Bluefruit.Scanner.start(1000); // 0 = Don't stop scanning or after n, in units of 10 ms (hundredth of a second)
  TimeCaptureMillis = millis();
  while (Bluefruit.Scanner.isRunning()) { // do nothing else but scanning....
    ProgressBar(TimeCaptureMillis, 10000); // in ms -> 10* 1000 -> 10 seconds
  }
  // ---------------------
  delay(1000); // Necessary, otherwise Oled Display messages are missed and/or invisible
  // --------------------
  // Second step: scan for CPS\trainer
  /* Setup Central Scanning for an advertising CPS/trainer...
     - Enable auto scan if disconnected
     - Interval = 100 ms, window = 80 ms
     - Filter only to accept CPS service
     - We use active scan
  */
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80); // in units of 0.625 ms
  Bluefruit.Scanner.useActiveScan(true);  // ...
  Bluefruit.Scanner.filterRssi(-70);      // original value of -80 , we want to scan only nearby peripherals !!
  Bluefruit.Scanner.clearFilters();
  Bluefruit.Scanner.filterUuid(0x1818);   //UUID16_SVC_CYCLING_POWER);  Use filterUuid for CPS 0x1818
  // Show Scanning message on the Oled
  ShowOnOledLarge("Scanning", "CyclePower", 1000);
  Bluefruit.Scanner.start(1000); // 0 = Don't stop scanning or after n, in units of 10 ms (hundredth of a second)
  TimeCaptureMillis = millis();
  while (Bluefruit.Scanner.isRunning()) { // do nothing else but scanning....
    ProgressBar(TimeCaptureMillis, 10000); // in ms -> 10 seconds
  }
  // ---------------------
  delay(1000); // Necessary, otherwise Oled Display messages are missed and/or invisible
  // --------------------
#ifdef DEBUGAIR
  Serial.println(F("Use Airflow Companion App on your Phone!"));
#endif
  // Initialize and setup BLE Uart functionality for connecting to smartphone
  bleuart.begin();
  bleuart.setRxCallback(prph_bleuart_rx_callback);
  // Advertising packet construction
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.Advertising.setStopCallback(adv_stop_callback);
  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  // A.K.A. secondary response packet
  Bluefruit.ScanResponse.addName();
  /* Declare further advertising properties
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)
  */
  Bluefruit.Advertising.restartOnDisconnect(true);  // Initiative (autoreconnect) with smartphone
  // but peripheral must advertise first!
  Bluefruit.Advertising.setInterval(32, 244);       // in units of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(ADVERTISE_TIME);// was 30   number of seconds in fast mode
  // Start advertising: to be picked up by a Smartphone with the Companion App!
#ifdef DEBUGAIR
  Serial.println(F("Start Advertising..."));
#endif
  // Show Advertising message on the Oled
  ShowOnOledLarge("Advertise", "SmartPhone", 1000);
  Bluefruit.Advertising.start(ADVERTISE_TIME);   // 0 = Don't stop advertising or after ADVERTISE_TIME (!) seconds
  TimeCaptureMillis = millis();
  while (Bluefruit.Advertising.isRunning()) {
    ProgressBar(TimeCaptureMillis, ADVERTISE_TIME * 1000);
  }

  //---------------------------------------------
  delay(2000); // Necessary, otherwise Oled Display messages are missed and/or invisible
  //---------------------------------------------

  // ISR Function is called 2(!) times in one full 50Hz cycle --> 100 times in a second
  attachInterrupt(digitalPinToInterrupt(ISR_PIN), zero_Cross_ISR, RISING);  // Set Zero-Crossing Interrupt Service Routine
  AreFansWorking = true; // needs a more sophisticated test !!!!!!!!!!!!!

  // Set up a repeating softwaretimer that fires every SCHEDULED_TIME seconds to invoke the OLED Time Sharing schedular
  RoundRobin.begin(SCHEDULED_TIME, DisplaySchedular_callback);
  RoundRobin.start();

  OneMinuteInterval   = millis() + SIXTY_SECONDS_TIME; // Set first Delay Heat Balance Algoritm calculation
  SomeSecondsInterval = millis() + SOME_SECONDS_TIME; // Set first Delay POWER Algorithm calculation

  // This will just show up......during first phase of data collection
  ShowOnOledLarge("", "Ready!", 0); // No delay

  // Allow callback functions from now on to COMPLETELY dominate operations
  if (hrmc.enableNotify()) { // allow from now on HBM measurements
#ifdef DEBUGAIR
    Serial.print("HBM enabled..\n");
#endif
  }
  if (cpmc.enableNotify()) { // allow from now on Power measurements
#ifdef DEBUGAIR
    Serial.print("PWR enabled..\n");
#endif
  }
} // end of SETUP() --> Ready to go ...

void loop()
{ // Do not use ... !!!
  // -------------------------------------------------------
  // The callback/interrupt functions are dominating completely the
  // processing and loop() would never have been called,
  // since there is a constant stream of BLE packets and interrupts that
  // are coming in!
  // -------------------------------------------------------
}
bool SetCurrentTemperatureAndHumidity(void) {
  float t = sht31.readTemperature();
  float h = sht31.readHumidity();
  if ((!isnan(t)) && (!isnan(h))) {  // check if 'is not a number'
    Tair = t;
    RH = h / 100;
    return true;
  } else {
    Tair = 19.5; // set the default average values
    RH = 0.40;
#ifdef DEBUGAIR
    Serial.println("Failed to read temperature and/or humidity\n");
#endif
    return false;
  }
}

void zero_Cross_ISR(void) {
  // Normal mode
  digitalWrite(PWM_PIN_L_FAN, LOW); // Set AC cycle OFF --> LOW
  digitalWrite(PWM_PIN_U_FAN, LOW); // Set AC cycle OFF --> LOW

  NrfTimer2.attachInterrupt(&LFanDutyCycleEnds, ActualLowerFanPeriod); // microseconds !
  NrfTimer1.attachInterrupt(&UFanDutyCycleEnds, ActualUpperFanPeriod); // microseconds !

  // -------------------------------------------------------------------------------------
}

void UFanDutyCycleEnds(void) {
  digitalWrite(PWM_PIN_U_FAN, HIGH); // Set AC cycle ON --> HIGH
}

void LFanDutyCycleEnds(void) {
  digitalWrite(PWM_PIN_L_FAN, HIGH); // Set AC cycle ON --> HIGH
}

// Adjust for Fan Balance settings ----------------------------------------
void AdjustForFanBalance(void)
{
  ActualLowerFanPerc = ActualUpperFanPerc; // Upper was set to a new value, Lower is equal unless
  if (FanBalance != 0) { // Need to adjust for Fan Balance?
    if (FanBalance < 0) { // < 0 --> adjust Upper Fan
      ActualUpperFanPerc += -(FanBalance); // Notice negation (!) Increase percentage
    } else { // > 0 --> adjust Lower Fan
      ActualLowerFanPerc += FanBalance;    // Increase percentage
    }
  }
}
// ----------------------------------------------------------------------

// Translate Fan Air Flow Percentage Settings to low level AC duty cycle times
void SetFanThresholds(void) {
  // Set Upper threshold to UpperThreshold value, MODE independent !!!
  if (ActualUpperFanPerc > UpperThreshold) {
    ActualUpperFanPerc = UpperThreshold;
  }
  // independent of each other !
  if (ActualLowerFanPerc > UpperThreshold) {
    ActualLowerFanPerc = UpperThreshold;
  }

  // Set Lower threshold to LowerThreshold value, MODE independent !!!
  if (ActualUpperFanPerc < LowerThreshold) {
    ActualUpperFanPerc = LowerThreshold;
  }
  // independent of each other !
  if (ActualLowerFanPerc < LowerThreshold) {
    ActualLowerFanPerc = LowerThreshold;
  }
}

void SetBothFanPeriods(void) {
  // ------------------------------------------------------------------------
  ActualUpperFanPeriod = int(map(ActualUpperFanPerc, 0, 100, MAX_TIME_OFF_UPPER, MIN_TIME_OFF));
  ActualLowerFanPeriod = int(map(ActualLowerFanPerc, 0, 100, MAX_TIME_OFF_LOWER, MIN_TIME_OFF));
}

// LittleFS --------------------------------------------------
bool getPersistentData(void) { // Get variable settings from persistant storage -> PRSdata
  uint32_t readLen;
  char buffer[RECORD_LENGTH] = { 0 };
  file.open(PRS_FILENAME, FILE_O_READ);
  if (!file) {
    return false;  // No such file exists ... take action elsewhere ..
  }
  file.seek(1);
  readLen = file.read(buffer, sizeof(buffer));
  // WARNING: arduino sscanf can't process floating points and NOTICE: CRITICAL sscanf format specifiers !!!!
  // unsigned short int (uint16_t) --> 0 to 65000 -> Format specifier: %hu  // short int (int16_t) --> -32000 to +32000 --> Format specifier: %hd
  sscanf(buffer, "%hu %hu %hu %hd %hu %hu %hu %hu %hu %hu %hu %s", &UpperThreshold, &LowerThreshold, &OpMode, &FanBalance, &BikePos, &GE, &CP60, &TAWC_Mode, 
    &w_prime_usr, &BodyWeight, &BodyHeight, &DisplaySettings);
#ifdef DEBUGAIR
  Serial.printf("Retrieve: [%d] %hu %hu %hu %hd %hu %hu %hu %hu %hu %hu %hu %s\n", readLen, UpperThreshold, LowerThreshold, OpMode, FanBalance, BikePos, GE, CP60, TAWC_Mode,
    w_prime_usr, BodyWeight, BodyHeight, DisplaySettings);
#endif
  // Set estimated start values for variables that relate to CP and w_prime
  eCP = CP60;
  ew_prime_mod = w_prime_usr;
  ew_prime_test = w_prime_usr;
  file.close();
  return true;
}

bool setDataToPersistence(void) { // Store Variable Settings Persistant -> PRSdata
  char buffer[RECORD_LENGTH] = { 0 };
  // New PRS_FILENAME is ceated when it is not present!
  file.open(PRS_FILENAME, FILE_O_WRITE);
  if (!file) {
#ifdef DEBUGAIR
    Serial.println(F("File not found, created new one!"));
#endif
  }
  file.seek(1);
  sprintf(buffer, "%hu %hu %hu %hd %hu %hu %hu %hu %hu %hu %hu %s", UpperThreshold, LowerThreshold, OpMode, FanBalance, BikePos, GE, CP60, TAWC_Mode, 
    w_prime_usr, BodyWeight, BodyHeight, DisplaySettings);
#ifdef DEBUGAIR
  Serial.printf("Store: %hu %hu %hu %hd %hu %hu %hu %hu %hu %hu %hu %s\n", UpperThreshold, LowerThreshold, OpMode, FanBalance, BikePos, GE, CP60, TAWC_Mode,
    w_prime_usr, BodyWeight, BodyHeight, DisplaySettings);
#endif
  file.write(buffer, sizeof(buffer));
  file.close();
  return true;
} // LittleFS --------------------------------------------------

/*------------------------------------------------------------------*/
/* Peripheral callback functions
  ------------------------------------------------------------------*/
void prph_connect_callback(uint16_t conn_handle) {
  // stop advertising... now that we are connected to avoid messing up until timeout!
  if (Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.stop();
  }
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);
  char peer_name[32] = { 0 };
  connection->getPeerName(peer_name, sizeof(peer_name));
  ConnectedToMobilePhone = true;
#ifdef DEBUGAIR
  Serial.print(F("[Prph] Connected to "));
  Serial.println(peer_name);
#endif
  peer_name[10] = 0; // only 11 char long allowed
  // Show the message on the Oled
  ShowOnOledLarge("Paired", peer_name, 1000);
  // nRF52 code needs about 750 ms to setup an UART Peripheral Service with
  // a working TX & RX connection, before we can use it for transmission !
  while (!bleuart.notifyEnabled()) {
  } // !!!! wait until ready !!!!
  // Send persistent values to Mobile Phone for correct settings page on the Phone
  char TXpacketBuffer[20 + 1] = { 0 };
  sprintf(TXpacketBuffer, "!S%d;%d;%d;", UpperThreshold, LowerThreshold, GE);
  // send these persistent data to the Settings page on the smartphone
  bleuart.print(String(TXpacketBuffer));
#ifdef DEBUGAIR
  Serial.print(F("S-Persistent data sent to Phone: ")); Serial.println(TXpacketBuffer);
#endif

  // do we need a delay here for Companion App ? YES we do !!
  delay(1000);
  //---------------------------------------------

  memset(TXpacketBuffer, 0, sizeof(TXpacketBuffer)); // clear the buffer again and set new data
  sprintf(TXpacketBuffer, "!P%d;%d;", BodyWeight, BodyHeight);
  // send these persistent data to the Settings page on the smartphone
  bleuart.print(String(TXpacketBuffer));
#ifdef DEBUGAIR
  Serial.print(F("P-Persistent data sent to Phone: ")); Serial.println(TXpacketBuffer);
#endif

  // do we need a delay here for Companion App ? YES we do !!
  delay(1000);
  //---------------------------------------------

  memset(TXpacketBuffer, 0, sizeof(TXpacketBuffer)); // clear the buffer again and set new data
  sprintf(TXpacketBuffer, "!O%d;%d;%d;", OpMode, FanBalance, (BikePos + 1)); // Bikepos (0-2) needs to be converted to App SelectionIndex (1-3)
  // send these persistent data to the Settings page on the smartphone
  bleuart.print(String(TXpacketBuffer));
#ifdef DEBUGAIR
  Serial.print(F("O-Persistent data sent to Phone: ")); Serial.println(TXpacketBuffer);
#endif

  // do we need a delay here for Companion App ? YES we do !!
  delay(1000);
  //---------------------------------------------

  memset(TXpacketBuffer, 0, sizeof(TXpacketBuffer)); // clear the buffer again and set new data
  sprintf(TXpacketBuffer, "!C%d;%d;%d;", TAWC_Mode, CP60, w_prime_usr); // 
  // send these persistent data to the Settings page on the smartphone
  bleuart.print(String(TXpacketBuffer));
#ifdef DEBUGAIR
  Serial.print(F("C-Persistent data sent to Phone: ")); Serial.println(TXpacketBuffer);
#endif

  // do we need a delay here for Companion App ? YES we do !!
  delay(1000);
  //---------------------------------------------

  memset(TXpacketBuffer, 0, sizeof(TXpacketBuffer)); // clear the buffer again and set new data
  sprintf(TXpacketBuffer, "!D%s;", DisplaySettings); // Fill D-packet with Display Settings string
  // send these persistent data to the Settings page on the smartphone
  bleuart.print(String(TXpacketBuffer));
#ifdef DEBUGAIR
  Serial.print(F("D-Persistent data sent to Phone: ")); Serial.println(TXpacketBuffer);
#endif

} // -------------------------------

void prph_disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void) conn_handle;
  (void) reason;
  ConnectedToMobilePhone = false;
  // Show the message on the Oled
  ShowOnOledLarge("Smartphone", "Lost!", 1000);
#ifdef DEBUGAIR
  Serial.println(F("[Prph] Disconnected"));
#endif
}

void prph_bleuart_rx_callback(uint16_t conn_handle) {
  (void) conn_handle;
  // Read data received over BLE Uart from Mobile Phone
  char RXpacketBuffer[20] = { 0 };
  bleuart.read(RXpacketBuffer, 20);
#ifdef DEBUGAIR
  Serial.print(F("\n[Prph] RX: "));
  Serial.println(RXpacketBuffer);
#endif

  // The following routines parse and process the incoming commands
  // Every RXpacket starts with a '!' otherwise corrupt/invalid
  if (RXpacketBuffer[0] != '!') {
#ifdef DEBUGAIR
    Serial.println(F("[Prph] RX: packet does not start with a ! "));
#endif
    return; // invalid RXpacket: do not further parse and process
  }
  // RXpacket buffer has IdCode = "S" SSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSSS
  if (RXpacketBuffer[1] == 'S') { // Settings packet
    // Besides what is mechanically possible there are also limits in what is physically pleasant/comfortable
    // New Settings have arrived --> parse, set values and store persistently
    // WARNING: arduino sscanf can't process floating points and NOTICE: CRITICAL sscanf format specifiers !!!!!!!!
    sscanf(RXpacketBuffer, "!S%hu;%hu;%hu;", &UpperThreshold, &LowerThreshold, &GE);
#ifdef DEBUGAIR
    Serial.print(F("S-Settings "));
    Serial.println(RXpacketBuffer);
#endif
    // LittleFS for persistent storage of these values
    if (setDataToPersistence()) {
      // Confirm to the PHONE: settings rcvd and set to persistent
      // send message to phone
      bleuart.print("!SSet & Stored!;");
    }
    // Now set the scheduled OLED Display to show the SETTINGS Display
    Scheduled = -1; // Special value (-2) to be sure that the full ## seconds the display is shown during (-1)
    CallOledDisplaySchedular(); // Show the values it's showtime
    return; // Settings rcvd and set to persistent
  }// ----------end of Settings

  // RXpacket buffer has IdCode = "P" PPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPPP
  if (RXpacketBuffer[1] == 'P') { // P-Settings packet
    // New User Settings have arrived --> parse, set values and store persistently
    // WARNING: arduino sscanf can't process floating points and NOTICE: CRITICAL sscanf format specifiers !!!!!!!!
    sscanf(RXpacketBuffer, "!P%hu;%hu;", &BodyWeight, &BodyHeight);
    // Update relevant global variables accordingly !!!
    Ad = DuboisBodyArea((float)BodyWeight, (float)BodyHeight);
    CHeatCapacity = HeatCapacity((float)BodyWeight, Ad);
#ifdef DEBUGAIR
    Serial.print(F("P-Settings "));
    Serial.println(RXpacketBuffer);
    Serial.printf("Ad: %5.3f BW: %d BH: %d ", Ad, BodyWeight, BodyHeight);
    Serial.printf("CHeatCapacity: %f \n", CHeatCapacity);
#endif
    // LittleFS for persistent storage of these values
    if (setDataToPersistence()) {
      // Confirm to the PHONE: settings rcvd and set to persistent
      // send message to phone
      bleuart.print("!SSet & Stored!;");
    }
    // Now set the scheduled OLED Display to show the SETTINGS Display
    Scheduled = -1; // Special value to be sure that the full ## seconds the display is shown during (-1)
    CallOledDisplaySchedular(); // Show the values it's showtime
    return; // Settings rcvd and set to persistent
  } // end of P settings

  // RXpacket buffer has IdCode = "D" DDDDDDDDDDDDDDDDDDDDDDDDDDDDDD Display settings....
  if (RXpacketBuffer[1] == 'D') { // Settings packet
    // This packet holds a string of MAX_SCHEDULED display settings --> which screen to show (1) and which NOT (0) !!!
    // New Settings have arrived --> parse, set values and store persistently
    RXpacketBuffer[MAX_SCHEDULED + 2] = 0; // set ending separator ";" to zero (null terminated string)
    strcpy(DisplaySettings, &RXpacketBuffer[2]); // Skip "!D" and copy ONLY the MAX_SCHEDULED settings to global string variable
#ifdef DEBUGAIR
    Serial.print(F("D-Settings: "));
    Serial.printf("%s - DisplaySettings\n", DisplaySettings);
#endif
    // LittleFS for persistent storage of these values
    if (setDataToPersistence()) {
      // Confirm to the PHONE: settings rcvd and set to persistent
      // send message to phone
      bleuart.print("!DSet & Stored!;");
    }
    // Now set the scheduled OLED Display to show the SETTINGS Display
    Scheduled = -1; // Special value (-2) to be sure that the full ## seconds the display is shown during (-1)
    CallOledDisplaySchedular(); // Show the values it's showtime
    return; // Settings rcvd and set to persistent
  }// ----------end of Settings

  // RXpacket buffer has IdCode = "O" OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
  if (RXpacketBuffer[1] == 'O' && AreFansWorking) { // Operation Mode Settings packet
    // New Settings have arrived --> parse, set values and store persistently
    uint16_t iPos = 0;
    // WARNING: arduino sscanf can't process floating points and NOTICE: CRITICAL sscanf format specifiers !!!!!!!!
    sscanf(RXpacketBuffer, "!O%hu;%hd;%hu;", &OpMode, &FanBalance, &iPos);
#ifdef DEBUGAIR
    Serial.print(F("O-Settings "));
    Serial.println(RXpacketBuffer);
    Serial.printf(" --> Mode %hu Bal %hd Pos: %hu\n", OpMode, FanBalance, iPos);
#endif
    if (iPos > 0) {
      BikePos = (iPos - 1); // convert Spinner App selection index from 1-3 to 0-2
    } else BikePos = 0;
    // LittleFS for persistent storage of these values
    if (setDataToPersistence()) {
      // Confirm to the PHONE: settings rcvd and set to persistent
      //send message to phone
      bleuart.print("!OSet & Stored!;");
    }
    // Now set the scheduled OLED Display to show the SETTINGS Display
    Scheduled = -1; // Special value (-2) to be sure that the full ## seconds the display is shown during (-1)
    CallOledDisplaySchedular(); // Show the values it's showtime
    return; // Settings rcvd and set to persistent
  } else { // Fans are not working end message to phone
    if (RXpacketBuffer[1] == 'O') {
      bleuart.print("!OOut of Order!;");
      return;
    }
  } // end of O settings
  
  // RXpacket buffer has IdCode = "C" CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC
  if (RXpacketBuffer[1] == 'C') { // Settings packet
    // Track Anaerobic Capacity, CP60 and W-Prime settings
    // New Settings have arrived --> parse, set values and store persistently
    // WARNING: arduino sscanf can't process floating points and NOTICE: CRITICAL sscanf format specifiers !!!!!!!!
    sscanf(RXpacketBuffer, "!C%hu;%hu;%hu;", &TAWC_Mode, &CP60, &w_prime_usr);
#ifdef DEBUGAIR
    Serial.print(F("C-Settings: "));
    Serial.println(RXpacketBuffer);
#endif
    // Check floor and ceiling values for user set CP60 and w_prime, to garantee best possible estimates !!
    ConstrainW_PrimeValue(CP60, w_prime_usr);
    // Set estimated start values for variables that relate to CP and w_prime
    eCP = CP60;
    ew_prime_mod = w_prime_usr;
    ew_prime_test = w_prime_usr;
    IsShowWprimeValuesDominant = false;
    if (TAWC_Mode == 1) { // Set Yes/No Show Values on Oled Display
      DisplaySettings[10] = '1'; // Yes ShowWprimeValuesOnOled
    } else (DisplaySettings[10] = '0'); // Do not show
#ifdef DEBUGAIR
    Serial.printf("%d ", TAWC_Mode);
    Serial.print(F("Display-Settings: "));
    Serial.println(DisplaySettings);
#endif
    // LittleFS for persistent storage of these values
    if (setDataToPersistence()) {
    // Confirm to the PHONE: settings rcvd and set to persistent
    // send message to phone
      bleuart.print("!CSet & Stored!;");
    }
    // Now set the scheduled OLED Display to show the SETTINGS Display
    Scheduled = -1; // Special value (-2) to be sure that the full ## seconds the display is shown during (-1)
    CallOledDisplaySchedular(); // Show the values it's showtime
    return; // Settings rcvd and set to persistent
  } // ----------end of C-Settings

  // Manual Control Settings get parsed and processed! MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM
  if (RXpacketBuffer[1] == 'M' && AreFansWorking) {
#ifdef DEBUGAIR
    Serial.println("Process Manual Percentage Setting for both fans!");
#endif
    uint16_t iUFperc = 0, iLFperc = 0;
    // WARNING: arduino sscanf can't process floating points and NOTICE: CRITICAL sscanf format specifiers !!!!!!!!
    sscanf(RXpacketBuffer, "!M%hu;%hu;", &iUFperc, &iLFperc);
    ActualUpperFanPerc = constrain(iUFperc, 0, 100);
    ActualLowerFanPerc = constrain(iLFperc, 0, 100);
    // Hard overruling of any algorithmic interference: we are running MANUAL mode NOW!
    OpMode = MANUAL;
    // Notice: Current FAN BALANCE Value IS IGNORED, SINCE IT IS SET NOW by the user....
    SetBothFanPeriods(); // Translate Perc to Millis Duty cycle Fan operation
    // Now set the scheduled OLED Display to show the FAN-SETTINGS Display
    Scheduled = 0; // First display is shown
    CallOledDisplaySchedular(); // Show the values if showtime
    return;
  } else { // Fans are not working end message to phone
    if (RXpacketBuffer[1] == 'M') {
      bleuart.print("!MOut of Order!;");
      return;
    }
  }
}//--------------------------

void scan_stop_callback(void) {
#ifdef DEBUGAIR
  Serial.println(F("Scanning by Central is stopped after timeout..."));
#endif
  // Show the message on the Oled
  ShowOnOledLarge("Scanning", "Timeout!", 1000);
  if (!IsConnectedToHRM) {
    HRM_conn_handle = 0;
  }
  if (!ConnectedToCPS) {
    CPS_conn_handle = 0;
  }
}

void adv_stop_callback(void) {
#ifdef DEBUGAIR
  Serial.println(F("Advertising by Peripheral is stopped after timeout..."));
#endif
  ConnectedToMobilePhone = false;
  // Show the message on the Oled
  ShowOnOledLarge("Advertise", "Timeout!", 1000);
}

/**
   Callback invoked when scanner picks up advertising data
   @param report Structural advertising data
*/
void scan_callback(ble_gap_evt_adv_report_t* report) {
  // Since we configured the scanner with filterUuid, scan_callback is invoked for
  // all desired BLE devices..... we  still have to handle the specific devices
  // Check for POWER Sensor/trainer and/or HRM strap...
  // It is critical to stop further scanning in the scan process before timeout!

  // --------------------------------------------------------------------------------------
  uint8_t buffer[32];
  memset(buffer, 0, sizeof(buffer));
#ifdef DEBUGAIR
  Serial.println(F("Device found:"));
  /* Shortened Local Name */
  if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, buffer, sizeof(buffer)))
  {
    Serial.printf("%14s %s | ", "SHORT NAME", buffer);
    memset(buffer, 0, sizeof(buffer));
  }
  Serial.println(F("Timestamp Addr              Rssi Data"));
  Serial.printf("%09d ", millis());
  // MAC is in little endian --> print reverse
  Serial.printBufferReverse(report->peer_addr.addr, 6, ':');
  Serial.print(F(" "));
  Serial.print(report->rssi);
  Serial.print(F("  "));
  Serial.printBuffer(report->data.p_data, report->data.len, '-');
  Serial.println();
#endif

  // Check if picked-up advertising contains the wanted HRM strap service
  if (!IsConnectedToHRM) {
    if (Bluefruit.Scanner.checkReportForUuid(report, UUID16_SVC_HEART_RATE)) {
      /* Complete Local Name */
#ifdef DEBUGAIR
      Serial.println("HRM service detected");
#endif
      if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
      {
#ifdef DEBUGAIR
        Serial.printf("%14s %s\n", "COMPLETE NAME", buffer);
#endif
        memcpy(HRMdeviceName, buffer, 10);
        memset(buffer, 0, sizeof(buffer));
      }
      Bluefruit.Central.connect(report);
    }
  }

  // Check if picked-up advertising contains the wanted CPS service
  if (!ConnectedToCPS) {
    if (Bluefruit.Scanner.checkReportForUuid(report, 0x1818)) {  //UUID16_SVC_CYCLING_POWER 0x1818
#ifdef DEBUGAIR
      Serial.println("CPS service detected");
#endif
      // Get Complete Local Name if reported
      if (Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, buffer, sizeof(buffer)))
      {
#ifdef DEBUGAIR
        Serial.printf("Cycling Power Sevice delivered by: %14s %s\n", "COMPLETE NAME", buffer);
#endif
        memcpy(CPSdeviceName, buffer, 10);
        memset(buffer, 0, sizeof(buffer));
      }
      Bluefruit.Central.connect(report);
    }
  }
} // end

/**
   Callback invoked when a connection is established
   @param conn_handle
*/
void connect_callback(uint16_t conn_handle) {
  // Check for HRM or CPS conn_handle else failed and disconnect
  uint8_t loc_value = 0;
  char SensorLoc[10] = {0};
  // If HRM is found check for body location
  if (hrms.discover(conn_handle) ) {
    // Once HRM service is found, we continue to discover its characteristic
#ifdef DEBUGAIR
    Serial.println("Found HRM!");
#endif
    if ( !hrmc.discover() ) {
      // Measurement chr is mandatory, if it is not found (valid), then disconnect
#ifdef DEBUGAIR
      Serial.println(F("Failed HRMC and disconnecting ..."));
#endif
      Bluefruit.disconnect(conn_handle);
      return;
    }
    if ( bslc.discover() ) { // NOT fatal if not found
      // Read 8-bit BSLC value from peripheral
      loc_value = bslc.read8();
#ifdef DEBUGAIR
      Serial.print("Found Body Location of Sensor: "); Serial.println(HRMSensorLoc_str[loc_value]);
#endif
      // convert to string of chars
      memset(SensorLoc, 0, sizeof(SensorLoc));
      strcpy(SensorLoc, HRMSensorLoc_str[loc_value]);
    } // bslc
    // Reaching here means we are ready to go, let's enable notification on measurement heartbeats/m
    // CHANGED THAT AND POSTPONED INCOMING MEASUREMENTS AND THEIR PROCESSING SINCE THEY INTERFERED WITH A STABLE
    // PROCESSING OF MANY OTHER TASKS IN SETUP !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    /*    if ( hrmc.enableNotify() ) { // First CONNECT !! */
    IsConnectedToHRM = true;
    if (HRM_conn_handle != 0) { // in the case of a RECONNECT (!) of a lost HRM connection !!
      hrmc.enableNotify();       // let's enable notification on measurement heartbeats/m
    }
    HRM_conn_handle = conn_handle;
#ifdef DEBUGAIR
    Serial.println("Ready to receive HRM Measurement value");
#endif
    ShowOnOledLarge(HRMdeviceName, SensorLoc, 3000);
    /*    } else {
      #ifdef DEBUGAIR
          Serial.println("Couldn't enable notify for HRM Measurement. Increase DEBUG LEVEL for troubleshooting");
      #endif
        }
    */
    return; // Found HRM now return otherwise it will disconnect!!!
  } // end hrms  ---------------------------------

  /* Check for CPS services to be present  */
  if (cps.discover(conn_handle) ) {
#ifdef DEBUGAIR
    Serial.println(F("Found CPS!"));
#endif
    // If CPMC is not found, disconnect and return
    if ( !cpmc.discover() ) {
#ifdef DEBUGAIR
      Serial.println(F("Failed CPMC and disconnecting ..."));
#endif
      // disconnect since we couldn't find crucial service
      Bluefruit.disconnect(conn_handle);
      return;
    }
    // CPLC
    if ( cplc.discover() ) {
      // The Sensor Location characteristic
      // See: https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.characteristic.sensor_location.xml
      // power sensor location value is 16 bit
      // Read 16-bit cplc value from peripheral
      loc_value = cplc.read8();
#ifdef DEBUGAIR
      Serial.print("Power Location Sensor: ");
      Serial.printf("Loc#: %d %s\n", loc_value, CPSSensorLoc_str[loc_value]);
#endif
    } else {
#ifdef DEBUGAIR
      Serial.println("Found NO Sensor Location");
#endif
    }
    if (strlen(CPSdeviceName) == 0) { // CPS did not deliver a device name
      strcpy(CPSdeviceName, "No Name\0");
      // check the BLE DIS Service (if present) to insert a name...
      if (TryDISservice(conn_handle)) {
      }
    }
    // Put it all together and show
    // convert to string of chars
    memset(SensorLoc, 0, sizeof(SensorLoc));
    strcpy(SensorLoc, CPSSensorLoc_str[loc_value]);
    strcat(CPSdeviceName, " ");
    strcat(CPSdeviceName, CPSModelNum);
    CPSdeviceName[10] = 0; // Limit the length of the string to 10 chars
    // ---------------------------------------------------------------------------------
    // Reaching here means we are ready to go, let's enable notification on reading data
    // ---------------------------------------------------------------------------------
    // CHANGED THAT AND POSTPONED INCOMING MEASUREMENTS AND THEIR PROCESSING SINCE THEY INTERFERED WITH A STABLE
    // PROCESSING OF MANY OTHER TASKS IN SETUP !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

    /*    if ( cpmc.enableNotify() ) { // To set ALL CLEAR: READY TO ROCK AND ROLL */
    ConnectedToCPS = true;
    if (CPS_conn_handle != 0) { // in the case of a RECONNECT (!) of a lost CPS connection !!
      cpmc.enableNotify();      // let's enable notification on reading data
    }
    CPS_conn_handle = conn_handle;
#ifdef DEBUGAIR
    Serial.println(F("Ready to receive CPS data"));
#endif
    ShowOnOledLarge(CPSdeviceName, SensorLoc, 3000);
    /*    }  else {
      #ifdef DEBUGAIR
          Serial.println(F("Couldn't enable notify for CPS data"));
      #endif
        }
    */
    return; // Found & enabled CPS, now return otherwise it will disconnect!!!
  }// END CPS

  // Reaching here means NO or invalid HRM or CPS handle.....
#ifdef DEBUGAIR
  Serial.println(F("Failed and disconnecting ..."));
#endif
  // disconnect since we couldn't find crucial service
  Bluefruit.disconnect(conn_handle);
} // Done connect callback --------------------------------------------

bool TryDISservice(uint16_t conn_handle) {
  char buffer[32] = {0};
  // Look for diss and find name!
  if ( diss.discover(conn_handle) ) {
#ifdef DEBUGAIR
    Serial.print(F("Found DIS !\n"));
#endif
    if ( disma.discover() ) {
      // read and print out CPSdeviceName
      memset(buffer, 0, sizeof(buffer));
      if ( disma.read(buffer, sizeof(buffer)) ) {
#ifdef DEBUGAIR
        Serial.print("DIS Manufacturer name: ");
        Serial.println(buffer);
#endif
        memcpy(CPSdeviceName, buffer, 10);
      }
    } // end disma

    if ( dismo.discover() ) {
      // read and print out Model Number
      memset(buffer, 0, sizeof(buffer));
      if ( dismo.read(buffer, sizeof(buffer)) ) {
#ifdef DEBUGAIR
        Serial.print("DIS Model Number: ");
        Serial.println(buffer);
#endif
        memcpy(CPSModelNum, buffer, 10);
      }
    } // end dismo
    return true;
  } // end diss --> no more relevant characteristics of Device Information Service
  else { // NOT fatal !!
#ifdef DEBUGAIR
    Serial.println(F("Found NO Device Information!"));
#endif
    return false;
  }
}// end function

// Function to count up after SCHEDULED_TIME for Oled display sequence
void DisplaySchedular_callback(TimerHandle_t _handle) {
  // 0 to MAX_SCHEDULED are assigned
  do {
    if (++Scheduled < MAX_SCHEDULED) {
    } else { // MAX is reached
      Scheduled = 0; // start all over again
    }
  } while (DisplaySettings[Scheduled] == '0');
  TimeCaptureMillis = millis();
}

// Function to sequentially show different Oled displays with data/values
void CallOledDisplaySchedular(void) {
  const unsigned long TimeSpan = SCHEDULED_TIME/2; // Half of TIME: 5 seconds
  unsigned long TimeProgressed = millis() - TimeCaptureMillis;
  if (IsShowWprimeValuesDominant) { Scheduled = 10; } // ONLY ShowWprimeValuesOnOled will be shown !!
  switch (Scheduled) {
    case -1: { // SHOW NEW SETTINGS DISPLAY
        ShowSettingsOnOled();
        break;
      }
    case 0 : { // Fan Operational Values, divide screen time
        if (TimeProgressed < TimeSpan) {
          ShowFanSettingsOnOled();
        } else {
          ShowRequestedAirflowValueOnOled();
        }
        break;
      }
    case 1 : { //
        ShowEnergyProductionValuesOnOled();
        break;
      }
    case 2 : { //
        ShowHeatProductionValuesOnOled();
        break;
      }
    case 3 : { //
        ShowStoredHeatValuesOnOled();
        break;
      }
    case 4 : { //
        ShowRad_ConvValuesOnOled();
        break;
      }
    case 5 : { //
        ShowEreq_SwrateValuesOnOled();
        break;
      }
    case 6 : { //
        ShowTcoreValuesOnOled();
        break;
      }
    case 7 : { //
        ShowTbody_TskinValuesOnOled();
        break;
      }
    case 8 : {  //
        if (TimeProgressed < TimeSpan) { // divide screen time
          ShowAirTemp_HumidityValuesOnOled();
        } else {
          ShowPwr_HbmValuesOnOled();
        }
        break;
      }
    case 9 : { //
        ShowWorkValueOnOled();
        break;
      }
    case 10 : { // DO NOT CHANGE this scheduled position nr# 10 see TAWC_Mode and appropriate DisplaySettings[10]
      /*
        if (TimeProgressed < TimeSpan) {
        ShowWprimeValuesOnOled(0);
        } else {
        ShowWprimeValuesOnOled(1);  
        }
      */
      ShowWprimeValuesOnOled();
        break;
      }
    case MAX_SCHEDULED : {
        break;
      } // NO ACTION YET!!!
  }
} // ---------

void ProgressBar(unsigned long StartTime, unsigned long TimeSpan) {
  // Located at the bottom of the screen
  static uint16_t X0 = 1, Y0 = 59, W = 127, H = 5;
  unsigned long TimeProgressed = millis() - StartTime;
  // if (TimeProgressed>=TimeSpan) { return; }
  uint16_t Fw = (uint16_t)(((double)TimeProgressed / (double)(TimeSpan)) * 127); // Type cast to get rid of decimals
  display.setTextColor(SSD1306_WHITE);
  display.drawRect( X0, Y0, W, H, SSD1306_WHITE);
  display.fillRect( X0, (Y0 + 2), Fw, (H - 4), SSD1306_WHITE);
  display.display();
  delay(200);
}

void ShowIconsOnTopBar(uint8_t Mode) {
  // Show heat State on Top Bar
  if (HeatState == 1) { display.drawBitmap(1, 0, chevron_up_icon16x16, 16, 16, 1); }
  if (HeatState == 2) { display.drawBitmap(1, 0, chevron_down_icon16x16, 16, 16, 1); }
  if (Mode == 1) { // Show Heat Balance State Text on Top Bar
    const unsigned long TimeSpan = 2000; // 2 seconds
    unsigned long TimeProgressed = millis() - TimeCaptureMillis;
    if (TimeProgressed < TimeSpan) {
      char HeatBalance[11];
      memset(HeatBalance, 0, sizeof(HeatBalance));
      display.setTextSize(2);
      strcpy(HeatBalance, State_str[HeatState]);
      display.setCursor(32, 0); 
      display.print(HeatBalance);
      return;
    }
  }
  // Show Icons on Top Bar
  if (IsConnectedToHRM) { // show icon
    display.drawBitmap(32, 0, heart_icon16x16, 16, 16, 1); // 37, 0
  }
  if (ConnectedToCPS) { // show icon
    display.drawBitmap(52, 0, power_icon16x16, 16, 16, 1); // 8,0
  }
  if (ConnectedToMobilePhone) { // show icon
    display.drawBitmap(72, 0, mobile_icon16x16, 16, 16, 1); //64,0
  }
  if (IsTempSensor) { // show icon
    display.drawBitmap(96, 0, humidity_icon16x16, 16, 16, 1); // 92,0
    display.drawBitmap(112, 0, temperature_icon16x16, 16, 16, 1); //108,0
  }
}

// Notice: Character set is default 5*7 PLUS extra bit space in height and width
// Textsize= 1*(5*7) characters  6*8 pixels wide-height
// Textsize= 2*(5*7) characters 12*15 pixels wide-height 4 lines with 10 chars
// Textsize= 3*(5*7) characters 18*22 pixels wide-height

void ShowOnOledLarge(char *Line1, char *Line2, uint16_t Pause) {
  // Clear and set Oled to display 3 line info -> centered
  int pos = 1;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(0);
  display.setTextSize(2);
  if (Line1) {
    pos = round( (127 - (12 * strlen(Line1))) / 2 );
    display.setCursor(pos, 20); // 16 + 4
    display.print(Line1);
  }
  if (Line2) {
    pos = round( (127 - (12 * strlen(Line2))) / 2 );
    display.setCursor(pos, 40); // 2*(16 + 4)
    display.print(Line2);
  }
  display.display();
  delay(Pause);  // Pause indicated time in ms
}

// Notice: Character set is default 5*7 PLUS (!) 1 bit space in height and width
// Textsize= 1*(5*7) characters  6*8 pixels wide-height
// Textsize= 2*(5*7) characters 12*15 pixels wide-height 4 lines with 10 chars
// Textsize= 3*(5*7) characters 18*22 pixels wide-height

// Funtion to show some settings values on Oled screen
void ShowSettingsOnOled(void) {
  char tmp[12];
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(6, 0);
  display.printf("%3d %2d %3d", CP60, BodyWeight, BodyHeight);
  display.setCursor(6, 17); // 50
  if (FanBalance < 0) {
    display.printf("%3d%%  U%2d%%", GE, -FanBalance);
  } else {
    if (FanBalance > 0) {
      display.printf("%3d%%  L%2d%%", GE, FanBalance);
    } else { // == 0 no setting
      display.printf("%3d%%   %2d%%", GE, FanBalance);
    }
  }
  display.setCursor(6, 33);
  display.printf("%3d%%  %3d%%", UpperThreshold, LowerThreshold);
  // Show Operation Mode ------------------
  display.setCursor(6, 50);
  strcpy(tmp, OpMode_str[OpMode]);
  display.print(tmp);
  strcpy(tmp, BikePos_str[BikePos]);
  display.print(tmp);
  display.display();
}// -----------------------------------

// Line numbers for Display layout zones: icontopbar, line #1 en line #2
#define HLINE1 18
#define HLINE2 43 // was 44 onderste cijfers missen een streepje...

// Funtion to show Fan Settings/Values on Oled screen
void ShowFanSettingsOnOled(void) {
  char tmp[12];
  const double MAX = 100;
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  // Show Upper Fan Values ----------------
  display.setTextSize(2);
  display.setCursor(8, 1); // 20
  sprintf(tmp, "%3d%%  %4d", ActualUpperFanPerc, ActualUpperFanPeriod);
  display.print(tmp);
  uint16_t X0 = 1, Y0 = 20, W = 127, H = 10;
  double Tscale = ((ActualUpperFanPerc / MAX) * 127);
  uint16_t Fw = uint16_t(Tscale); // Type cast to get rid of decimals
  display.drawRect( X0, Y0, W, H, SSD1306_WHITE);
  display.fillRect( X0, (Y0 + 2), Fw, (H - 4), SSD1306_WHITE);
  // Show Lower Fan values -----------------
  display.setCursor(8, 36); // 20
  sprintf(tmp, "%3d%%  %4d", ActualLowerFanPerc, ActualLowerFanPeriod);
  display.print(tmp);
  X0 = 1, Y0 = 54, W = 127, H = 10;
  Tscale = ((ActualLowerFanPerc / MAX) * 127);
  Fw = uint16_t(Tscale); // Type cast to get rid of decimals
  display.drawRect( X0, Y0, W, H, SSD1306_WHITE);
  display.fillRect( X0, (Y0 + 2), Fw, (H - 4), SSD1306_WHITE);
  display.display();
}// -----------------------------------

// Funtion to show Requested AirFlow as a result of Energy Production
void ShowRequestedAirflowValueOnOled(void) {
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(10, 1);
  display.print("Air Speed");
  // Show operation and bike position Mode ------------------
  display.setCursor(6, 50);
  strcpy(tmp, OpMode_str[OpMode]);
  display.print(tmp);
  strcpy(tmp, BikePos_str[BikePos]);
  display.print(tmp);
  //--------------------------------
  display.setTextSize(2);
  if (AirSpeedUnit == MPS) {
    display.drawBitmap(104, (HLINE1+2), perSecond_icon24x22, 24, 22, 1); 
    display.setCursor(102, (HLINE1+2)); // 102 for k only
    display.print("m");  // units in m/s  (mps)
  } else {
    display.drawBitmap(104, (HLINE1+2), perHour_icon24x22, 24, 22, 1); 
    display.setCursor(88, (HLINE1+2));
    if (AirSpeedUnit == KPH) {
      display.print("km");  // units in km/h  (kph)
    } else { if (AirSpeedUnit == MPH) {
        display.print("mi");  // units in mi/h  (mph)
        }
      }
    }

  //---------------------------------
  display.setTextSize(3);
  if (AirSpeedUnit == MPS) {
    display.setCursor(26, (HLINE1+2) ); //  26 HLINE2
    dtostrf(Vair, 4, 1, tmp ); 
  } else { 
      display.setCursor(12, (HLINE1+2) ); 
      if (AirSpeedUnit == KPH) {
      dtostrf((Vair * 3.6), 4, 1, tmp ); // convert m/s to km/h
      } else { if (AirSpeedUnit == MPH) {
          dtostrf((Vair * 2.24), 4, 1, tmp ); // convert m/s to miles/h
        }
      }
    }
  display.print(tmp);
  display.display();
}// -----------------------------------

// Funtion to show Metabolic Energy Production as a result of cycling
void ShowEnergyProductionValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(1);
  display.setTextSize(2);
  //--------------------------------
  display.setCursor(102, HLINE1); 
  display.print("J");  // units in J/m2
  display.drawBitmap(104, HLINE1, perSquareMeter_icon24x22, 24, 22, 1);
  //---------------------------------
  display.drawBitmap(1, HLINE2, sigma_icon16x20, 16, 20, 1);
  display.drawBitmap(104, HLINE2, JouleSquareMeter_icon24x22, 24, 22, 1);  
  display.setCursor(101, (HLINE2+2) ); // +6
  display.print("k"); // units in kJ/m2
  display.setTextSize(3);
  display.setCursor(1, HLINE1);
  display.print("M");

  dtostrf(MetEnergy, 4, 0, tmp );
  display.setCursor(26, HLINE1 ); //  HLINE1
  display.print(tmp);

  dtostrf(SumEnergyProduced, 4, 0, tmp);
  display.setCursor(26, HLINE2 ); //  HLINE1
  display.print(tmp);

  display.display();
}// -----------------------------------

// Funtion to show Net Metabolic Heat Production as a result of cycling
void ShowHeatProductionValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(1);
  display.setTextSize(2); // 1 is small and 2 not so Large characters
  //--------------------------------
  display.setCursor(102, HLINE1); 
  display.print("J");  // units in J/m2
  display.drawBitmap(104, HLINE1, perSquareMeter_icon24x22, 24, 22, 1);
  //---------------------------------
  display.drawBitmap(1, HLINE2, sigma_icon16x20, 16, 20, 1);
  display.drawBitmap(104, HLINE2, JouleSquareMeter_icon24x22, 24, 22, 1);  
  display.setCursor(101, (HLINE2+2) ); // +6
  display.print("k"); // units in kJ/m2
  display.setTextSize(3);
  display.setCursor(1, HLINE1);
  display.print("H");

  dtostrf(HeatProduced, 4, 0, tmp );
  display.setCursor(26, HLINE1 ); //  HLINE2
  display.print(tmp);

  dtostrf(SumHeatProduced, 4, 0, tmp);
  display.setCursor(26, HLINE2 ); //  HLINE1
  display.print(tmp);

  display.display();
}// -----------------------------------


// Funtion to show Internal Heat Stored (S) during workout
void ShowStoredHeatValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  //--------------------------------
  display.setCursor(102, 1); 
  display.print("J");  // units in J/m2
  display.drawBitmap(104, 1, perSquareMeter_icon24x22, 24, 22, 1);
  //---------------------------------
  display.drawBitmap(1, HLINE2, sigma_icon16x20, 16, 20, 1);
  //--------------------------------
  display.setCursor(102, HLINE2); 
  display.print("J");  // units in J/m2
  display.drawBitmap(104, HLINE2, perSquareMeter_icon24x22, 24, 22, 1);
  //---------------------------------
  display.setTextSize(3);
  display.setCursor(1, 1);
  display.print("S");
  display.setCursor(26, 1);
  dtostrf(DeltaStoredHeat, 4, 0, tmp); // show sign only if negative
  if (DeltaStoredHeat > 0) { // Show a + sign to emphasize heat gain !
    tmp[0] = '+';
  }
  display.print(tmp);

  // --------------------------------
  uint16_t X0 = 1, Y0 = 27, W = 127, H = 10;
  const double MAXStoredHeat = 3500; // This value needs to be checked in practice !!!!!!!!!!!!!!!
  double Tscale = ((StoredHeat / MAXStoredHeat) * 127);
  uint16_t Fw = uint16_t(Tscale); // Type cast to get rid of decimals
  display.drawRect( X0, Y0, W, H, SSD1306_WHITE);
  display.fillRect( X0, (Y0 + 2), Fw, (H - 4), SSD1306_WHITE);
  // --------------------------------

  dtostrf(StoredHeat, 4, 0, tmp); // show sign only if negative
  display.setCursor(26, HLINE2);
  display.print(tmp);

  display.display();
}// -----------------------------------

// Funtion to show Radiation and Convection Heat Transfer Values on Oled screen
void ShowRad_ConvValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(1);
  display.setTextSize(3);
  display.setCursor(1, (HLINE1) );
  display.print("R");
  display.setCursor(1, (HLINE2) );
  display.print("C");
  display.setTextSize(2);
  //--------------------------------
  display.setCursor(102, HLINE1); 
  display.print("J");  // units in J/m2
  display.drawBitmap(104, HLINE1, perSquareMeter_icon24x22, 24, 22, 1);
  //--------------------------------
  display.setCursor(102, HLINE2); 
  display.print("J");  // units in J/m2
  display.drawBitmap(104, HLINE2, perSquareMeter_icon24x22, 24, 22, 1);
  //---------------------------------
  dtostrf(Radiation, 4, 0, tmp); // Show Heat Exchange by Radiation
  display.setCursor(26, HLINE1 );
  display.setTextSize(3);
  display.print(tmp);
  dtostrf(Convection, 4, 0, tmp); // Show Heat Exchange by Convection
  display.setCursor(26, HLINE2 );
  display.print(tmp);
  display.display();
}// -----------------------------------

// Funtion to show Ereq and Sweatrate Values on Oled screen
void ShowEreq_SwrateValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(1);
  display.setTextSize(3);
  display.setCursor(1, (HLINE1) );
  display.print("E");
  display.setTextSize(2);
  display.setCursor(1, (HLINE2 + 6) );
  display.print("Sw");
  //--------------------------------
  display.setCursor(102, HLINE1); 
  display.print("J");  // units in J/m2
  display.drawBitmap(104, HLINE1, perSquareMeter_icon24x22, 24, 22, 1);
  //--------------------------------
  display.setCursor(102, (HLINE2-2)); // -2 lower case g
  display.print("g");  // units in grams/h == mL/h
  display.drawBitmap(104, (HLINE2), perHour_icon24x22, 24, 22, 1);
  //---------------------------------
  dtostrf(Ereq, 4, 0, tmp); // Show Evaporative Heat Exchange Requested
  display.setCursor(26, HLINE1 ); // 38
  display.setTextSize(3);
  display.print(tmp);
  dtostrf(SwRate, 4, 0, tmp); // show Sweat Rate in g (or mL) per hour
  display.setCursor(26, HLINE2 );
  display.setTextSize(3);
  display.print(tmp);
  display.display();
}// -----------------------------------

// Funtion to show Temp related values on Oled screen
void ShowTcoreValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(1);
  display.drawBitmap(112, (HLINE1+2), DegreesCelsius_icon16x20, 16, 20, 1);
  display.drawBitmap(112, (HLINE2+2), DegreesCelsius_icon16x20, 16, 20, 1);
  display.setTextSize(2);
  display.setCursor(1, (HLINE2 + 4) );
  display.print("Tc"); // Tc
  display.drawBitmap(1, (HLINE1+4), delta_icon16x16, 16, 16, 1);
  display.setCursor(21, (HLINE1) ); // 14
  dtostrf(DeltaTc, 5, 2, tmp); // show sign only if negative
  if (DeltaTc > 0) { // Show a + sign to emphasize heat gain !
    tmp[0] = '+';
  }
  display.setTextSize(3);
  display.print(tmp);
  dtostrf(Tcore_cur, 4, 1, tmp); // show sign only if negative
  display.setCursor(38, HLINE2); // 32
  display.print(tmp);
  display.display();
}// -----------------------------------

// Funtion to show Body and Skin Temp values on Oled screen
void ShowTbody_TskinValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(1);
  display.drawBitmap(112, (HLINE1+2), DegreesCelsius_icon16x20, 16, 20, 1);
  display.drawBitmap(112, (HLINE2+2), DegreesCelsius_icon16x20, 16, 20, 1);
  display.setTextSize(2);
  display.setCursor(1, (HLINE1 + 4) );
  display.print("Ts");
  display.setCursor(1, (HLINE2 + 4) );
  display.print("Tb");
  dtostrf(Tskin, 4, 1, tmp); // show sign only if negative
  display.setCursor(38, HLINE1); // 32
  display.setTextSize(3);
  display.print(tmp);
  display.setCursor(38, (HLINE2) ); // 32
  dtostrf(Tbody_cur, 4, 1, tmp); // show sign only if negative
  display.print(tmp);
  display.display();
}// -----------------------------------

/* Funtion to show W-Prime data on Oled screen
void ShowWprimeValuesOnOled(uint8_t Mode) {
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  if (Mode == 0) {
    display.setCursor(1, 5); // was 1, 5
    display.print("eCP");
    display.setCursor(116, 5); //was 112, 5
    display.print("W");
    dtostrf(eCP, 4, 0, tmp); // show sign only if negative
    display.setCursor(26, 1); // was 42, 1
    display.setTextSize(3);
    display.print(tmp);
  } else {
    display.setCursor(1, 5); // was 1, 48
    display.print("W");
    display.setTextSize(1);
    display.print("'");
    display.setTextSize(2);
    display.setCursor(104, 5); // was 112, 48
    display.print("kJ");
    if (ew_prime < 10000) { // Personal setting
      dtostrf((double)ew_prime / 1000, 4, 2, tmp); // show sign only if negative
    } else dtostrf((double)ew_prime / 1000, 4, 1, tmp); // show sign only if negative
    display.setCursor(26, 1); // was 42, 1
    display.setTextSize(3);
    display.print(tmp);
  }
  display.setTextSize(2);
  display.setCursor(104, (HLINE2+6)); // was 112, 48
  display.print("kJ");
  uint16_t X0 = 1, Y0 = 27, W = 127, H = 10;
  double Tscale = ( ((double)w_prime_balance / (double)ew_prime) * 127);
  uint16_t Fw = uint16_t(Tscale); // Type cast to get rid of decimals
  display.drawRect( X0, Y0, W, H, SSD1306_WHITE);
  display.fillRect( X0, (Y0 + 2), Fw, (H - 4), SSD1306_WHITE);
  if (ew_prime < 10000) { // Personal setting
      dtostrf((double)w_prime_balance / 1000, 4, 2, tmp); // show sign only if negative
    } else dtostrf((double)w_prime_balance / 1000, 4, 1, tmp); // show sign only if negative
  display.setCursor(26, HLINE2);
  display.setTextSize(3);
  display.print(tmp);
  display.display();
}// -----------------------------------


// Funtion to show W-Prime data on Oled screen
void ShowWprimeValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(1, 5); // was 1, 5
  display.print("CP");
  display.setCursor(1, 48); // was 1, 48
  display.print("W");
  display.setTextSize(1);
  display.print("'");
  display.setTextSize(2);
  display.setCursor(116, 5); //was 112, 5
  display.print("W");
  display.setCursor(104, 48); // was 112, 48
  display.print("kJ");
  dtostrf(eCP, 4, 0, tmp); // show sign only if negative
  display.setCursor(26, 1); // was 26, 1
  display.setTextSize(3);
  display.print(tmp);
  uint16_t X0 = 1, Y0 = 27, W = 127, H = 10;
  // code accounts for complex manipulations of eW-Prime during continued depletion
  double w_prime_balance_alt = ((double)ew_prime_mod-(double)w_prime_usr ) + (double)w_prime_balance; 
  double Tscale = ((w_prime_balance_alt/(double)ew_prime_mod) * 127);
  // ---------------------------------------------------------------------------- 
  uint16_t Fw = (uint16_t)Tscale; // Type cast to get rid of decimals
  display.drawRect( X0, Y0, W, H, SSD1306_WHITE);
  display.fillRect( X0, (Y0 + 2), Fw, (H - 4), SSD1306_WHITE);
  if (ew_prime_test < 10000) { // Personal setting
    dtostrf((double)ew_prime_test / 1000, 4, 2, tmp); // show sign only if negative
  } else dtostrf((double)ew_prime_test / 1000, 4, 1, tmp); // show sign only if negative
  display.setCursor(26, HLINE2); // was 26
  display.setTextSize(3);
  display.print(tmp);
  display.display();
}// -----------------------------------
*/

// Funtion to show W-Prime data on Oled screen
void ShowWprimeValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(1, 3); // was 1, 5
  display.print("CP");
  display.setCursor(1, 48); // was 1, 48
  display.print("W");
  display.setTextSize(1);
  display.print("'");
  display.setTextSize(2);
  display.setCursor(116, 3); //was 112, 5
  display.print("W");
  display.setCursor(104, 48); // was 112, 48
  display.print("kJ");
  dtostrf(eCP, 4, 0, tmp); // show sign only if negative
  display.setCursor(26, 0); // was 42, 1
  display.setTextSize(3);
  display.print(tmp);
  uint16_t X0 = 1, Y0 = 23, W = 127, H = 18;
  // code accounts for complex manipulations of eW-Prime during continued depletion
  double w_prime_balance_alt = ((double)ew_prime_mod-(double)w_prime_usr) + (double)w_prime_balance; 
  double Tscale = ((w_prime_balance_alt/(double)ew_prime_mod) * 127);
  // ------------------------------------------------------------------------------
  uint16_t Fw = (uint16_t)(Tscale+0.5); // Type cast to get rid of decimals
  display.drawRect( X0, Y0, W, H, SSD1306_WHITE);
  display.fillRect( X0, (Y0 + 3), Fw, (H - 6), SSD1306_WHITE); // Y0+2   H-4
  // ----------------------------------------------------------------------------
  // Show W Prime balance as a percentage of what's left in the tank!!
  display.setTextSize(2);
  Tscale = ((Tscale/127))*100;
  if ((Fw-36) > 1) { // 48
    display.setTextColor(SSD1306_BLACK);
    display.setCursor((((Fw/2)+0.5)-16), (Y0+2));
  } else { 
    display.setTextColor(SSD1306_WHITE);
    display.setCursor((Fw+4), (Y0+2)); //
    }
  if (Tscale == 100) {
    dtostrf(Tscale, 3, 0, tmp); // 100% max value is 3 digits!!!
  } else {
    dtostrf(Tscale, 2, 0, tmp); // < 100% max value is 2 digits!!!
  }
  display.print(tmp);
  display.print("%");
  display.setTextColor(SSD1306_WHITE);
  // ------------------------------------------------------------------------------
  if (ew_prime_test < 10000) { // Personal setting
    dtostrf((double)ew_prime_test / 1000, 4, 2, tmp); // show sign only if negative
  } else dtostrf((double)ew_prime_test / 1000, 4, 1, tmp); // show sign only if negative
  display.setCursor(26, HLINE2);
  display.setTextSize(3);
  display.print(tmp);
  display.display();
}// -----------------------------------

// Funtion to show Temp related values on Oled screen
void ShowAirTemp_HumidityValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay(); // clean the oled screen
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(1);
  display.drawBitmap(1, (HLINE1 + 4), temperature_icon16x16, 16, 16, 1);
  display.drawBitmap(112, (HLINE1+2), DegreesCelsius_icon16x20, 16, 20, 1);
  dtostrf(Tair, 4, 1, tmp);  // show sign only if negative
  display.setCursor(38, HLINE1); // was 22
  display.setTextSize(3);
  display.print(tmp);
  display.setCursor(38, HLINE2); // was 22
  dtostrf((RH * 100), 4, 1, tmp); // show current Relative Humidity in %
  display.print(tmp);
  display.drawBitmap(1, (HLINE2 + 4), humidity_icon16x16, 16, 16, 1);
  display.setCursor(114, (HLINE2 + 6) ); // was 114, 40
  display.setTextSize(2);
  display.print(F("%"));
  display.display();
}// -----------------------------------

// Function to show current WORK in kJ and kcal (C) as a result of cycling
void ShowWorkValueOnOled(void) {
#define GARMIN_GE 24  // Garmin a.o. use this fixed GE-value in % !!
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(1);
  display.setTextSize(2);
  display.setCursor(104, (HLINE1 + 6) );
  display.print("kJ");  // units in kJoules
  display.setCursor(104, (HLINE2 + 6) ); 
  display.print("C"); // units in kcal -> Cal
  display.setCursor(116, (HLINE2 + 13) ); 
  display.setTextSize(1);
  display.print("al");
  display.drawBitmap(1, (HLINE1+4), power_icon16x16, 16, 16, 1); 
  display.drawBitmap(1, (HLINE2+4), power_icon16x16, 16, 16, 1); 
  display.setTextSize(3);
  // apply some corrections to make the result value comparable
  double SEP = ( SumEnergyProduced * (double)Ad ); // Undo individualised (div Ad) --> kJ --> NOT kJ/m2 !!
  SEP = (SEP * double(GE) / double(GARMIN_GE) ); // Correct for difference in GE and the Garmin GE
  dtostrf(SEP, 4, 0, tmp );
  display.setCursor(26, HLINE1 ); //  HLINE1
  display.print(tmp);
  dtostrf((SEP / 4.1868), 4, 0, tmp); // Convert to kcal = Calorie
  display.setCursor(26, HLINE2 ); //  HLINE1
  display.print(tmp);

  display.display();
}// -----------------------------------

// Function to show current Power and Heartbeat values as a result of cycling
void ShowPwr_HbmValuesOnOled(void) {
  char tmp[12];
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  ShowIconsOnTopBar(1);
  display.setTextSize(2);
  display.drawBitmap(1, HLINE1 + 4, power_icon16x16, 16, 16, 1);
  display.drawBitmap(1, HLINE2 + 4, heart_icon16x16, 16, 16, 1);
  display.setCursor(116, (HLINE1 + 6) );
  display.print("W");  // units in Watts
  //------------------
  display.drawBitmap(104, (HLINE2), perMinute_icon24x22, 24, 22, 1);  
  display.setCursor(102, (HLINE2) );
  display.print("B"); // units in Heart Beats per Minute
  // ----------------
  display.setTextSize(3);
  dtostrf(CPS_average, 4, 0, tmp );
  display.setCursor(26, HLINE1 ); //  HLINE1
  display.print(tmp);

  dtostrf((double)HBM_average, 4, 0, tmp);
  display.setCursor(26, HLINE2 ); //  HLINE1
  display.print(tmp);

  display.display();
}// -----------------------------------

/**
   Callback invoked when a connection is dropped
   @param conn_handle
   @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
*/
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  /*  (void) conn_handle;
    (void) reason; */
  if (HRM_conn_handle == conn_handle) {
    // in case of autoreconnect scan for:
    Bluefruit.Scanner.filterUuid(UUID16_SVC_HEART_RATE); // Use filterUuid for HRM strap
    IsConnectedToHRM = false;
    //HRM_conn_handle = 0;
    // Show the message on the Oled
    ShowOnOledLarge("HRM strap", "Lost!", 500);
#ifdef DEBUGAIR
    Serial.print(F("HRM is disconnected, reason = 0x")); Serial.println(reason, HEX);
#endif
  }
  if (CPS_conn_handle == conn_handle) {
    // in case of autoreconnect scan for:
    Bluefruit.Scanner.filterUuid(0x1818);   //UUID16_SVC_CYCLING_POWER);  Use filterUuid for CPS 0x1818
    ConnectedToCPS = false;
    //CPS_conn_handle = 0;
    // Show the message on the Oled
    ShowOnOledLarge("CyclePower", "Lost!", 500);
#ifdef DEBUGAIR
    Serial.print(F("Power is disconnected, reason = 0x")); Serial.println(reason, HEX);
#endif
  }
  Bluefruit.Scanner.start(1000); // 0 = Don't stop scanning or after n, in units of 10 ms (hundredth of a second) 1000 = 10 seconds
}

// Estimate Skin Temperature as a function of Air temperature, Air Flow, Vapour pressur and Core Temperature
float EstimateSkinTemp(float AirTemp, float PresAir, float AirVelo, float Met, float CoreTemp) {
  // Mehnert P. et al., 2000    --> Notice that these equations are valid in relative steady state situations....
  // Tradiant == Tair !!
  // Tskin =  7.19 + 0.064*Tair + 0.061Tradiant + 0.198*Pair - 0.348*Vair + 0.616*Tcore // Nude subjects
  // Tskin = 12.17 + 0.020*Tair + 0.044Tradiant + 0.194*Pair - 0.253*Vair + 0.0029*Met + 0.513*Tcore // clothed subjects

  //return (7.19 + 0.125*AirTemp + 0.198*PresAir - 0.348*AirVelo + 0.616*CoreTemp); // Nude

  // FILTER/SMOOTH: To avoid that (sprint generated) bursts in MetEnergy will cause spikes in Tskin
  float Met_Filtered = (float)EMA_MetEnergyFilter( int(Met + 0.5) );
  return (12.17 + 0.064 * AirTemp + 0.194 * PresAir - 0.253 * AirVelo + 0.0029 * Met_Filtered + 0.513 * CoreTemp); // Clothed
}


double PercentageofChange(double Initial, double Final)
{
  return (100 * (Final - Initial) / Initial);
}

// Calculate P: the partial water vapor pressure at T(emperature) in mmHg or pKa
double Pantoine(double T)
{
  // outcome in milliBar
  double A = 18.956;
  double B = 4030.18;
  double C = 235;
  double exponent = A - (B / (C + T));
  return exp(exponent) / 10; // div 10 -> Convert standard milliBar (mB) to pKa
}

// Calculate Ad = Body Surface Area according to Dubois
float DuboisBodyArea(float BW, float BH) {
  return (float)(0.00718 * pow((double)BW, 0.425) * pow((double)BH, 0.725));
}

// HeatCapacity Constant 3747 (Joules/kg/C) Bw in Kg Ad in m2
float HeatCapacity(float BW, float Area) {
  return (0.83 * 4184 * BW) / Area;
}

float HeatChange(float Tc, float Tp, float Tdiff)
{
  if ((Tc - Tp) == 0) {
    return 0.0;
  }
  return CHeatCapacity * (Tc - Tp) / Tdiff;
}

/*
  Author: Mark J. Buller
  Adapted from: Buller MJ, Tharion WJ, Cheuvront SN, Montain SJ, Kenefick RW, Castellani J,
  Latzka WA, Roberts WS, Richter M, Jenkins OC, Hoyt RW. (2013)
  Estimation of human core temperature from sequential heart rate observations.
  Physiological Measurement 34 781–798. (Matlab Function Appendix)

  Inputs: Previous or intial core temperature, Previous or initial variance, Current heart rate.
  Output: Current core temperature, and Current variance

  TCprev floating point - previous Tcore
  Vprev floating point - previous Variance
  HRcurrent - floating point - current Heart Rate Value
  Sets values to global variable Tcore_cur current (core body temperature)
  and v_cur (current variance)

*/
void EstimateTcore(double TCprev, double Vprev, double HRcurrent) {
  //Model Parameters
  double a = 1;
  double gamma = pow(0.022, 2);
  double b_0 = -7887.1;
  double b_1 = 384.4286;
  double b_2 = -4.5714;
  double sigma = pow(18.88, 2);

  //Initialize Kalman filter
  double x = TCprev;
  double v = Vprev;

  //Time Update Phase
  double x_pred = a * x;                    //Equation 3
  double v_pred = (pow(a, 2)) * v + gamma;  //Equation 4

  //Observation Update Phase
  double z = HRcurrent;
  double c_vc = 2 * b_2 * x_pred + b_1;                     //Equation 5
  double k = (v_pred * c_vc) / (pow(c_vc, 2) * v_pred + sigma); //Equation 6
  x = x_pred + k * (z - (b_2 * pow(x_pred, 2) + b_1 * x_pred + b_0)); //Equation 7
  v = (1 - k * c_vc) * v_pred;                              //Equation 8

  // set global variables with result value
  Tcore_cur = x;
  v_cur = v;
}

uint8_t HeatBalanceState(int current_value) { // Determine the state of the Heat equation
  int EMA_Value = 0, EMA_Value_Diff = 0;
  static int EMA_Value_prev = 0;
  int BandWidth = int(0.0025 * (float)EMA_Value_prev); // Plus or Minus 0.25% Bandwidth with respect to previous EMA_value
  // Update HeatBalance values now !!
  EMA_Value = EMA_HeatBalanceFilter(current_value);
  EMA_Value_Diff = (EMA_Value - EMA_Value_prev);
  EMA_Value_prev = EMA_Value;
  if ( (EMA_Value_Diff < -BandWidth) ) {
    return 2;  // Heat Loss
  }
  if ( (EMA_Value_Diff > +BandWidth) ) {
    return 1;  // Heat Gain
  }
  return 0; // Balanced within the bandwidth --> Steady State reached
}

void HeatBalanceAlgorithm(void) {
  double F2, F3;
  float DeltaPska, DeltaTska, Psa;
  float Psk = 0.0; // Partial vapor pressure at skin temperature
  float Pair = 0.0; // Partial vapor pressure at ambient air temperature
  // -----------------------------------------------------------------------------------------------
  // NOTICE: Dry-Bulb Temperature EQUALS, in indoors situation, the Ambient Air Temperature --> Tdb = Tair
  //------------------------------------------------------------------------------------------------
  // Algorithm that estimates Core Temp. based on HeartRateMeasurement >>>>> AT ONE MINUTE INTERVALS !!
  if (IsConnectedToHRM) { // Only calculate and update Core Temperature when connected to HRM-strap
    EstimateTcore(Tcore_prev, v_prev, (double)HBM_average); // Estimate the Core Body Temperature from Heart Beat sequence
  }
  v_prev = v_cur;

  // Calculate environmental variables
  Psa = (float)Pantoine((double)Tair); // in units kPa
  Pair = RH * Psa; // saturated water vapour pressure at ambient temperature corrected for Relative Humidity (default = 70%)

  // Calculate Metabolic Energy
  MetEnergy = (double)( CPS_average * ((float)(100 / GE)) ) / Ad; // in Joules/m2 --> individualized (div Ad)

  // Estimate now the mean body skin temperature from
  Tskin = EstimateSkinTemp(Tair, Pair, Vair, MetEnergy, (float)Tcore_cur);

  // Continued: Calculate environmental variables and determine the appropriate Delta's
  Psk = (float)Pantoine((double)Tskin); // saturated water vapour pressure at the wetted skin surface in units kPa
  DeltaPska = (Psk - Pair); // Calculate the DeltaPska pKa
  DeltaTska = (Tskin - Tair); // Calculate DeltaTska in degrees Celsius

  // Determine now the Mean Body Temperature from estimated Core Temperature and mean estimated Skin temperature
  // Burton, 1935 (64*36) and Kerslake, 1972 (67*33)
  //Tbody_cur = float( (float)0.64*Tcore_cur + 0.36*Tskin ); // equation according to Burton, 1935 (confirmed in other papers)
  Tbody_cur = (float)0.67 * Tcore_cur + 0.33 * Tskin; // equation according to Kerslake, 1972 (confirmed in other papers)

  // Calculate how much heat has been produced since previous round
  if (IsConnectedToHRM) { // Only calculate and update HeatChange when connected to HRM-strap
    if (Tcore_cur > Tcore_start) { // Skip first measurement(s) until a steady effort is delivered --> Tcore_start temperature has reached!
      DeltaStoredHeat = HeatChange(Tbody_cur, Tbody_prev, HC_Interval); // Heat Change calculated in J/m2
    } else {
      DeltaStoredHeat = 0.0;
    }
  }
  // ...... some tweaking ....
  if ( (DeltaStoredHeat < 0) && TWEAK ) { // TWEAK Heat loss  !
    StoredHeat += (1.50*DeltaStoredHeat); // Tcore-algorithm underestimates Heat Loss (Cool down) --> Increase artificially Heat Loss contribution !!!
  } else {
    StoredHeat += DeltaStoredHeat;   // equation  S == internally stored body heat NO Tweaking
  }

  // Calculate Core Temp change
  DeltaTc = (Tcore_cur - Tcore_prev);

#ifdef DEBUGAIR
  Serial.printf("Tc_cur:%f ", Tcore_cur);
  Serial.printf("Tc_prev: %f ", Tcore_prev);
  Serial.printf("DeltaStoredHeat: %f \n", DeltaStoredHeat);
#endif

  // Store the actual T-results now for comparison in the next round to determine the T-Delta's
  Tcore_prev = Tcore_cur;
  Tbody_prev = Tbody_cur;

  // Calculate Terms from Heat balance equation
  F2 = (double)(Hc * DeltaTska + He * DeltaPska); // equation  1 (C+E) without Va(exp 0.84)
  Radiation = (double)Hr * DeltaTska;        // equation  Radiation heat exchange

  // equation component H  use CPS_average for "External Work"
  HeatProduced = (double)( CPS_average * ((float)(100/GE) - 1) ) / Ad; // in J/m2 equation 1 H = (M-W)/Ad & GE = W/M -> M = W/GE
  SumHeatProduced += HeatProduced * HC_Interval / 1000; // sum of total heat produced during the workout, units in kJ/m2
  SumEnergyProduced += MetEnergy * HC_Interval / 1000; // in kJ/m2 --> individualized

#ifdef DEBUGAIR
  Serial.printf("H:%5.1f GE: %4.2f \n", HeatProduced, (float)(100 / GE));
#endif
  // Solve Full Heat Balance equation with Power HeatProduced (H) AND HeatStored (S) to find air velocity !
  // Heat Balance equation:
  // F3 = (H-R-S)/F2;             --> right side of the heat balance equation
  // However we want the airflow optimised in such a way the S is minimized or equals null...
  // If StoredHeat > 0 after an interval, it means NOT all heat is exchanged so we have to increase the airflow to
  // increase the heat exchange (by C+E), therefore we swap (H-S-R) for (H+S-R). The argument is:
  // StoredHeat represents the stored internal heat that is to be removed together with the produced heat (H),
  // the sum of both (!) is the heat we want to exchange with the environment...
  F3 = (HeatProduced + StoredHeat - Radiation) / F2; // right side of the FULL heat balance equation S = StoredHeat
  // POW with decimal exponent (1.1905) can't process a negative base (F3), --> POW equation gives "nan" result!
  if ( F3 > 0 ) {
    Vair = (float)pow(F3, 1.1905);         // in equation exp == 0.84 --> 84/100 -> 100/84 = 1.1905
  } else Vair = 0.0;

  // Now that we know Va, C and E can be re-calculated !! Notice H and R are independent of airflow speed so they hold the same values!!
  Convection = (float)pow((double)Vair, 0.84) * Hc * DeltaTska; // 0.6 standard or 0.84 to conform with previous calculations !!
  //float Emax = (float)pow((double)Vair, 0.84)*He*DeltaPska; // 0.6 standard or 0.84 to conform with previous calculations !!
  Ereq = (float)(HeatProduced + StoredHeat - Radiation - Convection); // Requested Evaporative exchange with the environment
  if (Ereq < 0) {
    Ereq = 0;  // in startup fase value can be negative force to zero !
  }
  //Sweat Rate = (147 + 1.527*Ereq - 0.87*Emax) --> The Original equation of Conzalez et al. 2009
  // Adapted since in our heat balance equation Ereq = Emax (by definition!) -> rewritten to: SwRate = 147 +(0.657*Ereq)
  if (Ereq > 0) {
    SwRate = (147 + (0.657 * Ereq)); // per milli liter fluid is equal to weight in grams -> Notice SwRate is in gr/m2/hour
  } else SwRate = 0;

  HeatState = HeatBalanceState((int)Ereq);

#ifdef DEBUGAIR
  Serial.printf("Psk: %0.3fkPa | Pair: %0.3fkPa -> DeltaPska: %0.3fkPa Tsk: %0.2fC-> DeltaTska %0.2fC ", Psk, Pair, DeltaPska, Tskin, DeltaTska);
  Serial.printf(" H: %f F2: %f F3: %f S: %0.2f -> Va: %0.2f m/s\n", HeatProduced, F2, F3, StoredHeat, Vair);
  Serial.printf("Rate of Body Heat Change: %1.4f J/m2 | Tc Change: %03.2f% OpMode: %s\n", DeltaStoredHeat, DeltaTc, OpMode_str[OpMode]);
  Serial.printf("C:%5.1f  ", Convection);
  Serial.printf("R:%5.1f  ", Radiation);
  Serial.printf("Ereq:%6.1f ",  Ereq);
  Serial.printf("Sweat rate: %4.0f(g/m2/h) \n", SwRate);
#endif

  if (OpMode == HEATBALANCE) { // Values have changed -> translate to Airflow intensity
    ActualUpperFanPerc = (uint16_t)(AirFlowToFanPercFactor[BikePos]*Vair + 0.5); // conversion from outcome Heat-Balance-Equation to percentage Fan intensity
    ActualUpperFanPerc = constrain(ActualUpperFanPerc, 0, 100);
    AdjustForFanBalance();
    SetFanThresholds();
    SetBothFanPeriods(); // Translate Perc to Millis Duty cycle Fan operation
  }
}

/**
   Hooked callback that triggered when a measurement value is sent from peripheral
   @param chr   Pointer client characteristic that even occurred,
                in this example it shoul  d be hrmc
   @param data  Pointer to received data
   @param len   Length of received data
*/
void hrmc_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
  // New heartrate data are available so record heartrate value and do calculations
  uint16_t HBM_cur = (uint16_t)data[1]; // Notice we assume data[1] to be 8 bit value NOT 16 bit !!!

  HBM_average = (uint16_t)sma_filter(HBM_cur, HBM_history, HBM_Length); // Average over 60 seconds

  // Sensiron SHT31 sensor gets and updates Temperature and Relative Humidity (Tair & RH)
  IsTempSensor = SetCurrentTemperatureAndHumidity();

#ifdef DEBUGAIR
  Serial.printf("HBM: %02d | TC Value: %04.4f | V Value: %f \n",  HBM_cur, Tcore_cur, v_cur);
#endif
  // Called also at cpmc_notify_callback function, the "first one" gets the job done!
  if (millis() > OneMinuteInterval)
  {
    OneMinuteInterval = millis() + SIXTY_SECONDS_TIME;
    HeatBalanceAlgorithm();
  }
  CallOledDisplaySchedular();
}

int EMA_MetEnergyFilter(int current_value) {
  static uint16_t exponential_average = current_value;

  exponential_average = (EMA_ALPHA * (uint32_t)current_value + (100 - EMA_ALPHA) * (uint32_t)exponential_average) / 100;
  return exponential_average;
}

int EMA_HeatBalanceFilter(int current_value) {
  static uint16_t exponential_average = current_value;

  exponential_average = (EMA_ALPHA * (uint32_t)current_value + (100 - EMA_ALPHA) * (uint32_t)exponential_average) / 100;
  return exponential_average;
}

// Simple Moving Average filter (Reentrant programmed!!)
long sma_filter(uint16_t current_value, uint16_t history_SMA[], uint8_t SMA_Length)
{
  long sum = 0;
  uint8_t i;

  for (i = 1; i < SMA_Length; i++) {
    history_SMA[i - 1] = history_SMA[i];
  }
  history_SMA[SMA_Length - 1] = current_value;
  for (i = 0; i < SMA_Length; i++) {
    sum += history_SMA[i];
  }
  return sum / SMA_Length;
}
  
uint16_t CalculateAveragePowerBelowCP(uint16_t iPower, uint16_t iCP){
  // calculate avg_power_below_cp real time using a running sum and counter
  static unsigned long int CountPowerBelowCP = 0;
  static unsigned long int SumPowerBelowCP = 0;
    if (iPower < iCP) { 
      SumPowerBelowCP += (unsigned long int)iPower;
      CountPowerBelowCP++;
      }
  return uint16_t(SumPowerBelowCP/CountPowerBelowCP); // average power below CP
}   // end calculate avg_power_below_cp

void CalculateAveragePowerAboveCP(uint16_t iPower, uint16_t &iavPwr, unsigned long int &iCpACp){
  // calculate avg_power_above_cp real time using a running sum and counter
  // returning the values by C++ reference!
  static unsigned long int SumPowerAboveCP = 0;
      SumPowerAboveCP += (unsigned long int)iPower;
      iCpACp++;
      iavPwr = uint16_t(SumPowerAboveCP/iCpACp); // average power above CP
}   // end calculate avg_power_above_cp

double tau_w_prime_balance(uint16_t iPower, uint16_t iCP){  
    uint16_t avg_power_below_cp = CalculateAveragePowerBelowCP(iPower, iCP);
    double delta_cp = double(iCP - avg_power_below_cp);
    return (double(546.00) * exp(-0.01 * delta_cp) + double(316.00));
}   // end Tau W Prime Balance

void w_prime_balance_waterworth(uint16_t iPower, uint16_t iCP, uint16_t iw_prime) {
    // Most power meters measure power, torque a.o. in a high frequency (20-60 Hz) but 
    // transmit (BLE) datasets to a monitoring device in much lower frequency: 1-4 times per second.
    int power_above_cp = 0; // Power > CP
    static double T_lim = 0; // Time (duration) while Power is above CP, the summed value of every sample time value P > CP
    double w_prime_expended = 0.0; // Expended energy in Joules
    double ExpTerm1 = 0.0, ExpTerm2 = 0.0;
    static double TimeSpent = 0.0; // Total Time spent in the workout, the summed value of every sample time value
    static double running_sum = 0.0;
    static unsigned long int CountPowerAboveCP = 0; // Count the Power readings above CP 
    static uint16_t avPower = 0; // Average power above CP
    const long int NextLevelStep = 1000; // Stepsize of the next level of w-prime modification --> 1000 Joules step
    static long int NextUpdateLevel = 0; // The next level at which to update eCP, e_w_prime_mod and ew_prime_test
    // Quarq Dfour Zero Spider power meter sends between 2 and 1.2 power readings per second, dependent of POWER level !!!
    // We assume that the sample frequency (number of samples per second) is VARIABLE !!!
    // Determine the individual sample time in seconds, it may/will vary during the workout !!! 
    static unsigned long PrevReadingTime = 0;
    double SampleTime  = double(millis()-PrevReadingTime)/1000; // Time or duration since the previous sample, convert from millis to seconds
    PrevReadingTime = millis(); // Update for the next sample
    double tau = tau_w_prime_balance(iPower, iCP); // Determine the value for tau
    TimeSpent += SampleTime ; // The summed value of all sample time values during the workout
    power_above_cp = (iPower - iCP);
#ifdef DEBUGAIR
    Serial.printf("Time:%6.1f ST: %4.2f tau: %f ", TimeSpent, SampleTime , tau);
#endif
    // w_prime is energy and measured in Joules = Watt*second
    // Determine the expended energy above CP since the previous measurement (--> i.e. during sample time)
    w_prime_expended = double(max(0, power_above_cp))*SampleTime; // Determine (Watts_above_CP) * (its duration in seconds) = expended energy in Joules!
    // Calculate some terms of the equation
    ExpTerm1 = exp(TimeSpent/tau); // Exponential term1
    ExpTerm2 = exp(-TimeSpent/tau); // Exponential term2
#ifdef DEBUGAIR
    Serial.printf("W prime expended: %3.0f exp-term1: %f exp-term2: %f ", w_prime_expended , ExpTerm1, ExpTerm2);
#endif
    running_sum = running_sum + (w_prime_expended*ExpTerm1); // Determine the running sum
#ifdef DEBUGAIR
    Serial.printf("Running Sum: %f ", running_sum);
#endif    
    w_prime_balance = (long int)( (double)iw_prime - (running_sum*ExpTerm2) ) ; // Determine w prime balance and cast from double to int
#ifdef DEBUGAIR
    Serial.printf(" w_prime_balance: %d ", w_prime_balance);
#endif
    //--------------- extra --------------------------------------------------------------------------------------
    // Workout starts at a certain W'= ##,### Joules and CP = ### watts, set by the user; the algorithm increases CP and W' stepwise
    // to more realistic values every time when W'balance is depleted to a certain level; -> 2-Parameter Algorithm updates CP and W'
    if (power_above_cp > 0) { 
      CalculateAveragePowerAboveCP(iPower, avPower, CountPowerAboveCP); // Average power above CP is to be calculated for future use
      T_lim += SampleTime ; // Time to exhaustion: the accurate sum of every second spent above CP, calculated for future use
    }
#ifdef DEBUGAIR
    Serial.printf(" [%d]\n", CountPowerAboveCP);
#endif
    // When working above CP, the moment comes that we need to update eCP and ew_prime !!
    if ( (w_prime_balance < NextUpdateLevel) && (w_prime_expended > 0) ) { // W' balance is further depleted --> test for an update moment
       NextUpdateLevel -= NextLevelStep; // Move down another level of depletion, update eCP, ew_prime_mod and ew_prime_test
       eCP = GetCPfromTwoParameterAlgorithm(avPower, T_lim, iw_prime); // Estimate a new eCP value
       ew_prime_mod = w_prime_usr - NextUpdateLevel; // Adjust ew_prime_modified to the next level of depletion to be checked
       ew_prime_test = GetWPrimefromTwoParameterAlgorithm(uint16_t(eCP*1.045), double(1200), eCP); // 20-Min-test estimate for W-Prime
#ifdef DEBUGAIR
       Serial.printf("Update of eCP - ew_prime %5d - avPower: %3d - T-lim:%6.1f --> eCP: %3d ", ew_prime_mod, avPower, T_lim, eCP);
       Serial.printf("--> Test estimate of W-Prime: %d \n", ew_prime_test );
#endif
    }
    //-----------------extra -------------------------------------------------------------------------------
} // end

/* 19 mei 2021
void w_prime_balance_waterworth(uint16_t iPower, uint16_t iCP, uint16_t iw_prime) {
    // Most power meters measure power, torque a.o. in a high frequency (20-60 Hz) but 
    // transmit (BLE) datasets to a monitoring device only about 1 per second.
    // Quarq Dfour Zero Spider power meter sends between 2 and 1.0 power readings per second dependent of POWER level !!!
    int power_above_cp = 0;
    static double T_lim = 0; // Time spent during power_above_cp
    double w_prime_expended = 0.0;
    double ExpTerm1 = 0.0, ExpTerm2 = 0.0;
    static double running_sum = 0.0;
    static unsigned long int CountPowerAboveCP = 0; // Count the power readings above CP 
    static uint16_t avPower = 0; // Average power above CP
    const long int NextLevelStep = 1000; // Stepsize of the next level of w-prime modification --> 1000 Joules
    static long int NextUpdateLevel = 0; // The next level at which to update eCP, e_w_prime_mod and ew_prime_test
    
    double tau = tau_w_prime_balance(iPower, iCP); // Determine the value for tau
    
    // Accurately calculate sample time in seconds and count power readings
    // sampling rate is the number of samples per second and equals 1/sample_time ! 
    static unsigned long PrevReadingTime = 0;
    double Sample_Time = double(millis()-PrevReadingTime)/1000; // millis to seconds
    PrevReadingTime = millis();
    static unsigned long NumOfPowerReadings = 0;
    NumOfPowerReadings++;
    power_above_cp = (iPower - iCP);
#ifdef DEBUGAIR
    Serial.printf("CNT:%4d ST: %5.3f tau: %f ", NumOfPowerReadings, Sample_Time, tau);
#endif
    // w_prime is energy (unit in Joules) so we need to detemine Watts * seconds to get Energy!
    // Determine the spent energy (above CP) since the last measurement (--> during sample time)
    w_prime_expended = double(max(0, power_above_cp))/Sample_Time;  // replaced *sampling_rate with /Sample_Time in seconds
    // WorkoutTime is the time spent during the workout in seconds (present-time - start-time) therefore
    ExpTerm1 = exp(double(NumOfPowerReadings)/tau);  // (WorkoutTime*sampling_rate) is equivalnet to NumOfPowerReadings
    ExpTerm2 = exp(-double(NumOfPowerReadings)/tau); // (WorkoutTime*sampling_rate) is equivalnet to NumOfPowerReadings
#ifdef DEBUGAIR
    Serial.printf("W prime expended: %3.0f exp-term1: %f exp-term2: %f ", w_prime_expended , ExpTerm1, ExpTerm2);
#endif
    running_sum = running_sum + (w_prime_expended*ExpTerm1); //
#ifdef DEBUGAIR
    Serial.printf("Running Sum: %f ", running_sum);
#endif    
    w_prime_balance = (long int)( (double)iw_prime - (running_sum*ExpTerm2) ) ; // cast from double to int
#ifdef DEBUGAIR
    Serial.printf(" w_prime_balance: %d ", w_prime_balance);
#endif
    //--------------- extra --------------------------------------------------------------------------------------
    // Workout starts at a certain W'= ##,### Joules and CP = ### watts, set by the user; the algorithm increases CP and W' stepwise
    // to more realistic values every time when W'balance is depleted to a certain level; -> 2-Parameter Algorithm updates CP and W'
    if (power_above_cp > 0) { 
      CalculateAveragePowerAboveCP(iPower, avPower, CountPowerAboveCP); // Average power > CP is to be calculated for later use
      T_lim += Sample_Time; // Limit Time is Sum of Sample Time spent above CP, calculated for later use
    }
#ifdef DEBUGAIR
//    Serial.printf(" [%d]\n", CountPowerAboveCP);
#endif
    // When working above CP, the moment comes that we need to update eCP and ew_prime !!
    if ( (w_prime_balance < NextUpdateLevel) && (w_prime_expended > 0) ) { // W' balance is further depleted --> test for an update moment
       NextUpdateLevel -= NextLevelStep; // Move down another level of depletion, update eCP, ew_prime_mod and ew_prime_test
#ifdef DEBUGAIR
//       UpdateCNT++; // Count number of eCP updates FOR TESTING ONLY
#endif
       eCP = GetCPfromTwoParameterAlgorithm(avPower, T_lim, iw_prime); // Estimate a new eCP value
       ew_prime_mod = w_prime_usr - NextUpdateLevel; // Adjust ew_prime_modified to the next level of depletion to be checked
       ew_prime_test = GetWPrimefromTwoParameterAlgorithm(uint16_t(eCP*1.045), double(1200), eCP); // 20-min-test estimate for W-Prime
#ifdef DEBUGAIR
       Serial.printf("Update of eCP - ew_prime %5d - avPower: %3d - T-lim:%6.1f --> eCP: %3d ", ew_prime_mod, avPower, T_lim, eCP);
       Serial.printf("--> 20-min-test-estimate of W-Prime: %d \n", ew_prime_test );
#endif
    }
    //-----------------extra -------------------------------------------------------------------------------
} // end
*/

// Check and Set starting value of w_prime to realistic numbers!!
void ConstrainW_PrimeValue(uint16_t &iCP, uint16_t &iw_prime) {
    if (iCP < 100) { iCP = 100; } // Update to lowest level that we allow for
    // First determine the "minimal" value for W_Prime according to a 20-min-test estimate, given the iCP value! 
    uint16_t w_prime_estimate = GetWPrimefromTwoParameterAlgorithm(uint16_t(iCP*1.045), double(1200), iCP); 
    if (iw_prime < w_prime_estimate) { iw_prime = w_prime_estimate; } // Update iw_prime to a realistic level
    return;
} // end

uint16_t GetCPfromTwoParameterAlgorithm(uint16_t iav_Power, double iT_lim, uint16_t iw_prime) {
     uint16_t WprimeDivTlim = uint16_t( double(iw_prime)/iT_lim ); // type cast for correct calculations
     if (iav_Power > WprimeDivTlim){ // test for out of scope
      return (iav_Power - WprimeDivTlim); // Solve 2-parameter algorithm to estimate CP
      } else {
      return eCP; // Something went wrong do'nt allow an update of CP
     }
} // end

uint16_t GetWPrimefromTwoParameterAlgorithm(uint16_t iav_Power, double iT_lim, uint16_t iCP) {
     if (iav_Power > iCP){ // test for out of scope
      return (iav_Power-iCP)*((uint16_t)iT_lim); // Solve 2-parameter algorithm to estimate new W-Prime
      } else {
      return w_prime_usr; // Something went wrong do'nt allow an update of w_prime
     }
} // end

/**
   Hooked callback that triggered when a measurement value is sent from peripheral
   @param chr   Pointer client characteristic that even occurred,
                in this example it should be cpmc
   @param data  Pointer to received data
   @param len   Length of received data
*/
void cpmc_notify_callback(BLEClientCharacteristic* chr, uint8_t* data, uint16_t len)
{
#ifdef DEBUGAIR
  uint8_t buffer[35] = {0};
  // Transfer the contents of data to buffer (array of chars)
  Serial.print("CPS Data: ");
  for (int i = 0; i < len; i++) {
    if ( i <= sizeof(buffer)) {
      buffer[i] = *data++;
      Serial.printf("%02X ", buffer[i], HEX);
    }
  }
  Serial.println();
#else
  uint8_t buffer[4] = {0};
  buffer[0] = *data++; // Flags: (uint8_t)(cpmcDef & 0xff)
  buffer[1] = *data++; // Flags: (uint8_t)(cpmcDef >> 8)
  buffer[2] = *data++; // (uint8_t)(powerOut & 0xff)
  buffer[3] = *data++; // (uint8_t)(powerOut >> 8)
#endif
  // POWER is stored in 2 bytes !!!
  uint8_t lsb_InstantaneousPower = buffer[2]; // Instantaneous Power LSB
  uint8_t msb_InstantaneousPower = (buffer[3] & 0xFF); // Instantaneous Power MSB
  CPS_cur = lsb_InstantaneousPower + msb_InstantaneousPower * 256;
#ifdef DEBUGAIR
  Serial.printf("Power Value:   %4d\n", CPS_cur);
#endif

  // Smooth the values of the power over CPS_Length seconds (about 1 reading/sec) average...
  CPS_average = (uint16_t)sma_filter(CPS_cur, CPS_history, CPS_Length);
  
  // Activate Track Anaerobic Work Capacity depletion algorithm ------------------------
  if (TAWC_Mode == 1) { // Only calculate when TAWC_Mode is selected !!!!
    // Only apply the values of CP and W-Prime set by the user !! DO NOT CHANGE DURING WORKOUT !!
    // NO FILTERING of Power reading !!!
    w_prime_balance_waterworth((unsigned int)CPS_cur, CP60, w_prime_usr);
    // Check for continued depletion then set for dominance to show !!
    if ((CPS_cur - eCP) > 0) { IsShowWprimeValuesDominant = true; 
    } else { IsShowWprimeValuesDominant = false; }
  }
  // Activate Track Anaerobic Work Capacity depletion algorithm ------------------------
  
  // Called also at hrmc_notify_callback function, the "first one" gets the job done!
  if (millis() > OneMinuteInterval)
  {
    OneMinuteInterval = millis() + SIXTY_SECONDS_TIME;
    HeatBalanceAlgorithm();
  }
  CallOledDisplaySchedular();
}
//////////////////////// DONE! /////////////////////////
