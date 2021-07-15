#include <math.h>
#include <avr/dtostrf.h>

// Necessary libraries for use of Oled display(s)
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// Additional splash screen bitmap and icon(s) for Oled
//#include "Adafruit_SSD1306_Icons.h" // needs to be in directory of main code
// Declarations for an SSD1306 128x64 display connected to I2C (SDA, SCL pins)
#define SCREEN_WIDTH 128            // SSD1306-OLED display width, in pixels
#define SCREEN_HEIGHT 64            // SSD1306-OLED display height, in pixels
#define OLED_RESET -1               // No reset pin on this OLED display
#define SSD1306_I2C_ADDRESS 0x3C    // I2C Address for SSD1306-OLED display
// Declare the display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);



//************** TEST VALUES **********************
uint16_t CPSPowerValue = 0; 
const uint16_t INTERVALPOWER = 175; // Max power during peak interval time
#define DEBUGAIR 
#ifdef DEBUGAIR
      uint8_t UpdateCNT = 0; // Count number of eCP updates
#endif

// Global Variables ******************************
uint16_t CP60 = 140;  // Your (estimate of) Critical Power, more or less the same as FTP
uint16_t eCP = CP60;  // Algorithmic estimate of Critical Power during intense workout
uint16_t w_prime_usr = 7200;          // Your (estimate of) W-prime or a base value
uint16_t ew_prime_mod = w_prime_usr;  // First order estimate of W-prime modified during intense workout
uint16_t ew_prime_test = w_prime_usr;   // 20-Min-test algorithmic estimate (20 minute @ 5% above eCP) of W-prime for a given eCP! 
long int w_prime_balance = 0;  // Can be negative !!!
//************************************************

// Line numbers for Display layout zones: icontopbar, line #1 en line #2
#define HLINE1 18
#define HLINE2 43

void ShowWprimeValuesOnOled(void);


// ------------------------   W'Balance Functions  -----------------------------------
uint16_t CalculateAveragePowerBelowCP(uint16_t iPower, uint16_t iCP);
void CalculateAveragePowerAboveCP(uint16_t iPower, uint16_t &iavPwr, unsigned long int &iCpACp);
double tau_w_prime_balance(uint16_t iPower, uint16_t iCP);
void w_prime_balance_waterworth(uint16_t iPower, uint16_t iCP, uint16_t iw_prime);
void ConstrainW_PrimeValue(uint16_t &iCP, uint16_t &iw_prime);
uint16_t GetCPfromTwoParameterAlgorithm(uint16_t iav_Power, double iT_lim, uint16_t iw_prime);
uint16_t GetWPrimefromTwoParameterAlgorithm(uint16_t iav_Power, double iT_lim, uint16_t iCP);

// ------------------------   W'Balance Functions  ------------------------------------


  
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
//#ifdef DEBUGAIR
//    Serial.printf("AvP<CP: %4d dCP: %6.2f ", avg_power_below_cp, delta_cp);
//#endif
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
    
    // FOR FAST TESTING ONLY --> OVERRULES PREVIOUS CODE !!
    SampleTime  = 0.833; // --------------------------------
    // -----------------------------------------------------
    
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
#ifdef DEBUGAIR
       UpdateCNT++; // Count number of eCP updates FOR TESTING ONLY
#endif
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

// Check and Set starting values of w_prime to reasonable numbers!!
void ConstrainW_PrimeValue(uint16_t &iCP, uint16_t &iw_prime) {
    if (iCP < 100) { iCP = 100; } // Update to lowest level that we allow for
    // First determine the "minimal" value for W_Prime according to a 20-Min-test estimate, given the iCP value! 
    uint16_t w_prime_test = GetWPrimefromTwoParameterAlgorithm(uint16_t(iCP*1.045), double(1200), iCP); 
    if (iw_prime < w_prime_test) { iw_prime = w_prime_test; } // Update iw_prime to a realistic level
    return;
} // end

double GetTlimfromTwoParameterAlgorithm(uint16_t iav_Power, uint16_t iCP, uint16_t iw_prime) {
     return double(iw_prime)/double(iav_Power-iCP); // type cast for correct calculations
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

void setup() {
  // put your setup code here, to run once:
  
#ifdef DEBUGAIR
  Serial.begin(115200);
  while ( !Serial ) delay(10);   // for nrf52840 with native usb
  Serial.println("Start HIIT Workout and calculate W' Balance and estimate CP");
#endif

  // Start the show for the SSD1306 Oled display
  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS)) {
#ifdef DEBUGAIR
    Serial.println(F("SSD1306 allocation failed!"));
#endif
  }
  else {
    display.clearDisplay();
#ifdef DEBUGAIR
//    Serial.println(F("SSD1306 is running..."));
#endif
  }

// Check floor and ceiling values for starting CP and w_prime to garantee best estimates !!
ConstrainW_PrimeValue(eCP, ew_prime_mod);

#ifdef DEBUGAIR
    Serial.printf("Starting values -->  eCP: %3d --> ew_prime: %5d Interval Power: %3d \n", eCP, ew_prime_mod, INTERVALPOWER);
    Serial.printf("Tlim: %5.3f --> ew_prime: %5d \n", GetTlimfromTwoParameterAlgorithm(INTERVALPOWER, eCP, ew_prime_mod), 
      GetWPrimefromTwoParameterAlgorithm(uint16_t(CP60*1.045), double(1200), CP60) );
#endif

// Start of a simulated HIIT workout... 
uint16_t t = 0;
  for (t = 0; t < 10; t++) { // warmup 10 seconds
    CPSPowerValue = 100;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
    ShowWprimeValuesOnOled();
  }

  for (t = 0; t < 1008; t++) {// 1 Interval takes 14*60=840 seconds with sample rate of 1.2 --> 1008 ticks
    CPSPowerValue = INTERVALPOWER;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
    ShowWprimeValuesOnOled();
  }
#ifdef DEBUGAIR
    Serial.printf("Test-estimate of WPrime: %d \n", ew_prime_test ); // 4.5% above CP during 20 minutes
#endif

  for (t = 70; t < 130; t++) { // recovery 60 seconds
    CPSPowerValue = 100;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
    ShowWprimeValuesOnOled();
  }

  for (t = 130; t < 190; t++) {// 2 // Interval 60 seconds
    CPSPowerValue = INTERVALPOWER+20;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
    ShowWprimeValuesOnOled();
  }
#ifdef DEBUGAIR
    Serial.printf("Test-estimate of WPrime: %d \n", ew_prime_test ); // 4.5% above CP during 20 minutes
#endif

  for (t = 190; t < 250; t++) { // recovery 1*60 seconds
    CPSPowerValue = 100;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
    ShowWprimeValuesOnOled();
  }

/*
  for (t = 250; t < 310; t++) {// 3  Interval 60 seconds
    CPSPowerValue = INTERVALPOWER;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
  }

  for (t = 310; t < 380; t++) { // recovery 60 seconds
    CPSPowerValue = 100;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
  }

  for (t = 380; t < 440; t++) {// 4  Interval 60 seconds
    CPSPowerValue = INTERVALPOWER;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
  }

  for (t = 440; t < 500; t++) { // recovery 60 seconds
    CPSPowerValue = 100;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
  }

  for (t = 500; t < 560; t++) {// 5  Interval 60 seconds
    CPSPowerValue = INTERVALPOWER;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
  }

  for (t = 560; t < 800; t++) { // cooldown/recovery 4*60 seconds
    CPSPowerValue = 100;
    w_prime_balance_waterworth(CPSPowerValue, CP60, w_prime_usr );
  }
*/

#ifdef DEBUGAIR
  Serial.println(); Serial.println( "------------ Done ------------"); Serial.println();
  Serial.printf("Starting values -->  w_prime: %5d -->  CP: %3d \n", w_prime_usr , CP60);
  Serial.printf("[%3d] estimates --> ew_prime: %5d --> eCP: %3d \n", UpdateCNT, ew_prime_mod, eCP);
#endif
} // end setup

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

void loop() {
  //
}
