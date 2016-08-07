// Please see README.md

#include <avr/interrupt.h>
#include <limits.h>
#include <Arduino.h>
#include "SerialCommand.h"

#if defined(__AVR_ATmega8__)
#define AREF_INTERNAL (2.56)
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega88__)
#define AREF_INTERNAL (1.1)
#else
#error "Parameters are not defined for this chip"
#endif

// Firmware version
#define FW_VER "1.3.13"

// Use oversampling to reduce noise
#define ADC_RESOLUTION 10 // adc_bits
#define OVERSAMPLE_BITS 6
#define OVERSAMPLE_SAMPLES (1UL << (OVERSAMPLE_BITS * 2))
#define OVERSAMPLE_MAX_VALUE (1UL << (ADC_RESOLUTION + OVERSAMPLE_BITS))

// CA3162E updates 3-digit display at about 67 Hz, this gives 5 ms/digit.
#define DIGIT_REFRESH_TIME    5000  // In microseconds
#define ARRAY_SIZE(x) (sizeof((x))/sizeof((x)[0]))

// CA3161 accepts following codes as special symbols
#define L_DASH  10
#define L_E     11
#define L_H     12
#define L_L     13
#define L_P     14
#define L_SPACE 15

// EEPROM address locations
#define DOUBLE_SIZE 4  // double is only 4byte on ATmega
#define EEPROM_CALIBMUL 0x00
#define EEPROM_CALIBADD 0x05

// Default demo string and time
static byte init_string[] = { 
  L_SPACE, L_SPACE, L_SPACE, 
  L_H, L_E, L_L, 0, L_SPACE, 
  L_SPACE, L_SPACE
}; 
static int init_delay = 100;

// ADC vars
static unsigned long cumul_reading = 0;
static double readval, convval;
static int showval = 0;


// Serial object
// https://github.com/scogswell/ArduinoSerialCommand
SerialCommand serial_cmd;

// See
// http://41.media.tumblr.com/6a7069c75223ee39d701e7c93bd8613b/tumblr_n8t59l7b2L1s5t695o1_1280.png
// for bare ATmega328 based Arduino pinout. See
// http://arduino.cc/en/Hacking/PinMapping for bare ATmega8 Arduino pinout.

// Cable pinout to interface with CA3162E and CA3161E socket:
//
// CA3162E    CA3161E    Arduino    ATmega DIP28
// 14/VCC     16/VCC     --         07/VCC, 20/AVCC
// --         15/SGF     DPin08     14/PD2
// --         14/SGG     DPin07     13/PD3
// --         13/SGA     DPin06     12/PD4
// --         12/SGB     DPin05     11/PD5
// --         11/SGC     DPin04     06/PD6
// --         10/SGD     DPin03     05/PD7
// --         09/SGE     DPin02     04/PB0
// 03/NSD     --         DPin11     17/PB3
// 04/MSD     --         DPin10     16/PB2
// 05/LSD     --         DPin09     15/PB1
// 07/GND     08/GND     --         08/GND, 22/GND
// 11/AIN     --         APin00     23/PC0

// Pinouts
static const byte analog_input = A0;

// LSD, NSD, MSD, active low (PNP BJT, for common anode display module)
static const byte digit_drivers[] = { 9, 11, 10 };

// SGG to SGA, active low (for common anode display module)
static const byte segment_drivers[] = { 7, 8, 2, 3, 4, 5, 6 };

// CA3161 symbols table (hex value)
static const byte symbols_table[] = {
  0x01, 0x4F, 0x12, 0x06,   // 0, 1, 2, 3 
  0x4C, 0x24, 0x20, 0x0F,   // 4, 5, 6, 7
  0x00, 0x04, 0x7E, 0x30,   // 8, 9, -, E
  0x48, 0x71, 0x18, 0x7F    // H, L, P, 
};

// Digits
static byte outdigits[ARRAY_SIZE(digit_drivers)] = { L_SPACE, L_SPACE, L_DASH };

static inline void set_digits(byte d2, byte d1, byte d0)
{
  outdigits[2] = d2;
  outdigits[1] = d1;
  outdigits[0] = d0;
}

// EEPROM double w/r func 
double eeprom_read_double( unsigned int addr ){
  union{
    byte b[DOUBLE_SIZE];
    double d;
  } data;
  for( int i = 0; i < 4; i++ ){
    data.b[i] = eeprom_read_byte( ( uint8_t* ) addr + i );
  }
  return data.d;
}
void eeprom_update_double( unsigned int addr, double x ){
  union{
    byte b[DOUBLE_SIZE];
    double d;
  } data;
  data.d = x;
  while( !eeprom_is_ready() ); // Wait for EEPROM ready status
  cli(); // clear interrupts
  for( int i = 0; i < 4; i++ ){
    eeprom_update_byte( ( uint8_t* ) addr + i, data.b[i] );
  }
  sei(); // set interrupts
}

// Calibration by real measured voltage
// See README.md for calibration
static double calib_mul = eeprom_read_double( EEPROM_CALIBMUL );
static double calib_add = eeprom_read_double( EEPROM_CALIBADD );

// Set pins according to current digit
void output_one_digit(void)
{
  static byte current_digit = 0;
  byte ii;

  // All digits off
  for (ii = 0; ii < ARRAY_SIZE(outdigits); ii++)
    digitalWrite(digit_drivers[ii], HIGH);

  // Configure the segment pins
  for (ii = 0; ii < ARRAY_SIZE(segment_drivers); ii++)
    digitalWrite(segment_drivers[ii], ((symbols_table[outdigits[current_digit]] >> ii) & 1) ? HIGH : LOW);

  // Switch current digit on
  digitalWrite(digit_drivers[current_digit], LOW);

  current_digit++;
  if (current_digit >= ARRAY_SIZE(outdigits))
    current_digit = 0;
}

// Show digits for specified time
static inline void show_digits(byte d2, byte d1, byte d0, unsigned long time_ms)
{
  unsigned long start_ms, elapsed_ms;
  
  set_digits(d2, d1, d0);
  start_ms = millis();

  while (1) {
    elapsed_ms = millis() - start_ms;
    if (elapsed_ms > ULONG_MAX / 2) // millis() wrapped around
      elapsed_ms += time_ms;
    if (elapsed_ms >= time_ms)
      break;

    output_one_digit();
    delayMicroseconds(DIGIT_REFRESH_TIME);
    output_one_digit();
    delayMicroseconds(DIGIT_REFRESH_TIME);
    output_one_digit();
    delayMicroseconds(DIGIT_REFRESH_TIME);
  }
}

void set_calib(){
  char *add_value;
  char *mul_value;

  mul_value = serial_cmd.next();
  add_value = serial_cmd.next();

  if( add_value == NULL || mul_value == NULL )
    Serial.println( "Missing parameters (2)" );
  else{
    calib_mul = strtod( mul_value, (char **) NULL);
    calib_add = strtod( add_value, (char **) NULL);
    Serial.print( "New param: ");
    Serial.print( mul_value );
    Serial.print( "x, " );
    Serial.print( add_value );
    Serial.println( "+" );
  }
}

void save_vals(){
  eeprom_update_double( EEPROM_CALIBMUL, calib_mul );
  eeprom_update_double( EEPROM_CALIBADD, calib_add );
  Serial.println( "OK");
}

void get_status(){
  Serial.print( "millis=" );
  Serial.print( millis() );
  Serial.print( ", adc=" );
  Serial.print( cumul_reading );
  Serial.print( ", read=" );
  Serial.print( readval, 5 );
  Serial.print( ", conv=" );
  Serial.print( convval, 5 );
  Serial.print( ", out=" );
  Serial.println( showval );
}

void get_eeprom_values(){
  Serial.print( "calib_mul: " );
  Serial.println( eeprom_read_double( EEPROM_CALIBMUL ), 5 );
  Serial.print( "calib_add: " );
  Serial.println( eeprom_read_double( EEPROM_CALIBADD ), 5 );
}

void unknow_command(){
  Serial.println( "Unknow command.");
}

void firmware_ver(){
  Serial.print( "Firmware ver. " );
  Serial.println( FW_VER );
}

void setup()
{
  int ii;
  
  // Setup pins
  for (ii = 0; ii < ARRAY_SIZE(digit_drivers); ii++) {
    pinMode(digit_drivers[ii], OUTPUT);
    digitalWrite(digit_drivers[ii], HIGH);
  }

  for (ii = 0; ii < ARRAY_SIZE(segment_drivers); ii++) {
    pinMode(segment_drivers[ii], OUTPUT);
    digitalWrite(segment_drivers[ii], HIGH);
  }

  pinMode(analog_input, INPUT);

  // Init string
  for( ii=1; ii < ARRAY_SIZE( init_string ); ii++){
    show_digits( init_string[ii-1], init_string[ii], init_string[ii+1], init_delay);
  }
  
  // Serial Setup
  Serial.begin(9600);
  Serial.println("CA3162/3161 emulator - https://github.com/lorf/CA3162E-emulator");

  serial_cmd.addCommand( "CAL", set_calib );
  serial_cmd.addCommand( "STOR", save_vals );
  serial_cmd.addCommand( "STAT", get_status );
  serial_cmd.addCommand( "VAL", get_eeprom_values );
  serial_cmd.addCommand( "VER", firmware_ver );
  serial_cmd.addDefaultHandler( unknow_command );

  // Note that this is 2.56V on ATmega8 and 1.1V on ATmega328. One can also use
  // EXTERNAL reference of about 1V on ATmega8 for better resolution.
  analogReference(INTERNAL);

}

void loop(){
  static int num_samples = 0;
  static unsigned long last_us = 0;

  unsigned long elapsed_us;
  byte d0, d1, d2;

  /* Serial
  static char inputString[SERIAL_STR_SIZE] = "";       // a vector to hold incoming data
  static boolean stringComplete = false;                  // whether the string is complete
  static boolean vector_pos = 0; */

  elapsed_us = micros() - last_us;
  if (elapsed_us > ULONG_MAX / 2)                         // micros() wrapped around
    elapsed_us += DIGIT_REFRESH_TIME;
  if (elapsed_us >= DIGIT_REFRESH_TIME) {
    output_one_digit();
    last_us += elapsed_us;
  }

  cumul_reading += analogRead(analog_input);
  num_samples++;

  if (num_samples == OVERSAMPLE_SAMPLES) {
    cumul_reading >>= OVERSAMPLE_BITS;

    // Convert reading to millivolts
    readval = (((double)cumul_reading * AREF_INTERNAL) / (double)OVERSAMPLE_MAX_VALUE) * 1000.0;
    convval = readval * calib_mul + calib_add;
    showval = (int)(convval + 0.5);

    if( showval > 999 ){
      // Overvoltage
      set_digits(L_E, L_E, L_E);
    } else if( cumul_reading == 0 || showval < 0 ){
      // Possibly negative voltage
      set_digits(L_DASH, L_DASH, L_DASH);
    } else{
      d0 = ( byte )( showval % 10 );
      d1 = ( byte )( ( showval / 10 ) % 10 );
      d2 = ( byte )( ( showval / 100 ) % 10 );
      // Convert zero padding to spaces
      if( d2 == 0 ){
        d2 = L_SPACE;
        if( d1 == 0 )
          d1 == L_SPACE;
      }
      set_digits( d2, d1, d0 );
    }

    cumul_reading = 0;
    num_samples = 0;
  }

  // Serial commands process
  serial_cmd.readSerial();

}
