#define SN "MyWindow"
#define SV "2.2"
// Enable debug prints to serial monitor
//#define MY_DEBUG

#define MY_RADIO_RFM69
#define MY_RFM69_FREQUENCY RFM69_433MHZ
#define MY_IS_RFM69HW
#define MY_RFM69_NEW_DRIVER

// Message signing settings
#define MY_SIGNING_ATSHA204
#define MY_SIGNING_REQUEST_SIGNATURES

//Enable OTA feature
#define MY_OTA_FIRMWARE_FEATURE
#define MY_SMART_SLEEP_WAIT_DURATION_MS 1000

// Optimizations when running on battery.
#define MY_TRANSPORT_UPLINK_CHECK_DISABLED
#define MY_TRANSPORT_WAIT_READY_MS  5000
#define MY_SLEEP_TRANSPORT_RECONNECT_TIMEOUT_MS 2000
#define MY_PARENT_NODE_ID 0
#define MY_PARENT_NODE_IS_STATIC
#define MY_SPLASH_SCREEN_DISABLED

#include <MySensors.h>

typedef enum
{
  INIT,
  ARMED,
  WAITING_ECHO,
  RETRY,
  GOTO_SLEEP,
  FOTA_ONGOING,
} States;
States m_state;

MyMessage msg(1, V_TRIPPED);
MyMessage msg_batt(201, V_VOLTAGE);

#define REQUEST_ECHO true
#define ECHO_TIMEOUT 2000 //ms
#define BATT_SKIP_COUNT 4

unsigned long timing_echo_timeout;
unsigned long trip_counter;

void setup()
{
  pinMode(3, INPUT);
  m_state = INIT;
  trip_counter = 0;
  timing_echo_timeout = 0;
}

void presentation()
{
  sendSketchInfo(SN, SV);
  present(1, S_DOOR);
  present(201, S_MULTIMETER);
}

void loop()
{
  if (isFirmwareUpdateOngoing()) m_state = FOTA_ONGOING;

  switch (m_state) {    
    case INIT: {
        DEBUG_OUTPUT(PSTR("SKETCH DEBUGGING: Sending initial status.\n"));
        long vcc = readVcc();
        send(msg_batt.set(float(vcc) / 1000.0f, 2));
        sendBatteryLevel( (long)((vcc - 1900) / 14.0) );
        send(msg.set(digitalRead(3) == HIGH));
        m_state = GOTO_SLEEP;
        break;
      }
    case ARMED: {
        DEBUG_OUTPUT(PSTR("SKETCH DEBUGGING: Window tripped.\n"));
        if (trip_counter % BATT_SKIP_COUNT == 0) {
          DEBUG_OUTPUT(PSTR("SKETCH DEBUGGING: Sending battery level.\n"));
          long vcc = readVcc();
          send(msg_batt.set(float(vcc) / 1000.0f, 2));
          sendBatteryLevel( (long)((vcc - 1900) / 14.0) );
        }
        // Short delay to allow buttons to properly settle
        sleep(1500);
        send(msg.set(digitalRead(3) == HIGH), REQUEST_ECHO);
        trip_counter++;
        timing_echo_timeout = millis();
        m_state = WAITING_ECHO;
        break;
      }
    case WAITING_ECHO: {
        if (millis() - timing_echo_timeout >= ECHO_TIMEOUT) {
          DEBUG_OUTPUT(PSTR("SKETCH DEBUGGING: Timeout. Retrying in 60s.\n"));
          m_state = RETRY;
          sleep(60000);
        }
        break;
      }
    case RETRY: {
        DEBUG_OUTPUT(PSTR("SKETCH DEBUGGING: Send retry.\n"));
        send(msg.set(digitalRead(3) == HIGH), REQUEST_ECHO);
        timing_echo_timeout = millis();
        m_state = WAITING_ECHO;
        break;
      }
    case GOTO_SLEEP: {
        DEBUG_OUTPUT(PSTR("SKETCH DEBUGGING: Going to sleep.\n"));
        m_state = ARMED;
        sleep(1, CHANGE, 43200000, true);
        break;
      }
    case FOTA_ONGOING: {
        DEBUG_OUTPUT(PSTR("SKETCH DEBUGGING: FOTA ongoing. Do nothing and reinit node.\n"));
        m_state = INIT;
        break;
      }
    default:
      break;
  }
}

void receive(const MyMessage &message)
{
  if (message.type == V_TRIPPED) {
    if (message.isEcho()) {
      DEBUG_OUTPUT(PSTR("SKETCH DEBUGGING: Echo received.\n"));
      m_state = GOTO_SLEEP;
    }
  }
}

/*
   Taken from Sensebender Micro sketch
*/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADcdMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
