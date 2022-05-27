#include <MIDI.h>
#include <arduino-timer.h>
/* 
 * TR-6S Buddy by Warpeggio
 * The goal here is to bolt TRS MIDI and some rudimentary note-to-gate 
 * conversion onto David Haillant's UC Divider. I'll let him take it from
 * here:
 *    UC Divider is a configurable step/beat divider for musical application
 *    It features: 
 *    8 gated/trigged outputs (5V)
 *    1 clock input (positive voltage detection, max +5V)
 *    1 reset input (positive voltage detection, max +5V)
 * 
 *    2021 David Haillant
 * 
 *    See hardware and additional information:
 *    www.davidhaillant.com
 * 
 */

/*
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/


// Uncomment the following line to allow serial debugging (115200 bauds)
//#define DEBUG

// I like to use the LED for debugging purposes
#define LED           13    // Arduino Board LED is on Pin 13

// We'll keep David's stuff on pin 4, but we have other functions 
// to program. 
// The PPQN_PIN has pulses of 1PPQN and faster.
// The QNPP_PIN has pulses of 1PPQN and slower.
// The START_PIN goes high specifically when the START button is
// pressed on the TR-6S. In MIDI terms, this is System Real Time::Start.
// The VOICE_PIN follows the gates from the selected TR-6s channel
#define PPQN_PIN      4
#define QNPP_PIN      5
#define START_PIN     6
#define VOICE_PIN     7

// pin definitions (see Arduino pin mapping)
#define CLOCK_PIN     2
#define RESET_PIN     3

#define PPQN_POT_PIN      A0
#define QNPP_POT_PIN      A1
#define VOICE_POT_PIN     A2

// We're going to use this as a hack to make the clock divider work
// with MIDI signals, where true is like high and false is like low.
bool fake_pin = false;

// This was originally used as the global PPQN setting.
// TR-6S uses 24PPQN; we will use var pqnnSelect to determine
// the output PPQN.
#define MAX_RESOLUTION 24

// Max number of quarter notes per pulse (QNPP)
#define MAX_QNPP 24

// Number of channels on the TR-6S
#define MAX_VOICE 6

// PPQN counter, incremented on each pulse on clock input. 
// From 0 to (MAX_RESOLUTION - 1)
uint8_t ppqn_counter = 0;
// QNPP counter, incremented when a quarter note has passed.
uint8_t qnpp_counter = 0;

// Initialize pin state variables
uint8_t ppqn_state = LOW;
uint8_t qnpp_state = LOW;
uint8_t start_state = LOW;
uint8_t voice_state = LOW;

uint8_t clock_state = LOW;
uint8_t reset_state = LOW;

// and setup potentiometer setting vars
int ppqnPotVal =  0;
int qnppPotVal =  0;
int voicePotVal = 0;

int ppqnSelect =  0;
int qnppSelect =  0;
int voiceSelect = 0;
int voiceMap = 0;

// debouncing
uint8_t last_clock_pin_state = HIGH;
bool last_fake_pin_state = false;

#ifdef DEBUG
uint8_t debug_counter = 0;
#endif

// I *think* these are function prototypes
void reset_outputs(void);
void read_pots(void);
void manage_clock();
void manage_MIDI_clock();
void StartHandler();
void ClockHandler();
bool timer_handler(void *) {
  // The first fake_pin is related to MIDI clock pulses.
  if (fake_pin)
  // Set the fake_pin back to false, this is the equivalent of the falling edge of a clock pulse
  {
    fake_pin = false;
    // Back to the clock subroutine to manage pin states
    manage_midi_clock();
  }
  // The start pin only fires once, so we just manage its state right here.
  if (start_state == HIGH)
  {
    start_state = LOW;
    digitalWrite(START_PIN,LOW);
  }  
  
  return false; // to repeat the action - false to stop
}
MIDI_CREATE_DEFAULT_INSTANCE();
// timers
auto timer = timer_create_default();

void setup() {

  
  // inputs
  pinMode(CLOCK_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, INPUT_PULLUP);

  // outputs
  //for (uint8_t i = 0; i < NUMBER_OUTS; i++) pinMode(out_pins[i], OUTPUT);
  pinMode(PPQN_PIN, OUTPUT);
  pinMode(QNPP_PIN, OUTPUT);  
  pinMode(START_PIN, OUTPUT);
  pinMode(VOICE_PIN, OUTPUT);
  
  // set up interrupt on clock pin
  attachInterrupt(digitalPinToInterrupt(CLOCK_PIN), manage_midi_clock, CHANGE);

  // set up interrupt on reset pin
  attachInterrupt(digitalPinToInterrupt(RESET_PIN), reset_outputs, FALLING);

  interrupts();
  
  // Initialize and configure MIDI
  MIDI.begin(10); // The TR-6S uses the conventional Channel 10 for percussion.
  MIDI.setHandleNoteOn(MyHandleNoteOn);
  MIDI.setHandleNoteOff(MyHandleNoteOff);
  MIDI.setHandleClock(ClockHandler);
  MIDI.setHandleStart(StartHandler);
#ifdef DEBUG
  Serial.begin(115200);
  Serial.println("hello"); 
#endif
  // Reset counters & set all inputs LOW
  reset_outputs();
  // Initialize potentiometer settings
  read_pots();
}

void loop() {  
  MIDI.read();
  digitalWrite(PPQN_PIN,  (clock_state == HIGH) ? ppqn_state : LOW);
  digitalWrite(QNPP_PIN,  (clock_state == HIGH) ? qnpp_state : LOW);
  digitalWrite(VOICE_PIN, voice_state);
  digitalWrite(START_PIN, (clock_state == HIGH) ? start_state : LOW);
  timer.tick();
}

void reset_outputs(void)
{
  // load each counter with its MAX value -> Ready for the beat!
  //for (uint8_t i = 0; i < NUMBER_OUTS; i++) out_states[i] = LOW;
  ppqn_state = LOW;
  qnpp_state = LOW;
  start_state = LOW;
  voice_state = LOW;
  ppqn_counter = 0;
  qnpp_counter = 0;
  
  read_pots();

#ifdef DEBUG
  Serial.print("PPQN: ");
  Serial.println(ppqn_counter);
  for (uint8_t i = 0; i < NUMBER_OUTS; i++)
  {
    Serial.print("\t");
    Serial.print(out_states[i]);
  }
  Serial.println("\n\t----------------------------------------------------------------------");
#endif
}

void read_pots(void)
{
  ppqnPotVal =  analogRead(PPQN_POT_PIN);
  qnppPotVal =  analogRead(QNPP_POT_PIN);
  voicePotVal = analogRead(VOICE_POT_PIN);
  
  // map it to the range of the analog out:
  ppqnSelect =  map(ppqnPotVal, 0, 1023, 0, 7);
  qnppSelect =  map(qnppPotVal, 0, 1023, 1, MAX_QNPP);
  voiceSelect = map(voicePotVal, 0, 1023, 1, MAX_VOICE);

  // The voice selection has to be mapped to a note number
  // from the TR-6S MIDI Implementation datasheet.
  switch (voiceSelect) {
    case 1:
      // BD / Bass Drum
      voiceMap = 36;
      break;
    case 2:
      // SD / Snare Drum
      voiceMap = 38;
      break;
    case 3:
      // LT / Low Tom
      voiceMap = 43;
      break;
    case 4:
      // HC / Hand Clap
      voiceMap = 39;
      break;
    case 5:
      // CH / Closed Hat
      voiceMap = 42;
      break;
    case 6:
      // OH / Open Hat
      voiceMap = 46;
      break;    
  }
}

// MyHandleNoteON is the function that will be called by the Midi Library
// when a MIDI NOTE ON message is received.
// It will be passed bytes for Channel, Pitch, and Velocity
void MyHandleNoteOn(byte channel, byte pitch, byte velocity) {
  //TODO: Only bring the LED and VOICE_PIN high if the pitch byte
  //      matches our selected note number.
  if (pitch == voiceMap)
  {
    //digitalWrite(VOICE_PIN,HIGH); // Bring the VOICE output high
    voice_state = HIGH;
  }
}

// MyHandleNoteOFF is the function that will be called by the Midi Library
// when a MIDI NOTE OFF message is received.
// * A NOTE ON message with Velocity = 0 will be treated as a NOTE OFF message *
// It will be passed bytes for Channel, Pitch, and Velocity
void MyHandleNoteOff(byte channel, byte pitch, byte velocity) {
  if (pitch == voiceMap) 
  {
    //digitalWrite(VOICE_PIN,LOW); // Set VOICE output back to low
    voice_state = LOW;
  }
}

void ClockHandler(void) {
  // Set the pseudo_pin to true, the logical equivalent of a clock pulse for manage_clock()
  fake_pin = true;
  // Call the clock management sub to manage pin states
  manage_midi_clock();
  // Nobody is going to call us back, so we need to set a timer.
  timer.in(5, timer_handler);
}

void StartHandler(void) {
  // First, set all the outputs low and restart the counters.
  reset_outputs();
  // TR-6S sends out a pulse when the START button is pressed. 
  // Set the START_PIN high
  //digitalWrite(START_PIN,HIGH);
  // Set the global start_state for the timer_handler
  start_state = HIGH;
  // Call me, maybe?
  timer.in(5, timer_handler);
}

void manage_midi_clock()
{
  bool current_fake_pin_state = fake_pin;
  // We'll debounce this, i guess...
  if (current_fake_pin_state != last_fake_pin_state)
  {
    last_fake_pin_state = current_fake_pin_state; // keep that in mind for the next call
    // manage either start or end of pulse (the logic is negative: LOW means voltage is present)
    if (current_fake_pin_state) // true is like HIGH
    {
      clock_state = HIGH;
      // Just the downbeat for now
      //out_states[0] = (ppqn_counter == 0);
      //ppqn_state = (ppqn_counter == 0);
      /*
      binary_1_out = (ppqn_counter == 0);
      binary_2_out = (ppqn_counter == 0) || (ppqn_counter == 12);
      binary_4_out = (ppqn_counter == 0) || (ppqn_counter == 6) || (ppqn_counter == 12) || (ppqn_counter == 18);
      binary_8_out = (ppqn_counter == 0) || (ppqn_counter == 3) || (ppqn_counter == 6) || (ppqn_counter == 9) || (ppqn_counter == 12) || (ppqn_counter == 15) || (ppqn_counter == 18) || (ppqn_counter == 21);

      ternary_3_out = (ppqn_counter == 0) || (ppqn_counter == 8) || (ppqn_counter == 16);
      ternary_6_out = (ppqn_counter == 0) || (ppqn_counter == 4) || (ppqn_counter == 8) || (ppqn_counter == 12) || (ppqn_counter == 16) || (ppqn_counter == 20);
      ternary_12_out = ~(ppqn_counter & 0b00000001);
      */
      switch (ppqnSelect) {
        case 0:
          // Quarter notes (1/24 MIDI)
          ppqn_state = (ppqn_counter == 0);
          break;
        case 1:
          // Eighth Notes (1/12)
          ppqn_state = (ppqn_counter == 0) || (ppqn_counter == 12);
          break;
        case 2:
          // Sixteenth Notes (1/6)
          ppqn_state = (ppqn_counter == 0) || (ppqn_counter == 6) || (ppqn_counter == 12) || (ppqn_counter == 18);
          break;
        case 3:
          // Thirty-second Notes (1/3)
          ppqn_state = (ppqn_counter == 0) || (ppqn_counter == 3) || (ppqn_counter == 6) || (ppqn_counter == 9) || (ppqn_counter == 12) || (ppqn_counter == 15) || (ppqn_counter == 18) || (ppqn_counter == 21);
          break;
        case 4:
          // Triplets
          ppqn_state = (ppqn_counter == 0) || (ppqn_counter == 8) || (ppqn_counter == 16);
          break;
        case 5:
          // Sextuplets
          ppqn_state = (ppqn_counter == 0) || (ppqn_counter == 4) || (ppqn_counter == 8) || (ppqn_counter == 12) || (ppqn_counter == 16) || (ppqn_counter == 20);
          break;
        case 6:
          // 12uplets
          ppqn_state = ~(ppqn_counter & 0b00000001);
          break;
        case 7:
          // 24PPQN
          ppqn_state = HIGH;
          break;
      }
      qnpp_state = ((qnpp_counter == 0) && (ppqn_counter == 0));
    }
    else
    {
      // Falling Edge -> be ready for next step
      clock_state = LOW;

      // increment ppqn_counter
      ppqn_counter++;
  
      // reset ppqn_counter if it overflow its resolution
      if (ppqn_counter >= MAX_RESOLUTION)
      {
        ppqn_counter = 0;
        // If we've pulsed through an entire quarter note, we can increment the QNPP counter.
        qnpp_counter++;
        // likewise, it must reset on overflow
        if (qnpp_counter >= qnppSelect)
        {
          qnpp_counter = 0;
        }
      }      
    }
  }
  else
  {
    // we are in the same state as previous call... this shouldn't happen (bounces?)
    // do nothing
  }
}

//void manage_clock()
//TODO DRY this out. We want to let a pulse on the CLOCK_PIN advance 
//     the ppqn_counter and qnpp_counter.
//{
//  uint8_t current_clock_pin_state = digitalRead(CLOCK_PIN);
//
//  // check for effective change (removes some bouncy effect)
//  if (current_clock_pin_state != last_clock_pin_state)
//  {
//    last_clock_pin_state = current_clock_pin_state; // keep that in mind for the next call
//
//    // manage either start or end of pulse (the logic is negative: LOW means voltage is present)
//    if (current_clock_pin_state == LOW)
//    {
//      // Rising Edge -> start the beat!

//      // the clock state is useful for activating the outputs only while the clock is high
//      clock_state = HIGH;
//      // -> the outputs are allowed to turn ON
//      ppqn_state = (ppqn_counter == 0);
//      qnpp_state = (qnpp_counter == 0);
/*
  binary_1_out = (ppqn_counter == 0);
  binary_2_out = (ppqn_counter == 0) || (ppqn_counter == 12);
  binary_4_out = (ppqn_counter == 0) || (ppqn_counter == 6) || (ppqn_counter == 12) || (ppqn_counter == 18);
  binary_8_out = (ppqn_counter == 0) || (ppqn_counter == 3) || (ppqn_counter == 6) || (ppqn_counter == 9) || (ppqn_counter == 12) || (ppqn_counter == 15) || (ppqn_counter == 18) || (ppqn_counter == 21);

  ternary_3_out = (ppqn_counter == 0) || (ppqn_counter == 8) || (ppqn_counter == 16);
  ternary_6_out = (ppqn_counter == 0) || (ppqn_counter == 4) || (ppqn_counter == 8) || (ppqn_counter == 12) || (ppqn_counter == 16) || (ppqn_counter == 20);
  ternary_12_out = ~(ppqn_counter & 0b00000001);
 */
      // BINARY 1 (1)
      //out_states[0] = (ppqn_counter == 0);
      // BINARY 2 (1/2)
      //out_states[1] = (ppqn_counter == 0) || (ppqn_counter == 12);
      // BINARY 4 (1/4)
      //out_states[2] = (ppqn_counter == 0) || (ppqn_counter == 6) || (ppqn_counter == 12) || (ppqn_counter == 18);
      // BINARY 8 (1/8)
      //out_states[3] = (ppqn_counter == 0) || (ppqn_counter == 3) || (ppqn_counter == 6) || (ppqn_counter == 9) || (ppqn_counter == 12) || (ppqn_counter == 15) || (ppqn_counter == 18) || (ppqn_counter == 21);

      // TERNARY 3 (1/3)
      //out_states[4] = (ppqn_counter == 0) || (ppqn_counter == 8) || (ppqn_counter == 16);
      // TERNARY 6 (1/6)
      //out_states[5] = (ppqn_counter == 0) || (ppqn_counter == 4) || (ppqn_counter == 8) || (ppqn_counter == 12) || (ppqn_counter == 16) || (ppqn_counter == 20);
      // TERNARY 12 (1/12)
      //out_states[6] = (~(ppqn_counter & 0b00000001) & 0b00000001);

      // 24 PPQN (copy from input clock) or any other function ----- TBD ------
      //out_states[7] = 0;  // TBD
//#ifdef DEBUG
//      Serial.print("\t");
//      Serial.print(debug_counter);
//      debug_counter++;
      
//      Serial.print("\t");
//      Serial.print(ppqn_counter);

//      for (uint8_t i = 0; i < NUMBER_OUTS; i++)
//      {
//        Serial.print("\t");
//        Serial.print(out_states[i]);
//      }
//      Serial.println("");
//#endif
//    }
//    else
//    {
//      // Falling Edge -> be ready for next step
//      clock_state = LOW;

//      // increment ppqn_counter
//      ppqn_counter++;
  
//      // reset ppqn_counter if it overflow its resolution
//      if (ppqn_counter >= MAX_RESOLUTION)
//      {
//        ppqn_counter = 0;
//        qnpp_counter++;
//        if (qnpp_counter >= qnppSelect)
//        {
//          qnpp_counter = 0;
//        }
//      }
//#ifdef DEBUG
//      Serial.println(".");
//#endif
//    }
//  }
//  else
//  {
//    // we are in the same state as previous call... this shouldn't happen (bounces?)
//    // do nothing
//#ifdef DEBUG
//      Serial.println("B");
//#endif
//  }
//}
