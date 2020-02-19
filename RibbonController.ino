/*
 Created: Chip Audette, Feb/Mar 2016
 Changed: Jim Foster, Feb 2020
 Purpose: Sense Velocity, Pressure and Possition to be translated into corresponding midi values.
 Physical Setup:
 Ribbon;
 Top - A2 (VccPin)
 Wiper - A3 (RibbonPin) 
 Bottom - A1 (GndPin) 
 Fsr Top - 5v (Vcc) 
 Fsr Bottom - A0 (fsrPin)  //pulled to gnd via 10k resistor

 */

// initialize variables for analogIns
const int ribbonVccPin = A2;  //this is the "top" of the ribbon (the "bottom" is gnd)
const int ribbonPin = A3;     //this is the "wiper" of the ribbon
const int ribbonGnd = A1;     //this is the "bottom" of the ribbon
const int fsrPin = A0;
//Define ribbon parameters
#define RIBBON_SPAN (1023)  // the Arduino AnalogRead() spans 0-1023
const int ribbon_max_val = 382; //for my ribbon on my Arduino UNO
const int ribbon_min_val = 16; //for my ribbon on my Arduino UNO
float R_pullup_A3_kOhm = 0.0;  //computed in setup()
float R_ribbon_min_kOhm = 0.0; //computed in setup()
const float R_ribbon_max_kOhm = 17.45; //for my ribbon, measured via multi-meter
const float ribbon_span_half_steps_float = 24;  //how many half-steps do I want my ribbon to represent
const float ribbon_span_extra_half_steps_float = 0.5; //twiddle factor

//what note do you want the bottom of the ribbon to represent?
#define MIDI_note_bottom (43-7)  //Note C1

//what MIDI channel do you want to use
#define MIDI_CHANNEL (0x01)  //let's use channel 1.  Channel 0 is universal.

//define MIDI message numbers
#define NOTE_ON (0x90+MIDI_CHANNEL)
#define NOTE_OFF (0x80+MIDI_CHANNEL)
#define PITCH_BEND (0xE0+MIDI_CHANNEL)
#define CHANNEL_PRESSURE (0xD0+MIDI_CHANNEL)

#define NOTE_ON_VEL (127)  //0x7F
#define NOTE_OFF_VEL (0)  //0x7F

//Below, set the range of the synth's bend wheel.
//Many synths default to +/- 2 half steps.  You can
//often change it to a bigger value.  Bigger values
//permit more smooth pitch changes
#define synth_bend_half_steps (24) 

//define some global variables
int prev_note_num = 0;
boolean prev_note_on = false;
int MIDI_note_num = 0, prev_MIDI_note_num = 0;
unsigned int bend_int = 0, prev_bend_int = 0;
byte pitch_bend_LSB, pitch_bend_MSB;
boolean flag_pitch_is_bent = false;
int MIDI_note_vel = 0, prev_note_vel = 0, aftertouch = 0, prev_aftertouch = 0;


//setup routine
void setup() {
  //setup the ribbon pins and the analogRead reference
  pinMode(ribbonVccPin, INPUT); //set to high impedance
  pinMode(ribbonPin, INPUT_PULLUP); //set as input, but with pullup
  pinMode(ribbonGnd, OUTPUT); digitalWrite(ribbonGnd,LOW); //use as ground for the ribbon

  //compute scaling parameters for the ribbon
  R_pullup_A3_kOhm = R_ribbon_max_kOhm * ( ((float)RIBBON_SPAN) / ((float)ribbon_max_val) - 1.0 );
  R_ribbon_min_kOhm = ((float)ribbon_min_val) / ((float)ribbon_max_val) * R_ribbon_max_kOhm;

  //initialize the MIDI communications:
  //Serial.begin(115200);  //for debugging only (not sending MIDI to synth!)
  Serial.begin(31250);  // MIDI standard speed
}

void loop() {
 // read the ribbon and send the MIDI commands
  updateTheSystem();
 // debug the raw values 
  //debug();
  // delay a bit so that we only get an update rate of 60-100Hz
  delay(10);
}
void debug() {
  int ribbon = analogRead(A3);
  int fsr = analogRead(A0);
  Serial.print("ribbon;");
  Serial.print("||");
  Serial.print("fsrRaw");
  Serial.print("||");
  Serial.println(" ");
  Serial.print(ribbon);
  Serial.print("||");
  Serial.print(fsr);
  Serial.print("||");
  Serial.println(" ");
  delay(20);
}

void updateTheSystem() {

  //read input
  int ribbon_value = analogRead(ribbonPin);
  int fsr = analogRead(fsrPin);

  //process the ribbon value
  float note_num_float = processRibbonValue(ribbon_value); //zero is bottom of ribbon, 1.0 is top of ribbon

  //figure out which note number the ribbon value corresponds to
  note_num_float = note_num_float - ribbon_span_extra_half_steps_float/2.0;
  int note_num = ((int)(note_num_float + 0.5)); //round

  //is the ribbon being pressed?  Is a note on?
  boolean note_on = false;
  if (ribbon_value < (RIBBON_SPAN-10)) {
    if ((note_num >= 0) && (note_num <= ribbon_span_half_steps_float)) note_on = true;
  } else {
    //the ribbon value is weird.  use the old note number because it's safer
    note_num = prev_note_num; //just in case there was a bad value
  }
  
  //calc MIDI note number/ velocity/aftertouch
  MIDI_note_vel = map(fsr, 0, 990, 30, 127);
  aftertouch = map(fsr, 400, 980, 0, 127);
  note_num = constrain(note_num, 0, ribbon_span_half_steps_float);
  MIDI_note_num = note_num + MIDI_note_bottom;
  boolean note_change = (note_on != prev_note_on) | (note_num != prev_note_num);

  //compute the amount of bend to apply to the note
  computePitchBend(note_num_float,note_num);

  //write MIDI commands
   writeMIDI(note_change,note_on,prev_note_on,MIDI_note_num,MIDI_note_vel);
   Aftertouch(aftertouch);
   
  //save state for next time through the loop
  prev_note_vel = MIDI_note_vel;
  prev_aftertouch = aftertouch;
  prev_note_num = note_num;
  prev_MIDI_note_num = MIDI_note_num;
  prev_note_on = note_on;
  prev_bend_int = bend_int;
}

float processRibbonValue(int ribbon_value) {
   //apply the first scaling to do the primary linearization of the response.
  //This chunk of code assumes that the soft pot is a perfect potentiometer
  //and that the wiper is being pulled up via a pullup resistor (built into the
  //arduino).  When the pullup resistor is a similar value as the ribbon (which it is)
  //the voltage at the wiper is not as linear as we'd like.  The equations below
  //are simply solve the resistor divider to figure out where the user touched
  //the potentiometer.
  float ribbon_value_float = (float)ribbon_value;
  float foo_float = 1.0 / (((float)RIBBON_SPAN) / ribbon_value_float - 1.0);
  float ribbon_frac = (R_pullup_A3_kOhm / R_ribbon_max_kOhm) * foo_float;
  float ribbon_R_kOhm = R_pullup_A3_kOhm * foo_float;
  float ribbon_span_frac = (ribbon_R_kOhm - R_ribbon_min_kOhm) / (R_ribbon_max_kOhm - R_ribbon_min_kOhm);
  ribbon_span_frac = max(0.0, ribbon_span_frac);
  float note_num_float = ribbon_span_frac * (ribbon_span_half_steps_float + ribbon_span_extra_half_steps_float);
  note_num_float = note_num_float - ribbon_span_extra_half_steps_float/2.0;

  //could apply more code here to calibrate for additional weirdness in your
  //own ribbon
  //
  //more code
  //more code

  return note_num_float;
}

void computePitchBend(float note_num_float, int note_num) {
  float partial_note_num_float = note_num_float - ((float)note_num);
  float partial_bend_float = partial_note_num_float / ((float)synth_bend_half_steps);
  partial_bend_float = constrain(partial_bend_float, -1.0, 1.0);
  #define MIDI_MAX_BEND  (8192)  //can go up and down by this amount
  
  //these are the values that are output and used by the rest of the system
  bend_int = (unsigned int)(MIDI_MAX_BEND + (partial_bend_float * ((float)MIDI_MAX_BEND)));
  pitch_bend_LSB = ((byte)bend_int) & 0b01111111;  //get lower 7 bits, ensure first bit to zero
  pitch_bend_MSB = ((byte)(bend_int >> 7)) & 0b01111111;  //get upper 7 bits, set first bit to zero
}
void Aftertouch(int aftertouch) {
  if (aftertouch != prev_aftertouch) {
  Serial.write((byte)CHANNEL_PRESSURE);
  Serial.write((byte)aftertouch);
  }
}

void transmitPitchBend(void) {
    Serial.write((byte)PITCH_BEND);
    Serial.write((byte)pitch_bend_LSB);
    Serial.write((byte)pitch_bend_MSB);
    flag_pitch_is_bent = true;
}

void recenterMidiPitchBend(void) {
  Serial.write((byte)PITCH_BEND);
  Serial.write((byte)0x00);  //center,LSB
  Serial.write((byte)64);  //center,MSB}
  flag_pitch_is_bent = false;
}

void writeMIDI(boolean note_change,boolean note_on,boolean prev_note_on,int MIDI_note_num,int MIDI_note_vel) { 
//write MIDI messages
if (note_change) { //has a note turned on/off or changed value?
    if (note_on) {
      //turn on new note
      Serial.write((byte)NOTE_ON);
      Serial.write((byte)MIDI_note_num);
      Serial.write((byte)MIDI_note_vel);  //velocity NOTE_ON_VEL
  
      //send pitch bend info
      transmitPitchBend();
  
      //turn off the old note?
      if (prev_note_on == true) {
        //previous note was on...so assume legato...turn off old note now that new one is already started
        Serial.write((byte)NOTE_OFF);
        Serial.write((byte)prev_MIDI_note_num);
        Serial.write((byte)NOTE_OFF_VEL);
      }
      
    } else if ((note_on == false) && (prev_note_on == true)) {
      //no other notes are active.  turn note off
      Serial.write((byte)NOTE_OFF);
      Serial.write((byte)prev_MIDI_note_num);
      Serial.write((byte)NOTE_OFF_VEL);
  
      //remove the pitch bend command
      recenterMidiPitchBend();
    }
  } else {
    //we're not changing note number...but we should still update the pitch bend info
    if (note_on == true) {
      if (bend_int != prev_bend_int) { //but only if it has changed
        transmitPitchBend();
      }
    }
  }
}
//Open note not yet implemented, unsure how to go about it at this point
void Open_note(int MIDI_note_vel, boolean open_note, boolean note_on) { 
      Serial.write((byte)NOTE_ON);
      Serial.write((byte)36);
      Serial.write((byte)MIDI_note_vel);
}
//what do? 
