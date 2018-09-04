#include <Adafruit_NeoPixel.h>
#include "avr/pgmspace.h"
#include "DAC_MCP4X.h"
#include "Oscillator.h"


#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


#define CAPTEUR1 0
#define CAPTEUR2 1
#define CAPTEUR3 2
#define CAPTEUR4 3
#define CAPTEUR5 4
#define CAPTEUR6 5
#define DELAYTIME 50

#define NUMPIXELS 12
#define PIN 2

#define CONSTRAINMAX 50
#define CONSTRAINMIN 0

#define INTENSITY 255
#define VOLUME 80

#define NUMREADINGS 10

const unsigned int sine1024[1024] PROGMEM = {32, 32, 32, 32, 32, 32, 33, 33, 33, 33, 33, 34, 34, 34, 34, 34, 35, 35, 35, 35, 35, 36, 36, 36, 36, 36, 37, 37, 37, 37, 37, 37, 38, 38, 38, 38, 38, 39, 39, 39, 39, 39, 40, 40, 40, 40, 40, 40, 41, 41, 41, 41, 41, 42, 42, 42, 42, 42, 42, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 44, 45, 45, 45, 45, 45, 45, 46, 46, 46, 46, 46, 47, 47, 47, 47, 47, 47, 48, 48, 48, 48, 48, 48, 49, 49, 49, 49, 49, 49, 49, 50, 50, 50, 50, 50, 50, 51, 51, 51, 51, 51, 51, 51, 52, 52, 52, 52, 52, 52, 53, 53, 53, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 57, 57, 57, 57, 58, 58, 58, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 63, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 62, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 61, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 60, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 59, 58, 58, 58, 58, 58, 58, 58, 58, 58, 57, 57, 57, 57, 57, 57, 57, 57, 57, 56, 56, 56, 56, 56, 56, 56, 56, 55, 55, 55, 55, 55, 55, 55, 55, 54, 54, 54, 54, 54, 54, 54, 53, 53, 53, 53, 53, 53, 53, 53, 52, 52, 52, 52, 52, 52, 51, 51, 51, 51, 51, 51, 51, 50, 50, 50, 50, 50, 50, 49, 49, 49, 49, 49, 49, 49, 48, 48, 48, 48, 48, 48, 47, 47, 47, 47, 47, 47, 46, 46, 46, 46, 46, 45, 45, 45, 45, 45, 45, 44, 44, 44, 44, 44, 44, 43, 43, 43, 43, 43, 42, 42, 42, 42, 42, 42, 41, 41, 41, 41, 41, 40, 40, 40, 40, 40, 40, 39, 39, 39, 39, 39, 38, 38, 38, 38, 38, 37, 37, 37, 37, 37, 37, 36, 36, 36, 36, 36, 35, 35, 35, 35, 35, 34, 34, 34, 34, 34, 33, 33, 33, 33, 33, 32, 32, 32, 32, 32, 32, 31, 31, 31, 31, 31, 30, 30, 30, 30, 30, 29, 29, 29, 29, 29, 28, 28, 28, 28, 28, 27, 27, 27, 27, 27, 26, 26, 26, 26, 26, 26, 25, 25, 25, 25, 25, 24, 24, 24, 24, 24, 23, 23, 23, 23, 23, 23, 22, 22, 22, 22, 22, 21, 21, 21, 21, 21, 21, 20, 20, 20, 20, 20, 19, 19, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31};
unsigned int phaseStep[8] = {10, 20, 30, 15, 40, 25};

MCP4X dac;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int prevVal = 0, prevVal2 = 0;
const int lookUpTableSize = 1024;
unsigned int cos1 = 0, cos2 = 0, cos3 = 0, sine1 = 0, sine2 = 0, sine3 = 0;

void setupTimers() {


  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 25Khz increments
  OCR1A = 639 ;// = (16*10^6) / (25000*1) - 1 (must be <65536) = 639
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  //TCCR1A |= (1 << COM1A0); // togle PORT1A0 a chaque fois que le compteur atteind sa valeure max. Permet de debuguer la frequence du compteur à l'oscilo
  // Set CS10  bits for 1 prescaler
  TCCR1B |= (1 << CS10);
  //TCCR1B = (1 << CS10) | (1 << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

}


struct OscillatorState {
  unsigned int freq;
  unsigned int vol;
  unsigned int phaseAccu;
  unsigned int phaseStep;
};

struct OscillatorState oscillators[6];
//struct LedState led[2];

ISR(TIMER1_COMPA_vect) { //timer 1 interrupt



  //PORTD |= _BV(PD2);
  updatePhaseAccu();

  //Serial.println(oscillators[0].phaseAccu);
  sine1 = (pgm_read_word(&(sine1024[oscillators[0].phaseAccu]))) * oscillators[0].vol;  // BLEU
  cos1 = (pgm_read_word(&(sine1024[oscillators[1].phaseAccu]))) * oscillators[1].vol;   // VERT

  sine2 = (pgm_read_word(&(sine1024[oscillators[2].phaseAccu]))) * oscillators[2].vol;  // VIOLET
  cos2 = (pgm_read_word(&(sine1024[oscillators[3].phaseAccu]))) * oscillators[3].vol;   // JAUNE

  sine3 = (pgm_read_word(&(sine1024[oscillators[4].phaseAccu]))) * oscillators[4].vol;  // ROUGE
  cos3 = (pgm_read_word(&(sine1024[oscillators[5].phaseAccu]))) * oscillators[5].vol;   // ORANGE

  prevVal = sine1 + sine2 + sine3;
  prevVal2 = cos1 + cos2 + cos3;

  dac.output2(prevVal, prevVal2);
  //PORTD &= ~_BV(PD2);



  //PORTD |= _BV(PD2);
  //    m_phaseAccu += m_phaseStep;
  //    (m_phaseAccu >> 6);
  //(uint8_t)(m_phaseAccu = sineA.setPhaseAccu());
  //(uint8_t)(prevVal = pgm_read_word(&(sine512[m_phaseAccu])));
  //dac.output2(pgm_read_word(&(sine512[m_phaseAccu])),pgm_read_word(&(sine512[m_phaseAccu])));
  //PORTD &= ~_BV(PD2);

  //    if(prevVal == 0){
  //    dac.output(0);
  //    //dac.output2(0,4096);
  //    prevVal = 4096;
  //    }
  //    else{
  //    //dac.output(4096);
  //    //dac.output2(4096,0);
  //    prevVal = 0;
  //    }

}

inline void updatePhaseAccu() {
  for (int i = 0; i < 6; i++) {
    oscillators[i].phaseAccu += oscillators[i].phaseStep;
    oscillators[i].phaseAccu &= 0b0000001111111111;
  }
}

inline void updatePhaseStep() { // Both pair of oscillators must have the same phase step in order to draw a full circle.
  for (int i = 0; i < 6; i++) {
    oscillators[i].phaseStep = phaseStep[i];
  }
}

void readCapteurValue() {

  unsigned int c1 = analogRead(CAPTEUR1);
  unsigned int c2 = analogRead(CAPTEUR2);
  unsigned int c3 = analogRead(CAPTEUR3);
  unsigned int c4 = analogRead(CAPTEUR4);
  unsigned int c5 = analogRead(CAPTEUR5);
  unsigned int c6 = analogRead(CAPTEUR6), c7;

  

  c1 = constrain(c1, CONSTRAINMIN, CONSTRAINMAX);
  c1 = map(c1, CONSTRAINMAX, CONSTRAINMIN, 0, INTENSITY);
  oscillators[0].vol = map(c1, CONSTRAINMIN,INTENSITY, 0, VOLUME);
  //Serial.println(oscillators[0].vol);
  
  c2 = constrain(c2, CONSTRAINMIN, CONSTRAINMAX);
  c2 = map(c2, CONSTRAINMAX, CONSTRAINMIN, 0, INTENSITY);
  oscillators[1].vol = map(c2,CONSTRAINMIN,INTENSITY,0, VOLUME);


  c3 = constrain(c3, CONSTRAINMIN, CONSTRAINMAX);
  c3 = map(c3, CONSTRAINMAX, CONSTRAINMIN, 0, INTENSITY);
  oscillators[2].vol = map(c3,CONSTRAINMIN,INTENSITY,0, VOLUME);

  c4 = constrain(c4, CONSTRAINMIN, CONSTRAINMAX);
  c4 = map(c4, CONSTRAINMAX, CONSTRAINMIN, 0, INTENSITY);
  oscillators[3].vol = map(c4, CONSTRAINMIN,INTENSITY,0, VOLUME);

  c5 = constrain(c5, CONSTRAINMIN, CONSTRAINMAX);
  c5 = map(c5, CONSTRAINMAX, CONSTRAINMIN, 0, INTENSITY);
  oscillators[4].vol = map(c5,CONSTRAINMIN,INTENSITY,0, VOLUME);

  c6 = constrain(c6, CONSTRAINMIN, CONSTRAINMAX);
  c7 = map(c6, CONSTRAINMAX, CONSTRAINMIN, 0, INTENSITY); // 155
  oscillators[5].vol = map(c7,CONSTRAINMIN,INTENSITY,0, VOLUME);


  pixels.setPixelColor(0, 0, 0, c1); //Serial.println(c2);BLEU
  pixels.setPixelColor(1, 0, 0, c1); //Serial.println(c2);BLEU
  pixels.setPixelColor(2, 0, c2, 0); //Serial.println(c2);VERT
  pixels.setPixelColor(3, 0, c2, 0); //Serial.println(c2);VERT
  pixels.setPixelColor(4, c3, 0, c3); //Serial.println(c2);VIOLET
  pixels.setPixelColor(5, c3, 0, c3); //Serial.println(c2);VIOLET
  pixels.setPixelColor(6, c4, c4, 0); //Serial.println(c2);VIOLET
  pixels.setPixelColor(7, c4, c4, 0); //Serial.println(c2);JAUNE
  pixels.setPixelColor(8, c5, 0, 0); //Serial.println(c2);ROUGE
  pixels.setPixelColor(9, c5, 0, 0); //Serial.println(c2);ROUGE

  if (c7 == 0) {
    pixels.setPixelColor(10, 0, 0, 0); //Serial.println(c2);ORANGE
    pixels.setPixelColor(11, 0, 0, 0); //Serial.println(c2);ORANGE
  }
  else {
    pixels.setPixelColor(10, 255, c7, 0); //Serial.println(c2);ORANGE
    pixels.setPixelColor(11, 255, c7, 0); //Serial.println(c2);ORANGE
  }
}



int smoothing(int value) {

  // subtract the last reading:
  total = total - readings[readIndex];
  // Udpdate array with last value:
  readings[readIndex] = value;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / NUMREADINGS;
  //Serial.println(average);
  return average;

}

void setup() {

  cli();//stop interrupts
  //pinMode(2, OUTPUT);
  //pinMode(9, OUTPUT);
  //Serial.begin(9600);

  // Init Timers
  setupTimers();

  // Init Pixels
  pixels.begin(); // This initializes the NeoPixel library.

  // Init DAC
  dac.init(MCP4X_4822, 5000, 5000, 10, 7, 1);
  dac.setGain2x(MCP4X_CHAN_A, 1);
  dac.setGain2x(MCP4X_CHAN_B, 1);
  dac.begin(1);

  // Init Oscillators Volume
  for (int i = 0; i < 6; i++) {
    oscillators[i].vol = 0;
  }

  // Init Oscillators phaseStep
  updatePhaseStep();

  // Init phaseAccu
  oscillators[0].phaseAccu = 0, oscillators[2].phaseAccu = 0, oscillators[4].phaseAccu = 0;
  oscillators[1].phaseAccu = 256, oscillators[3].phaseAccu = 256, oscillators[5].phaseAccu = 256; // We shift one oscillator out of two to get the cosinus value instead of the sinus (sin(x+pi/2)=cos(x))

  sei();//allow interrupts
}




void loop() {

  readCapteurValue();
  pixels.show();
  delay(DELAYTIME);

}
