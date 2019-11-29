//
// LOAD THIS ONTO THE NANO AND SET SLAVE NUMBER
//  - 8 = left channel
//  - 9 = right channel
//
// THIS SKETCH SIMPLY CAPTURES, FORMATS VU VALUES BETWEEN 0 AND 8 FOR THE NUMBER COLUMNS (MATRIX_WIDTH)
// IT THEN STORES THEM INTO data_avgs UNTIL A REQUEST FROM MASTER RECEIVED TO PROVIDE IT.
//

#include <Wire.h>
#include <arduinoFFT.h>

// #define DEBUG             TRUE

#define SLAVE_ID_PIN        2                 // Pin tied to ground = LEFT, Pin high = RIGHT
#define SLAVE_LEFT_CHANNEL  8
#define SLAVE_RIGHT_CHANNEL 9
int     slaveNumber = SLAVE_RIGHT_CHANNEL;    // Default it to RIGHT channel unless the SLAVE_ID_PIN is tied to ground...

#define SAMPLES            64                 // Must be a power of 2

#define MATRIX_HEIGHT       8
#define MATRIX_WIDTH       16

double vReal[SAMPLES];
double vImag[SAMPLES];
char data_avgs[MATRIX_WIDTH];
char prev_avgs[MATRIX_WIDTH];

int yvalue;
int displaycolumn , displayvalue;
int peaks[MATRIX_WIDTH];

arduinoFFT FFT = arduinoFFT();  // FFT object

#ifdef DEBUG
char dbgMsg[32];
#endif

void setup() {

// #ifdef DEBUG
  Serial.begin(115200);
// #endif

  // Read the SLAVE_ID_PIN to see if this is the left or right channel...
  pinMode(SLAVE_ID_PIN, INPUT_PULLUP);
  if (digitalRead(SLAVE_ID_PIN) == LOW) {
    slaveNumber = SLAVE_LEFT_CHANNEL;
// #ifdef DEBUG
    Serial.println("VU Analyser: Setup as Left channel");
// #endif
  }
  else {
    slaveNumber = SLAVE_RIGHT_CHANNEL;
// #ifdef DEBUG
    Serial.println("VU Analyser: Setup as Right channel");
// #endif
  }

  Wire.begin(slaveNumber);     // join i2c bus with address #8
  Wire.onRequest(requestEvent); // register event

  // ADCSRA = 0b11100101;       // set ADC to free running mode and set pre-scalar to 32 (0xe5)
  ADCSRA = 0b11100010;          // set ADC to free running mode and set pre-scalar to 32 (0xe5)
  ADMUX = 0b00000000;           // use pin A0 and external voltage reference

  delay(50);                    // wait to get reference voltage stabilized
}

void loop() {
  takeSample();
  processSample();
  formatSample();
}

int lastVal = 0;

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {

  char vals[MATRIX_WIDTH + 1];

  for (int i = 0; i < MATRIX_WIDTH; i++)
  {
    vals[i] = data_avgs[i] + '0';
  }
  vals[MATRIX_WIDTH] = '\0';
  Wire.write(vals);

#ifdef DEBUG
  Serial.println(vals);
#endif
}

// ++ Sampling
void takeSample() {
  int value;

  for (int i = 0; i < SAMPLES; i++)
  {
    while (!(ADCSRA & 0x10));         // wait for ADC to complete current conversion ie ADIF bit set
    ADCSRA = 0b11110101 ;             // clear ADIF bit so that ADC can do next operation (0xf5)
    value = ADC - 512 ;               // Read from ADC and subtract DC offset caused value
    vReal[i] = value / MATRIX_HEIGHT; // Copy to bins after compressing
    vImag[i] = 0;
  }
}

// ++ FFT
void processSample() {
  // FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_WELCH, FFT_FORWARD);
  // FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_RECTANGLE, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
}


// ++ re-arrange FFT result to match with no. of columns on display ( MATRIX_WIDTH )
void formatSample() {
  int step = (SAMPLES / 2) / MATRIX_WIDTH;
  int c = 0;

  for (int i = 0; i < (SAMPLES / 2); i += step)
  {
    data_avgs[c] = 0;
    for (int k = 0 ; k < step ; k++) {
      data_avgs[c] = data_avgs[c] + vReal[i + k];
    }
    data_avgs[c] = data_avgs[c] / step;

    // Normalise the values to between 0 and MATRIX_HEIGHT...
    data_avgs[c] = constrain(data_avgs[c], 0, 64);                        // set max & min values for buckets
    data_avgs[c] = map(data_avgs[c], 0, 64, 0, MATRIX_HEIGHT);            // remap averaged values to MATRIX_HEIGHT

    c++;
  }
}
