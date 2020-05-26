/*

	Example of use of the FFT libray to compute FFT for a signal sampled through the ADC.
        Copyright (C) 2017 Enrique Condes

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "arduinoFFT.h"
#include <SparkFun_ADXL345.h>

ADXL345 adxl = ADXL345();

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
/*
  These values can be changed in order to evaluate the functions
*/
#define CURR A0
#define testLED 11
#define accLED 13
#define currLED 10
#define tempLED 12

const uint16_t samples = 128; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1000;

unsigned int delayTime = 0;



/*
  These are the input and output vectors
  Input vectors receive computed results from FFT
*/
double vReal[samples];
double vImag[samples];

double vRealAvg[samples];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02

int incomingByte = 0;

void setup()
{
  pinMode(testLED, OUTPUT);
  pinMode(accLED, OUTPUT);
  pinMode(currLED, OUTPUT);
  pinMode(tempLED, OUTPUT);

  pinMode(CURR, INPUT);


  digitalWrite(testLED, HIGH);
  delay(100);
  digitalWrite(accLED, HIGH);
  delay(100);
  digitalWrite(currLED, HIGH);
  delay(100);
  digitalWrite(tempLED, HIGH);
  delay(100);
  digitalWrite(testLED, LOW);
  delay(100);
  digitalWrite(accLED, LOW);
  delay(100);
  digitalWrite(currLED, LOW);
  delay(100);
  digitalWrite(tempLED, LOW);
  delay(100);
  init_acc();
  if (samplingFrequency <= 1000)
    delayTime = 1000 / samplingFrequency;
  else
    delayTime = 1000000 / samplingFrequency;
  SerialUSB.begin(115200);
  while (!SerialUSB);
  //SerialUSB.println("Ready");
}

void loop()
{
  char select = 'n';
  if (SerialUSB.available() > 0) {
    // read the incoming byte:
    incomingByte = SerialUSB.read();
    select = char(incomingByte);
    // say what you got:
    //SerialUSB.print("I received: ");
    //SerialUSB.println(select);
    SerialUSB.flush();

    switch (select) {
      case 'f':
        {digitalWrite(accLED, HIGH);
        AvgACC(10);
        SerialUSB.println();
        digitalWrite(accLED, LOW);
        break;}
      case 'c':
        {digitalWrite(currLED, HIGH);
        float current = measCurrent(1000);
        SerialUSB.print(current, 4);
        SerialUSB.println();
        digitalWrite(currLED, LOW);
        break;}
      case 't':
        {digitalWrite(testLED, HIGH);
        digitalWrite(accLED, HIGH);
        AvgACC(10);
        digitalWrite(accLED, LOW);
        digitalWrite(currLED, HIGH);
        float testcurr = measCurrent(1000);
        SerialUSB.print(',');
        SerialUSB.print(testcurr, 4);
        SerialUSB.println();
        digitalWrite(currLED, LOW);
        digitalWrite(testLED, LOW);
        break;}
    }
  }
}


double measCurrent(int samples) {
  int avg = 0;
  for (int sample = 0; sample < samples; sample++) {
    avg = analogRead(CURR) + avg;
  }
  avg = avg / samples;
  //SerialUSB.println(avg);
  avg = map(avg, 0, 1023, 0, 10000);
  float avgnorm = (float) avg / 10000;
  return avgnorm;
}

void init_acc() {
  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(2);           // Give the range settings
  // Accepted values are 2g, 4g, 8g or 16g
  // Higher Values = Wider Measurement Range
  // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
  // Default: Set to 1
  // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library

  adxl.setActivityXYZ(0, 0, 0);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)

  adxl.setInactivityXYZ(0, 0, 0);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(0, 0, 0); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)

  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment

  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment

  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);"
  // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
  // This library may have a problem using INT2 pin. Default to INT1 pin.

  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(0);
  adxl.ActivityINT(0);
  adxl.FreeFallINT(0);
  adxl.doubleTapINT(0);
  adxl.singleTapINT(0);
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
        break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
        break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
        break;
    }
    SerialUSB.print(abscissa, 6);
    SerialUSB.print(" ");
    SerialUSB.print(vData[i], 4);
    SerialUSB.println();
  }
  SerialUSB.println();
}

void SumVector(double *vData, uint16_t bufferSize) {

  for (uint16_t i = 0; i < bufferSize; i++)
  {
    vRealAvg[i] = vData[i] + vRealAvg[i];
    //SerialUSB.print(vData[i], 4);
    //SerialUSB.println();
  }

}

void AvgACC(int avgNum) {
  for (uint16_t j = 0; j < (samples >> 1); j++)
  {
    vRealAvg[j] = 0;
  }
  for (int i = 0; i < avgNum; i++) {
    CalcFFT();
    SumVector(vReal, (samples >> 1));
  }
  for (uint16_t k = 0; k < (samples >> 1); k++)
  {
    vRealAvg[k] = vRealAvg[k] / avgNum;
    vRealAvg[k] = vRealAvg[k] / 1000;
    SerialUSB.print(vRealAvg[k], 4);
    if (k < ((samples>>1)-1)){
    SerialUSB.print(",");
    }
  }
}

void CalcFFT() {
  for (uint16_t i = 0; i < samples; i++)
  {
    int x, y, z;
    adxl.readAccel(&x, &y, &z);
    vReal[i] = double(z);
    vImag[i] = 0.0; //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
    if (samplingFrequency <= 1000)
      delay(delayTime);
    else
      delayMicroseconds(delayTime);
  }
  /* Print the results of the sampling according to time */
  //SerialUSB.println("Data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  //SerialUSB.println("Weighed data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  //SerialUSB.println("Computed Real values:");
  //PrintVector(vReal, samples, SCL_INDEX);
  // SerialUSB.println("Computed Imaginary values:");
  //PrintVector(vImag, samples, SCL_INDEX);
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  //SerialUSB.println("Computed magnitudes:");
  //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //SerialUSB.println(x, 6);
  //while (1); /* Run Once */

}

