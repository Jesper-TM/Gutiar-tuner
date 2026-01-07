#include "arduinoFFT.h"
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/*
Project: Guitar tuner 
Date: 2025-12-19
Authours: Arvid Melberg, Jesper Treutiger Milén 

About:
The projct is a piano tuner that, by an Arduino module can show how wrongly tuned a guitar string is.
The module consists of 3 main components, an electret condenser microphone, LED-display and a LED-ring. When 
a button is preseed the program analyses the singal from the microphone and sets a pre defined target-string,
depending on which string is played on the guitar. A band-pass filter is then applied around the target string
in order to remove unessecary frequencies. Depending on how off-tune the guitar string is, an error is 
calculated. The target sting and error is displayd on the LED-screen. The LED-ring shows the error with light 
indicators which makes the tuner easier to use at a longer distance. 
*/

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C  
#define BUTTON_PIN 2

#define SAMPLES 512
#define Fs 8000
#define MIC_PIN A0 
#define ledPin 11

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(24, ledPin, NEO_RGB + NEO_KHZ800);


double vReal[SAMPLES];
double vRealHPS[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT(vReal, vImag, SAMPLES, Fs);

//Guitar string properis
const double fTarget[6] = {82.41,110.00,146.83,196.00,246.94,329.63};
String tones = "Ebgdae";
int targetStringIndex;

int samplingPeriodUs;

//Error cents
static inline double log2d(double x){ return log(x)/log(2.0); }
static inline double centsErr(double f, double f0){ return 300*log2d(f/f0); }

//Harmonic Product Spectrum function
void HPS(){
  double p;
  for(int i=1;i<SAMPLES/10;i++){
    p=vReal[i]*vReal[2*i]*vReal[3*i]*vReal[4*i]*vReal[5*i]; //Negelcts the overtones
    vReal[i]=p;
  }
}


//Sample function, 1024 sampels per call
void sampleBlock(){
  unsigned long t0 = micros();

  for (int i=0;i<SAMPLES;i++){
    unsigned long target = t0 + (unsigned long)i*samplingPeriodUs;
    while ((long)(micros()-target) < 0) { /* wait */ }
    vReal[i] = analogRead(MIC_PIN); // 0..1023
    vImag[i] = 0.0;
  }
}


//Band limit, +-15%  around target string (target)
void bandLimitAroundTarget(double target){
  double fmin = target*0.75;
  double fmax = target*1.15;
  double df = (double)Fs / (double)SAMPLES;

  int maxBin=SAMPLES/2;
  for (int bin=0; bin<maxBin; bin++){
    double f = bin*df;
    if (f<fmin || f>fmax) vReal[bin]=0.0; 
  }


/*
Sets the target string depending on the input argument(peakHz). If the dominant peak from the FFT result is
within +-15% of a string, the desierd value, targetStringIndex is updated 
*/
void setTargetString(double peakHz) {
  for(int i=0;i<=5;i++){
    if((peakHz <= fTarget[i]*1.15 && peakHz >= fTarget[i]*0.75) || peakHz > 330){
      targetStringIndex = i;
    } 
  }
}


/*
Update the led-strip depending on the error: cent/10. There are 24 led-lights on the ring and center led is nr 12
*/
void stripColor(double cents) {
  strip.clear();
  int numLeds = (int)round(cents/10); 
  int center = 12;
  if (numLeds > 12) numLeds = 12;
  if (numLeds < -12) numLeds = -12;

  if (numLeds == 0) {
    strip.setPixelColor(center, strip.Color(0, 50, 0)); //center = correct tone 
  } 
  else if (numLeds > 0) {
    for (int i = 0; i <= numLeds; i++) {
      strip.setPixelColor(center - i, strip.Color(0, 0, 50)); //blue = to low tone 
    }
  } 
  else {
    for (int i = 0; i <= abs(numLeds); i++) {
      strip.setPixelColor(center + i, strip.Color(50, 0, 0)); //red = to high tone
    }
  }
  strip.show();
}


void displayInfo(double cents) {
  display.clearDisplay();     
  display.setCursor(0, 10);   // position for led-display
  display.print("Target:");
  display.println(tones[targetStringIndex]);
  display.println();
  display.print("Error:");
  display.print((int)cents);
  display.display();  // show the changes
}



void setup(){

  Serial.begin(115200);
  analogReference(DEFAULT);
  pinMode(MIC_PIN, INPUT);
  samplingPeriodUs = (unsigned int)(1000000.0 / Fs);
  Serial.println(F("UNO FFT tuner – DC-removal + HP + band mask"));

  //LED-screen 
  Wire.begin();
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  //LED ring
  strip.begin();
  strip.show();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
}


void loop(){

  int buttonState = digitalRead(BUTTON_PIN);

  //Calculate FFT
  sampleBlock();
  FFT.dcRemoval();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  //Calculate HPS
  HPS();
  double target = fTarget[targetStringIndex];

  //Set a banlimit around the targeted frequency when the button is pressed 
  if(buttonState == HIGH) bandLimitAroundTarget(target); 

  double peakHz = FFT.majorPeak(); //Most dominant peak
  double cents  = centsErr(peakHz, target); 
   
  if(buttonState==LOW) setTargetString(peakHz); //Button pressed -> call target string function

  //Show targetString and the error on the led-screen
  displayInfo(cents);
  
  //Show the led-strip
  stripColor(cents);

  //Print the values
  Serial.print("Peak: "); Serial.print(peakHz,2);
  Serial.print(" Hz   Δc: "); Serial.print(cents,1);
  Serial.print("   (target "); Serial.print(target,2); Serial.println(" Hz)");
  delay(25);
}
