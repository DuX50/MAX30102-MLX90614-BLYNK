//Blynk
#define BLYNK_TEMPLATE_ID"........"
#define BLYNK_TEMPLATE_NAME"Quickstart Template"
#define BLYNK_AUTH_TOKEN"........"

#include <Adafruit_GFX.h>    //OLED libraries
#include <Adafruit_SSD1306.h> //OLED libraries
#include <Wire.h>
#include <Adafruit_MLX90614.h>


#include "MAX30105.h"           //MAX3010x library
#include "heartRate.h"          //Heart rate calculating algorithm
#include "ESP32Servo.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>




#define FINGER_ON 7000    //Minimum amount of infrared ray (whether the finger is on or not)
#define MINIMUM_SPO2 90.0 

//OLED
#define SCREEN_WIDTH 128 //OLED WIDTH
#define SCREEN_HEIGHT 64 //OLED HEIGHT
#define OLED_RESET    -1 //Reset pin
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32


Adafruit_MLX90614 mlx = Adafruit_MLX90614();
MAX30105 particleSensor;

char ssid[] = "Beeeee";
char pass[] = "*12345678#";


//Variables to calculate heartbeat
const byte RATE_SIZE = 4; //
byte rates[RATE_SIZE]; //heartbeat array
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

//Variables to calculate SpO2
double avered = 0;
double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;

double SpO2 = 0;
double ESpO2 = 90.0;//initial value
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
int i = 0;
int Num = 30;//Calculate only once after sampling 30 times

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)

//MLX
double temp_obj;

//small heartbeat 
static const unsigned char PROGMEM logo2_bmp[] =
{ 0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,        
0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,
};
//big heartbeat
static const unsigned char PROGMEM logo3_bmp[] =
{ 0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00
};
//O2 icon
static const unsigned char PROGMEM O2_bmp[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x00, 0x3f, 0xc3, 0xf8, 0x00, 0xff, 0xf3, 0xfc,
0x03, 0xff, 0xff, 0xfe, 0x07, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0xfe, 0x0f, 0xff, 0xff, 0x7e,
0x1f, 0x80, 0xff, 0xfc, 0x1f, 0x00, 0x7f, 0xb8, 0x3e, 0x3e, 0x3f, 0xb0, 0x3e, 0x3f, 0x3f, 0xc0,
0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3f, 0x1f, 0xc0, 0x3e, 0x3e, 0x2f, 0xc0,
0x3e, 0x3f, 0x0f, 0x80, 0x1f, 0x1c, 0x2f, 0x80, 0x1f, 0x80, 0xcf, 0x80, 0x1f, 0xe3, 0x9f, 0x00,
0x0f, 0xff, 0x3f, 0x00, 0x07, 0xfe, 0xfe, 0x00, 0x0b, 0xfe, 0x0c, 0x00, 0x1d, 0xff, 0xf8, 0x00,
0x1e, 0xff, 0xe0, 0x00, 0x1f, 0xff, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00,
0x0f, 0xe0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

int Tonepin = 4; //LED blink on beat, can be a buzzer

void setup() {

Serial.begin(115200);
Serial.println("System Start");

//Blynk
Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);


//MLX
if (!mlx.begin()) {
Serial.println("Error connecting to MLX sensor. Check wiring.");
while (1);
};
Serial.println("Temperature Sensor MLX90614");

//MAX30201
if (!particleSensor.begin(Wire,I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
{
Serial.println("MAX30102 not found");
while (1);
}

byte ledBrightness = 0x7F;
byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR
int sampleRate = 800; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
int pulseWidth = 215; //Options: 69, 118, 215, 411
int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
// Set up
particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
particleSensor.enableDIETEMPRDY();
particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

//OLED
if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
Serial.println(F("SSD1306 allocation failed"));
for(;;); // Don't proceed, loop forever
}
display.display();delay(2000);
display.clearDisplay();

display.setCursor(30,15);
display.setTextSize(1);
display.setTextColor(WHITE);
display.println("Sensor");
display.setCursor(25,35);
display.setTextSize(1);
display.println("Initializing");
display.display();
Serial.println("OLED activated");


}

void loop() {
long irValue = particleSensor.getIR();    //Reading the IR value it will permit us to know if there's a finger on the sensor or not
//Did you put your fingers?
if (irValue > FINGER_ON ) {

//Check if there is a heartbeat, measure the heartbeat
if (checkForBeat(irValue) == true) {
  //heartbeat animation
  display.clearDisplay();//clear screen
  display.drawBitmap(0, 0, logo3_bmp, 16, 16, WHITE);//Show large heartbeat icon
  display.setTextSize(2);//Set text size
  display.setTextColor(WHITE);//text color
  display.setCursor(42, 10);//Set cursor position
  //display.print(beatAvg); display.println(" BPM");//Display heartbeat value
  display.drawBitmap(0, 35, O2_bmp, 16, 16, WHITE);//Show oxygen icon
  display.setCursor(42, 40);//Set cursor positio
  //Display blood oxygen value
  if (beatAvg > 30) display.print(String(ESpO2) + "%");
  else display.print("---- %" );
  display.display();//display screen
  tone(Tonepin, 1000);//Led or buzzer on
  delay(10);
  noTone(Tonepin);//Led off
  
  //Serial.print("Bpm="); Serial.println(beatAvg);//Display heartbeat to serial
  long delta = millis() - lastBeat;//Calculate heartbeat difference
  lastBeat = millis();
  beatsPerMinute = 60 / (delta / 1000.0);//Calculate average heartbeat
  if (beatsPerMinute < 255 && beatsPerMinute > 20) {
    //The heartbeat must be between 20-255
    rates[rateSpot++] = (byte)beatsPerMinute; //Array to store heartbeat values
    rateSpot %= RATE_SIZE;
    beatAvg = 0;//Calculate average
    for (byte x = 0 ; x < RATE_SIZE ; x++) beatAvg += rates[x];
    beatAvg /= RATE_SIZE;
  }
}

//Measure SpO2
uint32_t ir, red ;
double fred, fir;
particleSensor.check(); //Check the sensor, read up to 3 samples
if (particleSensor.available()) {
  i++;
  ir = particleSensor.getFIFOIR(); //Read infrared
  red = particleSensor.getFIFORed(); //Read red light
  //Serial.println("red=" + String(red) + ",IR=" + String(ir) + ",i=" + String(i));
  fir = (double)ir;//Convert double
  fred = (double)red;//Convert double
  aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
  avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
  sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level
  sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level

  if ((i % Num) == 0) {
    double R = (sqrt(sumirrms) / aveir) / (sqrt(sumredrms) / avered);
    SpO2 = -23.3 * (R - 0.4) + 100;
    ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
    if (ESpO2 <= MINIMUM_SPO2) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
    if (ESpO2 > 100) ESpO2 = 99.9;
    //Serial.print(",SPO2="); Serial.println(ESpO2);
    sumredrms = 0.0; sumirrms = 0.0; SpO2 = 0;
    i = 0;
  }
  particleSensor.nextSample(); //We're finished with this sample so move to next sample
}


//display data to serial
Serial.print("Bpm:" + String(beatAvg));
if (beatAvg > 30)  Serial.println(",SPO2:" + String(ESpO2));
else Serial.println(",SPO2:" + String(ESpO2));

//MLX
temp_obj = mlx.readObjectTempC();
//Temp Serial Monitor
Serial.print("Object temp = ");
Serial.println(temp_obj);

//Display data to OLED display not done temp display
display.clearDisplay();
display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE);
display.setTextSize(2);
display.setTextColor(WHITE);
display.setCursor(42, 10);
display.print(beatAvg); display.println(" BPM");
display.drawBitmap(0, 35, O2_bmp, 32, 32, WHITE);
display.setCursor(42, 40);
if (beatAvg > 30) display.print(String(ESpO2) + "%");
else display.print("---- %" );
display.display();

//Send data to Blynk
Blynk.virtualWrite(V0, beatAvg);
Blynk.virtualWrite(V1, String(ESpO2));
Blynk.virtualWrite(V2, temp_obj);

}
//No finger detected, clears all data and displays "Finger ??????"
else {
//Clear heartbeat data
for (byte rx = 0 ; rx < RATE_SIZE ; rx++) rates[rx] = 0;
beatAvg = 0; rateSpot = 0; lastBeat = 0;
//Clear blood oxygen data
avered = 0; aveir = 0; sumirrms = 0; sumredrms = 0;
SpO2 = 0; ESpO2 = 90.0;
//Display Finger ??????
display.clearDisplay();
display.setTextSize(2);
display.setTextColor(WHITE);
display.setCursor(30, 5);
display.println("Finger");
display.setCursor(30, 35);
display.println("??????");
display.display();
noTone(Tonepin);
}
}
