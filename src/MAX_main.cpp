/***************************************************
  This is a library for the Adafruit PT100/P1000 RTD Sensor w/MAX31865

  Designed specifically to work with the Adafruit RTD Sensor
  ----> https://www.adafruit.com/products/3328

  This sensor uses SPI to communicate, 4 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "../lib/MUX_Temperature/MUX_Temperature.h"
#include <Wire.h>
#include "Fan.h"
#include "../lib/controller_pins.h"
#include <Adafruit_NeoPixel.h>
#include "EEPROMstorage.h"
#include "StepperErrorHandler.h"
//// new: display
#include "image_data.h"
// #include <Fonts/InterSemiBold24pt7b.h>

#include <GxEPD2_BW.h>
GxEPD2_BW<GxEPD2_213_BN, GxEPD2_213_BN::HEIGHT> display(GxEPD2_213_BN(/*CS=D8*/ 2, /*DC=D3*/ 9, /*RST=D4*/ 8, /*BUSY=D2*/ 7)); // DEPG0213BN 122x250, SSD1680, TTGO T5 V2.4.1, V2.3.1
Adafruit_MAX31865 thermo = Adafruit_MAX31865(CS_PIN);
CD74HC4067 my_mux(S0_PIN, S1_PIN, S2_PIN, S3_PIN);
MUX_Temperature mux_temperature = MUX_Temperature(&thermo, &my_mux);
Fan fan_control0(FAN0_PIN);
Fan fan_control1(FAN1_PIN);
#if (STRIP_24V)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(6, STRIP_DPIN, NEO_WRGB + NEO_KHZ800);
#else
Adafruit_NeoPixel strip = Adafruit_NeoPixel(36, STRIP_DPIN, NEO_GRB + NEO_KHZ800); // NEO_WRGB
#endif
#if (RALPH_STEPPER)
StepperErrorHandler stepperError(STEPPER_PIN, STEPPER_THRESHOLD);
#endif

String SerialBuffer;
char CharBuffer[5];
void serialEvent();
void requestEvent();
void receiveEvent(int len);
String array2msg(double *temperatures);
String array2msg_int(int *temperatures);
double *temperatures;
String message;
String I2Cmsg;
double *flowArray;
String dataWire;
bool stepperErrorRequest = false;

bool checkSensors = false;
bool printSensors = false;

//variables to save the values
String warningValue = "water level";
String connectionValue = "connected";
String tempValue = "4";
String modeValue = "parameter";
String lightValue = "on";

EEPROMstorage eeprom_led;
int LED_r = 0;
int LED_g = 0;
int LED_b = 0;
int LED_w = 0;
int old_LED_r = 28;
int old_LED_g = 58;
int old_LED_b = 44;
int old_LED_w = 0;



void setColor(uint32_t c)
{
  for (uint16_t i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, c);
    strip.show();
  }
}

//functions

void init_base() {
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(2);
  // display.setFont(&Inter_SemiBold24pt7b);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_BLACK);
    display.drawInvertedBitmap(220, 0, divider_line, 76, 152, GxEPD_WHITE);
    display.drawInvertedBitmap(12, 12, tick, 30, 30, GxEPD_WHITE);
    display.drawInvertedBitmap(12, 110, tick, 30, 30, GxEPD_RED);
    display.drawInvertedBitmap(243, 23, tick, 30, 30, GxEPD_WHITE);
    display.drawInvertedBitmap(243, 99, tick, 30, 30, GxEPD_WHITE);
    display.drawInvertedBitmap(276, 23, arrow, 30, 30, GxEPD_WHITE);
    display.drawInvertedBitmap(276, 99, arrow, 30, 30, GxEPD_WHITE);
    display.drawInvertedBitmap(174, 54, c_symbol, 34, 34, GxEPD_WHITE);

    // display.drawCircle(166, 42, 8, GxEPD_WHITE);
    // display.setCursor(150, 56);
    // display.print("C");   
  }
  while(display.nextPage());
}

void base_with_parameters() {
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);
  display.setTextSize(2);
  // display.setFont(&Inter_SemiBold24pt7b);
  display.setFullWindow();
  display.firstPage();
  do
  {
  if (modeValue == "eco") {
    display.drawInvertedBitmap(243, 23, eco, 30, 30, GxEPD_WHITE);
  }
    }
  while(display.nextPage());
}
void mode(String m)
{
  if (m == "eco") {
    modeValue = "eco";
  }
  else if (m == "parameter") {
    modeValue = "parameter";
  }
  else if (m == "sleep") {
    modeValue = "sleep";
  }
  else if (m == "run") {
    modeValue = "run";
  }
  else if (m == "manual") {
    modeValue = "run";
  }
  
  // display.setPartialWindow(132, 23, 30, 30);
  // display.firstPage();
  // do{
  //   // base();
  //   // display.fillScreen(GxEPD_BLACK);
  //   display.fillRect(130, 23, 30, 30, GxEPD_WHITE);
  //   if (m == "eco") {
  //     Serial.println("eco");
  //     display.drawInvertedBitmap(130, 23, eco, 30, 30, GxEPD_WHITE);
  //   }
  //   else if (m == "parameter") {
  //     Serial.println("mode_parameter_run");
  //     display.drawInvertedBitmap(130, 23, parameter_run, 30, 30, GxEPD_WHITE);
  //   }
  //   else if (m == "sleep") {
  //     Serial.println("sleep");
  //     display.drawInvertedBitmap(130, 23, sleep, 30, 30, GxEPD_WHITE);
  //   }
  //   else if (m == "run") {
  //     Serial.println("run");
  //     // display.drawInvertedBitmap(130, 23, mode_run, 30, 30, GxEPD_WHITE);
  //   }
  //   else if (m == "manual") {
  //     Serial.println("manual");
  //     display.drawInvertedBitmap(130, 23, manual, 30, 30, GxEPD_WHITE);
  //   }
  //   delay(500);
  // } while(display.nextPage());
}

void warning(String m)
{
  display.setPartialWindow(8, 90, 30, 30);
  display.firstPage();
  do {
    display.fillScreen(GxEPD_BLACK);
    if (m == "environment") {
      Serial.println("warning_environment_temp");
      display.drawInvertedBitmap(8, 90, environment, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, environment, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, environment, 30, 30, GxEPD_WHITE);
      delay(3000);
    }
    else if (m == "general") {
      Serial.println("general");
      display.drawInvertedBitmap(8, 90, general, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, general, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, general, 30, 30, GxEPD_WHITE);
      delay(3000);
    }
    else if (m == "door") {
      Serial.println("door is open");
      display.drawInvertedBitmap(8, 90, door, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, door, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, door, 30, 30, GxEPD_WHITE);
      delay(3000);
    }
    else if (m == "pressure") {
      Serial.println("pressure");
      display.drawInvertedBitmap(8, 90, pressure, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, pressure, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, pressure, 30, 30, GxEPD_WHITE);
      delay(3000);
    }
    else if (m == "water level") {
      Serial.println("general");
      display.drawInvertedBitmap(8, 90, water_level, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, water_level, 30, 30, GxEPD_WHITE);
      delay(300);
      display.drawRect(8, 90, 30, 30, GxEPD_BLACK);
      delay(200);
      display.drawInvertedBitmap(8, 90, water_level, 30, 30, GxEPD_WHITE);
      delay(3000);
    }
    delay(50);
  } while(display.nextPage());
}
void wifi_connection(String m) {
  String connected_compare = "connected"; //creating a String "connected" to be able to compare it with the input data (input is a String)
  if (m == connected_compare) {
    display.fillRoundRect(127, 8, 55.5, 32.6, 10, GxEPD_WHITE); //draw a filled round rectangle
    display.drawBitmap(143, 12, connected, 25, 25, GxEPD_BLACK); //draw the wifi icon inside
  }
  else {
    display.fillRoundRect(127, 8, 55.5, 32.6, 10, GxEPD_BLACK); //draw a filled round rectangle
    display.drawRoundRect(127, 8, 55.5, 32.6, 10, GxEPD_WHITE); //draw a white border around the rectangle
    display.drawBitmap(143, 12, disconnected, 25, 25, GxEPD_WHITE); //draw the no wifi icon inside
  }
}
void connection(String m)
{
  // display.setPartialWindow(8, 8, 30, 30);
  display.firstPage();
  do{
  //   display.fillScreen(GxEPD_BLACK);
    if (m == "connected") {
      Serial.println("connected");
      display.drawInvertedBitmap(8, 8, connected, 30, 30, GxEPD_WHITE);
    }
    else {
      Serial.println("no connection");
      display.drawInvertedBitmap(8, 8, connected, 30, 30, GxEPD_WHITE);
      display.drawInvertedBitmap(8, 8, disconnected, 30, 30, GxEPD_WHITE);
    }
  //   delay(50);
  } while(display.nextPage()); 
}


void light(String m)
{
  display.setPartialWindow(130, 90, 30, 30); // x,y,width,height
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_BLACK);
    if (m == "on") {
      Serial.println("light on");
      display.drawBitmap(130, 90, light_on, 30, 30, GxEPD_WHITE);
    } else {
      Serial.println("light off");
      display.drawBitmap(130, 90, light_off, 30, 30, GxEPD_WHITE);
    }
    delay(50);
  }
  while (display.nextPage());
}

void temp(String m) {
  display.setPartialWindow(64, 21, 92, 110);
  display.firstPage();
  do{
    // display.setFont(&Inter_SemiBold24pt7b);
    // display.setRotation(1);
    display.fillScreen(GxEPD_BLACK);
    display.setCursor(64, 110);
    display.setTextColor(GxEPD_WHITE);
    display.setTextSize(1.5);
    display.print(m);
    delay(50);
  } while(display.nextPage());
}

struct ShowBoxParameters
{
  uint16_t x, y, w, h;
  uint16_t color;
};

struct ShowValueParameters
{
  uint16_t x, y, w, h, cursor_y;
  float value;
};
struct coordinates
{
  uint16_t x;
  uint16_t y;
};
const char HelloWorld[] = "Hello World!";
void helloWorldCallback(const void* p)
{
  const coordinates& where(*reinterpret_cast<const coordinates*>(p));
  display.setCursor(where.x, where.y);
  display.print(HelloWorld);
}

void helloWorld()
{
  //Serial.println("helloWorld");
  coordinates cursor;
  display.setRotation(1);
  // display.setFont(&Inter_SemiBold24pt7b);
  if (display.epd2.WIDTH < 104) display.setFont(0);
  display.setTextColor(GxEPD_BLACK);
  int16_t tbx, tby; uint16_t tbw, tbh;
  display.getTextBounds(HelloWorld, 0, 0, &tbx, &tby, &tbw, &tbh);
  // center bounding box by transposition of origin:
  cursor.x = ((display.width() - tbw) / 2) - tbx;
  cursor.y = ((display.height() - tbh) / 2) - tby;
  display.setFullWindow();
  display.drawPaged(helloWorldCallback, &cursor);
  //Serial.println("helloWorld done");
}

void showBoxCallback(const void* parameters)
{
  const ShowBoxParameters* p = reinterpret_cast<const ShowBoxParameters*>(parameters);
  display.fillRect(p->x, p->y, p->w, p->h, p->color);
}

void showValueBoxCallback(const void* parameters)
{
  const ShowValueParameters* p = reinterpret_cast<const ShowValueParameters*>(parameters);
  display.fillRect(p->x, p->y, p->w, p->h, GxEPD_WHITE);
  display.setCursor(p->x, p->cursor_y);
  display.print(p->value);
}


void showPartialUpdate()
{
  // some useful background
  helloWorld();
  // use asymmetric values for test
  ShowBoxParameters boxParameters{10, 15, 70, 20, GxEPD_WHITE};
  ShowValueParameters valueParameters{10, 15, 70, 20, 0, 13.95};
  valueParameters.cursor_y = valueParameters.y + valueParameters.h - 6;
  float value = 13.95;
  uint16_t incr = display.epd2.hasFastPartialUpdate ? 1 : 3;
  // display.setFont(&Inter_SemiBold24pt7b);
  display.setTextColor(GxEPD_BLACK);
  // show where the update box is
  for (uint16_t r = 0; r < 4; r++)
  {
    display.setRotation(r);
    display.setPartialWindow(boxParameters.x, boxParameters.y, boxParameters.w, boxParameters.h);
    boxParameters.color = GxEPD_BLACK;
    display.drawPaged(showBoxCallback, &boxParameters);
    delay(2000);
    boxParameters.color = GxEPD_WHITE;
    display.drawPaged(showBoxCallback, &boxParameters);
    while (display.nextPage());
    delay(1000);
  }
}



void setup()
{
  Serial.begin(250000);
  while (!Serial);
 
  Serial.println("Adafruit MAX31865 PT100 Sensor Test!");
  display.init(250000, true, 2, false);
  // display.setRotation(1);

  // display.fillScreen(GxEPD_BLACK);
  // display.display();
  // delay(500);
  // base();
  init_base();
  // wifi_connection("connected");
  // display.firstPage();
  // display.setPartialWindow(115, 50, 50, 40);
  // display.fillScreen(GxEPD_BLACK);
  // do{
    
  //   display.setCursor(120, 80);
  //   display.print(millis());
  //   display.drawBitmap(12, 12, disconnected, 30, 30, GxEPD_WHITE);
  // } while(display.nextPage());
  // showPartialUpdate();
  // Timer 0 and 1 for hardware PWM
  // Set the PWM frequency for hardware PWM pins (pins 3 and 6)
  // TCA0.SINGLE.CTRLA = 0b00000100; // Enable TCA0, single-slope PWM
  // TCA0.SINGLE.CTRLB = 0b00011000; // Set TCA0 to use 8-bit resolution

  // // Set the prescaler for the PWM frequency
  // TCA0.SINGLE.PER = 104; // Adjust this value to set the desired PWM frequency

  // // Start TCA0 timer
  // TCA0.SINGLE.CTRLA |= 0b00000001; // Start the timer
  // Fan:
  fan_control0.begin();
  fan_control1.begin();

  // I2C:
  Wire.begin(SLAVEI2C_ADRESS);
  Wire.onRequest(requestEvent); // Create an Interrup when Master request data
  Wire.onReceive(receiveEvent); // Create an Interrup when Master request data
  // mux temperature reading:
  mux_temperature.begin();

  // LED
  if (!STRIP_24V)
  {
    pinMode(STRIP_VCC, OUTPUT);
    digitalWrite(STRIP_VCC, HIGH);
  }
  strip.begin();

  LED_r = eeprom_led.read_r();
  LED_g = eeprom_led.read_g();
  LED_b = eeprom_led.read_b();
  LED_w = eeprom_led.read_w();

  if ((LED_r == 0 && LED_g == 0 && LED_b == 0 && LED_w == 0) || (LED_r == 255 && LED_g == 255 && LED_b == 255 && LED_w == 255))
  {
    eeprom_led.store(old_LED_r, old_LED_g, old_LED_b, old_LED_w);
    setColor(strip.Color(old_LED_r, old_LED_g, old_LED_b, old_LED_w));
  }
  else
  {
    setColor(strip.Color(LED_r, LED_g, LED_b, LED_w));
  }
#if (RALPH_STEPPER)
  stepperError.begin();
#endif

}
unsigned long oldOut;
bool test = false;

void loop()
{ 
  
  // display.firstPage();
  // display.setPartialWindow(115, 50, 50, 40);
  // do{
    // display.fillScreen(GxEPD_BLACK);
    // display.setCursor(120, 80);
    // display.print(millis());
    // display.drawBitmap(115, 50, warning_door, 30, 30, GxEPD_WHITE);
    
  Serial.println(display.getCursorX());
  


  mux_temperature.async_ReadAllRTD();
  temperatures = mux_temperature.async_ReadAllTemperatures();
  I2Cmsg = array2msg(temperatures);

#if (RALPH_STEPPER)
  stepperError.readSensor();
#endif

  // Print every second
  if (millis() >= oldOut + 1000L)
  {
    for (size_t i = 0; i < 16; i++)
    {
      Serial.print((String)temperatures[i] + " ");
    }
    Serial.println();
    oldOut = millis();
  }

  if (checkSensors)
  {
    mux_temperature.checkAvailableSensors();
    checkSensors = false;
  }
  // if (printSensors)
  // {
  //   mux_temperature.printAvailableSensors();
  //   // mux_temperature.printI2CAvailableSensors();
  //   I2Cmsg = array2msg_int(mux_temperature.availableSensors, "A");
  //   Wire.write(I2Cmsg.c_str()); // Send ASII String
  //   printSensors = false;
  // }

  if (Serial.available() > 0)
  {
    serialEvent();
  }
}

void receiveEvent(int len)
{
  while (Wire.available() > 0)
  {
    dataWire += (char)Wire.read();
  }
  Serial.println(dataWire);
  float Input = 0;
  char decision = dataWire[0];
  dataWire.remove(0, 1);
  Input = dataWire.toFloat();

  switch (decision)
  {
  case 'A':
    Serial.println("Update TemperatureSensors.");
    checkSensors = true;
    break;
    
  case 'R':
    if (Input > 0)
    {
      fan_control0.setPWM(Input);
      fan_control1.setPWM(Input);
    }
    else
    {
      fan_control0.reset();
      fan_control1.reset();
    }
    break;

    
#if (RALPH_STEPPER)
  case 'S':
    stepperErrorRequest = true;
    break;
#endif
  case 'r':
    if (Input == -1)
    {
      eeprom_led.store(old_LED_r, old_LED_g, old_LED_b, old_LED_w);
      setColor(strip.Color(old_LED_r, old_LED_g, old_LED_b, old_LED_w));
    }
    else
    {
      LED_r = (int)Input;
      eeprom_led.store(LED_r, LED_g, LED_b, LED_w);
      setColor(strip.Color(LED_r, LED_g, LED_b, LED_w));
    }
    break;
  case 'g':
    LED_g = (int)Input;
    eeprom_led.store(LED_r, LED_g, LED_b, LED_w);
    setColor(strip.Color(LED_r, LED_g, LED_b, LED_w));
    break;
  case 'b':
    LED_b = (int)Input;
    eeprom_led.store(LED_r, LED_g, LED_b, LED_w);
    setColor(strip.Color(LED_r, LED_g, LED_b, LED_w));
    break;
  case 'w':
    LED_w = (int)Input;
    eeprom_led.store(LED_r, LED_g, LED_b, LED_w);
    setColor(strip.Color(LED_r, LED_g, LED_b, LED_w));
    break;
  default:
    Serial.println("I2C command not recognized.");
    break;
  }
  dataWire = "";
}

void requestEvent()
{
#if (RALPH_STEPPER)
  if (stepperErrorRequest == true)
  {
    Wire.println("S" + (String)stepperError.checkStepperError());
    stepperErrorRequest = false;
  }
  else
    Wire.write(I2Cmsg.c_str()); // Send ASII String
#else
  Wire.write(I2Cmsg.c_str()); // Send ASII String
#endif
}

String array2msg(double *array)
{
  message = "/*";
  for (size_t i = 0; i < 16; i++)
  {
    message.concat(array[i]);
    message.concat(",");
  }
  message.concat("*/");
  return message;
}

String array2msg_int(int *array)
{
  message = "/*";
  for (size_t i = 0; i < 16; i++)
  {
    message += String(array[i]);
    message += ",";
  }
  message += "*/";
  return message;
}

void serialEvent()
{

  float Input = 0;
  SerialBuffer = Serial.readString();
  SerialBuffer.toCharArray(CharBuffer, 5);
  char decision = CharBuffer[0];
  SerialBuffer.remove(0, 1);
  Input = SerialBuffer.toFloat();
  SerialBuffer.remove(0, 2);
  int lastIndex = SerialBuffer.length();
  SerialBuffer.remove(lastIndex);
  SerialBuffer.trim();
  Serial.println(SerialBuffer);
  // display.setRotation(1);
  // display.fillScreen(GxEPD_BLACK);
  switch (decision)

  {
  case 'A':
    Serial.println("Update TemperatureSensors.");
    checkSensors = true;
    break;
  case 'W':
    Serial.println("Command Warning.");
    Serial.println(SerialBuffer);
    warning(SerialBuffer);
    break;
  case 'M':
    Serial.println("message is");
    Serial.println(SerialBuffer);
    mode(SerialBuffer);
    base_with_parameters();
    break;
  case 'C':
    Serial.println("message is");
    Serial.println(SerialBuffer);
    // wifi_connection(SerialBuffer);
    // connection(SerialBuffer);
    //code with drawpaged
    display.setPartialWindow(12, 12, 30, 30);
    ShowBoxParameters boxParameters{12, 12, 30, 30, GxEPD_WHITE};
    ShowValueParameters valueParameters{12, 12, 30, 30, 0, 13.95};
    // display.fillScreen(GxEPD_BLACK);
    boxParameters.color = GxEPD_BLACK;
    display.drawPaged(showBoxCallback, &boxParameters);
    delay(2000);
    boxParameters.color = GxEPD_WHITE;
    display.drawPaged(showBoxCallback, &boxParameters);
    // while (display.nextPage());
    delay(1000);
    // display.setPartialWindow(12, 12, 30, 30);
    // // display.firstPage();
    // // do{
    //   display.fillScreen(GxEPD_BLACK);
    //   if (SerialBuffer == "connected") {
    //     Serial.println("connected");
    //     display.drawInvertedBitmap(12, 12, connected, 30, 30, GxEPD_WHITE);
    //   }
    //   else {
    //     Serial.println("no connection");
    //     display.drawInvertedBitmap(12, 12, connected, 30, 30, GxEPD_WHITE);
    //     display.drawInvertedBitmap(12, 12, disconnected, 30, 30, GxEPD_WHITE);
    //   }
    //   delay(50);
    // // } while(display.nextPage()); 
    break;
  case 'E':
    Serial.println("message is");
    Serial.println(SerialBuffer);
    temp(SerialBuffer);
    break;
  case 'L':
    Serial.println("message is");
    Serial.println(SerialBuffer);
    light(SerialBuffer);
    break;
  case 'T':
    for (size_t i = 0; i < 16; i++)
    {
      Serial.print((String)temperatures[i] + " ");
    }
    Serial.println();
    break;
  case 'R':
    Serial.println("Set Radiator with Speed: " + (String)Input);
    if (Input > 0)
    {
      fan_control0.setPWM(Input);
      fan_control1.setPWM(Input);
    }
    else
    {
      fan_control0.reset();
      fan_control1.reset();
    }
    break;

  case 'r':
    Serial.println("Set LED r: " + (String)Input);
    if (Input == -1)
    {
      eeprom_led.store(old_LED_r, old_LED_g, old_LED_b, old_LED_w);
      setColor(strip.Color(old_LED_r, old_LED_g, old_LED_b, old_LED_w));
    }
    else
    {
      LED_r = (int)Input;
      eeprom_led.store(LED_r, LED_g, LED_b, LED_w);
      setColor(strip.Color(LED_r, LED_g, LED_b, LED_w));
    }
    break;
  case 'g':
    Serial.println("Set LED g: " + (String)Input);
    LED_g = (int)Input;
    eeprom_led.store(LED_r, LED_g, LED_b, LED_w);
    setColor(strip.Color(LED_r, LED_g, LED_b, LED_w));
    break;
  case 'b':
    Serial.println("Set LED b: " + (String)Input);
    LED_b = (int)Input;
    eeprom_led.store(LED_r, LED_g, LED_b, LED_w);
    setColor(strip.Color(LED_r, LED_g, LED_b, LED_w));
    break;

  case 'w':
    Serial.println("Set LED w: " + (String)Input);
    LED_w = (int)Input;
    eeprom_led.store(LED_r, LED_g, LED_b, LED_w);
    setColor(strip.Color(LED_r, LED_g, LED_b, LED_w));
    break;
  default:
    Serial.println("unknown Command");
    break;
  }
}
