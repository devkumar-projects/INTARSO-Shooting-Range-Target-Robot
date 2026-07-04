#include <FastLED.h>


#define NUM_LEDS 1
#define DATA_PIN 23  
#define CLOCK_PIN 19 

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(115200);
  Serial.println("--- P9813 LED UNIT TEST START ---");
 
  FastLED.addLeds<P9813, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
 
  FastLED.clear();
  FastLED.show();
  delay(1000);
}

void loop() {
 
  Serial.println("Red...");
  leds[0] = CRGB::Red;
  FastLED.show();
  delay(1000);

 
  Serial.println("Green...");
  leds[0] = CRGB::Green;
  FastLED.show();
  delay(1000);

 
  Serial.println("Blue...");
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(1000);

  
  Serial.println("White (full power)...");
  leds[0] = CRGB::White;
  FastLED.show();
  delay(1000);

  
  Serial.println("Off...");
  leds[0] = CRGB::Black;
  FastLED.show();
  delay(2000);
}
