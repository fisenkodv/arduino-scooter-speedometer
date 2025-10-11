#include <SPI.h>
#include <U8g2lib.h>
#include <math.h>
#include <Adafruit_NeoPixel.h>

#ifdef __AVR__
#include <avr/power.h>  // Required for 16 MHz Adafruit Trinket
#endif

// Display setup - adjust pins as needed
#define OLED_CS 10
#define OLED_DC 7
#define OLED_RESET 8

U8G2_SSD1327_MIDAS_128X128_1_4W_HW_SPI u8g2(U8G2_R0, OLED_CS, OLED_DC, OLED_RESET);

// NeoPixels
#define LED_SWITCH_PIN 12
#define LED_DATA_PIN 5
#define LED_COUNT 60  // How many NeoPixels are attached to the Arduino?

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

// Speedometer parameters
#define SPEEDO_CENTER_X 64
#define SPEEDO_CENTER_Y 70
#define SPEEDO_RADIUS 60
#define MAX_SPEED 16

// Hall sensor setup
#define HALL_SENSOR_PIN 2  // Use interrupt pin (pin 2 or 3 on most Arduinos)

// Speed calculation parameters
#define WHEEL_CIRCUMFERENCE_MM 798.0  // Adjust for your scooter wheel (10" wheel ≈ 798mm circumference)
#define MAGNETS_PER_REVOLUTION 2      // Number of magnets on the wheel
#define MIN_PULSE_INTERVAL 50         // Minimum ms between pulses to avoid bouncing

// Speed measurement variables
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;
volatile bool newPulseReceived = false;

float currentSpeedMph = 0.0;
unsigned long lastSpeedUpdate = 0;
unsigned long speedTimeout = 3000;  // Reset speed to 0 if no pulse for 3 seconds

bool currentLightsSwitchState = false;

void setup() {
  Serial.begin(9600);

  pinMode(LED_SWITCH_PIN, INPUT_PULLUP);
  currentLightsSwitchState = isLightsSwitchOn();

  // Setup hall sensor interrupt
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);

  u8g2.begin();

#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  strip.begin();             // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();              // Turn OFF all pixels ASAP
  strip.setBrightness(128);  // Set BRIGHTNESS to about 50% (max = 255)

  Serial.println("Scooter Speedometer Ready");
}

void loop() {
  // Update speed calculation
  updateSpeed();

  bool mostRecentLightsSwitchState = isLightsSwitchOn();

  if (mostRecentLightsSwitchState != currentLightsSwitchState) {
    clearStrip();
    clearDisplay();
  }

  currentLightsSwitchState = mostRecentLightsSwitchState;

  if (currentLightsSwitchState) {
    runningLights(currentSpeedMph, 10, 2);
    delay(30);
  } else {
    drawClassicSpeedometer((int)currentSpeedMph);
  }

  // delay(50);  // Update display ~20 times per second
  // colorWipe(valueToColor(currentSpeedMph), 10);  // Red
  // colorWipe(strip.Color(  0, 255,   0), 50); // Green
  // colorWipe(strip.Color(  0,   0, 255), 50); // Blue
  // strip.clear();               //   Set all pixels in RAM to 0 (off)
  // theaterChase(strip.Color(255, 0, 0), 30);

  // currentSpeedMph++;
  // if (currentSpeedMph > 16) currentSpeedMph = 1;
}

/**
 * Update speed calculation based on hall sensor pulses
 */
void updateSpeed() {
  // Check if we received a new pulse
  if (newPulseReceived) {
    newPulseReceived = false;
    lastSpeedUpdate = millis();

    // Calculate speed
    // Speed = (wheel circumference / magnets per rev) / time interval
    // Convert from inches per millisecond to miles per hour
    if (pulseInterval > 0) {
      float distancePerPulseMm = WHEEL_CIRCUMFERENCE_MM / MAGNETS_PER_REVOLUTION;
      float mmPerMs = distancePerPulseMm / pulseInterval;
      float mmPerHour = mmPerMs * 3600000.0;    // ms to hours conversion (1000ms * 3600s)
      currentSpeedMph = mmPerHour / 1609344.0;  // mm to miles conversion (1 mile = 1,609,344 mm)

      // Apply smoothing filter to reduce jitter
      static float filteredSpeed = 0;
      filteredSpeed = (filteredSpeed * 0.7) + (currentSpeedMph * 0.3);
      currentSpeedMph = filteredSpeed;

      // Limit maximum displayed speed for safety
      if (currentSpeedMph > MAX_SPEED) {
        currentSpeedMph = MAX_SPEED;
      }
    }
  }

  // Reset speed to 0 if no pulse received for timeout period
  if (millis() - lastSpeedUpdate > speedTimeout) {
    currentSpeedMph = 0.0;
  }
}

bool isLightsSwitchOn() {
  return digitalRead(LED_SWITCH_PIN) == LOW;
}

void clearDisplay() {
  u8g2.firstPage();
  do {
    // do nothing
  } while (u8g2.nextPage());
}

void clearStrip() {
  strip.clear();
  strip.show();
}

uint32_t valueToColor(float value) {
  const float deltaVal = 2.0f;
  const float minVal = 0 + deltaVal;
  const float maxVal = MAX_SPEED - deltaVal;

  // Clamp
  if (value <= minVal) {
    return strip.Color(0, 0, 255);  // Blue
  }
  if (value >= maxVal) {
    return strip.Color(255, 0, 0);  // Red
  }

  // Normalize [0,1]
  float t = (value - minVal) / (maxVal - minVal);

  // Hue interpolation (240°=blue → 0°=red)
  float hue = (1.0f - t) * 240.0f;
  float saturation = 1.0f;
  float v = 1.0f;

  float c = v * saturation;
  float x = c * (1 - fabs(fmod(hue / 60.0f, 2) - 1));
  float m = v - c;

  float r1, g1, b1;
  if (hue < 60) {
    r1 = c;
    g1 = x;
    b1 = 0;
  } else if (hue < 120) {
    r1 = x;
    g1 = c;
    b1 = 0;
  } else if (hue < 180) {
    r1 = 0;
    g1 = c;
    b1 = x;
  } else if (hue < 240) {
    r1 = 0;
    g1 = x;
    b1 = c;
  } else {
    r1 = c;
    g1 = 0;
    b1 = x;
  }

  int r = (int)((r1 + m) * 255);
  int g = (int)((g1 + m) * 255);
  int b = (int)((b1 + m) * 255);

  return strip.Color(r, g, b);
}

void runningLights(float value, int tailLength, int speed) {
  static int head = 0;
  uint32_t baseColor = valueToColor(value);

  uint8_t rBase = (baseColor >> 16) & 0xFF;
  uint8_t gBase = (baseColor >> 8) & 0xFF;
  uint8_t bBase = (baseColor)&0xFF;

  strip.clear();

  for (int i = 0; i < tailLength; i++) {
    int pos = (head - i + 60) % 60;

    float fade = 1.0f - (float)i / tailLength;
    uint8_t r = (uint8_t)(rBase * fade);
    uint8_t g = (uint8_t)(gBase * fade);
    uint8_t b = (uint8_t)(bBase * fade);

    strip.setPixelColor(pos, strip.Color(r, g, b));
  }

  strip.show();

  // Move the comet by more than 1 pixel per frame
  head = (head + speed) % 60;
}


/**
 * Interrupt Service Routine for hall sensor
 */
void hallSensorISR() {
  unsigned long currentTime = millis();

  // Debounce - ignore pulses that are too close together
  if (currentTime - lastPulseTime > MIN_PULSE_INTERVAL) {
    pulseInterval = currentTime - lastPulseTime;
    lastPulseTime = currentTime;
    newPulseReceived = true;
  }
}

/**
 * Main function to draw the complete speedometer
 */
void drawClassicSpeedometer(int speed) {
  u8g2.firstPage();

  do {
    // Draw the outer circle/arc
    drawSpeedometerArc();

    // Draw tick marks and numbers
    drawSpeedometerTicks();

    // // Draw the needle pointing to current speed
    drawSpeedometerNeedle(speed);

    // // Draw center dot
    u8g2.drawDisc(SPEEDO_CENTER_X, SPEEDO_CENTER_Y, 3);

    // // Draw digital readout
    drawSpeedNumber(speed);
  } while (u8g2.nextPage());
}

/**
 * Draw the outer arc of the speedometer
 * Classic design uses roughly 240 degrees (from 7:30 to 4:30 on a clock)
 */
void drawSpeedometerArc() {
  // Draw main arc - we'll draw it as segments
  for (int angle = 150; angle <= 390; angle += 5) {
    float radian = angle * PI / 180.0;
    int x = SPEEDO_CENTER_X + cos(radian) * SPEEDO_RADIUS;
    int y = SPEEDO_CENTER_Y + sin(radian) * SPEEDO_RADIUS;
    u8g2.drawPixel(x, y);

    // Make it thicker
    u8g2.drawPixel(x + 1, y);
    u8g2.drawPixel(x, y + 1);
  }
}

/**
 * Draw tick marks and speed numbers
 */
void drawSpeedometerTicks() {
  u8g2.setFont(u8g2_font_6x10_tr);

  for (int speed = 0; speed <= MAX_SPEED; speed += 2) {
    // Calculate angle for this speed (150° to 390° span)
    float angle = 150 + (speed * 240.0 / MAX_SPEED);
    float radian = angle * PI / 180.0;

    // Determine tick length (longer for major marks)
    int tickLength = (speed % 4 == 0) ? 8 : 4;

    // Outer point of tick
    int x1 = SPEEDO_CENTER_X + cos(radian) * SPEEDO_RADIUS;
    int y1 = SPEEDO_CENTER_Y + sin(radian) * SPEEDO_RADIUS;

    // Inner point of tick
    int x2 = SPEEDO_CENTER_X + cos(radian) * (SPEEDO_RADIUS - tickLength);
    int y2 = SPEEDO_CENTER_Y + sin(radian) * (SPEEDO_RADIUS - tickLength);

    // Draw the tick mark
    u8g2.drawLine(x1, y1, x2, y2);

    // Draw numbers for major ticks (every 2)
    if (speed % 2 == 0) {
      int textX = SPEEDO_CENTER_X + cos(radian) * (SPEEDO_RADIUS - 16);
      int textY = SPEEDO_CENTER_Y + sin(radian) * (SPEEDO_RADIUS - 16);

      char speedText[4];
      sprintf(speedText, "%d", speed);

      // Center the text roughly
      textX -= strlen(speedText) * 3;
      textY += 3;

      u8g2.drawStr(textX, textY, speedText);
    }
  }
}

/**
 * Draw the speedometer needle
 */
void drawSpeedometerNeedle(int speed) {
  // Calculate needle angle
  float angle = 150 + (speed * 240.0 / MAX_SPEED);
  float radian = angle * PI / 180.0;

  // Needle tip coordinates
  int tipX = SPEEDO_CENTER_X + cos(radian) * (SPEEDO_RADIUS - 5);
  int tipY = SPEEDO_CENTER_Y + sin(radian) * (SPEEDO_RADIUS - 5);

  // Draw main needle line
  u8g2.drawLine(SPEEDO_CENTER_X, SPEEDO_CENTER_Y, tipX, tipY);

  // Draw needle with some thickness
  u8g2.drawLine(SPEEDO_CENTER_X + 1, SPEEDO_CENTER_Y, tipX + 1, tipY);
  u8g2.drawLine(SPEEDO_CENTER_X, SPEEDO_CENTER_Y + 1, tipX, tipY + 1);

  // Optional: draw a small tail on the opposite side
  int tailX = SPEEDO_CENTER_X - cos(radian) * 8;
  int tailY = SPEEDO_CENTER_Y - sin(radian) * 8;
  u8g2.drawLine(SPEEDO_CENTER_X, SPEEDO_CENTER_Y, tailX, tailY);
}

/**
 * Draw digital speed display
 */
void drawSpeedNumber(int speed) {
  u8g2.setFont(u8g2_font_logisoso16_tr);

  char speedText[4];
  sprintf(speedText, "%d", speed);

  // Position at bottom center
  int textWidth = u8g2.getStrWidth(speedText);
  int x = SPEEDO_CENTER_X - textWidth / 2;
  int y = 115;

  // Draw background box
  u8g2.drawRFrame(x - 3, y - 20, textWidth + 6, 24, 2);

  // Draw the speed number
  u8g2.drawStr(x, y, speedText);

  // Draw "MPH" label
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.drawStr(SPEEDO_CENTER_X - 8, 128, "MPH");
}

void colorWipe(uint32_t color, int wait) {
  for (int i = 0; i < strip.numPixels(); i++) {  // For each pixel in strip...
    strip.setPixelColor(i, color);               //  Set pixel's color (in RAM)
    strip.show();                                //  Update strip to match
    delay(wait);                                 //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for (int a = 0; a < 10; a++) {   // Repeat 10 times...
    for (int b = 0; b < 3; b++) {  //  'b' counts from 0 to 2...
      strip.clear();               //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for (int c = b; c < strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color);  // Set pixel 'c' to value 'color'
      }
      strip.show();  // Update strip with new contents
      delay(wait);   // Pause for a moment
    }
  }
}
