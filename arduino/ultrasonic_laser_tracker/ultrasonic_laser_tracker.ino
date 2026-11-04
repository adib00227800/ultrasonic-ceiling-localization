#include <Wire.h>
#include <Adafruit_VL53L0X.h>   // اضافه برای VL53L0X

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// ===== Ultrasonic config =====
const int N = 5; // 1 + 4
const int trigPins[N] = {22,24,26,28,32};
const int echoPins[N] = {23, 25,27,29,33};

const float maxDistanceM = 4.0;
const float speedOfSound = 343.0;

unsigned long timeoutUs;


static inline float usToCm(unsigned long durUs) {
  if (durUs == 0) return -1.0f;
  return (durUs * 0.0343f);
}

static inline void triggerPair(int tx, int rx) {
  digitalWrite(trigPins[tx], LOW);
  digitalWrite(trigPins[rx], LOW);
  delayMicroseconds(2);

  digitalWrite(trigPins[tx], HIGH);
  digitalWrite(trigPins[rx], HIGH);
  delayMicroseconds(10);

  digitalWrite(trigPins[tx], LOW);
  digitalWrite(trigPins[rx], LOW);
}

static inline float measureSingle(int i) {
  for (int k = 0; k < N; k++) digitalWrite(trigPins[k], LOW);

  delayMicroseconds(2);
  digitalWrite(trigPins[i], HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPins[i], LOW);

  unsigned long durUs = pulseIn(echoPins[i], HIGH, timeoutUs);
  return usToCm(durUs);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) {}

 

  // VL53L0X init
  Serial.println("Initializing VL53L0X...");
  Wire.begin();                // already called but safe
  if (!lox.begin()) {
    Serial.println("VL53L0X not found! Continuing without laser.");
    // می‌توانی قبل از ادامه loop یک fallback قرار دهی
  } else {
    Serial.println("VL53L0X initialized.");
  }

  // Ultrasonic pins
  for (int i = 0; i < N; i++) {
    pinMode(trigPins[i], OUTPUT);
    digitalWrite(trigPins[i], LOW);
    pinMode(echoPins[i], INPUT);
  }
  timeoutUs = (unsigned long)((2.0 * maxDistanceM) / speedOfSound * 1e6);
  if (timeoutUs < 30000UL) timeoutUs = 30000UL;
}

void loop() {
  // --- 1) فاصله سنسور 1 خودش ---
  float d1 = measureSingle(0);
  if (d1 < 0) Serial.print(-1);
  else       Serial.print(d1, 2);

  // --- 2) فاصله جفت‌ها ---
  for (int k = 1; k < N; k++) {
    triggerPair(0, k);
    delayMicroseconds(100);
    unsigned long usk = pulseIn(echoPins[k], HIGH, timeoutUs);
    float dk = usToCm(usk);

    Serial.print(',');
    if (dk < 0) Serial.print(-1);
    else        Serial.print(dk, 2);

    delay(50);
  }

  // --- 3) خواندن ارتفاع از VL53L0X (لیزر) ---
  float laserHeight_cm = -1.0f;
  if (lox.begin()) { // اگر init موفق بود
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // blocking
    if (measure.RangeStatus != 4) { // 4 = out of range per Adafruit lib
      // measure.RangeMilliMeter -> میلی‌متر
      laserHeight_cm = measure.RangeMilliMeter / 10.0-2.9; // تبدیل به سانتی‌متر
    } else {
      laserHeight_cm = -1.0f;
    }
  }

  // اضافه کردن شتاب‌ها و ارتفاع لیزری به همان خط
  Serial.print(',');
  // چاپ ارتفاع لیزری (به سانتی‌متر)
  if (laserHeight_cm < 0) Serial.print(-1);
  else                   Serial.print(laserHeight_cm, 2);

  Serial.println();
  delay(10);
}
