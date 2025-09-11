#include <EEPROM.h>

const int VIBRATION_PIN = 2;
const int SOUND_SENSOR_PIN = 8;
const int MOTION_PIN = 6;
const int STATUS_LED_PIN = 13;

const int ULTRASONIC_TRIG_PIN = 30;
const int ULTRASONIC_ECHO_PIN = 31;

const int IR_VCC_PIN = 42; 
const int IR_SIGNAL_PIN = 43;
const int IR_GND_PIN = 44;

const unsigned long VIBRATION_DEBOUNCE = 10;  
const unsigned long MOTION_DEBOUNCE = 50;      
const unsigned long SOUND_DEBOUNCE = 30;      
const unsigned long CO2_READ_INTERVAL = 2000;
const unsigned long ULTRASONIC_READ_INTERVAL = 500;
const unsigned long IR_READ_INTERVAL = 200;      
const unsigned long STATS_DISPLAY_INTERVAL = 30000;
const unsigned long CALIBRATION_TIME = 50000; // 50 secs calibrating 

// ===== ULTRASONIC CONSTANTS =====
const float SOUND_SPEED = 0.0343; 
const int MAX_DISTANCE = 1000;
const int MIN_DISTANCE = 1;
const int PROXIMITY_THRESHOLD = 50;

const unsigned char CO2_CMD_READ[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
const unsigned char CO2_CMD_CALIBRATE[] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};
const int CO2_RESPONSE_LENGTH = 9;
const int CO2_NORMAL_LEVEL = 400;
const int CO2_ELEVATED_LEVEL = 800;
const int CO2_HIGH_LEVEL = 1000;
const int CO2_CRITICAL_LEVEL = 2000;
const int CO2_MIN_VALID = 300;  
const int CO2_MAX_VALID = 5000;  

// ===== INFRARED SENSOR CONSTANTS =====
const int IR_HUMAN_THRESHOLD = 100;  
const int IR_LIFE_THRESHOLD = 50; 
const int IR_BASELINE_SAMPLES = 50; 
const int IR_DETECTION_SAMPLES = 5;  

// ===== SOUND SENSITIVITY CONSTANTS =====
const int SOUND_THRESHOLD = 300;    // LOWER SINSI SOUND // 2000 orgi
const int SOUND_MIN_DURATION = 20;      

const int VIBRATION_CONSECUTIVE_HITS = 2; // SENSI VIB

// ===== EEPROM ADDRESSES =====
const int EEPROM_INIT_FLAG = 0;
const int EEPROM_VIBRATION_COUNT = 4;
const int EEPROM_SOUND_COUNT = 8;
const int EEPROM_MOTION_COUNT = 12;
const int EEPROM_PROXIMITY_COUNT = 16;
const int EEPROM_LIFE_COUNT = 20;

struct SensorState {
  volatile bool ralphVibrationFlag = false;
  volatile int ralphVibrationHits = 0;
  unsigned long ralphLastVibTime = 0;
  
  int ralphSoundState = LOW;
  unsigned long ralphLastSoundTime = 0;
  unsigned long ralphSoundStart = 0;
  
  int ralphMotionState = LOW;
  int ralphLastMotionRead = LOW;
  unsigned long ralphLastMotionTime = 0;
  unsigned long ralphMotionStart = 0;
  unsigned long ralphTotalMotionTime = 0;
  
  unsigned long ralphLastCO2Time = 0;
  unsigned long ralphLastUltrasonicTime = 0;
  unsigned long ralphLastIRTime = 0;
  unsigned long ralphLastStatsTime = 0;
  
  bool ralphCalibrating = false;
  unsigned long ralphCalibStart = 0;
 
  float ralphLastDistance = 0.0;
  bool ralphProximityDetected = false;
  unsigned long ralphLastProximityTime = 0;
  
  int ralphIRBaseline = 0;
  bool ralphIRBaselineSet = false;
  int ralphIRDetectionCount = 0;
  bool ralphLifeDetected = false;
  unsigned long ralphLastLifeTime = 0;
  
  int ralphCO2Buffer[5] = {0, 0, 0, 0, 0};
  int ralphCO2BufferIndex = 0;
  bool ralphCO2BufferFull = false;
};

struct SensorData {
  unsigned long ralphVibCount = 0;
  unsigned long ralphSoundCount = 0;
  unsigned long ralphMotionCount = 0;
  unsigned long ralphProximityCount = 0;
  unsigned long ralphLifeCount = 0;
  
  int ralphCurrentCO2 = 0;
  int ralphMinCO2 = 9999;
  int ralphMaxCO2 = 0;
  unsigned long ralphTotalCO2Readings = 0;
  unsigned long ralphSumCO2 = 0;

  float ralphCurrentDistance = 0.0;
  float ralphMinDistance = MAX_DISTANCE;
  float ralphMaxDistance = 0.0;
  
  // Infrared data
  int ralphCurrentIRValue = 0;
  int ralphMaxIRValue = 0;
  int ralphMinIRValue = 1023;
  
  float ralphAvgCO2 = 0.0;
  unsigned long ralphUptime = 0;
};

SensorState ralphieState;
SensorData ralphieData;
unsigned char ralphCO2Response[CO2_RESPONSE_LENGTH];

void setup() {
  Serial.begin(9600); // BAUFU
  Serial1.begin(9600);
  
  pinMode(VIBRATION_PIN, INPUT_PULLUP);
  pinMode(SOUND_SENSOR_PIN, INPUT);
  pinMode(MOTION_PIN, INPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  // Ralph's IR sensor setup - importante to
  pinMode(IR_VCC_PIN, OUTPUT);
  pinMode(IR_GND_PIN, OUTPUT);
  pinMode(IR_SIGNAL_PIN, INPUT);
  digitalWrite(IR_VCC_PIN, HIGH);  
  digitalWrite(IR_GND_PIN, LOW);   
  
  attachInterrupt(digitalPinToInterrupt(VIBRATION_PIN), ralphVibrationISR, CHANGE);
  
  ralphLoadDataFromEEPROM();

  ralphDisplayStartupMessage();
  
  ralphInitializeIRBaseline();
  
  ralphStartCalibration();
}

void loop() {
  unsigned long ralphCurrentTime = millis();
  ralphieData.ralphUptime = ralphCurrentTime;

  ralphHandleVibrationSensor(ralphCurrentTime);
  ralphHandleSoundSensor(ralphCurrentTime);
  ralphHandleMotionSensor(ralphCurrentTime);
  
  // Read CO2 - importante to
  if (ralphCurrentTime - ralphieState.ralphLastCO2Time >= CO2_READ_INTERVAL) {
    ralphHandleCO2Sensor();
    ralphieState.ralphLastCO2Time = ralphCurrentTime;
  }

  if (ralphCurrentTime - ralphieState.ralphLastUltrasonicTime >= ULTRASONIC_READ_INTERVAL) {
    ralphHandleUltrasonicSensor();
    ralphieState.ralphLastUltrasonicTime = ralphCurrentTime;
  }
  
  if (ralphCurrentTime - ralphieState.ralphLastIRTime >= IR_READ_INTERVAL) {
    ralphHandleInfraredSensor();
    ralphieState.ralphLastIRTime = ralphCurrentTime;
  }
  
  if (ralphCurrentTime - ralphieState.ralphLastStatsTime >= STATS_DISPLAY_INTERVAL) {
    ralphDisplayStatistics();
    ralphSaveDataToEEPROM();
    ralphieState.ralphLastStatsTime = ralphCurrentTime;
  }
  
  if (ralphieState.ralphCalibrating) {
    ralphHandleCalibration(ralphCurrentTime);
  }
  
  // Update status LED - visual lang to
  ralphUpdateStatusLED();

  if (Serial.available()) {
    ralphSerialEvent();
  }
  
  delay(20);
}

// ===== INFRARED SENSOR FUNCTIONS =====
void ralphInitializeIRBaseline() {
  Serial.println(F("🔥 Initializing infrared baseline..."));
  
  long ralphSum = 0;
  for (int i = 0; i < IR_BASELINE_SAMPLES; i++) {
    ralphSum += analogRead(IR_SIGNAL_PIN);
    delay(50);
  }
  
  ralphieState.ralphIRBaseline = ralphSum / IR_BASELINE_SAMPLES;
  ralphieState.ralphIRBaselineSet = true;
  
  Serial.print(F("🔥 Infrared baseline set to: "));
  Serial.println(ralphieState.ralphIRBaseline);
  Serial.println();
}

void ralphHandleInfraredSensor() {
  if (!ralphieState.ralphIRBaselineSet) return;
  
  int ralphIRValue = analogRead(IR_SIGNAL_PIN);
  ralphieData.ralphCurrentIRValue = ralphIRValue;
  
  // Update min/max values
  if (ralphIRValue > ralphieData.ralphMaxIRValue) ralphieData.ralphMaxIRValue = ralphIRValue;
  if (ralphIRValue < ralphieData.ralphMinIRValue) ralphieData.ralphMinIRValue = ralphIRValue;
  
  // Calculate difference from baseline
  int ralphIRDifference = abs(ralphIRValue - ralphieState.ralphIRBaseline);
  
  // Check for life detection - important detection to
  if (ralphIRDifference > IR_LIFE_THRESHOLD) {
    ralphieState.ralphIRDetectionCount++;
    
    // Require consecutive detections to avoid false positives
    if (ralphieState.ralphIRDetectionCount >= IR_DETECTION_SAMPLES && !ralphieState.ralphLifeDetected) {
      ralphieState.ralphLifeDetected = true;
      ralphieState.ralphLastLifeTime = millis();
      ralphieData.ralphLifeCount++;
      
      // Determine type of life detected
      if (ralphIRDifference > IR_HUMAN_THRESHOLD) {
        ralphDisplayLifeDetection("HUMAN", ralphIRValue, ralphIRDifference);
      } else {
        ralphDisplayLifeDetection("LIFE FORM (INSECT/SMALL ANIMAL)", ralphIRValue, ralphIRDifference);
      }
    }
  } else {
    if (ralphieState.ralphIRDetectionCount > 0) {
      ralphieState.ralphIRDetectionCount--;
    }
    
    if (ralphieState.ralphLifeDetected && ralphIRDifference < (IR_LIFE_THRESHOLD / 2)) {
      ralphieState.ralphLifeDetected = false;
      ralphDisplayLifeDetectionEnd();
    }
  }
}

// ===== IMPROVED SENSOR HANDLERS =====
void ralphHandleVibrationSensor(unsigned long ralphCurrentTime) {
  if (ralphieState.ralphVibrationFlag && (ralphCurrentTime - ralphieState.ralphLastVibTime > VIBRATION_DEBOUNCE)) {
    ralphieState.ralphLastVibTime = ralphCurrentTime;
    ralphieData.ralphVibCount++;
    
    ralphDisplayVibrationAlert();
    ralphieState.ralphVibrationFlag = false;
    ralphieState.ralphVibrationHits = 0; 
  }
}

void ralphHandleSoundSensor(unsigned long ralphCurrentTime) {
  int ralphSoundValue = analogRead(SOUND_SENSOR_PIN);
  
  // More sensitive sound detection using analog reading
  if (ralphSoundValue > SOUND_THRESHOLD && ralphieState.ralphSoundState == LOW && 
      (ralphCurrentTime - ralphieState.ralphLastSoundTime > SOUND_DEBOUNCE)) {
    
    ralphieState.ralphSoundState = HIGH;
    ralphieState.ralphSoundStart = ralphCurrentTime;
    ralphieState.ralphLastSoundTime = ralphCurrentTime;
    ralphieData.ralphSoundCount++;
    
    ralphDisplaySoundAlert(ralphSoundValue);
  } else if (ralphSoundValue <= SOUND_THRESHOLD && ralphieState.ralphSoundState == HIGH) {
    unsigned long ralphDuration = ralphCurrentTime - ralphieState.ralphSoundStart;
    if (ralphDuration >= SOUND_MIN_DURATION) {
      ralphieState.ralphSoundState = LOW;
      ralphDisplaySoundEnd(ralphDuration);
    }
  }
}

void ralphHandleMotionSensor(unsigned long ralphCurrentTime) {
  int ralphMotionReading = digitalRead(MOTION_PIN);
  
  // Improved motion detection with state tracking
  if (ralphMotionReading == HIGH && ralphieState.ralphLastMotionRead == LOW && 
      (ralphCurrentTime - ralphieState.ralphLastMotionTime > MOTION_DEBOUNCE)) {
    
    ralphieState.ralphMotionState = HIGH;
    ralphieState.ralphMotionStart = ralphCurrentTime;
    ralphieState.ralphLastMotionTime = ralphCurrentTime;
    ralphieData.ralphMotionCount++;
    
    ralphDisplayMotionStart();
  } else if (ralphMotionReading == LOW && ralphieState.ralphLastMotionRead == HIGH && 
             (ralphCurrentTime - ralphieState.ralphLastMotionTime > MOTION_DEBOUNCE)) {
    
    ralphieState.ralphMotionState = LOW;
    ralphieState.ralphLastMotionTime = ralphCurrentTime;
    ralphieState.ralphTotalMotionTime += (ralphCurrentTime - ralphieState.ralphMotionStart);
    
    ralphDisplayMotionEnd(ralphCurrentTime - ralphieState.ralphMotionStart);
  }
  
  ralphieState.ralphLastMotionRead = ralphMotionReading;
}

void ralphHandleCO2Sensor() {
  ralphSendCO2Command(CO2_CMD_READ);
  delay(100);
  
  if (ralphReadCO2Response()) {
    int ralphPPM = ralphCalculateCO2PPM();
    
    if (ralphIsValidCO2Reading(ralphPPM)) {
      // Add to buffer for averaging
      ralphAddCO2ToBuffer(ralphPPM);
      
      int ralphFilteredPPM = ralphGetFilteredCO2();
      
      ralphUpdateCO2Statistics(ralphPPM);
      ralphDisplayCO2Reading(ralphFilteredPPM, ralphPPM); 
    } else {
      ralphDisplayCO2Error("Invalid reading");
    }
  } else {
    ralphDisplayCO2Error("Communication error");
  }
}

bool ralphIsValidCO2Reading(int ralphPPM) {
  return (ralphPPM >= CO2_MIN_VALID && ralphPPM <= CO2_MAX_VALID);
}

void ralphAddCO2ToBuffer(int ralphPPM) {
  ralphieState.ralphCO2Buffer[ralphieState.ralphCO2BufferIndex] = ralphPPM;
  ralphieState.ralphCO2BufferIndex = (ralphieState.ralphCO2BufferIndex + 1) % 5;
  
  if (ralphieState.ralphCO2BufferIndex == 0) {
    ralphieState.ralphCO2BufferFull = true;
  }
}

int ralphGetFilteredCO2() {
  if (!ralphieState.ralphCO2BufferFull && ralphieState.ralphCO2BufferIndex == 0) {
    return ralphieState.ralphCO2Buffer[0]; // hindi pa sure CO2 ralph
  }
  
  int ralphCount = ralphieState.ralphCO2BufferFull ? 5 : ralphieState.ralphCO2BufferIndex;
  long ralphSum = 0;
  
  for (int i = 0; i < ralphCount; i++) {
    ralphSum += ralphieState.ralphCO2Buffer[i];
  }
  
  return ralphSum / ralphCount;
}

// ===== ENHANCED DISPLAY FUNCTIONS =====
void ralphDisplayStartupMessage() {
  Serial.println(F("\n╔══════════════════════════════════════╗"));
  Serial.println(F("║      BAYMAX ENVIRONMENTAL MONITOR    ║"));
  Serial.println(F("║            Version 3.0               ║"));
  Serial.println(F("╠══════════════════════════════════════╣"));
  Serial.println(F("║ Sensors: Vibration, Sound, Motion   ║"));
  Serial.println(F("║    CO2, Distance, Infrared Heat     ║"));
  Serial.println(F("║         LIFE DETECTION ENABLED      ║"));
  Serial.println(F("╚══════════════════════════════════════╝"));
  Serial.println();
  
  Serial.print(F("📊 Loaded Data - Vibrations: "));
  Serial.print(ralphieData.ralphVibCount);
  Serial.print(F(", Sounds: "));
  Serial.print(ralphieData.ralphSoundCount);
  Serial.print(F(", Motion: "));
  Serial.print(ralphieData.ralphMotionCount);
  Serial.print(F(", Proximity: "));
  Serial.print(ralphieData.ralphProximityCount);
  Serial.print(F(", Life Detection: "));
  Serial.println(ralphieData.ralphLifeCount);
  Serial.println();
}

void ralphDisplaySoundAlert(int ralphSoundLevel) {
  Serial.print(F("🔊 SOUND ["));
  Serial.print(ralphieData.ralphSoundCount);
  Serial.print(F("] │ Level: "));
  Serial.print(ralphSoundLevel);
  Serial.print(F(" │ "));
  ralphDisplayTimestamp();
}

void ralphDisplaySoundEnd(unsigned long ralphDuration) {
  Serial.print(F("🔇 SOUND END │ Duration: "));
  Serial.print(ralphDuration);
  Serial.print(F("ms │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayLifeDetection(const char* ralphLifeType, int ralphIRValue, int ralphDifference) {
  Serial.print(F("life detected."));
  Serial.print(F(" │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayLifeDetectionEnd() {
  unsigned long ralphDuration = millis() - ralphieState.ralphLastLifeTime;
  Serial.print(F("🔥 LIFE DETECTION END │ Duration: "));
  Serial.print(ralphDuration / 1000.0, 1);
  Serial.print(F("s │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayCO2Reading(int ralphFilteredPPM, int ralphRawPPM) {
  Serial.print(F("🌬️  CO2 READING [")); // co2 co2 ni ralph
  Serial.print(ralphieData.ralphTotalCO2Readings);
  Serial.print(F("] │ Filtered: "));
  Serial.print(ralphFilteredPPM);
  Serial.print(F(" ppm"));
  
  if (ralphFilteredPPM != ralphRawPPM) {
    Serial.print(F(" (raw: "));
    Serial.print(ralphRawPPM);
    Serial.print(F(")"));
  }
  
  Serial.print(F(" │ "));
  
  // Air quality indicator - importante to
  if (ralphFilteredPPM <= CO2_NORMAL_LEVEL) {
    Serial.print(F("✅ EXCELLENT"));
  } else if (ralphFilteredPPM <= CO2_ELEVATED_LEVEL) {
    Serial.print(F("🟡 GOOD"));
  } else if (ralphFilteredPPM <= CO2_HIGH_LEVEL) {
    Serial.print(F("🟠 MODERATE"));
  } else if (ralphFilteredPPM <= CO2_CRITICAL_LEVEL) {
    Serial.print(F("🔴 POOR"));
  } else {
    Serial.print(F("☠️  HAZARDOUS"));
  }
  
  Serial.print(F(" │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayCO2Error(const char* ralphErrorType) {
  Serial.print(F("❌ CO2 ERROR: "));
  Serial.print(ralphErrorType);
  Serial.print(F(" │ "));
  ralphDisplayTimestamp();
}

// ===== ULTRASONIC SENSOR FUNCTIONS (Same as before) =====
float ralphReadUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  
  long ralphDuration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 60000);
  
  if (ralphDuration == 0) {
    return -1.0;
  }
  
  float ralphDistance = ralphDuration * SOUND_SPEED / 2;
  
  if (ralphDistance < MIN_DISTANCE || ralphDistance > MAX_DISTANCE) {
    return -1.0;
  }
  
  return ralphDistance;
}

void ralphHandleUltrasonicSensor() {
  float ralphDistance = ralphReadUltrasonicDistance();
  
  if (ralphDistance > 0) {
    ralphieData.ralphCurrentDistance = ralphDistance;
    
    if (ralphDistance < ralphieData.ralphMinDistance) ralphieData.ralphMinDistance = ralphDistance;
    if (ralphDistance > ralphieData.ralphMaxDistance) ralphieData.ralphMaxDistance = ralphDistance;
    
    bool ralphIsProximity = (ralphDistance <= PROXIMITY_THRESHOLD);
    
    if (ralphIsProximity && !ralphieState.ralphProximityDetected) {
      ralphieState.ralphProximityDetected = true;
      ralphieState.ralphLastProximityTime = millis();
      ralphieData.ralphProximityCount++;
      ralphDisplayProximityAlert(ralphDistance);
    } 
    else if (!ralphIsProximity && ralphieState.ralphProximityDetected) {
      ralphieState.ralphProximityDetected = false;
      unsigned long ralphDuration = millis() - ralphieState.ralphLastProximityTime;
      ralphDisplayProximityEnd(ralphDuration);
    }
    
    static unsigned long ralphLastDisplayTime = 0;
    if (millis() - ralphLastDisplayTime >= 5000) {
      ralphDisplayDistanceReading(ralphDistance);
      ralphLastDisplayTime = millis();
    }
  }
}

// ===== REMAINING FUNCTIONS (CO2, Calibration, etc.) =====
void ralphSendCO2Command(const unsigned char* ralphCommand) {
  Serial1.write(ralphCommand, CO2_RESPONSE_LENGTH);
}

bool ralphReadCO2Response() {
  int ralphBytesRead = 0;
  unsigned long ralphTimeout = millis() + 1000;
  
  while (ralphBytesRead < CO2_RESPONSE_LENGTH && millis() < ralphTimeout) {
    if (Serial1.available()) {
      ralphCO2Response[ralphBytesRead] = Serial1.read();
      ralphBytesRead++;
    }
  }
  
  if (ralphBytesRead < CO2_RESPONSE_LENGTH) return false;
  
  if (ralphCO2Response[0] != 0xFF || ralphCO2Response[1] != 0x86) return false;
  
  byte ralphChecksum = 0;
  for (int i = 1; i < 8; i++) {
    ralphChecksum += ralphCO2Response[i];
  }
  ralphChecksum = 0xFF - ralphChecksum + 1;
  
  return (ralphCO2Response[8] == ralphChecksum);
}

int ralphCalculateCO2PPM() {
  int ralphHigh = ralphCO2Response[2];
  int ralphLow = ralphCO2Response[3];
  return (ralphHigh << 8) + ralphLow;
}

void ralphUpdateCO2Statistics(int ralphPPM) {
  ralphieData.ralphCurrentCO2 = ralphPPM;
  ralphieData.ralphTotalCO2Readings++;
  ralphieData.ralphSumCO2 += ralphPPM;
  ralphieData.ralphAvgCO2 = (float)ralphieData.ralphSumCO2 / ralphieData.ralphTotalCO2Readings;
  
  if (ralphPPM < ralphieData.ralphMinCO2) ralphieData.ralphMinCO2 = ralphPPM;
  if (ralphPPM > ralphieData.ralphMaxCO2) ralphieData.ralphMaxCO2 = ralphPPM;
}

// ===== IMPROVED VIBRATION INTERRUPT =====
void ralphVibrationISR() {
  static unsigned long ralphLastInterrupt = 0;
  unsigned long ralphCurrentTime = millis();
  
  if (ralphCurrentTime - ralphLastInterrupt > VIBRATION_DEBOUNCE) {
    ralphieState.ralphVibrationHits++;
    
    if (ralphieState.ralphVibrationHits >= VIBRATION_CONSECUTIVE_HITS) {
      ralphieState.ralphVibrationFlag = true;
    }
    
    ralphLastInterrupt = ralphCurrentTime;
  }
}

// ===== CALIBRATION FUNCTIONS =====
void ralphStartCalibration() {
  ralphieState.ralphCalibrating = true;
  ralphieState.ralphCalibStart = millis();
  ralphDisplayCalibrationStart();
}

void ralphHandleCalibration(unsigned long ralphCurrentTime) {
  if (ralphCurrentTime - ralphieState.ralphCalibStart >= CALIBRATION_TIME) {
    ralphSendCO2Command(CO2_CMD_CALIBRATE);
    delay(200);
    
    ralphieState.ralphCalibrating = false;
    ralphDisplayCalibrationComplete();
  } else {
    static unsigned long ralphLastProgressUpdate = 0;
    if (ralphCurrentTime - ralphLastProgressUpdate >= 2000) {
      int ralphProgress = ((ralphCurrentTime - ralphieState.ralphCalibStart) * 100) / CALIBRATION_TIME;
      ralphDisplayCalibrationProgress(ralphProgress);
      ralphLastProgressUpdate = ralphCurrentTime;
    }
  }
}

void ralphDisplayCalibrationStart() {
  Serial.println(F("🔧 CALIBRATION MODE STARTED"));
  Serial.println(F("   Please ensure sensor is in clean air..."));
  Serial.println(F("   Calibration will take 30 seconds"));
  Serial.println();
}

void ralphDisplayCalibrationProgress(int ralphProgress) {
  Serial.print(F("🔧 Calibration Progress: "));
  Serial.print(ralphProgress);
  Serial.println(F("%"));
}

void ralphDisplayCalibrationComplete() {
  Serial.println(F("✅ CALIBRATION COMPLETED"));
  Serial.println(F("   CO2 sensor calibrated to 400ppm"));
  Serial.println(F("   System ready for monitoring"));
  Serial.println();
}

// ===== DISPLAY FUNCTIONS =====
void ralphDisplayVibrationAlert() {
  Serial.print(F("🔴 VIBRATION ["));
  Serial.print(ralphieData.ralphVibCount);
  Serial.print(F("] │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayMotionStart() {
  Serial.print(F("🚶 MOTION START ["));
  Serial.print(ralphieData.ralphMotionCount);
  Serial.print(F("] │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayMotionEnd(unsigned long ralphDuration) {
  Serial.print(F("⏹️  MOTION END │ Duration: "));
  Serial.print(ralphDuration / 1000.0, 1);
  Serial.print(F("s │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayDistanceReading(float ralphDistance) {
  Serial.print(F("📏 DISTANCE: "));
  Serial.print(ralphDistance, 1);
  Serial.print(F(" cm │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayProximityAlert(float ralphDistance) {
  Serial.print(F("👤 PROXIMITY ["));
  Serial.print(ralphieData.ralphProximityCount);
  Serial.print(F("] │ Distance: "));
  Serial.print(ralphDistance, 1);
  Serial.print(F(" cm │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayProximityEnd(unsigned long ralphDuration) {
  Serial.print(F("👤 PROXIMITY END │ Duration: "));
  Serial.print(ralphDuration / 1000.0, 1);
  Serial.print(F("s │ "));
  ralphDisplayTimestamp();
}

void ralphDisplayStatistics() {
  Serial.println(F("\n╔══════════════ ENHANCED STATISTICS ══════════════╗"));
  
  unsigned long ralphUptimeSeconds = millis() / 1000;
  unsigned long ralphHours = ralphUptimeSeconds / 3600;
  unsigned long ralphMinutes = (ralphUptimeSeconds % 3600) / 60;
  unsigned long ralphSeconds = ralphUptimeSeconds % 60;
  
  Serial.print(F("⏱️  Uptime: "));
  Serial.print(ralphHours);
  Serial.print(F("h "));
  Serial.print(ralphMinutes);
  Serial.print(F("m "));
  Serial.print(ralphSeconds);
  Serial.println(F("s"));
  
  Serial.print(F("🔴 Vibrations: "));
  Serial.print(ralphieData.ralphVibCount);
  Serial.print(F(" │ 🔊 Sounds: "));
  Serial.print(ralphieData.ralphSoundCount);
  Serial.print(F(" │ 🚶 Motion: "));
  Serial.print(ralphieData.ralphMotionCount);
  Serial.print(F(" │ 👤 Proximity: "));
  Serial.println(ralphieData.ralphProximityCount);
  
  Serial.print(F("🔥 Life Detections: "));
  Serial.print(ralphieData.ralphLifeCount);
  Serial.print(F(" │ 🚶 Total Motion Time: "));
  Serial.print(ralphieState.ralphTotalMotionTime / 1000.0, 1);
  Serial.println(F("s"));
  
  if (ralphieData.ralphTotalCO2Readings > 0) {
    Serial.print(F("🌬️  CO2 │ Current: "));
    Serial.print(ralphieData.ralphCurrentCO2);
    Serial.print(F(" │ Avg: "));
    Serial.print(ralphieData.ralphAvgCO2, 0);
    Serial.print(F(" │ Min: "));
    Serial.print(ralphieData.ralphMinCO2);
    Serial.print(F(" │ Max: "));
    Serial.print(ralphieData.ralphMaxCO2);
    Serial.println(F(" ppm"));
  }
  
  Serial.print(F("📏 Distance │ Current: "));
  Serial.print(ralphieData.ralphCurrentDistance, 1);
  Serial.print(F(" │ Min: "));
  Serial.print(ralphieData.ralphMinDistance, 1);
  Serial.print(F(" │ Max: "));
  Serial.print(ralphieData.ralphMaxDistance, 1);
  Serial.println(F(" cm"));
  
  Serial.print(F("🔥 Infrared │ Current: "));
  Serial.print(ralphieData.ralphCurrentIRValue);
  Serial.print(F(" │ Baseline: "));
  Serial.print(ralphieState.ralphIRBaseline);
  Serial.print(F(" │ Min: "));
  Serial.print(ralphieData.ralphMinIRValue);
  Serial.print(F(" │ Max: "));
  Serial.println(ralphieData.ralphMaxIRValue);
  
  Serial.println(F("╚═════════════════════════════════════════════════╝\n"));
}

void ralphDisplayTimestamp() {
  unsigned long ralphCurrentTime = millis();
  unsigned long ralphSeconds = (ralphCurrentTime / 1000) % 60;
  unsigned long ralphMinutes = (ralphCurrentTime / (1000 * 60)) % 60;
  unsigned long ralphHours = (ralphCurrentTime / (1000 * 60 * 60)) % 24;
  
  Serial.print(ralphHours);
  Serial.print(F(":"));
  if (ralphMinutes < 10) Serial.print(F("0"));
  Serial.print(ralphMinutes);
  Serial.print(F(":"));
  if (ralphSeconds < 10) Serial.print(F("0"));
  Serial.println(ralphSeconds);
}

// ===== EEPROM FUNCTIONS =====
void ralphLoadDataFromEEPROM() {
  if (EEPROM.read(EEPROM_INIT_FLAG) == 0xAA) {
    EEPROM.get(EEPROM_VIBRATION_COUNT, ralphieData.ralphVibCount);
    EEPROM.get(EEPROM_SOUND_COUNT, ralphieData.ralphSoundCount);
    EEPROM.get(EEPROM_MOTION_COUNT, ralphieData.ralphMotionCount);
    EEPROM.get(EEPROM_PROXIMITY_COUNT, ralphieData.ralphProximityCount);
    EEPROM.get(EEPROM_LIFE_COUNT, ralphieData.ralphLifeCount);
  } else {
    EEPROM.write(EEPROM_INIT_FLAG, 0xAA);
    ralphSaveDataToEEPROM();
  }
}

void ralphSaveDataToEEPROM() {
  EEPROM.put(EEPROM_VIBRATION_COUNT, ralphieData.ralphVibCount);
  EEPROM.put(EEPROM_SOUND_COUNT, ralphieData.ralphSoundCount);
  EEPROM.put(EEPROM_MOTION_COUNT, ralphieData.ralphMotionCount);
  EEPROM.put(EEPROM_PROXIMITY_COUNT, ralphieData.ralphProximityCount);
  EEPROM.put(EEPROM_LIFE_COUNT, ralphieData.ralphLifeCount);
}

// ===== STATUS LED AND SERIAL FUNCTIONS =====
void ralphUpdateStatusLED() {
  static unsigned long ralphLastBlink = 0;
  static bool ralphLedState = false;
  
  unsigned long ralphBlinkRate = 1000; //  1 second

  if (ralphieState.ralphCalibrating) {
    ralphBlinkRate = 250;
  }
  else if (ralphieState.ralphLifeDetected) {
    ralphBlinkRate = 100;
  }
  else if (ralphieState.ralphMotionState == HIGH) {
    ralphBlinkRate = 500;
  }
  
  if (millis() - ralphLastBlink > ralphBlinkRate) {
    ralphLedState = !ralphLedState;
    digitalWrite(STATUS_LED_PIN, ralphLedState);
    ralphLastBlink = millis();
  }
}

void ralphSerialEvent() {
  while (Serial.available()) {
    String ralphCommand = Serial.readStringUntil('\n');
    ralphCommand.trim();
    ralphCommand.toLowerCase();
    
    if (ralphCommand == "stats") {
      ralphDisplayStatistics();
    } else if (ralphCommand == "reset") {
      ralphResetCounters();
    } else if (ralphCommand == "calibrate") {
      ralphStartCalibration();
    } else if (ralphCommand == "distance") {
      float ralphDist = ralphReadUltrasonicDistance();
      if (ralphDist > 0) {
        Serial.print(F("📏 Current Distance: "));
        Serial.print(ralphDist, 1);
        Serial.println(F(" cm"));
      } else {
        Serial.println(F("❌ Ultrasonic sensor error"));
      }
    } else if (ralphCommand == "ir" || ralphCommand == "infrared") {
      ralphDisplayIRStatus();
    } else if (ralphCommand == "co2") {
      ralphDisplayCO2Status();
    } else if (ralphCommand == "sensitivity") {
      ralphDisplaySensitivitySettings();
    } else if (ralphCommand == "test") {
      ralphRunSensorTest();
    } else if (ralphCommand == "help") {
      ralphDisplayHelp();
    } else {
      Serial.println(F("❓ Unknown command. Type 'help' for available commands."));
    }
  }
}

void ralphDisplayIRStatus() {
  Serial.println(F("\n🔥 INFRARED HEAT SENSOR STATUS:"));
  Serial.print(F("   Baseline: "));
  Serial.println(ralphieState.ralphIRBaseline);
  Serial.print(F("   Current Value: "));
  Serial.println(ralphieData.ralphCurrentIRValue);
  Serial.print(F("   Difference: "));
  Serial.println(abs(ralphieData.ralphCurrentIRValue - ralphieState.ralphIRBaseline));
  Serial.print(F("   Life Detected: "));
  Serial.println(ralphieState.ralphLifeDetected ? F("YES") : F("NO"));
  Serial.print(F("   Total Life Detections: "));
  Serial.println(ralphieData.ralphLifeCount);
  Serial.println();
}

void ralphDisplayCO2Status() {
  Serial.println(F("\n🌬️  CO2 SENSOR STATUS:"));
  Serial.print(F("   Current (Filtered): "));
  Serial.print(ralphieData.ralphCurrentCO2);
  Serial.println(F(" ppm"));
  Serial.print(F("   Average: "));
  Serial.print(ralphieData.ralphAvgCO2, 1);
  Serial.println(F(" ppm"));
  Serial.print(F("   Min/Max: "));
  Serial.print(ralphieData.ralphMinCO2);
  Serial.print(F("/"));
  Serial.print(ralphieData.ralphMaxCO2);
  Serial.println(F(" ppm"));
  Serial.print(F("   Total Readings: "));
  Serial.println(ralphieData.ralphTotalCO2Readings);
  
  // Show buffer contents - importante ang filtering to
  Serial.print(F("   Buffer: ["));
  int ralphCount = ralphieState.ralphCO2BufferFull ? 5 : ralphieState.ralphCO2BufferIndex;
  for (int i = 0; i < ralphCount; i++) {
    Serial.print(ralphieState.ralphCO2Buffer[i]);
    if (i < ralphCount - 1) Serial.print(F(", "));
  }
  Serial.println(F("]"));
  Serial.println();
}

void ralphDisplaySensitivitySettings() {
  Serial.println(F("\n⚙️  SENSITIVITY SETTINGS:"));
  Serial.print(F("   Vibration Debounce: "));
  Serial.print(VIBRATION_DEBOUNCE);
  Serial.println(F(" ms"));
  Serial.print(F("   Sound Threshold: "));
  Serial.print(SOUND_THRESHOLD);
  Serial.println(F(" (0-1023)"));
  Serial.print(F("   Motion Debounce: "));
  Serial.print(MOTION_DEBOUNCE);
  Serial.println(F(" ms"));
  Serial.print(F("   IR Life Threshold: "));
  Serial.println(IR_LIFE_THRESHOLD);
  Serial.print(F("   IR Human Threshold: "));
  Serial.println(IR_HUMAN_THRESHOLD);
  Serial.println();
}

void ralphRunSensorTest() {
  Serial.println(F("\n🧪 RUNNING SENSOR TEST..."));
  Serial.println(F("═══════════════════════════════════"));
  
  // Test vibration sensor
  Serial.print(F("🔴 Vibration Pin ("));
  Serial.print(VIBRATION_PIN);
  Serial.print(F("): "));
  Serial.println(digitalRead(VIBRATION_PIN) ? F("HIGH") : F("LOW"));
  
  // Test sound sensor
  int ralphSoundValue = analogRead(SOUND_SENSOR_PIN);
  Serial.print(F("🔊 Sound Level: "));
  Serial.print(ralphSoundValue);
  Serial.println(F(" (0-1023)"));
  
  // Test motion sensor
  Serial.print(F("🚶 Motion Pin ("));
  Serial.print(MOTION_PIN);
  Serial.print(F("): "));
  Serial.println(digitalRead(MOTION_PIN) ? F("HIGH") : F("LOW"));
  
  // Test ultrasonic sensor
  float ralphDistance = ralphReadUltrasonicDistance();
  Serial.print(F("📏 Distance: "));
  if (ralphDistance > 0) {
    Serial.print(ralphDistance, 1);
    Serial.println(F(" cm"));
  } else {
    Serial.println(F("ERROR"));
  }
  
  // Test infrared sensor
  int ralphIRValue = analogRead(IR_SIGNAL_PIN);
  Serial.print(F("🔥 Infrared Value: "));
  Serial.print(ralphIRValue);
  Serial.print(F(" (Baseline: "));
  Serial.print(ralphieState.ralphIRBaseline);
  Serial.print(F(", Diff: "));
  Serial.print(abs(ralphIRValue - ralphieState.ralphIRBaseline));
  Serial.println(F(")"));
  
  // Test CO2 sensor - importante ang test to
  Serial.print(F("🌬️  CO2 Sensor: "));
  ralphSendCO2Command(CO2_CMD_READ);
  delay(100);
  if (ralphReadCO2Response()) {
    int ralphPPM = ralphCalculateCO2PPM();
    Serial.print(ralphPPM);
    Serial.print(F(" ppm "));
    Serial.println(ralphIsValidCO2Reading(ralphPPM) ? F("(VALID)") : F("(INVALID)"));
  } else {
    Serial.println(F("ERROR - No Response"));
  }
  
  Serial.println(F("═══════════════════════════════════"));
  Serial.println();
}

void ralphResetCounters() {
  ralphieData.ralphVibCount = 0;
  ralphieData.ralphSoundCount = 0;
  ralphieData.ralphMotionCount = 0;
  ralphieData.ralphProximityCount = 0;
  ralphieData.ralphLifeCount = 0;
  ralphieData.ralphTotalCO2Readings = 0;
  ralphieData.ralphSumCO2 = 0;
  ralphieData.ralphMinCO2 = 9999;
  ralphieData.ralphMaxCO2 = 0;
  ralphieData.ralphMinDistance = MAX_DISTANCE;
  ralphieData.ralphMaxDistance = 0;
  ralphieData.ralphMinIRValue = 1023;
  ralphieData.ralphMaxIRValue = 0;
  ralphieState.ralphTotalMotionTime = 0;
  
  // Reset ng CO2 buffer ni ralph
  for (int i = 0; i < 5; i++) {
    ralphieState.ralphCO2Buffer[i] = 0;
  }
  ralphieState.ralphCO2BufferIndex = 0;
  ralphieState.ralphCO2BufferFull = false;
  
  ralphSaveDataToEEPROM();
  Serial.println(F("All counters and statistics reset"));
}

void ralphDisplayHelp() {
  Serial.println(F("\nAVAILABLE COMMANDS:"));
  Serial.println(F("═══════════════════════════════════════"));
  Serial.println(F("  stats         - Show detailed statistics"));
  Serial.println(F("  reset         - Reset all counters"));
  Serial.println(F("  calibrate     - Start CO2 calibration"));
  Serial.println(F("  distance      - Read current distance"));
  Serial.println(F("  ir/infrared   - Show infrared sensor status"));
  Serial.println(F("  co2           - Show CO2 sensor status"));
  Serial.println(F("  sensitivity   - Show sensitivity settings"));
  Serial.println(F("  test          - Run complete sensor test"));
  Serial.println(F("  help          - Show this help menu"));
  Serial.println(F("═══════════════════════════════════════"));
  Serial.println();
}
