/*
 * SmartKnob Menu System for Arduino Nano 33 BLE
 * Version WITHOUT Display (Serial Monitor Menu)
 * 
 * This version removes the TFT display dependency and uses Serial Monitor
 * for menu interaction until you configure TFT_eSPI properly.
 * 
 * Hardware:
 * - Arduino Nano 33 BLE
 * - SimpleFOC Mini Driver v1.0 (TMC6300)
 * - AS5600 Magnetic Encoder (I2C)
 * - BLDC Gimbal Motor
 * - Push Button on Pin D7
 */

#include <SimpleFOC.h>
#include <Wire.h>

// ============ PIN DEFINITIONS ============

// Motor Driver (TMC6300)
#define MOTOR_PIN_A 11
#define MOTOR_PIN_B 10
#define MOTOR_PIN_C 9
#define MOTOR_ENABLE 8

// Button Pin
#define BUTTON_PIN 7

// ============ MOTOR CONFIGURATION ============

#define MOTOR_POLE_PAIRS 7
#define MOTOR_VOLTAGE_LIMIT 3.0
#define MOTOR_VELOCITY_LIMIT 20

// PID Parameters
#define PID_P 6.0
#define PID_I 0.0
#define PID_D 0.06
#define PID_RAMP 1000.0
#define PID_LIMIT 10.0

// ============ MENU SYSTEM ============

enum MenuMode {
  MODE_FINE_CLICKS,      // Many fine detents (360 positions)
  MODE_COARSE_CLICKS,    // Fewer hard detents (24 positions)
  MODE_SPRING_RETURN,    // Spring return with endpoints
  MODE_SETTINGS,         // Settings submenu
  NUM_MODES
};

struct MenuItem {
  const char* name;
  const char* description;
  MenuMode mode;
};

MenuItem menuItems[] = {
  {"Fine Clicks", "360 smooth detents (1° each)", MODE_FINE_CLICKS},
  {"Coarse Clicks", "24 strong clicks (15° each)", MODE_COARSE_CLICKS},
  {"Spring Return", "Springs to center (±90°)", MODE_SPRING_RETURN},
  {"Settings", "Adjust parameters", MODE_SETTINGS}
};

enum SettingsItem {
  SETTING_STRENGTH,
  SETTING_RETURN,
  NUM_SETTINGS
};

const char* settingsNames[] = {
  "Haptic Strength",
  "Return to Menu"
};

// ============ HAPTIC CONFIGURATION ============

struct HapticConfig {
  float position_width_radians;
  float detent_strength;
  float endstop_strength;
  int num_positions;
  bool has_detents;
  float snap_point;
};

// ============ GLOBAL OBJECTS ============

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(MOTOR_PIN_A, MOTOR_PIN_B, MOTOR_PIN_C, MOTOR_ENABLE);
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// ============ STATE VARIABLES ============

MenuMode currentMode = MODE_FINE_CLICKS;
int currentMenuItem = 0;
int currentSettingItem = 0;
bool inSettingsMenu = false;
bool inModeDemo = false;
bool motorEngaged = true;

HapticConfig currentConfig;
float currentAngle = 0;
int currentPosition = 0;
int lastPosition = -999;

// Button debouncing
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;

// Haptic strength multiplier
float hapticStrengthMultiplier = 1.0;

// Status display
unsigned long lastStatusUpdate = 0;
const unsigned long statusUpdateInterval = 500;

// ============ SETUP ============

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);
  
  Serial.println("\n\n========================================");
  Serial.println("   SmartKnob Menu System - Nano 33 BLE");
  Serial.println("========================================\n");
  
  // Initialize button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("[✓] Button initialized on Pin D7");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize encoder
  sensor.init();
  motor.linkSensor(&sensor);
  Serial.println("[✓] AS5600 encoder initialized");
  
  // Initialize driver
  driver.voltage_power_supply = 5.0;
  driver.init();
  motor.linkDriver(&driver);
  Serial.println("[✓] Motor driver initialized");

  // Motor configuration
  motor.controller = MotionControlType::torque;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.velocity_limit = MOTOR_VELOCITY_LIMIT;
  
  // PID configuration
  motor.PID_velocity.P = PID_P;
  motor.PID_velocity.I = PID_I;
  motor.PID_velocity.D = PID_D;
  motor.PID_velocity.output_ramp = PID_RAMP;
  motor.PID_velocity.limit = PID_LIMIT;

  // Initialize motor
  motor.init();
  motor.initFOC();
  Serial.println("[✓] Motor FOC initialized");
  
  // Set initial mode
  setHapticMode(MODE_FINE_CLICKS);
  
  // Show menu
  printMenu();
  
  Serial.println("\n========================================");
  Serial.println("SmartKnob Ready!");
  Serial.println("Turn knob to navigate | Press button to select");
  Serial.println("========================================\n");
}

// ============ MAIN LOOP ============

void loop() {
  // Update FOC (critical timing)
  motor.loopFOC();
  
  // Handle button
  handleButton();
  
  // Update haptic feedback
  if (motorEngaged) {
    if (inSettingsMenu) {
      updateSettingsHaptic();
    } else if (inModeDemo) {
      updateModeHaptic();
    } else {
      updateMenuHaptic();
    }
  }
  
  // Move motor
  motor.move();
  
  // Update status display
  if (millis() - lastStatusUpdate > statusUpdateInterval) {
    updateStatus();
    lastStatusUpdate = millis();
  }
}

// ============ HAPTIC MODES ============

void setHapticMode(MenuMode mode) {
  currentMode = mode;
  
  switch(mode) {
    case MODE_FINE_CLICKS:
      currentConfig.position_width_radians = 1.0 * PI / 180.0;
      currentConfig.detent_strength = 0.4;
      currentConfig.endstop_strength = 0.8;
      currentConfig.num_positions = 360;
      currentConfig.has_detents = true;
      currentConfig.snap_point = 0.8;
      break;
      
    case MODE_COARSE_CLICKS:
      currentConfig.position_width_radians = 15.0 * PI / 180.0;
      currentConfig.detent_strength = 1.2;
      currentConfig.endstop_strength = 1.5;
      currentConfig.num_positions = 24;
      currentConfig.has_detents = true;
      currentConfig.snap_point = 1.5;
      break;
      
    case MODE_SPRING_RETURN:
      currentConfig.position_width_radians = 1.0 * PI / 180.0;
      currentConfig.detent_strength = 0.0;
      currentConfig.endstop_strength = 2.0;
      currentConfig.num_positions = 181;
      currentConfig.has_detents = false;
      currentConfig.snap_point = 0.0;
      break;
      
    case MODE_SETTINGS:
      currentConfig.position_width_radians = 60.0 * PI / 180.0;
      currentConfig.detent_strength = 1.0;
      currentConfig.endstop_strength = 1.2;
      currentConfig.num_positions = NUM_SETTINGS;
      currentConfig.has_detents = true;
      currentConfig.snap_point = 1.2;
      break;
  }
  
  currentPosition = 0;
  lastPosition = -999;
}

void setMenuHaptic() {
  currentConfig.position_width_radians = 60.0 * PI / 180.0;
  currentConfig.detent_strength = 1.0;
  currentConfig.endstop_strength = 1.2;
  currentConfig.num_positions = NUM_MODES;
  currentConfig.has_detents = true;
  currentConfig.snap_point = 1.2;
}

// ============ HAPTIC FEEDBACK ============

void updateMenuHaptic() {
  currentAngle = motor.shaft_angle;
  float position_f = currentAngle / currentConfig.position_width_radians;
  int new_position = round(position_f);
  new_position = constrain(new_position, 0, NUM_MODES - 1);
  
  if (new_position != currentPosition) {
    currentPosition = new_position;
    currentMenuItem = currentPosition;
  }
  
  calculateHapticTorque();
}

void updateModeHaptic() {
  currentAngle = motor.shaft_angle;
  float position_f = currentAngle / currentConfig.position_width_radians;
  int new_position = round(position_f);
  new_position = constrain(new_position, 0, currentConfig.num_positions - 1);
  
  if (new_position != currentPosition) {
    currentPosition = new_position;
  }
  
  calculateHapticTorque();
}

void updateSettingsHaptic() {
  currentAngle = motor.shaft_angle;
  float position_f = currentAngle / currentConfig.position_width_radians;
  int new_position = round(position_f);
  new_position = constrain(new_position, 0, NUM_SETTINGS - 1);
  
  if (new_position != currentPosition) {
    currentPosition = new_position;
    currentSettingItem = currentPosition;
  }
  
  calculateHapticTorque();
}

void calculateHapticTorque() {
  float target_detent = currentPosition * currentConfig.position_width_radians;
  
  // For spring return, target is center
  if (currentMode == MODE_SPRING_RETURN && inModeDemo) {
    target_detent = (currentConfig.num_positions / 2) * currentConfig.position_width_radians;
  }
  
  float angle_diff = currentAngle - target_detent;
  while (angle_diff > PI) angle_diff -= 2*PI;
  while (angle_diff < -PI) angle_diff += 2*PI;
  
  float torque = 0;
  
  // Detent force
  if (currentConfig.has_detents && currentConfig.detent_strength > 0) {
    torque += -angle_diff * currentConfig.detent_strength * currentConfig.snap_point * 5.0 * hapticStrengthMultiplier;
  }
  
  // Spring return force
  if (currentMode == MODE_SPRING_RETURN && inModeDemo) {
    torque = -angle_diff * 2.5 * hapticStrengthMultiplier;
  }
  
  // Endstop force
  if (currentConfig.endstop_strength > 0) {
    float min_angle = 0;
    float max_angle = (currentConfig.num_positions - 1) * currentConfig.position_width_radians;
    
    if (currentMode == MODE_SPRING_RETURN && inModeDemo) {
      float half_range = (currentConfig.num_positions - 1) * currentConfig.position_width_radians / 2.0;
      min_angle = target_detent - half_range;
      max_angle = target_detent + half_range;
    }
    
    float dist_from_min = currentAngle - min_angle;
    if (dist_from_min < 0) {
      torque += -dist_from_min * currentConfig.endstop_strength * 10.0 * hapticStrengthMultiplier;
    }
    
    float dist_from_max = currentAngle - max_angle;
    if (dist_from_max > 0) {
      torque += -dist_from_max * currentConfig.endstop_strength * 10.0 * hapticStrengthMultiplier;
    }
  }
  
  motor.target = constrain(torque, -motor.voltage_limit, motor.voltage_limit);
}

// ============ BUTTON HANDLING ============

void handleButton() {
  bool reading = digitalRead(BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      onButtonPress();
    }
  }
  
  lastButtonState = reading;
}

void onButtonPress() {
  Serial.println("\n[BUTTON PRESSED]");
  
  if (inModeDemo) {
    // Return to menu from demo
    inModeDemo = false;
    currentPosition = 0;
    currentMenuItem = 0;
    setMenuHaptic();
    printMenu();
  } else if (inSettingsMenu) {
    handleSettingsSelection();
  } else {
    handleMenuSelection();
  }
}

void handleMenuSelection() {
  if (currentMenuItem == MODE_SETTINGS) {
    inSettingsMenu = true;
    currentPosition = 0;
    currentSettingItem = 0;
    setHapticMode(MODE_SETTINGS);
    printSettingsMenu();
  } else {
    inModeDemo = true;
    setHapticMode((MenuMode)currentMenuItem);
    printModeDemo();
  }
}

void handleSettingsSelection() {
  switch(currentSettingItem) {
    case SETTING_STRENGTH:
      hapticStrengthMultiplier += 0.25;
      if (hapticStrengthMultiplier > 2.0) hapticStrengthMultiplier = 0.5;
      Serial.print("→ Haptic Strength: ");
      Serial.print(hapticStrengthMultiplier, 2);
      Serial.println("x");
      break;
      
    case SETTING_RETURN:
      inSettingsMenu = false;
      currentPosition = 0;
      currentMenuItem = 0;
      setMenuHaptic();
      printMenu();
      break;
  }
}

// ============ DISPLAY FUNCTIONS ============

void printMenu() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║          MAIN MENU                     ║");
  Serial.println("╠════════════════════════════════════════╣");
  
  for (int i = 0; i < NUM_MODES; i++) {
    if (i == currentMenuItem) {
      Serial.print("║ ► ");
    } else {
      Serial.print("║   ");
    }
    Serial.print(menuItems[i].name);
    
    // Pad with spaces
    int len = strlen(menuItems[i].name);
    for (int j = len; j < 35; j++) Serial.print(" ");
    Serial.println("║");
  }
  
  Serial.println("╠════════════════════════════════════════╣");
  Serial.print("║ ");
  Serial.print(menuItems[currentMenuItem].description);
  int len = strlen(menuItems[currentMenuItem].description);
  for (int j = len; j < 38; j++) Serial.print(" ");
  Serial.println("║");
  Serial.println("╚════════════════════════════════════════╝");
}

void printSettingsMenu() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║          SETTINGS MENU                 ║");
  Serial.println("╠════════════════════════════════════════╣");
  
  for (int i = 0; i < NUM_SETTINGS; i++) {
    if (i == currentSettingItem) {
      Serial.print("║ ► ");
    } else {
      Serial.print("║   ");
    }
    Serial.print(settingsNames[i]);
    
    int len = strlen(settingsNames[i]);
    
    // Show value for strength setting
    if (i == SETTING_STRENGTH) {
      Serial.print(" [");
      Serial.print(hapticStrengthMultiplier, 2);
      Serial.print("x]");
      len += 8;
    }
    
    for (int j = len; j < 35; j++) Serial.print(" ");
    Serial.println("║");
  }
  
  Serial.println("╚════════════════════════════════════════╝");
}

void printModeDemo() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.print("║  MODE: ");
  Serial.print(menuItems[currentMode].name);
  int len = strlen(menuItems[currentMode].name);
  for (int j = len; j < 31; j++) Serial.print(" ");
  Serial.println("║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║                                        ║");
  Serial.println("║  Turn the knob to feel the haptics!   ║");
  Serial.println("║                                        ║");
  Serial.println("║  Press button to return to menu       ║");
  Serial.println("║                                        ║");
  Serial.println("╚════════════════════════════════════════╝");
}

void updateStatus() {
  if (currentPosition == lastPosition) return;
  
  if (inModeDemo) {
    Serial.print("Position: ");
    
    if (currentMode == MODE_SPRING_RETURN) {
      int offset = currentPosition - (currentConfig.num_positions / 2);
      Serial.print(offset);
      Serial.println("°");
    } else {
      Serial.print(currentPosition);
      Serial.print(" / ");
      Serial.println(currentConfig.num_positions - 1);
    }
  } else if (!inSettingsMenu) {
    // Menu navigation feedback
    Serial.print("→ ");
    Serial.println(menuItems[currentMenuItem].name);
  } else {
    // Settings navigation
    Serial.print("→ ");
    Serial.println(settingsNames[currentSettingItem]);
  }
  
  lastPosition = currentPosition;
}
