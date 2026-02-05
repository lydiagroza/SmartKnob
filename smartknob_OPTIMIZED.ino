/*
 * SmartKnob Menu System - OPTIMIZED for Round Display
 * Features:
 * - Circular UI optimized for 240x240 round screen
 * - Higher refresh rate for smooth display
 * - Working button with hold-to-return (3 sec)
 * - Functional settings menu
 */

#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

// ============ PIN DEFINITIONS ============

// Motor Driver (TMC6300)
#define MOTOR_PIN_A 11
#define MOTOR_PIN_B 10
#define MOTOR_PIN_C 9
#define MOTOR_ENABLE 8

// Button Pin
#define BUTTON_PIN 7

// Display Pins - SOFTWARE SPI
#define TFT_CS   4
#define TFT_DC   3
#define TFT_RST  2
#define TFT_MOSI 12
#define TFT_SCK  13

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

// ============ DISPLAY CONFIGURATION ============

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 240
#define CENTER_X 120
#define CENTER_Y 120

// Colors
#define COLOR_BG GC9A01A_BLACK
#define COLOR_PRIMARY GC9A01A_CYAN
#define COLOR_SECONDARY 0x39E7      // Darker gray for round display
#define COLOR_TEXT GC9A01A_WHITE
#define COLOR_ACCENT GC9A01A_MAGENTA
#define COLOR_SUCCESS GC9A01A_GREEN
#define COLOR_WARNING GC9A01A_YELLOW
#define COLOR_DANGER GC9A01A_RED

// ============ MENU SYSTEM ============

enum MenuMode {
  MODE_FINE_CLICKS,
  MODE_COARSE_CLICKS,
  MODE_SPRING_RETURN,
  MODE_SETTINGS,
  NUM_MODES
};

struct MenuItem {
  const char* name;
  MenuMode mode;
};

MenuItem menuItems[] = {
  {"Fine", MODE_FINE_CLICKS},
  {"Coarse", MODE_COARSE_CLICKS},
  {"Spring", MODE_SPRING_RETURN},
  {"Settings", MODE_SETTINGS}
};

enum SettingsItem {
  SETTING_STRENGTH,
  SETTING_DETENT_STYLE,
  SETTING_RETURN,
  NUM_SETTINGS
};

const char* settingsNames[] = {
  "Strength",
  "Style",
  "< Back"
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
Adafruit_GC9A01A tft = Adafruit_GC9A01A(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);

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

// Button state
bool buttonPressed = false;
unsigned long buttonPressTime = 0;
unsigned long lastButtonChangeTime = 0;
const unsigned long debounceDelay = 50;
const unsigned long holdDuration = 3000; // 3 seconds

// Haptic strength multiplier (0.5x to 2.0x)
float hapticStrengthMultiplier = 1.0;

// Detent style (0 = soft, 1 = medium, 2 = hard)
int detentStyle = 1;

// Display update - HIGHER REFRESH RATE
unsigned long lastDisplayUpdate = 0;
const unsigned long displayUpdateInterval = 16; // ~60 FPS
bool needsFullRedraw = true;

// ============ SETUP ============

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n========================================");
  Serial.println("   SmartKnob - Optimized");
  Serial.println("========================================\n");
  
  // Initialize display
  Serial.println("[*] Initializing display...");
  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(COLOR_BG);
  Serial.println("[✓] Display initialized");
  
  // Show splash screen
  showSplashScreen();
  
  // Initialize button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("[✓] Button initialized");
  
  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);
  
  // Initialize encoder
  sensor.init();
  motor.linkSensor(&sensor);
  Serial.println("[✓] Encoder initialized");
  
  // Initialize driver
  driver.voltage_power_supply = 12.0;
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
  
  delay(1500);
  
  // Set initial mode
  setHapticMode(MODE_FINE_CLICKS);
  drawMenu();
  
  Serial.println("\n========================================");
  Serial.println("SmartKnob Ready!");
  Serial.println("Press: Select | Hold 3s: Return to menu");
  Serial.println("========================================\n");
}

// ============ MAIN LOOP ============

void loop() {
  // Update FOC (critical timing)
  motor.loopFOC();
  
  // Handle button with hold detection
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
  
  // Update display at higher refresh rate
  if (millis() - lastDisplayUpdate >= displayUpdateInterval) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}

// ============ HAPTIC MODES ============

void setHapticMode(MenuMode mode) {
  currentMode = mode;
  
  // Base strength multiplier based on style
  float styleMultiplier = 1.0;
  switch(detentStyle) {
    case 0: styleMultiplier = 0.6; break;  // Soft
    case 1: styleMultiplier = 1.0; break;  // Medium
    case 2: styleMultiplier = 1.5; break;  // Hard
  }
  
  switch(mode) {
    case MODE_FINE_CLICKS:
      currentConfig.position_width_radians = 1.0 * PI / 180.0;
      currentConfig.detent_strength = 0.4 * styleMultiplier;
      currentConfig.endstop_strength = 0.8;
      currentConfig.num_positions = 360;
      currentConfig.has_detents = true;
      currentConfig.snap_point = 0.8;
      break;
      
    case MODE_COARSE_CLICKS:
      currentConfig.position_width_radians = 15.0 * PI / 180.0;
      currentConfig.detent_strength = 1.2 * styleMultiplier;
      currentConfig.endstop_strength = 1.5;
      currentConfig.num_positions = 24;
      currentConfig.has_detents = true;
      currentConfig.snap_point = 1.5;
      break;
      
    case MODE_SPRING_RETURN:
      currentConfig.position_width_radians = 1.0 * PI / 180.0;
      currentConfig.detent_strength = 0.0;
      currentConfig.endstop_strength = 2.0 * styleMultiplier;
      currentConfig.num_positions = 181;
      currentConfig.has_detents = false;
      currentConfig.snap_point = 0.0;
      break;
      
    case MODE_SETTINGS:
      currentConfig.position_width_radians = 90.0 * PI / 180.0;
      currentConfig.detent_strength = 1.2;
      currentConfig.endstop_strength = 1.5;
      currentConfig.num_positions = NUM_SETTINGS;
      currentConfig.has_detents = true;
      currentConfig.snap_point = 1.2;
      break;
  }
  
  currentPosition = 0;
  lastPosition = -999;
}

void setMenuHaptic() {
  currentConfig.position_width_radians = 90.0 * PI / 180.0;
  currentConfig.detent_strength = 1.0;
  currentConfig.endstop_strength = 1.2;
  currentConfig.num_positions = NUM_MODES;
  currentConfig.has_detents = true;
  currentConfig.snap_point = 1.2;
}

// ============ HAPTIC FEEDBACK ============

void updateMenuHaptic() {
  currentAngle = sensor.getAngle();
  float position_f = currentAngle / currentConfig.position_width_radians;
  int new_position = round(position_f);
  new_position = constrain(new_position, 0, NUM_MODES - 1);
  
  if (new_position != currentPosition) {
    currentPosition = new_position;
    currentMenuItem = currentPosition;
    needsFullRedraw = true;
  }
  
  calculateHapticTorque();
}

void updateModeHaptic() {
  currentAngle = sensor.getAngle();
  float position_f = currentAngle / currentConfig.position_width_radians;
  int new_position = round(position_f);
  new_position = constrain(new_position, 0, currentConfig.num_positions - 1);
  
  if (new_position != currentPosition) {
    currentPosition = new_position;
  }
  
  calculateHapticTorque();
}

void updateSettingsHaptic() {
  currentAngle = sensor.getAngle();
  float position_f = currentAngle / currentConfig.position_width_radians;
  int new_position = round(position_f);
  new_position = constrain(new_position, 0, NUM_SETTINGS - 1);
  
  if (new_position != currentPosition) {
    currentPosition = new_position;
    currentSettingItem = currentPosition;
    needsFullRedraw = true;
  }
  
  calculateHapticTorque();
}

void calculateHapticTorque() {
  float target_detent = currentPosition * currentConfig.position_width_radians;
  
  if (currentMode == MODE_SPRING_RETURN && inModeDemo) {
    target_detent = (currentConfig.num_positions / 2) * currentConfig.position_width_radians;
  }
  
  float angle_diff = currentAngle - target_detent;
  while (angle_diff > PI) angle_diff -= 2*PI;
  while (angle_diff < -PI) angle_diff += 2*PI;
  
  float torque = 0;
  
  if (currentConfig.has_detents && currentConfig.detent_strength > 0) {
    torque = -angle_diff * currentConfig.detent_strength * currentConfig.snap_point * 5.0 * hapticStrengthMultiplier;
  }
  
  if (currentMode == MODE_SPRING_RETURN && inModeDemo) {
    torque = -angle_diff * 2.5 * hapticStrengthMultiplier;
  }
  
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
  
  motor.target = torque;
}

// ============ BUTTON HANDLING ============

void handleButton() {
  bool currentReading = digitalRead(BUTTON_PIN);
  unsigned long currentTime = millis();
  
  // Debounce
  if (currentReading != buttonPressed) {
    if (currentTime - lastButtonChangeTime > debounceDelay) {
      lastButtonChangeTime = currentTime;
      
      if (currentReading == LOW && !buttonPressed) {
        // Button just pressed
        buttonPressed = true;
        buttonPressTime = currentTime;
      } else if (currentReading == HIGH && buttonPressed) {
        // Button just released
        buttonPressed = false;
        unsigned long pressDuration = currentTime - buttonPressTime;
        
        if (pressDuration < holdDuration) {
          // Short press
          onButtonPress();
        }
        needsFullRedraw = true;
      }
    }
  }
  
  // Check for hold duration and show loading circle
  if (buttonPressed && (inModeDemo || inSettingsMenu)) {
    unsigned long holdTime = currentTime - buttonPressTime;
    if (holdTime >= holdDuration) {
      // Return to main menu
      returnToMainMenu();
      buttonPressed = false;
    }
  }
}

void onButtonPress() {
  Serial.println("[BUTTON] Short press");
  
  if (inModeDemo) {
    // In demo mode, short press does nothing (must hold to return)
    return;
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
    needsFullRedraw = true;
    Serial.println("[MENU] Entered settings");
  } else {
    inModeDemo = true;
    setHapticMode((MenuMode)currentMenuItem);
    needsFullRedraw = true;
    Serial.print("[MENU] Entered mode: ");
    Serial.println(menuItems[currentMenuItem].name);
  }
}

void handleSettingsSelection() {
  switch(currentSettingItem) {
    case SETTING_STRENGTH:
      hapticStrengthMultiplier += 0.25;
      if (hapticStrengthMultiplier > 2.0) hapticStrengthMultiplier = 0.5;
      needsFullRedraw = true;
      // Update current mode with new strength
      if (inModeDemo) {
        setHapticMode(currentMode);
      }
      Serial.print("[SETTINGS] Strength: ");
      Serial.println(hapticStrengthMultiplier);
      break;
      
    case SETTING_DETENT_STYLE:
      detentStyle = (detentStyle + 1) % 3;
      needsFullRedraw = true;
      // Update current mode with new style
      if (inModeDemo) {
        setHapticMode(currentMode);
      }
      Serial.print("[SETTINGS] Style: ");
      Serial.println(detentStyle == 0 ? "Soft" : detentStyle == 1 ? "Medium" : "Hard");
      break;
      
    case SETTING_RETURN:
      returnToMainMenu();
      Serial.println("[SETTINGS] Returned to menu");
      break;
  }
}

void returnToMainMenu() {
  inSettingsMenu = false;
  inModeDemo = false;
  currentPosition = 0;
  currentMenuItem = 0;
  setMenuHaptic();
  needsFullRedraw = true;
  Serial.println("[RETURN] Back to main menu");
}

// ============ DISPLAY FUNCTIONS ============

void showSplashScreen() {
  tft.fillScreen(COLOR_BG);
  
  // Draw outer ring
  tft.drawCircle(CENTER_X, CENTER_Y, 115, COLOR_PRIMARY);
  tft.drawCircle(CENTER_X, CENTER_Y, 114, COLOR_PRIMARY);
  
  // Draw title
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(3);
  tft.setCursor(35, 95);
  tft.print("SMART");
  tft.setCursor(45, 125);
  tft.print("KNOB");
  
  // Draw subtitle
  tft.setTextSize(1);
  tft.setTextColor(COLOR_SECONDARY);
  tft.setCursor(75, 155);
  tft.print("Ready");
}

void updateDisplay() {
  // Show hold-to-return loading circle if button held
  if (buttonPressed && (inModeDemo || inSettingsMenu)) {
    unsigned long holdTime = millis() - buttonPressTime;
    if (holdTime > 200) {  // Start showing after 200ms
      drawHoldProgress(holdTime);
      return;
    }
  }
  
  if (inSettingsMenu) {
    drawSettingsMenu();
  } else if (inModeDemo) {
    drawModeDemo();
  } else {
    drawMenu();
  }
}

void drawMenu() {
  // 1. STOP if nothing has changed!
  // If we assume the screen is already correct, we exit immediately.
  if (!needsFullRedraw) return; 

  // 2. Clear the background
  tft.fillScreen(COLOR_BG);
  
  // 3. Draw outer ring
  tft.drawCircle(CENTER_X, CENTER_Y, 115, COLOR_SECONDARY);
  
  // 4. Draw menu items in circle
  for (int i = 0; i < NUM_MODES; i++) {
    float angle = (2.0 * PI * i / NUM_MODES) - PI/2;
    int radius = 80;
    int x = CENTER_X + cos(angle) * radius;
    int y = CENTER_Y + sin(angle) * radius;
    
    if (i == currentMenuItem) {
      // Selected item
      tft.fillCircle(x, y, 25, COLOR_PRIMARY);
      tft.setTextColor(COLOR_BG);
      tft.setTextSize(1);
      
      int textLen = strlen(menuItems[i].name);
      tft.setCursor(x - (textLen * 3), y - 4);
      tft.print(menuItems[i].name);
    } else {
      // Unselected item
      tft.drawCircle(x, y, 20, COLOR_SECONDARY);
      tft.setTextColor(COLOR_SECONDARY);
      tft.setTextSize(1);
      
      int textLen = strlen(menuItems[i].name);
      tft.setCursor(x - (textLen * 3), y - 4);
      tft.print(menuItems[i].name);
    }
  }
  
  // 5. Draw center text
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  int textLen = strlen(menuItems[currentMenuItem].name);
  tft.fillRect(CENTER_X - 40, CENTER_Y - 10, 80, 20, COLOR_BG);
  tft.setCursor(CENTER_X - (textLen * 6), CENTER_Y - 8);
  tft.print(menuItems[currentMenuItem].name);
  
  // 6. Draw instruction
  tft.setTextSize(1);
  tft.setTextColor(COLOR_SECONDARY);
  tft.fillRect(0, 215, 240, 10, COLOR_BG);
  tft.setCursor(72, 218);
  tft.print("Press to select");

  // 7. MARK AS DONE
  // We finished drawing, so we set this to false.
  // The code won't draw again until you move the knob.
  needsFullRedraw = false;
}

void drawSettingsMenu() {
  if (needsFullRedraw) {
    tft.fillScreen(COLOR_BG);
    
    // Draw title
    tft.setTextColor(COLOR_ACCENT);
    tft.setTextSize(2);
    tft.setCursor(50, 25);
    tft.print("SETTINGS");
    
    needsFullRedraw = false;
  }
  
  // Draw settings items vertically
  int startY = 65;
  int itemHeight = 45;
  
  for (int i = 0; i < NUM_SETTINGS; i++) {
    int yPos = startY + i * itemHeight;
    
    if (i == currentSettingItem) {
      // Selected
      tft.fillRoundRect(15, yPos, 210, 38, 8, COLOR_ACCENT);
      tft.setTextColor(COLOR_BG);
      tft.setTextSize(2);
      tft.setCursor(25, yPos + 6);
      tft.print(settingsNames[i]);
      
      // Show value
      tft.setTextSize(1);
      tft.setCursor(25, yPos + 24);
      
      if (i == SETTING_STRENGTH) {
        tft.print(hapticStrengthMultiplier, 2);
        tft.print("x");
      } else if (i == SETTING_DETENT_STYLE) {
        if (detentStyle == 0) tft.print("Soft");
        else if (detentStyle == 1) tft.print("Medium");
        else tft.print("Hard");
      }
    } else {
      // Unselected
      tft.drawRoundRect(15, yPos, 210, 38, 8, COLOR_SECONDARY);
      tft.fillRoundRect(16, yPos + 1, 208, 36, 7, COLOR_BG);
      tft.setTextColor(COLOR_SECONDARY);
      tft.setTextSize(1);
      tft.setCursor(25, yPos + 15);
      tft.print(settingsNames[i]);
    }
  }
  
  // Instructions
  tft.setTextSize(1);
  tft.setTextColor(COLOR_SECONDARY);
  tft.fillRect(0, 205, 240, 20, COLOR_BG);
  tft.setCursor(20, 210);
  tft.print("Press: Change");
  tft.setCursor(20, 220);
  tft.print("Hold 3s: Exit");
}

void drawModeDemo() {
  if (needsFullRedraw) {
    tft.fillScreen(COLOR_BG);
    
    // Draw mode name at top
    tft.setTextColor(COLOR_SUCCESS);
    tft.setTextSize(2);
    int nameLen = strlen(menuItems[currentMode].name);
    tft.setCursor(CENTER_X - (nameLen * 6), 25);
    tft.print(menuItems[currentMode].name);
    
    needsFullRedraw = false;
  }
  
  // Draw position indicator
  if (currentMode == MODE_SPRING_RETURN) {
    drawSpringReturnIndicator();
  } else {
    drawPositionIndicator();
  }
  
  // Instructions at bottom
  tft.setTextSize(1);
  tft.setTextColor(COLOR_SECONDARY);
  tft.fillRect(0, 210, 240, 20, COLOR_BG);
  tft.setCursor(55, 218);
  tft.print("Hold 3s to exit");
}

void drawPositionIndicator() {
  // Clear center
  tft.fillCircle(CENTER_X, CENTER_Y, 85, COLOR_BG);
  
  // Draw ring with position markers
  int numDots = min(currentConfig.num_positions, 36);
  for (int i = 0; i < numDots; i++) {
    float angle = (2.0 * PI * i) / numDots - PI/2;
    int x = CENTER_X + cos(angle) * 95;
    int y = CENTER_Y + sin(angle) * 95;
    
    int positionIndex = (i * currentConfig.num_positions) / numDots;
    if (positionIndex == currentPosition) {
      tft.fillCircle(x, y, 6, COLOR_SUCCESS);
    } else {
      tft.drawCircle(x, y, 3, COLOR_SECONDARY);
    }
  }
  
  // Draw position number
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(4);
  String posStr = String(currentPosition);
  int textWidth = posStr.length() * 24;
  tft.fillRect(CENTER_X - 50, CENTER_Y - 25, 100, 30, COLOR_BG);
  tft.setCursor(CENTER_X - (textWidth / 2), CENTER_Y - 20);
  tft.print(posStr);
  
  // Draw total
  tft.setTextSize(2);
  tft.setTextColor(COLOR_SECONDARY);
  String totalStr = "/" + String(currentConfig.num_positions - 1);
  int totalWidth = totalStr.length() * 12;
  tft.fillRect(CENTER_X - 40, CENTER_Y + 15, 80, 16, COLOR_BG);
  tft.setCursor(CENTER_X - (totalWidth / 2), CENTER_Y + 15);
  tft.print(totalStr);
}

void drawSpringReturnIndicator() {
  // Clear center
  tft.fillCircle(CENTER_X, CENTER_Y, 85, COLOR_BG);
  
  int centerPos = currentConfig.num_positions / 2;
  int offset = currentPosition - centerPos;
  float maxAngle = PI / 2;
  float currentAngleRad = (float)offset / 90.0 * maxAngle;
  
  // Draw range arc
  for (int i = -90; i <= 90; i += 2) {
    float angle = (i * PI / 180.0) - PI/2;
    int x = CENTER_X + cos(angle) * 90;
    int y = CENTER_Y + sin(angle) * 90;
    tft.drawPixel(x, y, COLOR_SECONDARY);
    
    int x2 = CENTER_X + cos(angle) * 88;
    int y2 = CENTER_Y + sin(angle) * 88;
    tft.drawPixel(x2, y2, COLOR_SECONDARY);
  }
  
  // Draw center marker (zero position)
  tft.fillCircle(CENTER_X, CENTER_Y - 90, 5, COLOR_PRIMARY);
  
  // Draw current position pointer
  float pointerAngle = currentAngleRad - PI/2;
  int px = CENTER_X + cos(pointerAngle) * 70;
  int py = CENTER_Y + sin(pointerAngle) * 70;
  tft.fillCircle(px, py, 10, COLOR_SUCCESS);
  tft.drawLine(CENTER_X, CENTER_Y, px, py, COLOR_SUCCESS);
  
  // Draw degree value
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(4);
  String degStr = String(offset);
  if (offset > 0) degStr = "+" + degStr;
  int textWidth = degStr.length() * 24;
  tft.fillRect(CENTER_X - 60, CENTER_Y - 15, 120, 30, COLOR_BG);
  tft.setCursor(CENTER_X - (textWidth / 2), CENTER_Y - 15);
  tft.print(degStr);
  
  tft.setTextSize(2);
  tft.print("o");
}

void drawHoldProgress(unsigned long holdTime) {
  // Draw loading circle
  float progress = (float)holdTime / (float)holdDuration;
  progress = constrain(progress, 0.0, 1.0);
  
  // Darken screen slightly
  for (int r = 115; r > 30; r -= 10) {
    tft.drawCircle(CENTER_X, CENTER_Y, r, 0x2104);
  }
  
  // Draw progress ring
  int numSegments = 60;
  int completedSegments = progress * numSegments;
  
  for (int i = 0; i < numSegments; i++) {
    float angle = (2.0 * PI * i / numSegments) - PI/2;
    int x1 = CENTER_X + cos(angle) * 100;
    int y1 = CENTER_Y + sin(angle) * 100;
    int x2 = CENTER_X + cos(angle) * 110;
    int y2 = CENTER_Y + sin(angle) * 110;
    
    if (i < completedSegments) {
      tft.drawLine(x1, y1, x2, y2, COLOR_WARNING);
    } else {
      tft.drawLine(x1, y1, x2, y2, COLOR_SECONDARY);
    }
  }
  
  // Draw text
  tft.setTextColor(COLOR_TEXT);
  tft.setTextSize(2);
  tft.fillRect(CENTER_X - 60, CENTER_Y - 10, 120, 20, COLOR_BG);
  tft.setCursor(CENTER_X - 54, CENTER_Y - 8);
  tft.print("Returning");
  
  // Show percentage
  tft.setTextSize(3);
  tft.fillRect(CENTER_X - 30, CENTER_Y + 20, 60, 24, COLOR_BG);
  int percent = progress * 100;
  String percentStr = String(percent) + "%";
  tft.setCursor(CENTER_X - (percentStr.length() * 9), CENTER_Y + 20);
  tft.print(percentStr);
}
