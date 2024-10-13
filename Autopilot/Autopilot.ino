#include <WiFi.h>
#include <vector>
#include <SPI.h>
#include <Wire.h>
#include "AsyncTimer.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Other constants
#define SERIAL_BAUD_RATE 115200
#define WIFI_PORT 80
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define BNO055_SAMPLERATE_DELAY_MS 10
#define MAX_SSE_CLIENTS 1

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET - 1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, & Wire1, OLED_RESET);

#define DEBUG 1
#ifdef DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

const char * ssid = "ArduinoGiga"; // Name for your access point
const char * password = "8029802980"; // Password for your access point

const int configurableLineWidth = 2; // Configurable line width for flexible rendering

WiFiServer server(80);
std::vector < WiFiClient > sseClients;

// Pin definitions
#define LED_BUILTIN 13
#define MAGNET_SENSOR_PIN A0

const int MIN_HEADING = 0;
const int MAX_HEADING = 359;
const int MAX_CHANGE = 10;  // Maximum change in degrees per second

unsigned long lastEventTime = 0;
const long eventInterval = 1000; // Send an event every 5 seconds
const int HOME_POSITION = 5;

enum HomeMethod {
  TICK,
  MAGNET
};
struct SystemSettings {
  int magnetCntThreshold = 5;
  int magnetAnalogThreshold = 540;
  int magnetLoopThreshold = 100;
  HomeMethod homeMethod = TICK;
  int tickLengthRight = 5000;
  int tickLengthLeft = 5000;
  int tickLengthRightHome = 5000;
  int tickLengthLeftHome = 5000;
  int pauseLength = 2000;
  int tickLengthLimit = 10000;
  int headingThreshold = 20;
  int currentManualValue = 10;
  int manualValues[4] = {
    1000,
    2000,
    2000,
    1000
  };
};

struct CourseSettings {
  double xPos = 0;
  double yPos = 0;
  double headingVel = 0;
  double currHeading = 10;
  double holdHeading = -1;
  bool correctingCourse = false;
  bool okToHoldHeading = false;
  int maxDisplayChars = 9;
  String courseErrorStr = "";
  double courseError = 0;
};

enum class SystemState {
  INITIALIZING,
  READY,
  AP_ON,
  AP_OFF,
  ERROR
};

enum class ErrorCode {
  NONE,
  DISPLAY_INIT_FAILED,
  BNO055_NOT_DETECTED,
  WIFI_CONNECTION_FAILED,
  MOTOR_CONTROL_ERROR,
  SENSOR_READ_ERROR
};

enum RudderState {
  GO_HOME,
  GO_LEFT,
  GO_RIGHT,
  POT_SB,
  POT_PORT
};

enum MotorDirection {
  STOP,
  LEFT,
  RIGHT,
  LEFT_NUDGE,
  RIGHT_NUDGE
};

struct DirectionValue {
  MotorDirection direction; // LEFT or RIGHT
  int value;     // The value parsed from the string
};

struct ApSettings {
  RudderState rudderState = GO_HOME;
  bool BNO055Detected = true;
  bool LEDEnabled = false;
  String ipAddress = "";
};

const double ACCEL_VEL_TRANSITION = 0.01; // Assuming BNO055_SAMPLERATE_DELAY_MS = 10
const double ACCEL_POS_TRANSITION = 0.00005; // 0.5 * ACCEL_VEL_TRANSITION^2
const double DEG_2_RAD = 0.01745329251; // Conversion factor for degrees to radians

void sendToDisplay(const String lines[], const int sizes[], const int colors[], int numLines, int x = 0, int y = 0);
void sendToDisplay(const String& line1, int size1 = 2);
void sendToDisplay(const String& line1, const String& line2, int size1 = 2, int size2 = 2);
void sendToDisplay(const String& line1, const String& line2, const String& line3, int size1 = 2, int size2 = 2, int size3 = 2);

Adafruit_BNO055 bno = Adafruit_BNO055(12345);

void portClick();
void portNudgeClick();
void starboardClick();
void starboardNudgeClick();
void setClick();

AsyncTimer getHeadingTimer;
AsyncTimer getWiFiClientTimer;

SystemSettings settings;
CourseSettings courseSettings;
ApSettings apSettings;

void setup() {  
  randomSeed(analogRead(0));

  initializeSerial();
  initializeWire();
  initializeDisplay();
  initializeBNO055();
  setupWiFiAP();
  initializeLED();
  setupTimers();
}

void initializeSerial() {
  Serial.begin(SERIAL_BAUD_RATE);
}

void initializeWire() {
  Wire2.begin();
}

void initializeDisplay() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //logError(ErrorCode::DISPLAY_INIT_FAILED);
    handleInitializationError(ErrorCode::DISPLAY_INIT_FAILED, true);
  }  
}

void initializeBNO055() {
  apSettings.BNO055Detected = bno.begin();
  if (BNO055Detected()) {
    bno.setExtCrystalUse(true);
  } else {
    handleInitializationError(ErrorCode::BNO055_NOT_DETECTED, false);
    apSettings.BNO055Detected = true;
    logError(ErrorCode::BNO055_NOT_DETECTED);
    while (1) {
      toggleLED();
      delay(500);
    }
  }
  /*
  if (!bno.begin()) {
    logError(ErrorCode::BNO055_NOT_DETECTED);
    apSettings.BNO055Detected = false; //HARDCODED TO TRUE FOR NOW
    while (1) {
      toggleLED();
      delay(500);
    }
  } else {
    apSettings.BNO055Detected = true;
    bno.setExtCrystalUse(true);
  }
  */
}

String valueToArrowString(int value) {
  // Ensure the value is within the valid range
  value = constrain(value, -259, 259);
  
  // Determine the character to use
  char arrowChar = (value >= 0) ? '>' : '<';
  
  // Calculate the number of characters
  int absValue = abs(value);
  int charCount = 0;
  
  if (absValue >= settings.headingThreshold) {
    charCount = 5;
  } else if (absValue > 0) {
    charCount = map(absValue, 1, settings.headingThreshold, 1, 5);
  }
  if (charCount>5) charCount=5;
  
  // Create and return the string
  String result = "";
  for (int i = 0; i < charCount; i++) {
    result += arrowChar;
  }
  
  return result;
}

void handleInitializationError(ErrorCode err, bool stopExecution) {
  logError(err);
  if (stopExecution) {
    while (1) {
      toggleLED();
      delay(500);
    }
  }
}

void initializeLED() {
  pinMode(LED_BUILTIN, OUTPUT);
  turnLEDOff();
}

void setupTimers() {
  getHeadingTimer.setInterval(getHeading, 1000);
  getWiFiClientTimer.setInterval(evaluateWiFiClient, 500);
}

void loop() {  
  // Common tasks for all states
  getWiFiClientTimer.handle();
  getHeadingTimer.handle();
}

bool initializeSystem() {
  // Perform any remaining initialization tasks
  // Return true if successful, false otherwise
  return true;
}

void logError(ErrorCode error) {
  switch (error) {
    case ErrorCode::DISPLAY_INIT_FAILED:
      Serial.println(F("ERROR: Display initialization failed"));
      break;
    case ErrorCode::BNO055_NOT_DETECTED:
      Serial.println(F("ERROR: BNO055 not detected"));
      break;
    case ErrorCode::WIFI_CONNECTION_FAILED:
      Serial.println(F("ERROR: WiFi connection failed"));
      break;
    case ErrorCode::MOTOR_CONTROL_ERROR:
      Serial.println(F("ERROR: Motor control error"));
      break;
    case ErrorCode::SENSOR_READ_ERROR:
      Serial.println(F("ERROR: Sensor read error"));
      break;
    default:
      Serial.println(F("ERROR: Unknown error occurred"));
      break;
  }
}

// Function to calculate the total time and direction to reach the target position
void evaluateManualSteering(int targetPosition) {
    constexpr int MIN_POSITION = 1;
    constexpr int MAX_POSITION = 9;
    constexpr int HOME_POSITION = 5;

    DEBUG_PRINT("targetPosition = ");
    DEBUG_PRINTLN(targetPosition);

    int currentPosition = settings.currentManualValue;

    // Validate input
    if (targetPosition < MIN_POSITION || targetPosition > MAX_POSITION) {
        DEBUG_PRINTLN("Invalid target position");
        return;
    }

    // Check if already at target position
    if (currentPosition == targetPosition) {
        apSettings.rudderState = GO_HOME;
        DEBUG_PRINTLN("Already at the target position");
        return;
    }

    MotorDirection dir = (currentPosition < targetPosition) ? RIGHT : LEFT;
    int totalTime = calculateMoveTime(currentPosition, targetPosition);

    DEBUG_PRINT("Move ");
    DEBUG_PRINT(dir == RIGHT ? "RIGHT" : "LEFT");
    DEBUG_PRINT(" from position ");
    DEBUG_PRINT(currentPosition);
    DEBUG_PRINT(" to position ");
    DEBUG_PRINT(targetPosition);
    DEBUG_PRINT(" will take ");
    DEBUG_PRINT(totalTime);
    DEBUG_PRINTLN(" ms.");

    turnOffAP();
    moveMotor(dir, totalTime);

    settings.currentManualValue = targetPosition;
    updateRudderState(targetPosition);
}

int calculateMoveTime(int start, int end) {
    int totalTime = 0;
    int step = (start < end) ? 1 : -1;
    for (int i = start; i != end; i += step) {
        int stepIndex = abs((i < HOME_POSITION) ? HOME_POSITION - i - 1 : i - HOME_POSITION);
        totalTime += settings.manualValues[stepIndex];
    }
    return totalTime;
}

void updateRudderState(int position) {
    switch (position) {
        case 1:
        case 3:
        case 4:
            apSettings.rudderState = POT_PORT;
            break;
        case 2:
            apSettings.rudderState = GO_LEFT;
            break;
        case 5:
            apSettings.rudderState = GO_HOME;
            break;
        case 6:
        case 7:
        case 9:
            apSettings.rudderState = POT_SB;
            break;
        case 8:
            apSettings.rudderState = GO_RIGHT;
            break;
        default:
            DEBUG_PRINTLN("Invalid position for rudder state update");
    }
}

void moveMotor(MotorDirection dir, int duration) {
    controlMotor(dir);
    delay(duration);
    controlMotor(STOP);
}

void sendManualHome() {
  if (settings.currentManualValue != 5) {
    evaluateManualSteering(5);
  }
}

void sendSimpleResponse(WiFiClient& client, const char* status, const char* contentType, const char* content = "") {
    client.print("HTTP/1.1 ");
    client.println(status);
    client.print("Content-Type: ");
    client.println(contentType);
    client.println("Connection: close");
    client.println();
    if (strlen(content) > 0) {
        client.println(content);
    }
    client.stop();
}

int getManualMoveTotal(int st, int ed) {
  int tot = 0;

  for (int i = st; i <= ed; i++) {
    switch (i) {
    case 1:
      tot += settings.manualValues[0];
    case 2:
      tot += settings.manualValues[1];
    case 3:
      tot += settings.manualValues[2];
    case 4:
      tot += settings.manualValues[3];
    case 6:
      tot += settings.manualValues[3];
    case 7:
      tot += settings.manualValues[2];
    case 8:
      tot += settings.manualValues[1];
    case 9:
      tot += settings.manualValues[0];
    }
  }

  return tot;
}

void turnOnAP() {
  rudderHome();
  courseSettings.okToHoldHeading = true;
  apSettings.LEDEnabled = true;
}

void turnOffAP() {
  courseSettings.holdHeading = -1;
  courseSettings.okToHoldHeading = false;
  turnLEDOff();
  apSettings.LEDEnabled = false;
}

bool isAPOn() {
  return courseSettings.holdHeading > -1;
}

bool isAPOff() {
  return courseSettings.holdHeading == -1;
}
bool needToCourseCorrect() {
  if (isAPOn()) {
    double error = calculateCourseError(courseSettings.holdHeading, courseSettings.currHeading);
    return abs(error) > settings.headingThreshold;
  }
  return false;
}

bool isStringEmpty(String str) {
  return (str == NULL || str.length() == 0);
}

void sendToDisplay(const String lines[], const int sizes[], const int colors[], int numLines, int x, int y) {
    display.clearDisplay();
    int ycoord = y > 0 ? y : 0;

    for (int i = 0; i < numLines; i++) {
        if (!isStringEmpty(lines[i])) {
            //Serial.println(lines[i]);
            display.setTextSize(sizes[i] > 0 ? sizes[i] : 2);
            display.setTextColor(colors[i] > 0 ? colors[i] : SSD1306_WHITE);
            display.setCursor(x, ycoord);
            display.println(lines[i]);
            ycoord += sizes[i] * 8;
        }
    }

    display.display();
}

void sendToDisplay(const String& line1, int size1) {
    String lines[] = {line1};
    int sizes[] = {size1};
    int colors[] = {SSD1306_WHITE};
    sendToDisplay(lines, sizes, colors, 1);
}

void sendToDisplay(const String& line1, const String& line2, int size1, int size2) {
    String lines[] = {line1, line2};
    int sizes[] = {size1, size2};
    int colors[] = {SSD1306_WHITE, SSD1306_WHITE};
    sendToDisplay(lines, sizes, colors, 2);
}

void sendToDisplay(const String& line1, const String& line2, const String& line3, 
                   int size1, int size2, int size3) {
    String lines[] = {line1, line2, line3};
    int sizes[] = {size1, size2, size3};
    int colors[] = {SSD1306_WHITE, SSD1306_WHITE, SSD1306_WHITE};
    sendToDisplay(lines, sizes, colors, 3);
}

//DIAG FUNCTIONS
void sendToDebug(String ln1, String ln2, bool nl = false) {
  DEBUG_PRINT(ln1);
  if (nl) {
    DEBUG_PRINTLN(ln2);
  } else {
    DEBUG_PRINT(ln2);
  }
}
void sendToDebug(String ln, bool nl = false) {

  if (nl) {
    DEBUG_PRINTLN(ln);
  } else {
    DEBUG_PRINT(ln);
  }
}

void sendDiagToDebug() {

  sendToDebug(">> Heading: ", (String) courseSettings.currHeading);
  if (isAPOn()) {
    sendToDebug(" Holding: ", (String) courseSettings.holdHeading);
    sendToDebug(" Diff: ", (String) calculateCourseError(courseSettings.holdHeading, courseSettings.currHeading));
  }
  if (courseSettings.correctingCourse) {
    sendToDebug(" isCorrectingCourse");
  }
  if (!apSettings.BNO055Detected) {
    sendToDebug(" Simulating Heading ");
  }

  sendToDebug(" rudderState: ", convertRudderStateToStr(apSettings.rudderState), true);
}

String getStatusStr() {
  String str = (String) courseSettings.currHeading;

  if (isAPOn()) {
    str += " [";
    str += (String) courseSettings.holdHeading;
    str += "]";
  }

  return str;
}
void sendHome() {
  if (isRudderNotHome()) {
    turnOffAP();
    rudderHome();
  }
}
void setClick() {
  DEBUG_PRINTLN("SET PRESSED");
  DEBUG_PRINTLN(isAPOn());
  DEBUG_PRINTLN(isRudderHome());

  if ((isAPOff()) && (isRudderNotHome())) {
    //not stable - rudder needs to be sent home
    DEBUG_PRINTLN("SET PRESSED - Go Home");
    
    rudderHome();
  } else if ((isAPOff()) && (isRudderHome())) {
    //everything good - ready to start AP
    DEBUG_PRINTLN("SET PRESSED - Turn AP ON");

    turnOnAP();
  //} else if (isAPOn() && isRudderHome()) {
  } else if (isAPOn()) {
    //everything good - ready to stop AP
    DEBUG_PRINTLN("SET PRESSED - Turn AP OFF");
    turnOffAP();
  }
}

// Event handlers for other buttons
void portClick() {
  //DEBUG_PRINTLN("PORT PRESSED");  
  turnOffAP();
  if (apSettings.rudderState != GO_LEFT) {
    rudderHome();
    handleMotorMove(LEFT);    
  }
}

void portNudgeClick() {
  //DEBUG_PRINTLN("PORT NUDGE PRESSED"); 
  bool apStateWasOn = isAPOn();
  turnOffAP();
  if (apSettings.rudderState != GO_LEFT) {
    rudderHome();
    handleMotorMove(LEFT_NUDGE);
    //apSettings.rudderState = GO_HOME; ////
  }
  if (apStateWasOn) {
    turnOnAP();
  }
}

void starboardClick() {
  //DEBUG_PRINTLN("STARBOARD PRESSED");
  turnOffAP();
  if (apSettings.rudderState != GO_RIGHT) {
    rudderHome();
    handleMotorMove(RIGHT);
    //apSettings.rudderState = GO_RIGHT; ////
  }
}

void starboardNudgeClick() {
  //DEBUG_PRINTLN("STARBOARD NUDGE PRESSED");
  bool apStateWasOn = isAPOn();
  turnOffAP();
  if (apSettings.rudderState != GO_RIGHT) {
    rudderHome();
    handleMotorMove(RIGHT_NUDGE);
    //apSettings.rudderState = GO_HOME; ////
  }
  if (apStateWasOn) {
    turnOnAP();
  }
}

bool isRudderHome() {
  return (apSettings.rudderState == GO_HOME);
}

bool isRudderNotHome() {
  return (apSettings.rudderState != GO_HOME);
}

bool isInManualMode() {
  return ((apSettings.rudderState == POT_PORT) || (apSettings.rudderState == POT_SB));
}

bool isNotInManualMode() {
  return ((apSettings.rudderState != POT_PORT) && (apSettings.rudderState != POT_SB));
}
void turnLEDOn() {
  digitalWrite(LED_BUILTIN, LOW);
}

void turnLEDOff() {
  digitalWrite(LED_BUILTIN, HIGH);
}

void toggleLED() {
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

String getCourseErrorStr() {
  return getCourseErrorStr_C();
}

String getCourseErrorStr_C() {
  int maxChars = courseSettings.maxDisplayChars;

  String returnVal = "";
  double error = calculateCourseError(courseSettings.holdHeading, courseSettings.currHeading);
  int limit = settings.headingThreshold;

  // Calculate the ratio of error to limit and scale it for a max of 4 characters
  int scaledError = abs(error) * maxChars / limit;

  // Ensure scaled error is capped at 4 to not overflow the display
  if (scaledError > maxChars) {
    scaledError = maxChars;
  }

  int spaces = maxChars - scaledError;

  if (error > 0) {
    // PORT side, prepend the "X"s before the "|"
    returnVal = "";
    //for (int i = 0; i < spaces; i++) {
    //  returnVal += " ";
    //}
    for (int i = 0; i < scaledError; i++) {
      returnVal += "<";
    }
  } else if (error < 0) {
    // STARBOARD side, append the "X"s after the "|"
    returnVal = "";
    for (int i = 0; i < scaledError; i++) {
      returnVal += ">";
    }
  }

  return returnVal;
}

String getCourseErrorStr_A() {
  int maxChars = courseSettings.maxDisplayChars;

  String returnVal = "    |";
  double error = calculateCourseError(courseSettings.holdHeading, courseSettings.currHeading);
  int limit = settings.headingThreshold;

  // Calculate the ratio of error to limit and scale it for a max of 4 characters
  int scaledError = abs(error) * maxChars / limit;

  // Ensure scaled error is capped at 4 to not overflow the display
  if (scaledError > maxChars) {
    scaledError = maxChars;
  }

  int spaces = maxChars - scaledError;

  if (error > 0) {
    // PORT side, prepend the "X"s before the "|"
    returnVal = "";
    for (int i = 0; i < spaces; i++) {
      returnVal += " ";
    }
    for (int i = 0; i < scaledError; i++) {
      returnVal += "X";
    }
    returnVal += "|";
  } else if (error < 0) {
    // STARBOARD side, append the "X"s after the "|"
    returnVal = "    |";
    for (int i = 0; i < scaledError; i++) {
      returnVal += "X";
    }
  }

  return returnVal;
}

String getCourseErrorStr_B() {
  int maxChars = courseSettings.maxDisplayChars;

  String returnVal = "      |      "; // Central gauge with space for both sides
  double error = calculateCourseError(courseSettings.holdHeading, courseSettings.currHeading);
  int limit = settings.headingThreshold;

  // Scale the error to fit within the 6-character limit on each side
  int scaledError = abs(error) * maxChars / limit;

  // Ensure scaled error is capped at 6 to not overflow
  if (scaledError > maxChars) {
    scaledError = maxChars;
  }

  if (error > 0) {
    // PORT side, add characters before the "|"
    returnVal = String('=', scaledError) + String('-', maxChars - scaledError) + "|";
  } else if (error < 0) {
    // STARBOARD side, add characters after the "|"
    returnVal = "|" + String('-', maxChars - scaledError) + String('=', scaledError);
  }

  return returnVal;
}

double calculateCourseError(double target, double current) {
  double error = target - current;
  if (error > 180) {
    error -= 360;
  } else if (error < -180) {
    error += 360;
  }
  return error;
}

String convertBoolToStr(bool bl) {
  if (bl) {
    return "true";
  } else {
    return "false";
  }
}

void sendToSlave(String str) {
  Wire2.beginTransmission(8); // Start transmission to device with address 8
  Wire2.write(str.c_str()); // Send message
  Wire2.endTransmission();
}

MotorDirection getCourseCorrectionDirection() {
  double error = calculateCourseError(courseSettings.holdHeading, courseSettings.currHeading);
  return (error < 0) ? LEFT : RIGHT;
}

int getTickLength(MotorDirection dir, bool isGoingHome) {
  int tickLength = 0;

  if (dir == RIGHT || dir == RIGHT_NUDGE) {
    tickLength = isGoingHome ? settings.tickLengthRightHome : settings.tickLengthRight;
  } else if (dir == LEFT || dir == LEFT_NUDGE) {
    tickLength = isGoingHome ? settings.tickLengthLeftHome : settings.tickLengthLeft;
  }

  return min(tickLength, settings.tickLengthLimit);
}

//////////////////////
//MOTOR FUNCTIONS
void rudderHome() {
  if (isRudderNotHome()) {
    //DEBUG_PRINTLN("Sending Rudder To Home Position");
    if (isInManualMode()) {
      sendManualHome();
    }

    MotorDirection oppositeDir = rudderStateToOppositeMotorDirection(apSettings.rudderState); // Convert RudderState to MotorDirection
    //DEBUG_PRINT("Turning to: ");
    //DEBUG_PRINTLN(convertMotorDirectionToStr(oppositeDir));

    controlMotor(oppositeDir);

    if (settings.homeMethod == MAGNET) {
      waitUntilHome();
    } else {
      delay(getTickLength(oppositeDir, true)); // Use converted MotorDirection
    }

    controlMotor(STOP);
    apSettings.rudderState = GO_HOME;
    settings.currentManualValue = RudderStateToManualValue(apSettings.rudderState);
  }
}
String convertMotorDirectionToStr(MotorDirection dir) {
  switch (dir) {
  case RIGHT:
    return "RIGHT";
  case RIGHT_NUDGE:
    return "RIGHT NUDGE";
  case LEFT:
    return "LEFT";
  case LEFT_NUDGE:
    return "LEFT NUDGE";
  case STOP:
    return "STOP";
  default:
    DEBUG_PRINT("Unknown: ");
    DEBUG_PRINTLN(dir);
    return "???";
  }
}
String convertRudderStateToStr(RudderState rudderState) {
  switch (rudderState) {
  case GO_LEFT:
    return "P";
  case GO_RIGHT:
    return "S";
  case GO_HOME:
    return "H";
  case POT_PORT:
    return "L";
  case POT_SB:
    return "R";
  }
}
String convertRudderStateToStrOrig(RudderState rudderState) {
  switch (rudderState) {
  case GO_LEFT:
    return "LEFT";
  case GO_RIGHT:
    return "RIGHT";
  case GO_HOME:
    return "HOME";
  case POT_PORT:
    return "PPORT";
  case POT_SB:
    return "PSB";
  }
}
RudderState MotorDirectionToRudderState(MotorDirection dir) {
  switch (dir) {
  case LEFT_NUDGE:
  case LEFT:
    return GO_LEFT;
  case RIGHT_NUDGE:
  case RIGHT:
    return GO_RIGHT;
  case STOP:
    return GO_HOME;
  }
}
int RudderStateToManualValue(RudderState rudderState) {
  switch (rudderState) {
  case GO_LEFT:
    return 2;
  case GO_RIGHT:
    return 8;
  case GO_HOME:
    return 5;
  case POT_PORT:
    return 2;
  case POT_SB:
    return 8;
  }
}
bool isDirectionNudge(MotorDirection dir) {
  return (dir == LEFT_NUDGE || dir == RIGHT_NUDGE);
}
// Helper function to convert RudderState to MotorDirection
MotorDirection rudderStateToMotorDirection(RudderState state) {
  switch (state) {
  case GO_LEFT:
    return LEFT;
  case GO_RIGHT:
    return RIGHT;
  case GO_HOME:
    return STOP;
  case POT_PORT:
    return LEFT;
  case POT_SB:
    return RIGHT;
  }
}
MotorDirection getCoreMotorDirection(MotorDirection dir) {
  switch (dir) {
  case LEFT_NUDGE:
  case LEFT:
    return LEFT;
  case RIGHT_NUDGE:
  case RIGHT:
    return RIGHT;
  case STOP:
  default:
    return STOP;
  }
}
MotorDirection getOppositeMotorDirection(MotorDirection dir) {
  switch (dir) {
  case LEFT_NUDGE:
  case LEFT:
    return RIGHT;
  case RIGHT_NUDGE:
  case RIGHT:
    return LEFT;
  case STOP:
  default:
    return STOP;
  }
}
MotorDirection rudderStateToOppositeMotorDirection(RudderState state) {
  switch (state) {
  case GO_LEFT:
    return RIGHT;
  case GO_RIGHT:
    return LEFT;
  case GO_HOME:
    return STOP;
  case POT_PORT:
    return RIGHT;
  case POT_SB:
    return LEFT;
  }
}
void handleMotorMove(MotorDirection dir) {
  if (isRudderHome()) {
    //move rudder in dir
    controlMotor(dir);
    delay(getTickLength(dir, false));
    controlMotor(STOP);

    //set ruddetState
    apSettings.rudderState = MotorDirectionToRudderState(dir);
    settings.currentManualValue = RudderStateToManualValue(apSettings.rudderState);

    //if nudge, then bring back to home
    if (isDirectionNudge(dir)) {
      delay(settings.pauseLength);

      MotorDirection oppositeDir = getOppositeMotorDirection(dir);
      controlMotor(oppositeDir);
      if (settings.homeMethod == MAGNET) {
        waitUntilHome();
      } else {
        delay(getTickLength(oppositeDir, true));
      }
      controlMotor(STOP);
      apSettings.rudderState = GO_HOME;
      settings.currentManualValue = RudderStateToManualValue(apSettings.rudderState);
    }
  }
}

void controlMotor(MotorDirection dir) {

  //Serial.println(convertMotorDirectionToStr(dir));
  dir = getCoreMotorDirection(dir);
  //Serial.print("2.");
  //Serial.println(convertMotorDirectionToStr(dir));

  switch (dir) {
  case RIGHT:
    DEBUG_PRINTLN(">>RIGHT");
    sendToSlave("S");
    break;
  case LEFT:
    DEBUG_PRINTLN("<<LEFT");
    sendToSlave("P");
    break;
  case STOP:
    DEBUG_PRINTLN("XSTOPX");
    sendToSlave("X");
    break;
  }
}

// Function to wait until the rudder reaches home position using a magnet sensor
void waitUntilHome() {
  int cnt = 0;
  unsigned long startMillis = millis();
  bool ok = true;

  while (ok) {
    int magVal = analogRead(0);
    //DEBUG_PRINT("*mag value: ");
    //DEBUG_PRINTLN(magVal);

    if (magVal <= settings.magnetAnalogThreshold) {
      cnt++;
    }

    if (cnt >= settings.magnetCntThreshold) {
      //DEBUG_PRINT("Magnet count threshold reached: ");
      //DEBUG_PRINTLN(cnt);
      ok = false;
    }

    if ((millis() - startMillis) > settings.tickLengthLimit) {
      //DEBUG_PRINTLN("Tick length limit reached.");
      ok = false;
    }

    delay(50);
  }
}
void getHeading() {
  if (apSettings.LEDEnabled) {
    toggleLED();
  }

  updatePositionAndHeading();
  handleHeadingHold();
  updateWebpageCompass();
  //sendCompassData();
  updateCourseErrorDisplay();
  //sendCompassData(client);
  updateDisplay();
  handleCourseCorrection();
}

bool BNO055NotDetected(){
  return !apSettings.BNO055Detected;
}

bool BNO055Detected(){
  return apSettings.BNO055Detected;
}

int generateNextHeading(int heading) {
  // Generate a random change within the MAX_CHANGE range
  int change = random(-MAX_CHANGE, MAX_CHANGE + 1);
  
  // Calculate the new heading
  int newHeading = heading + change;
  
  // Ensure the heading stays within 0-359 range
  if (newHeading < MIN_HEADING) {
    newHeading += 360;
  } else if (newHeading > MAX_HEADING) {
    newHeading -= 360;
  }
  
  return newHeading;
}

String formatHeading(int heading) {
  // Format the heading as a 3-digit string with leading zeros
  char buffer[4];
  sprintf(buffer, "%03d", heading);
  return String(buffer);
}

void updatePositionAndHeading() {
  if (BNO055Detected()) {
    sensors_event_t orientationData, linearAccelData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

    updatePosition(linearAccelData);
    updateHeading(orientationData, linearAccelData);  
  } else {
    courseSettings.currHeading = generateNextHeading(courseSettings.currHeading);    
  } 
    
}

void updatePosition(const sensors_event_t& linearAccelData) {
  courseSettings.xPos += ACCEL_POS_TRANSITION * linearAccelData.acceleration.x;
  courseSettings.yPos += ACCEL_POS_TRANSITION * linearAccelData.acceleration.y;
}

void updateHeading(const sensors_event_t& orientationData, const sensors_event_t& linearAccelData) {
  courseSettings.headingVel = ACCEL_VEL_TRANSITION * linearAccelData.acceleration.x / cos(DEG_2_RAD * orientationData.orientation.x);
  courseSettings.currHeading = orientationData.orientation.x;
}

void handleHeadingHold() {
  if (courseSettings.okToHoldHeading) {
    courseSettings.holdHeading = courseSettings.currHeading;
    courseSettings.okToHoldHeading = false;
  }
}
void sendCompassData(WiFiClient &client) {
    double currentHeading = courseSettings.currHeading;
    double targetHeading = courseSettings.holdHeading;  // Only send target if autopilot is on
    int courseError = calculateCourseError(targetHeading, currentHeading);

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/event-stream");
    client.println("Cache-Control: no-cache");
    client.println("Connection: keep-alive");
    client.println();

    Serial.println("Error:");
    Serial.println(courseSettings.courseError);

    String data = "data: {";
    data += "\"currentCourse\": " + String(courseSettings.currHeading) + ",";
    data += "\"targetCourse\": " + (String(courseSettings.holdHeading) >= 0 ? String(String(courseSettings.holdHeading)) : "null") + ",";  // Send null if no target
    data += "\"courseError\": " + String(courseSettings.courseError);
    data += "}";
    
    client.println(data);
    client.println();
}
void updateWebpageCompass() {
  String jsonData = "{\"courseError\":\"" + String(courseSettings.courseError) + 
                      "\",\"currentCourse\":" + String(courseSettings.currHeading) + 
                      ",\"targetCourse\":" + String(courseSettings.holdHeading) + "}";
    sendSSEUpdate(jsonData);
}
void updateDisplay() {
  sendDiagToDebug();

  if (isAPOn()) {
    //updateWebpageCompass();
    updateCourseErrorDisplay();

    int ce = calculateCourseError(courseSettings.holdHeading, courseSettings.currHeading);
    String newCourseErrorStr = valueToArrowString(ce);

    drawCompass(courseSettings.currHeading, courseSettings.holdHeading, apSettings.ipAddress, newCourseErrorStr, convertRudderStateToStr(apSettings.rudderState));    
  } else {
    //sendToDisplay("AP READY", apSettings.ipAddress);
    drawCompass(courseSettings.currHeading, -1, apSettings.ipAddress, "", convertRudderStateToStr(apSettings.rudderState));
  }
}

void updateCourseErrorDisplay() {
  String newCourseErrorStr = getCourseErrorStr();
  if (newCourseErrorStr != courseSettings.courseErrorStr) {
    courseSettings.courseErrorStr = newCourseErrorStr;
    sendSSEUpdate(courseSettings.courseErrorStr);
  }
}

void handleCourseCorrection() {
  if (needToCourseCorrect()) {
    if (!courseSettings.correctingCourse) {
      startCourseCorrection();
    }
  } else if (courseSettings.correctingCourse) {
    endCourseCorrection();
  }
}

void startCourseCorrection() {
  MotorDirection correctionDir = getCourseCorrectionDirection();
  courseSettings.correctingCourse = true;
  handleMotorMove(correctionDir);
}

void endCourseCorrection() {
  rudderHome();
  courseSettings.correctingCourse = false;
}

void drawCompass(int currentHeading, int targetHeading, const String& ipAddress, const String& additionalInfo, String rudderState) {
  display.clearDisplay();

  //Serial.println(currentHeading);
  //Serial.println(targetHeading);
  
  // Define the center of the compass
  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2;
  int radius = min(SCREEN_WIDTH, SCREEN_HEIGHT) / 2 - 2;

  // Draw the compass circle
  display.drawCircle(centerX, centerY, radius, SSD1306_WHITE);

  // Draw the cardinal directions inside the circle
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(centerX - 3, centerY - radius + 2);
  display.print("N");
  display.setCursor(centerX - 3, centerY + radius - 8);
  display.print("S");
  display.setCursor(centerX + radius - 8, centerY - 3);
  display.print("E");
  display.setCursor(centerX - radius + 2, centerY - 3);
  display.print("W");

  // Function to draw a needle
  auto drawNeedle = [&](int heading, bool isCurrent) {
    float needleAngle = heading * PI / 180.0;
    int needleLength = radius - 4;
    int needleEndX = centerX + round(sin(needleAngle) * needleLength);
    int needleEndY = centerY - round(cos(needleAngle) * needleLength);

    if (isCurrent) {
      // Solid line for current heading
      display.drawLine(centerX, centerY, needleEndX, needleEndY, SSD1306_WHITE);
    } else {
      // Dashed line for target heading
      for (int i = 0; i < needleLength; i += 4) {
        int x = centerX + round(sin(needleAngle) * i);
        int y = centerY - round(cos(needleAngle) * i);
        display.drawPixel(x, y, SSD1306_WHITE);
      }
    }
  };

  // Draw both needles
  drawNeedle(currentHeading, true);  // Current heading (solid)
  if (targetHeading > -1) {
    drawNeedle(targetHeading, false);  // Target heading (dashed)
  }

  // Draw a small circle at the center
  display.fillCircle(centerX, centerY, 2, SSD1306_WHITE);

  // Display current heading
  display.setCursor(0, 0);
  display.print("C:");
  display.print(currentHeading);
  display.print((char)247);

  // Display target heading and rudder state
  if (targetHeading > -1) {
    display.setCursor(SCREEN_WIDTH - 30, 0);
    display.print("T:");
    display.print(targetHeading);
    display.print((char)247);
  }

    // Display rudder state below target heading
    display.setCursor(SCREEN_WIDTH - 8, 8);
    display.print(rudderState);

  if (targetHeading > -1) {
    // Calculate and display the difference between current and target heading
    int difference = currentHeading - targetHeading;
    // Normalize the difference to be between -180 and 180
    if (difference > 180) difference -= 360;
    if (difference < -180) difference += 360;

    display.setCursor(0, SCREEN_HEIGHT - 8);
    display.print("D:");
    if (difference < 0) display.print("+");
    else if (difference > 0) display.print("-");
    display.print(abs(difference));
    display.print((char)247);
  }

  // Display IP address in 4 parts
  int ipStartY = 12; // Start below the current heading
  int ipEndY = SCREEN_HEIGHT - 10; // End above the difference indicator
  int ipHeight = ipEndY - ipStartY;
  int chunkHeight = 8; // Height of each IP part
  int totalChunksHeight = 4 * chunkHeight; // Always 4 parts
  int startY = ipStartY + (ipHeight - totalChunksHeight) / 2; // Center vertically

  // Split IP address by periods
  int lastDot = -1;
  int dotCount = 0;
  for (int i = 0; i < 4; i++) {
    int nextDot = ipAddress.indexOf('.', lastDot + 1);
    String part;
    if (nextDot == -1) {
      part = ipAddress.substring(lastDot + 1);
    } else {
      part = ipAddress.substring(lastDot + 1, nextDot);
    }
    display.setCursor(0, startY + i * chunkHeight);
    display.print(part);
    lastDot = nextDot;
    if (nextDot == -1) break;
  }

  if (targetHeading > -1) {
    // Display additional information in the lower right corner
    display.setTextSize(1);
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(additionalInfo, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(SCREEN_WIDTH - w, SCREEN_HEIGHT - h);
    display.print(additionalInfo);
  }

  display.display();
}

void executeMotorTest() {
  sendToSlave("I");
}

void setupWiFiAP() {
  // Set up Access Point
  Serial.print("Setting up Access Point ...");
  WiFi.beginAP(ssid, password);

  sendToDisplay("AP Setup ...");

  while (WiFi.status() != WL_AP_LISTENING) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("Access Point started");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  apSettings.ipAddress = ipToString(WiFi.localIP());

  String line1 = "AP Ready";
  String line2 = apSettings.ipAddress;
  sendToDisplay(line1, line2);  

  //SendToDisplay("AP Ready",apSettings.ipAddress);

  server.begin();
}

// Convert IPAddress to String
String ipToString(IPAddress ip) {
  return String(ip[0]) + "." +
    String(ip[1]) + "." +
    String(ip[2]) + "." +
    String(ip[3]);
}

void parseMoveArg(String arg) {
  String cmd = "";
  int value = 0;

  int pipeIndex = arg.indexOf('|'); // Find the index of the pipe symbol

  if (pipeIndex != -1) { // Ensure there is a pipe in the argument
    // Extract the command part (before the pipe)
    cmd = arg.substring(0, pipeIndex);

    // Extract the value part (after the pipe) and convert it to an integer
    String valueStr = arg.substring(pipeIndex + 1);
    value = valueStr.toInt(); // Convert to integer
  } else {
    // If no pipe found, default cmd to the whole arg and value to 0
    cmd = arg;
    value = 0;
  }
}
void evaluateWiFiClient() {
    WiFiClient client = server.available();
    if (!client) return;

    DEBUG_PRINTLN("New client connected");
    String request = "";
    constexpr unsigned long REQUEST_TIMEOUT = 1000; // 1 second timeout
    unsigned long requestStartTime = millis();

    while (client.connected() && millis() - requestStartTime < REQUEST_TIMEOUT) {
        if (client.available()) {
            char c = client.read();
            request += c;
            if (c == '\n' && request.endsWith("\r\n\r\n")) {
                DEBUG_PRINTLN("Request received: " + request);
                handleRequest(client, request);
                break;
            }
        }
    }

    if (client.connected()) {
        client.stop();
    }
    DEBUG_PRINTLN("Client disconnected");
}

void handleRequest(WiFiClient& client, const String& request) {
    if (request.indexOf("GET /?arg=") >= 0) {
        handleButtonPress(client, request);
    } else if (request.indexOf("GET /events") >= 0) {
        handleSSE(client);
        sendCompassData(client);
    } else if (request.indexOf("GET /update") >= 0) {
        handleManualUpdate(client);
    } else if (request.indexOf("GET /moveMotor") >= 0) {
        handleMoveMotor(client, request);
    } else if (request.indexOf("GET /save") >= 0) {
        handleSaveSettings(client, request);
    } else {
        sendHTMLResponse(client);
    }
}

void handleButtonPress(WiFiClient& client, const String& request) {
    // Extract "arg" value from the request string.
    int argStartIndex = request.indexOf("arg=") + 4; // Start after "arg="
    int argEndIndex = request.indexOf(" ", argStartIndex); // Look for space after "arg"

    DEBUG_PRINT("request=");
    DEBUG_PRINTLN(request);

    DEBUG_PRINT("argStartIndex=");
    DEBUG_PRINTLN(argStartIndex);

    DEBUG_PRINT("argEndIndex=");
    DEBUG_PRINTLN(argEndIndex);

    // If no space is found, assume the argument is the last part of the request
    if (argEndIndex == -1) {
        argEndIndex = request.length();
    }

    // Get the actual argument
    String arg = request.substring(argStartIndex, argEndIndex);

    // Debugging print to see the extracted argument
    DEBUG_PRINTLN(arg + " button pressed");

    // Handle button actions based on extracted "arg"
    if (arg == "PT") portClick();
    else if (arg == "PTN") portNudgeClick();
    else if (arg == "SET") setClick();
    else if (arg == "SBN") starboardNudgeClick();
    else if (arg == "SB") starboardClick();

    // Send a simple response back to the client
    sendSimpleResponse(client, "200 OK", "text/html");
}

DirectionValue parseCommand(String arg) {
  DirectionValue dv;
  dv.direction = RIGHT; // Default direction
  dv.value = 0;

  // Split the string at '=' to separate the command and the value
  int equalIndex = arg.indexOf('=');

  if (equalIndex != -1) {
    // Get the value part after '='
    String valuePart = arg.substring(equalIndex + 1); // Everything after '=' is the command and value
    
    // Extract the last character as the value
    char lastChar = valuePart.charAt(valuePart.length() - 1);
    int value = lastChar - '0'; // Convert last character to integer
    
    // The rest is the command (everything except the last character)
    String command = valuePart.substring(0, valuePart.length() - 1);
    
    // Debug prints
    Serial.print("Command: ");
    Serial.println(command);
    Serial.print("Value: ");
    Serial.println(value);

    // Set direction based on the command
    if (command == "PT") {
      dv.direction = LEFT;
    } else if (command == "SB") {
      dv.direction = RIGHT;
    }

    // Set the value in the struct
    dv.value = value;
  } else {
    Serial.println("Invalid argument format");
  }

  return dv;
}

void handleManualUpdate(WiFiClient& client) {
    sendSimpleResponse(client, "200 OK", "text/plain", "Manual update triggered");
    String manualData = "Manual update at: " + String(millis());
    sendSSEUpdate(manualData);
    DEBUG_PRINTLN("Manual update sent");
}

void handleMoveMotor(WiFiClient& client, const String& request) {
    int directionStart = request.indexOf("direction=") + 10;
    int directionEnd = request.indexOf("&", directionStart);
    String direction = request.substring(directionStart, directionEnd);

    int timeStart = request.indexOf("time=") + 5;
    int timeEnd = request.indexOf(" ", timeStart);
    if (timeEnd == -1) timeEnd = request.indexOf("\r\n", timeStart);
    int moveTime = request.substring(timeStart, timeEnd).toInt();

    DEBUG_PRINT("Direction: ");
    DEBUG_PRINT(direction);
    DEBUG_PRINT(" Time: ");
    DEBUG_PRINTLN(moveTime);

    if (moveTime > 0) {
        if (direction == "left") moveMotor(LEFT, moveTime);
        else if (direction == "right") moveMotor(RIGHT, moveTime);
        else DEBUG_PRINTLN("Invalid direction parameter");
    } else {
        DEBUG_PRINTLN("Invalid time parameter");
    }

    String response = "Motor moved in direction: " + direction + " for " + String(moveTime) + " ms";
    sendSimpleResponse(client, "200 OK", "text/plain", response.c_str()); // Fixed line
}

void handleSaveSettings(WiFiClient& client, const String& request) {
    int paramStart = request.indexOf("?") + 1;
    int valueStart = request.indexOf("=", paramStart) + 1;
    int paramEnd = valueStart - 1;

    if (paramStart > 0 && valueStart > paramStart) {
        String paramName = request.substring(paramStart, paramEnd);
        int valueEnd = request.indexOf(" ", valueStart);
        if (valueEnd == -1) valueEnd = request.indexOf("\r\n", valueStart);
        String paramValue = request.substring(valueStart, valueEnd);
        int paramValInt = paramValue.toInt();

        saveChangedSetting(paramName, paramValInt);
        sendSimpleResponse(client, "200 OK", "text/plain", "Value saved");

        String updateData = "Parameter " + paramName + " updated to: " + paramValue;
        sendSSEUpdate(updateData);
    }
}

void handleSSE(WiFiClient & client) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/event-stream");
  client.println("Cache-Control: no-cache");
  client.println("Connection: keep-alive");
  client.println();
  client.flush(); 
  sseClients.push_back(client);
  Serial.println("New SSE client connected");
}

void sendSSEUpdate(const String & data) {
  for (auto it = sseClients.begin(); it != sseClients.end();) {
    if (it -> connected()) {
      it -> print("data: ");
      it -> println(data);
      it -> println();
      it -> flush();
      ++it;
    } else {
      Serial.println("SSE client disconnected");
      it = sseClients.erase(it);
    }
  }

/*
  // Check if too many clients are connected, and drop the oldest ones
  const int maxClients = 1;
  if (sseClients.size() > maxClients) {
    sseClients.erase(sseClients.begin());
    Serial.println("Dropped oldest SSE client due to exceeding max connections.");
  }
  */
}
/*
void sendHTMLResponseNew(WiFiClient & client) {
  client.println("HTTP/1.1 200 OK");
client.println("Content-Type: text/html");
client.println();

client.println("<!DOCTYPE html>");
client.println("<html lang='en'>");
client.println("<head>");
client.println("<meta charset='UTF-8'>");
client.println("<meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no'>");
client.println("<title>Nautical Nonsense Autopilot</title>");
client.println("<link href='https://fonts.googleapis.com/css2?family=Roboto:wght@400;700&display=swap' rel='stylesheet'>");
client.println("<style>");
client.println(":root { --primary-color: #0077be; --secondary-color: #f0f8ff; --accent-color: #ffd700;");
client.println("--text-color: #333; --background-color: #e6f2ff; }");
client.println("body { font-family: 'Roboto', sans-serif; background-color: var(--background-color);");
client.println("color: var(--text-color); padding: 20px; margin: 0; min-height: 100vh; }");
client.println(".container { max-width: 600px; margin: 0 auto; }");
client.println("h1, h2 { color: var(--primary-color); text-align: center; }");
client.println(".compass-container { display: flex; justify-content: center; margin-bottom: 20px; }");
client.println("#compassCanvas { background-color: var(--secondary-color); border-radius: 50%;");
client.println("box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); }");
client.println(".control-group { background-color: var(--secondary-color); border-radius: 10px;");
client.println("padding: 15px; margin-bottom: 20px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); }");
client.println(".button-group { display: flex; flex-wrap: wrap; justify-content: center; gap: 10px; }");
client.println("button { background-color: var(--primary-color); color: white; border: none;");
client.println("padding: 15px 20px; font-size: 16px; border-radius: 5px; cursor: pointer;");
client.println("transition: background-color 0.3s; min-width: 60px; }");
client.println("button:hover { background-color: #005c91; }");
client.println("input[type='number'] { width: 80px; padding: 10px; font-size: 16px; border: 1px solid var(--primary-color);");
client.println("border-radius: 5px; }");
client.println("#courseErrorDisplay { font-size: 24px; font-weight: bold; text-align: center;");
client.println("margin-bottom: 20px; color: var(--accent-color); background-color: var(--primary-color);");
client.println("padding: 10px; border-radius: 5px; }");
client.println(".input-group { display: flex; align-items: center; justify-content: space-between; margin-bottom: 10px; }");
client.println(".input-group label { flex: 1; }");
client.println(".input-group input { flex: 0 0 80px; }");
client.println(".input-group button { flex: 0 0 60px; }");
client.println("</style>");
client.println("</head>");
client.println("<body>");
client.println("<div class='container'>");
client.println("<h1>Nautical Nonsense Autopilot</h1>");
client.println("<div class='compass-container'>");
client.println("<canvas id='compassCanvas' width='250' height='250'></canvas>");
client.println("</div>");
client.println("<div id='courseErrorDisplay'></div>");

client.println("<div class='control-group'>");
client.println("<h2>Steering Control</h2>");
client.println("<div class='button-group'>");
client.println("<button onclick='fetch(\"/?arg=PT\")'>&lt;&lt;</button>");
client.println("<button onclick='fetch(\"/?arg=PTN\")'>&lt;</button>");
client.println("<button onclick='fetch(\"/?arg=SET\")'>SET</button>");
client.println("<button onclick='fetch(\"/?arg=SBN\")'>&gt;</button>");
client.println("<button onclick='fetch(\"/?arg=SB\")'>&gt;&gt;</button>");
client.println("</div>");
client.println("</div>");

client.println("<div class='control-group'>");
client.println("<h2>Fine Steering Control</h2>");
client.println("<div class='button-group'>");
client.println("<button onclick='fetch(\"/?arg=PT4\")'>&lt;&lt;&lt;&lt;</button>");
client.println("<button onclick='fetch(\"/?arg=PT3\")'>&lt;&lt;&lt;</button>");
client.println("<button onclick='fetch(\"/?arg=PT2\")'>&lt;&lt;</button>");
client.println("<button onclick='fetch(\"/?arg=PT1\")'>&lt;</button>");
client.println("<button onclick='fetch(\"/?arg=HOME\")'>HOME</button>");
client.println("<button onclick='fetch(\"/?arg=SB1\")'>&gt;</button>");
client.println("<button onclick='fetch(\"/?arg=SB2\")'>&gt;&gt;</button>");
client.println("<button onclick='fetch(\"/?arg=SB3\")'>&gt;&gt;&gt;</button>");
client.println("<button onclick='fetch(\"/?arg=SB4\")'>&gt;&gt;&gt;&gt;</button>");
client.println("</div>");
client.println("</div>");

client.println("<div class='control-group'>");
client.println("<h2>Custom Move</h2>");
client.println("<div class='button-group'>");
client.println("<button onclick='moveMotor(\"left\")'>&lt;</button>");
client.println("<input type='number' id='moveInput' value='500'>");
client.println("<button onclick='moveMotor(\"right\")'>&gt;</button>");
client.println("</div>");
client.println("</div>");

client.println("<div class='control-group'>");
client.println("<h2>Manual Steering</h2>");
client.println("<div id='manualSteeringInputs'></div>");
client.println("</div>");

client.println("<div class='control-group'>");
client.println("<h2>Tick Lengths</h2>");
client.println("<div id='tickLengthInputs'></div>");
client.println("</div>");

client.println("<div class='control-group'>");
client.println("<h2>Heading Threshold</h2>");
client.println("<div class='input-group'>");
client.println("<label for='HT'>Heading Threshold:</label>");
client.println("<input type='number' id='HT' value='"); client.println(settings.headingThreshold); client.println("'>");
client.println("<button onclick='saveValue(\"HT\", \"HT\")'>Save</button>");
client.println("</div>");
client.println("</div>");

client.println("</div>");
client.println("<script>");
client.println("let currentCourse = 0; let targetCourse = 0;");
//int courseError = calculateCourseError(targetHeading, currentHeading);
client.println("function drawCompass(currentCourse, targetCourse, courseError) {");
client.println("const canvas = document.getElementById('compassCanvas');");
client.println("const ctx = canvas.getContext('2d'); const centerX = canvas.width / 2;");
client.println("const centerY = canvas.height / 2; const radius = canvas.width / 2 - 10;");
client.println("ctx.clearRect(0, 0, canvas.width, canvas.height); ctx.beginPath();");
client.println("ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI); ctx.strokeStyle = '#0077be'; ctx.lineWidth = 3;");
client.println("ctx.stroke(); ctx.font = 'bold 16px Roboto'; ctx.fillStyle = '#0077be';");
client.println("ctx.textAlign = 'center'; ctx.textBaseline = 'middle'; ctx.fillText('N', centerX, centerY - radius + 20);");
client.println("ctx.fillText('E', centerX + radius - 20, centerY); ctx.fillText('S', centerX, centerY + radius - 20);");
client.println("ctx.fillText('W', centerX - radius + 20, centerY); for (let i = 0; i < 360; i += 30) {");
client.println("const angle = i * Math.PI / 180; const x1 = centerX + (radius - 10) * Math.cos(angle);");
client.println("const y1 = centerY + (radius - 10) * Math.sin(angle); const x2 = centerX + radius * Math.cos(angle);");
client.println("const y2 = centerY + radius * Math.sin(angle); ctx.beginPath(); ctx.moveTo(x1, y1);");
client.println("ctx.lineTo(x2, y2); ctx.strokeStyle = '#0077be'; ctx.lineWidth = 2; ctx.stroke(); }");
client.println("drawNeedle(ctx, centerX, centerY, radius - 20, currentCourse, '#ffd700', 4);");
client.println("drawNeedle(ctx, centerX, centerY, radius - 40, targetCourse, '#ff4500', 2);");
client.println("ctx.beginPath(); ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI); ctx.fillStyle = '#0077be'; ctx.fill(); }");

client.println("function drawNeedle(ctx, centerX, centerY, length, angle, color, width) {");
client.println("const radians = (angle - 90) * Math.PI / 180; ctx.beginPath(); ctx.moveTo(centerX, centerY);");
client.println("ctx.lineTo(centerX + length * Math.cos(radians), centerY + length * Math.sin(radians));");
client.println("ctx.strokeStyle = color; ctx.lineWidth = width; ctx.stroke(); }");

client.println("const evtSource = new EventSource('/events');");

client.println("evtSource.onmessage = function(event) {");
client.println("    const data = JSON.parse(event.data);");
client.println("    document.getElementById('courseErrorDisplay').innerText = `Course Error: ${data.courseError}°`;");
client.println("    // Update the compass");
client.println("    currentCourse = data.currentCourse;");
client.println("    targetCourse = data.targetCourse !== null ? data.targetCourse : currentCourse;  // If no target, only show current");
client.println("    drawCompass(currentCourse, targetCourse,courseError);");
client.println("};");

client.println("evtSource.onerror = function(error) {");
client.println("    console.error('EventSource failed:', error);");
client.println("};");

//client.println("// Draw initial compass state");

//client.println("drawCompass(0, 0);");
client.println("function moveMotor(direction) {");
client.println("const moveTime = document.getElementById('moveInput').value;");
client.println("fetch('/moveMotor?' + 'direction=' + direction + '&time=' + moveTime).then(response => {");
client.println("if (response.ok) { return response.text(); } else { throw new Error('Network response was not ok'); }");
client.println("}).then(data => { console.log('Motor moved in direction ' + direction + ' for ' + moveTime + ' ms');");
client.println("console.log('Response:', data); }).catch(error => {");
client.println("console.error('There was a problem with the fetch operation:', error); }); }");

client.println("function saveValue(param, id) {");
client.println("const value = document.getElementById(id).value;");
client.println("fetch('/save?' + param + '=' + value).then(response => {");
client.println("if (response.ok) { return response.text(); } else { throw new Error('Network response was not ok'); }");
client.println("}).then(data => { console.log('Value ' + param + ' saved: ' + value);");
client.println("console.log('Response:', data); }).catch(error => {");
client.println("console.error('There was a problem with the fetch operation:', error); }); }");

client.println("const manualSteeringDiv = document.getElementById('manualSteeringInputs');");
client.println("for (let i = 1; i <= 4; i++) { const inputGroup = document.createElement('div');");
client.println("inputGroup.className = 'input-group'; inputGroup.innerHTML = `<label for='MAN${i}'>Manual ${i}:</label>");
client.println("<input type='number' id='MAN${i}' value='0'><button onclick='saveValue(\"MAN${i}\", \"MAN${i}\")'>Save</button>`;");
client.println("manualSteeringDiv.appendChild(inputGroup); }");

client.println("const tickLengthDiv = document.getElementById('tickLengthInputs');");
client.println("const tickLengths = ['HTP', 'PTH', 'HTS', 'STH', 'NP', 'TL'];");
client.println("const tickLengthLabels = { 'HTP': 'HOME -> PORT:', 'PTH': 'PORT -> HOME:', 'HTS': 'HOME -> SB:', 'STH': 'SB -> HOME:', 'NP': 'Nudge Pause:', 'TL': 'Tick Limit:' };");
client.println("for (const tick of tickLengths) { const inputGroup = document.createElement('div'); inputGroup.className = 'input-group';");
client.println("inputGroup.innerHTML = `<label for='${tick}'>${tickLengthLabels[tick]}</label><input type='number' id='${tick}' value='0'><button onclick='saveValue(\"${tick}\", \"${tick}\")'>Save</button>`;");
client.println("tickLengthDiv.appendChild(inputGroup); }");
client.println("</script>");
client.println("</body>");
client.println("</html>");

}
*/
void sendHTMLResponse(WiFiClient & client) {
  if (!client.connected()) return;

  client.println(F("<!DOCTYPE html><html lang='en'>"));
  client.println(F("<head>"));
  sendHtmlHead(client); // Function to send the head content including meta tags and CSS
  sendJavaScript(client); // Function to send JavaScript for handling various tasks
  client.println(F("</head>"));
  client.println(F("<body>"));
  sendHtmlBody(client); // Function to send the main body content
  client.println(F("</body></html>"));
}

void sendHtmlHead(WiFiClient & client) {
  client.println(F("<meta charset='UTF-8'>"));
  client.println(F("<meta name='viewport' content='width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no'>"));
  client.println(F("<title>Nautical Nonsense Autopilot</title>"));
  client.println(F("<link href='https://fonts.googleapis.com/css2?family=Roboto:wght@400;700&display=swap' rel='stylesheet'>"));
  client.println(F("<style>"));
  client.println(F(":root { --primary-color: #0077be; --secondary-color: #f0f8ff; --accent-color: #ffd700;"));
  client.println(F("--text-color: #333; --background-color: #e6f2ff; }"));
  client.println(F("body { font-family: 'Roboto', sans-serif; background-color: var(--background-color);"));
  client.println(F("color: var(--text-color); padding: 20px; margin: 0; min-height: 100vh; }"));
  client.println(F(".container { max-width: 600px; margin: 0 auto; }"));
  client.println(F("h1, h2 { color: var(--primary-color); text-align: center; }"));
  client.println(F(".compass-container { display: flex; justify-content: center; margin-bottom: 20px; }"));
  client.println(F("#compassCanvas { background-color: var(--secondary-color); border-radius: 50%;"));
  client.println(F("box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); }"));
  client.println(F(".control-group { background-color: var(--secondary-color); border-radius: 10px;"));
  client.println(F("padding: 15px; margin-bottom: 20px; box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1); }"));
  client.println(F(".button-group { display: flex; flex-wrap: wrap; justify-content: center; gap: 10px; }"));
  client.println(F("button { background-color: var(--primary-color); color: white; border: none;"));
  client.println(F("padding: 15px 20px; font-size: 16px; border-radius: 5px; cursor: pointer;"));
  client.println(F("transition: background-color 0.3s; min-width: 60px; }"));
  client.println(F("button:hover { background-color: #005c91; }"));
  client.println(F("input[type='number'] { width: 80px; padding: 10px; font-size: 16px; border: 1px solid var(--primary-color);"));
  client.println(F("border-radius: 5px; }"));
  client.println(F("#courseErrorDisplay { font-size: 24px; font-weight: bold; text-align: center;"));
  client.println(F("margin-bottom: 20px; color: var(--accent-color); background-color: var(--primary-color);"));
  client.println(F("padding: 10px; border-radius: 5px; }"));
  client.println(F("</style>"));
}

void sendHtmlBody(WiFiClient & client) {
  client.println(F("<div class='container'>"));
  client.println(F("<h1>Nautical Nonsense Autopilot</h1>"));
  
  sendCompassCanvas(client);
  sendCourseErrorSection(client);
  sendSteeringControls(client);
  sendFineSteeringControls(client);
  sendCustomMove(client);
  sendManualSteering(client);
  sendTickLengths(client);
  sendHeadingThreshold(client);

  client.println(F("</div>"));
}

void sendCompassCanvas(WiFiClient & client) {
  client.println(F("<div class='compass-container'>"));
  client.println(F("<canvas id='compassCanvas' width='250' height='250'></canvas>"));
  client.println(F("</div>"));
}

void sendCourseErrorSection(WiFiClient & client) {
  client.println(F("<div id='courseErrorDisplay'></div>"));
}

void sendSteeringControls(WiFiClient & client) {
  client.println(F("<div class='control-group'><h2>Steering Control</h2><div class='button-group'>"));
  sendButton(client, "PT", "<<");
  sendButton(client, "PTN", "<");
  sendButton(client, "SET", "SET");
  sendButton(client, "SBN", ">");
  sendButton(client, "SB", ">>");
  client.println(F("</div></div>"));
}

void sendFineSteeringControls(WiFiClient & client) {
  client.println(F("<div class='control-group'><h2>Fine Steering Control</h2><div class='button-group'>"));
  sendButton(client, "PT4", "<<<<");
  sendButton(client, "PT3", "<<<");
  sendButton(client, "PT2", "<<");
  sendButton(client, "PT1", "<");
  sendButton(client, "HOME", "HOME");
  sendButton(client, "SB1", ">");
  sendButton(client, "SB2", ">>");
  sendButton(client, "SB3", ">>>");
  sendButton(client, "SB4", ">>>>");
  client.println(F("</div></div>"));
}

void sendCustomMove(WiFiClient & client) {
  client.println(F("<div class='control-group'><h2>Custom Move</h2><div class='button-group'>"));
  client.println(F("<button onclick='moveMotor(\"left\")'>&lt;</button>"));
  client.println(F("<input type='number' id='moveInput' value='500'>"));
  client.println(F("<button onclick='moveMotor(\"right\")'>&gt;</button>"));
  client.println(F("</div></div>"));
}
void sendInputWithSave(WiFiClient & client, const String& id, int value, const String& label = "") {
  client.print(label);
  client.print(F(": <input type='number' id='"));
  client.print(id);
  client.print(F("' value='"));
  client.print(value);
  client.print(F("'> <button onclick='saveValue(\""));
  client.print(id);
  client.print(F("\", \""));
  client.print(id);
  client.println(F("\")'>Save</button><br>"));
}
void sendManualSteering(WiFiClient & client) {
  client.println(F("<div class='control-group'><h2>Manual Steering</h2>"));
  for (int i = 0; i < 4; i++) {
    String str = "manual ";
    str += (i+1);
    sendInputWithSave(client, "MAN" + String(i+1), settings.manualValues[i], str);
  }
  client.println(F("</div>"));
}

void sendTickLengths(WiFiClient & client) {
  client.println(F("<div class='control-group'><h2>Tick Lengths</h2>"));
  sendInputWithSave(client, "HTP", settings.tickLengthLeft, "HOME -> PORT");
  sendInputWithSave(client, "PTH", settings.tickLengthLeftHome, "PORT -> HOME");
  sendInputWithSave(client, "HTS", settings.tickLengthRight, "HOME -> SB");
  sendInputWithSave(client, "STH", settings.tickLengthRightHome, "SB -> HOME");
  sendInputWithSave(client, "NP", settings.pauseLength, "Nudge Pause");
  sendInputWithSave(client, "TL", settings.tickLengthLimit, "Tick Limit");
  client.println(F("</div>"));
}

void sendHeadingThreshold(WiFiClient & client) {
  client.println(F("<div class='control-group'><h2>Heading Threshold</h2>"));
  sendInputWithSave(client, "HT", settings.headingThreshold, "Heading Threshold");
  client.println(F("</div>"));
}

void sendButton(WiFiClient & client, const char* arg, const char* label) {
  client.print(F("<button onclick='fetch(\"/?arg="));
  client.print(arg);
  client.print(F("\")'>"));
  client.print(label);
  client.println(F("</button>"));
}

void sendJavaScript(WiFiClient & client) {
  client.println(F("<script>"));
  includeAllJavaScript(client);
  client.println(F("</script>"));
}

void includeAllJavaScript(WiFiClient & client) {  
//client.println("let currentCourse = 0; let targetCourse = 0;");

client.println("function drawCompass(currentCourse, targetCourse, courseError) {");
client.println("    const canvas = document.getElementById('compassCanvas');");
client.println("    if (!canvas) { console.error('Canvas not found'); return; }");  // Check if canvas exists
client.println("    const ctx = canvas.getContext('2d');");
client.println("    if (!ctx) { console.error('Canvas context not found'); return; }");  // Check if canvas context is available
client.println("    const centerX = canvas.width / 2;");
client.println("    const centerY = canvas.height / 2;");
client.println("    const radius = canvas.width / 2 - 10;");

client.println("    ctx.clearRect(0, 0, canvas.width, canvas.height);");
client.println("    ctx.beginPath();");
client.println("    ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI);");
client.println("    ctx.strokeStyle = '#0077be';");
client.println("    ctx.lineWidth = 3;");
client.println("    ctx.stroke();");

client.println("    ctx.font = 'bold 16px Roboto';");
client.println("    ctx.fillStyle = '#0077be';");
client.println("    ctx.textAlign = 'center';");
client.println("    ctx.textBaseline = 'middle';");

client.println("    // Draw cardinal directions");
client.println("    ctx.fillText('N', centerX, centerY - radius + 20);");
client.println("    ctx.fillText('E', centerX + radius - 20, centerY);");
client.println("    ctx.fillText('S', centerX, centerY + radius - 20);");
client.println("    ctx.fillText('W', centerX - radius + 20, centerY);");

client.println("    // Draw tick marks");
client.println("    for (let i = 0; i < 360; i += 30) {");
client.println("        const angle = i * Math.PI / 180;");
client.println("        const x1 = centerX + (radius - 10) * Math.cos(angle);");
client.println("        const y1 = centerY + (radius - 10) * Math.sin(angle);");
client.println("        const x2 = centerX + radius * Math.cos(angle);");
client.println("        const y2 = centerY + radius * Math.sin(angle);");
client.println("        ctx.beginPath();");
client.println("        ctx.moveTo(x1, y1);");
client.println("        ctx.lineTo(x2, y2);");
client.println("        ctx.strokeStyle = '#0077be';");
client.println("        ctx.lineWidth = 2;");
client.println("        ctx.stroke();");
client.println("    }");

client.println("    // Draw current and target course needles");
client.println("    drawNeedle(ctx, centerX, centerY, radius - 20, currentCourse, '#ffd700', 4);");
client.println("    drawNeedle(ctx, centerX, centerY, radius - 40, targetCourse, '#ff4500', 2);");

client.println("    // Draw center circle");
client.println("    ctx.beginPath();");
client.println("    ctx.arc(centerX, centerY, 5, 0, 2 * Math.PI);");
client.println("    ctx.fillStyle = '#0077be';");
client.println("    ctx.fill();");
client.println("}");

client.println("function drawNeedle(ctx, centerX, centerY, length, angle, color, width) {");
client.println("    const radians = (angle - 90) * Math.PI / 180;");
client.println("    ctx.beginPath();");
client.println("    ctx.moveTo(centerX, centerY);");
client.println("    ctx.lineTo(centerX + length * Math.cos(radians), centerY + length * Math.sin(radians));");
client.println("    ctx.strokeStyle = color;");
client.println("    ctx.lineWidth = width;");
client.println("    ctx.stroke();");
client.println("}");

// EventSource for real-time updates
client.println("const evtSource = new EventSource('/events');");
client.println("evtSource.onmessage = function(event) {");
client.println("    const data = JSON.parse(event.data);");
client.println("    document.getElementById('courseErrorDisplay').innerText = `Course Error: ${data.courseError}°`;");
client.println("    currentCourse = data.currentCourse;");
client.println("    targetCourse = data.targetCourse;");
client.println("    drawCompass(currentCourse, targetCourse);");
client.println("};");

// Initial draw with default values
client.println("window.onload = function() {");
client.println("    let currentCourse = 0;");
client.println("    let targetCourse = 0;");

client.println("    drawCompass(currentCourse, targetCourse,courseError);");
client.println("};");

  includeMoveMotorJavaScript(client);
  includeSaveValueJavaScript(client);
}

void includeMoveMotorJavaScript(WiFiClient & client) {
  client.println("function moveMotor(direction) {");
  client.println("const moveTime = document.getElementById('moveInput').value;");
  client.println("fetch('/moveMotor?' + 'direction=' + direction + '&time=' + moveTime)");
  client.println(".then(response => { if (response.ok) { return response.text(); } else { throw new Error('Network response was not ok'); }})");
  client.println(".then(data => { console.log('Motor moved in direction ' + direction + ' for ' + moveTime + ' ms'); console.log('Response:', data); })");
  client.println(".catch(error => { console.error('There was a problem with the fetch operation:', error); }); }");
}

void includeSaveValueJavaScript(WiFiClient & client) {
  client.println("function saveValue(param, id) {");
  client.println("const value = document.getElementById(id).value;");
  client.println("fetch('/save?' + param + '=' + value).then(response => { if (response.ok) { return response.text(); } else { throw new Error('Network response was not ok'); }})");
  client.println(".then(data => { console.log('Value ' + param + ' saved: ' + value); console.log('Response:', data); })");
  client.println(".catch(error => { console.error('There was a problem with the fetch operation:', error); }); }");
}

void saveChangedSetting(String paramName, int paramValInt) {
  // Update the appropriate variable based on the parameter name
  if (paramName == "MAN1") {
    settings.manualValues[0] = paramValInt;
  } else if (paramName == "MAN2") {
    settings.manualValues[1] = paramValInt;
  } else if (paramName == "MAN3") {
    settings.manualValues[2] = paramValInt;
  } else if (paramName == "MAN4") {
    settings.manualValues[3] = paramValInt;
  } else if (paramName == "HTP") {
    settings.tickLengthLeft = paramValInt;
  } else if (paramName == "PTH") {
    settings.tickLengthLeftHome = paramValInt;
  } else if (paramName == "HTS") {
    settings.tickLengthRight = paramValInt;
  } else if (paramName == "STH") {
    settings.tickLengthRightHome = paramValInt;
  } else if (paramName == "NP") {
    settings.pauseLength = paramValInt;
  } else if (paramName == "TL") {
    settings.tickLengthLimit = paramValInt;
  } else if (paramName == "HT") {
    settings.headingThreshold = paramValInt;
  } else {
    Serial.println("Unknown parameter: " + paramName);
  }
}