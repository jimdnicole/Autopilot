#include <Wire.h>
#include <Arduino.h>

int currentDisplayRow=0;

enum class XYMode { ROWCOL, XCOL, ROWY, XY };
enum class Justification { LEFT, RIGHT };
enum Command {
  ROWCOL,
  YCOL,
  ROWX,
  XY
};

struct ParsedData {
  Command command;
  String lines[3];
  int lineCount;
  int row;
  int col;
  bool clr;
};

// Motor control pins
int enA = 3;
int MOTOR_LEFT = 5;
int MOTOR_RIGHT = 4;

// Motor control commands
const String stopMotor = "X";
const String goLeft = "P";
const String goRight = "S";
const String goInit = "I";

void setup() {
  Serial.begin(115200);         /* start serial for debug */

  pinMode(enA, OUTPUT);
  pinMode(MOTOR_LEFT, OUTPUT);
  pinMode(MOTOR_RIGHT, OUTPUT);
 
  Wire.begin(8);                /* join i2c bus with address 8 */
  Wire.onReceive(receiveEvent); /* register receive event */
 
  //Config_Init();
  //LCD_Init();
  //LCD_SetBacklight(100);

  digitalWrite(MOTOR_LEFT, LOW);
  digitalWrite(MOTOR_RIGHT, LOW);

  ////sendToDisplay("AP READY","192.168.3.1",XYMode::ROWCOL,true);
}

void loop() {
  delay(50);
}

// Function that executes when data is received from master
void receiveEvent(int howMany) {
  String data = "";
  while (0 < Wire.available()) {
    char c = Wire.read();
    data += c;
  }
  Serial.println(data);  // Print the received data
  
  //sendToDisplay(data,0,0,XYMode::ROWCOL,false);
  
  if (data.startsWith("DISP|")) {
    processDisplayCommand(data.substring(5));  // Remove "DISP|" prefix
  } else {
    processMotorCommand(data);
  }
}

void processMotorCommand(const String& command) {
  Serial.println("command received");
  Serial.println(command);

  //sendToDisplay("command ...",command,0,0,XYMode::ROWCOL,false);

  if (command == goRight) {
    digitalWrite(MOTOR_LEFT, LOW);
    digitalWrite(MOTOR_RIGHT, HIGH);
  } else if (command == goLeft) {
    digitalWrite(MOTOR_LEFT, HIGH);
    digitalWrite(MOTOR_RIGHT, LOW);
  } else if (command == stopMotor) {
    digitalWrite(MOTOR_LEFT, LOW);
    digitalWrite(MOTOR_RIGHT, LOW);
  } else if (command == goInit) {
    digitalWrite(MOTOR_LEFT, HIGH);
    delay(1000);
    digitalWrite(MOTOR_LEFT, LOW);  
  } else {
    Serial.println("Unknown motor command: " + command);
  }
}

ParsedData parseString(String input) {
  ParsedData result;
  result.lineCount = 0;

  // Find the positions of the main delimiters
  int cmdEnd = input.indexOf('|');
  int bracketStart = input.indexOf('[');
  int bracketEnd = input.indexOf(']');
  int rowStart = input.indexOf('|', bracketEnd + 1);
  int colStart = input.indexOf('|', rowStart + 1);
  int clrStart = input.indexOf('|', colStart + 1);

  // Parse command
  String cmdStr = input.substring(0, cmdEnd);
  if (cmdStr == "ROWCOL") result.command = ROWCOL;
  else if (cmdStr == "YCOL") result.command = YCOL;
  else if (cmdStr == "ROWX") result.command = ROWX;
  else if (cmdStr == "XY") result.command = XY;
  else result.command = ROWCOL; // Default value if invalid

  // Parse lines
  String linesStr = input.substring(bracketStart + 1, bracketEnd);
  int prevPos = 0;
  int pipePos;
  while ((pipePos = linesStr.indexOf('|', prevPos)) != -1 && result.lineCount < 3) {
    result.lines[result.lineCount++] = linesStr.substring(prevPos, pipePos);
    prevPos = pipePos + 1;
  }
  if (prevPos < linesStr.length() && result.lineCount < 3) {
    result.lines[result.lineCount++] = linesStr.substring(prevPos);
  }

  // Parse row, col, and clr
  result.row = input.substring(rowStart + 1, colStart).toInt();
  result.col = input.substring(colStart + 1, clrStart).toInt();
  result.clr = (input.substring(clrStart + 1) == "1" || input.substring(clrStart + 1).equalsIgnoreCase("true"));

  return result;
}
void processDisplayCommand(String str) {
  ParsedData p = parseString(str);
    
  if (p.lineCount == 1) {
    //sendToDisplay(p.lines[0], p.row, p.col, static_cast<XYMode>(p.command), p.clr);
  } else if (p.lineCount == 2) {
    //sendToDisplay(p.lines[0], p.lines[1], p.row, p.col, static_cast<XYMode>(p.command), p.clr);
  } else if (p.lineCount == 3) {
    //sendToDisplay(p.lines[0], p.lines[1], p.lines[2], p.row, p.col, static_cast<XYMode>(p.command), p.clr);
  }
}

