#include <Dynamixel2Arduino.h>
#include <math.h>

// Dynamixel configuration for OpenCM 9.04
#define DXL_SERIAL Serial3
#define DEBUG_SERIAL SerialUSB

const uint8_t DXL_DIR_PIN = 22;
const float DXL_PROTOCOL_VERSION = 2.0;

const int DXL_ID[] = {1, 2, 3, 4, 5, 6};
const int NUM_ACTUATORS = 6;
const int V_MAX = 15; // Max velocity (deg/sec)
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

// Stewart platform parameters
const float BASE_LARGE = 12.0;
const float PLATFORM_LARGE = 8.75;
const float BASE_SMALL = 10.95;
const float PLATFORM_SMALL = 5.5;
const float HEIGHT_OFFSET = 27.43;
const float CRANK_RADIUS = 4.5;
const float ROD_LENGTH = 27.8;
const int SUCCESS_LED = 18;
const int ERROR_LED = 19;

// Global variables
float limb_lengths[NUM_ACTUATORS];
float joint_angles[NUM_ACTUATORS];

void setup() {
  // Serial interfaces
  DEBUG_SERIAL.begin(115200);
  dxl.begin(57600);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // LED initialization
  pinMode(SUCCESS_LED, OUTPUT);
  pinMode(ERROR_LED, OUTPUT);
  digitalWrite(SUCCESS_LED, LOW);
  digitalWrite(ERROR_LED, LOW);

  // Dynamixel servos initialization
  DEBUG_SERIAL.println("Initializing servos...");
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    if (!dxl.ping(DXL_ID[i])) {
      DEBUG_SERIAL.print("Error: Servo ");
      DEBUG_SERIAL.print(DXL_ID[i]);
      DEBUG_SERIAL.println(" not responding!");
      delay(500);
      while (1) {
        digitalWrite(ERROR_LED, HIGH);
        delay(200);
        digitalWrite(ERROR_LED, LOW);
        delay(200);
        DEBUG_SERIAL.println(" Stuck in a loop.");
        if (dxl.ping(DXL_ID[i])) { break; }
      }
    }
    dxl.torqueOff(DXL_ID[i]);
    dxl.setOperatingMode(DXL_ID[i], OP_POSITION);
    dxl.writeControlTableItem(RETURN_DELAY_TIME, DXL_ID[i], 0);
    dxl.writeControlTableItem(DRIVE_MODE, DXL_ID[i], 0);
    dxl.writeControlTableItem(PROFILE_ACCELERATION, DXL_ID[i], 20);
    dxl.writeControlTableItem(PROFILE_VELOCITY, DXL_ID[i], V_MAX);
    dxl.torqueOn(DXL_ID[i]);
    DEBUG_SERIAL.print("  Servo ");
    DEBUG_SERIAL.print(DXL_ID[i]);
    DEBUG_SERIAL.println(": OK");
  }
  
  DEBUG_SERIAL.println("Stewart Platform Ready");
  DEBUG_SERIAL.println("Send commands: X Y Z Roll Pitch Yaw");

  // Move to Home Position
  processCommand("0 0 0 0 0 0");
}

void loop() {
  if (DEBUG_SERIAL.available() > 0) {
    String input = DEBUG_SERIAL.readStringUntil('\n');
    processCommand(input);
  }
}

void processCommand(String input) {
  float values[6] = {0, 0, 0, 0, 0, 0};
  int parsed = parseInput(input, values);

  if (parsed == 6) {
    float px = values[0];
    float py = values[1];
    float pz = values[2];
    float Roll = values[3];   // alpha
    float pitch = values[4];  // beta
    float yaw = values[5];    // gamma
    
    inverseKinematics(px, py, pz, Roll, pitch, yaw, limb_lengths);
    limbToJointAngles(limb_lengths, joint_angles);
    
    // Debug output for Python log confirmation (optional but helpful)
    DEBUG_SERIAL.print("CMD: ");
    DEBUG_SERIAL.print(px); DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.print(py); DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(pz);
    
    moveServos(joint_angles);
  } else {
    DEBUG_SERIAL.println("Error: Invalid command format");
    DEBUG_SERIAL.println("Use: X Y Z Roll Pitch Yaw");
    flashError(3);
  }
}

int parseInput(String input, float* values) {
  int count = 0;
  int startIndex = 0;
  input.trim();
  if (input == "0") {
    for (int i = 0; i < 6; i++) values[i] = 0;
    return 6;
  }
  for (int i = 0; i <= input.length(); i++) {
    if (input[i] == ' ' || i == input.length()) {
      if (startIndex < i) {
        values[count] = input.substring(startIndex, i).toFloat();
        count++;
        if (count >= 6) break;
      }
      startIndex = i + 1;
    }
  }
  return count;
}

void inverseKinematics(float px, float py, float pz, 
                      float alpha, float beta, float gamma, 
                      float* L) {
  pz += HEIGHT_OFFSET;
  float rad_alpha = radians(alpha);
  float rad_beta = radians(beta);
  float rad_gamma = radians(gamma);
  
  float c1 = cos(rad_alpha);
  float s1 = sin(rad_alpha);
  float c2 = cos(rad_beta);
  float s2 = sin(rad_beta);
  float c3 = cos(rad_gamma);
  float s3 = sin(rad_gamma);
  
  float base_points[6][3] = {
    { ((sqrt(3))/6)*(2*BASE_LARGE+BASE_SMALL),  0.5*BASE_SMALL,            0 },
    { (-((sqrt(3))/6)*(BASE_LARGE-BASE_SMALL)), 0.5*(BASE_LARGE+BASE_SMALL), 0 },
    { (-((sqrt(3))/6)*(BASE_LARGE+2*BASE_SMALL)), 0.5*BASE_LARGE,           0 },
    { (-((sqrt(3))/6)*(BASE_LARGE+2*BASE_SMALL)), -0.5*BASE_LARGE,          0 },
    { (-((sqrt(3))/6)*(BASE_LARGE-BASE_SMALL)), -0.5*(BASE_LARGE+BASE_SMALL),0 },
    { ((sqrt(3))/6)*(2*BASE_LARGE+BASE_SMALL),  -0.5*BASE_SMALL,           0 }
  };
  
  float platform_points[6][3] = {
    { ((sqrt(3))/6)*(2*PLATFORM_LARGE+PLATFORM_SMALL),  0.5*PLATFORM_SMALL,            0 },
    { (-((sqrt(3))/6)*(PLATFORM_LARGE-PLATFORM_SMALL)), 0.5*(PLATFORM_LARGE+PLATFORM_SMALL), 0 },
    { (-((sqrt(3))/6)*(PLATFORM_LARGE+2*PLATFORM_SMALL)), 0.5*PLATFORM_LARGE,           0 },
    { (-((sqrt(3))/6)*(PLATFORM_LARGE+2*PLATFORM_SMALL)), -0.5*PLATFORM_LARGE,          0 },
    { (-((sqrt(3))/6)*(PLATFORM_LARGE-PLATFORM_SMALL)), -0.5*(PLATFORM_LARGE+PLATFORM_SMALL),0 },
    { ((sqrt(3))/6)*(2*PLATFORM_LARGE+PLATFORM_SMALL),  -0.5*PLATFORM_SMALL,           0 }
  };
  
  float R[3][3] = {
    { c3*c2,         c3*s2*s1 - s3*c1,    c3*s2*c1 + s3*s1 },
    { s3*c2,         s3*s2*s1 + c3*c1,    s3*s2*c1 - c3*s1 },
    { -s2,           c2*s1,               c2*c1             }
  };
  
  float transformed_points[6][3];
  for (int i = 0; i < 6; i++) {
    transformed_points[i][0] = R[0][0]*platform_points[i][0] + 
                              R[0][1]*platform_points[i][1] + 
                              R[0][2]*platform_points[i][2] + px;
    transformed_points[i][1] = R[1][0]*platform_points[i][0] + 
                              R[1][1]*platform_points[i][1] + 
                              R[1][2]*platform_points[i][2] + py;
    transformed_points[i][2] = R[2][0]*platform_points[i][0] + 
                              R[2][1]*platform_points[i][1] + 
                              R[2][2]*platform_points[i][2] + pz;
  }
  
  for (int i = 0; i < 6; i++) {
    float dx = transformed_points[i][0] - base_points[i][0];
    float dy = transformed_points[i][1] - base_points[i][1];
    float dz = transformed_points[i][2] - base_points[i][2];
    L[i] = sqrt(dx*dx + dy*dy + dz*dz);
  }
}

void limbToJointAngles(float* L, float* joints) {
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    float numerator = CRANK_RADIUS*CRANK_RADIUS + L[i]*L[i] - ROD_LENGTH*ROD_LENGTH;
    float denominator = 2 * CRANK_RADIUS * L[i];
    float ratio = numerator / denominator;
    ratio = (ratio < -1) ? -1 : (ratio > 1) ? 1 : ratio;
    float angle = acos(ratio);
    float deg_angle1 = degrees(angle);
    float deg_angle2 = 180.0 - deg_angle1;
    
    if (i % 2 == 1) {  // Mirror for even servos
      deg_angle2 = 360.0 - deg_angle2;
    }
    joints[i] = deg_angle2;
  }
}

void moveServos(float* goal_pos) {
  bool valid = true;
  for (int i = 0; i < NUM_ACTUATORS; i++) {
    if ((i % 2 == 0) && (goal_pos[i] < 60 || goal_pos[i] > 180)) {
      DEBUG_SERIAL.println("INVALID (60-180°)");
      valid = false;
    } else if ((i % 2 == 1) && (goal_pos[i] < 180 || goal_pos[i] > 300)) {
      DEBUG_SERIAL.println("INVALID (180-300°)");
      valid = false;
    }
  }
  if (valid) {
    digitalWrite(SUCCESS_LED, HIGH);
    for (int i = 0; i < NUM_ACTUATORS; i++) {
      if (!dxl.setGoalPosition(DXL_ID[i], goal_pos[i], UNIT_DEGREE)) {
        DEBUG_SERIAL.print("  !! Servo ");
        DEBUG_SERIAL.print(DXL_ID[i]);
        DEBUG_SERIAL.println(": COMM FAILURE");
      }
    }
    delay(100);
    digitalWrite(SUCCESS_LED, LOW);
    DEBUG_SERIAL.println("Movement complete");
  } else {
    DEBUG_SERIAL.println("Movement aborted: Invalid positions");
    flashError(5);
  }
}

void flashError(int count) {
  for (int i = 0; i < count; i++) {
    digitalWrite(ERROR_LED, HIGH);
    delay(150);
    digitalWrite(ERROR_LED, LOW);
    delay(150);
  }
}
