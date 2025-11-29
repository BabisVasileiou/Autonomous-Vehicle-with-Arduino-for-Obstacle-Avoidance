/*
 * University of West Attica - MSc DRONES
 * Autonomous Vehicle with Arduino for Obstacle Avoidance
 */

#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ==========================================
// 1. CONFIGURATION & CONSTANTS
// ==========================================
const float SOUND_SPEED = 0.0343f; // cm/us
const int SERVO_PIN = 11;
const int ECHO_PIN = 9;
const int TRIG_PIN = 8;

// Motor Pins (A0..A3 mapped to 14..17 on UNO)
const int IN1 = 17; // A3
const int IN2 = 16; // A2
const int IN3 = 15; // A1
const int IN4 = 14; // A0

// Cliff Sensor Pins (Corrected based on Wiring Doc: 2=Right, 3=Left)
const int CLIFF_LEFT_PIN = 3; 
const int CLIFF_RIGHT_PIN = 2;

// LOGIC CONFIGURATION
// Set true if sensors output LOW when detecting floor (Common IR modules)
// This enables INPUT_PULLUP for safety (Wire break = HIGH = Cliff)
const bool CLIFF_ACTIVE_LOW = true;

// LCD Address (Usually 0x27 or 0x3F)
const int LCD_ADDRESS = 0x27;

// ==========================================
// 2. CLASS: DASHBOARD (LCD MANAGER)
// ==========================================
class Dashboard {
  private:
    LiquidCrystal_I2C lcd;
    String lastLine1;
    String lastLine2;

  public:
    Dashboard() : lcd(LCD_ADDRESS, 16, 2) {
      lastLine1 = "";
      lastLine2 = "";
    }

    void begin() {
      lcd.init();
      lcd.backlight();
      show("SYSTEM BOOT", "Initializing...");
      delay(300);
    }

    // Updates screen ONLY if text changes (prevents flickering)
    void show(const String &line1, const String &line2) {
      if (line1 != lastLine1 || line2 != lastLine2) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(line1);
        lcd.setCursor(0, 1);
        lcd.print(line2);
        lastLine1 = line1;
        lastLine2 = line2;
      }
    }
};

// ==========================================
// 3. CLASS: ADVANCED SENSOR (ADAPTIVE KALMAN)
// ==========================================
class Sonar {
  private:
    int trigPin, echoPin;
    float kalmanEstimate;
    float err_measure;
    float err_estimate;
    float q;
    bool isInitialized;

  public:
    Sonar(int trig, int echo) {
      trigPin = trig;
      echoPin = echo;
      kalmanEstimate = 0.0f;
      err_measure = 2.0f;
      err_estimate = 2.0f;
      q = 0.1f;
      isInitialized = false;
    }

    void begin() {
      pinMode(trigPin, OUTPUT);
      pinMode(echoPin, INPUT);
      digitalWrite(trigPin, LOW);
    }

    // Returns filtered distance in cm (float)
    float read() {
      digitalWrite(trigPin, LOW); delayMicroseconds(2);
      digitalWrite(trigPin, HIGH); delayMicroseconds(10);
      digitalWrite(trigPin, LOW);

      // Timeout 25ms (~4.3m max range)
      long duration = pulseIn(echoPin, HIGH, 25000L);

      float rawDist;
      if (duration == 0) {
        rawDist = 250.0f; // Timeout fallback
      } else {
        rawDist = (duration * SOUND_SPEED) / 2.0f;
      }

      // Initialize Kalman on first valid reading
      if (!isInitialized) {
        kalmanEstimate = rawDist;
        isInitialized = true;
      } else {
        float kalmanGain = err_estimate / (err_estimate + err_measure);
        kalmanEstimate = kalmanEstimate + kalmanGain * (rawDist - kalmanEstimate);
        err_estimate = (1.0f - kalmanGain) * err_estimate + q;
      }

      return kalmanEstimate;
    }
};

// ==========================================
// 4. CLASS: MOTOR ABSTRACTION LAYER
// ==========================================
enum MoveState { HALT, FWD, REV, TRN_L, TRN_R };

class DriveTrain {
  private:
    int p1, p2, p3, p4;

  public:
    DriveTrain(int in1, int in2, int in3, int in4) {
      p1 = in1; p2 = in2; p3 = in3; p4 = in4;
    }

    void begin() {
      pinMode(p1, OUTPUT); pinMode(p2, OUTPUT);
      pinMode(p3, OUTPUT); pinMode(p4, OUTPUT);
      set(HALT);
    }

    void set(MoveState state) {
      switch (state) {
        case FWD:   writeMotors(LOW, HIGH, LOW, HIGH); break;
        case REV:   writeMotors(HIGH, LOW, HIGH, LOW); break;
        case TRN_L: writeMotors(LOW, HIGH, HIGH, LOW); break;
        case TRN_R: writeMotors(HIGH, LOW, LOW, HIGH); break;
        case HALT:  writeMotors(LOW, LOW, LOW, LOW);   break;
      }
    }

  private:
    void writeMotors(int a, int b, int c, int d) {
      digitalWrite(p1, a); digitalWrite(p2, b);
      digitalWrite(p3, c); digitalWrite(p4, d);
    }
};

// ==========================================
// 5. CLASS: ROBOT BRAIN (FSM & AI)
// ==========================================
enum BrainState {
  CRUISING,
  DETECTED_OBSTACLE,
  SCANNING_LEFT,
  SCANNING_RIGHT,
  RESETTING_HEAD,
  DECIDING_PATH,
  EXECUTING_TURN,
  EXECUTING_ESCAPE,
  CLIFF_ESCAPE
};

class RobotBrain {
  private:
    Sonar* sonar;
    DriveTrain* motors;
    Dashboard* screen;
    Servo headServo;

    BrainState currentState;
    unsigned long timer;
    unsigned long actionDuration;
    unsigned long antiLoopTimer;

    float distCurrent, distLeft, distRight;
    int consecutiveTurns;

    // Tuning Params & Safe Limits
    const float SAFE_DIST = 25.0f;
    const float CRIT_DIST = 10.0f;
    const unsigned long SERVO_DELAY = 200;
    const unsigned long ESCAPE_TIME = 600;
    const unsigned long CLIFF_TIME = 700;
    const int SERVO_LEFT_LIM = 150;  // Limit servo sweep
    const int SERVO_RIGHT_LIM = 30; 

  public:
    RobotBrain(Sonar* s, DriveTrain* m, Dashboard* d) {
      sonar = s;
      motors = m;
      screen = d;
      currentState = CRUISING;
      consecutiveTurns = 0;
      antiLoopTimer = 0;
      distCurrent = distLeft = distRight = 250.0f;
    }

    void begin() {
      // Configure pins based on active-low logic
      if (CLIFF_ACTIVE_LOW) {
        pinMode(CLIFF_LEFT_PIN, INPUT_PULLUP);
        pinMode(CLIFF_RIGHT_PIN, INPUT_PULLUP);
      } else {
        pinMode(CLIFF_LEFT_PIN, INPUT);
        pinMode(CLIFF_RIGHT_PIN, INPUT);
      }

      headServo.attach(SERVO_PIN);
      headServo.write(90);
      antiLoopTimer = millis();
      delay(200);
    }

    void update() {
      // --- PRIORITY 1: CLIFF SAFETY CHECK ---
      int rawLeft = digitalRead(CLIFF_LEFT_PIN);
      int rawRight = digitalRead(CLIFF_RIGHT_PIN);
      bool cliffDetected;

      if (CLIFF_ACTIVE_LOW) {
        // With Pullup: Floor=LOW, Cliff/Disconnect=HIGH
        cliffDetected = (rawLeft == HIGH) || (rawRight == HIGH);
      } else {
        cliffDetected = (rawLeft == HIGH) || (rawRight == HIGH);
      }

      if (cliffDetected && currentState != CLIFF_ESCAPE) {
        motors->set(REV);
        screen->show("! WARNING !", "CLIFF DETECTED");
        Serial.println("[SAFETY] CLIFF detected! Reversing.");
        transitionTo(CLIFF_ESCAPE, CLIFF_TIME);
        return;
      }

      // --- PRIORITY 2: FINITE STATE MACHINE ---
      switch (currentState) {
        case CRUISING:
          distCurrent = sonar->read();
          screen->show("MODE: DRIVING", "Dist: " + String((int)distCurrent) + "cm");
          
          if (distCurrent < CRIT_DIST) {
            motors->set(REV);
            transitionTo(EXECUTING_ESCAPE, ESCAPE_TIME);
          } else if (distCurrent < SAFE_DIST) {
            motors->set(HALT);
            transitionTo(DETECTED_OBSTACLE, 100);
          } else {
            motors->set(FWD);
            // Reset anti-loop logic periodically
            if (millis() - antiLoopTimer >= 2000UL) {
              consecutiveTurns = 0;
              antiLoopTimer = millis();
            }
          }
          break;

        case DETECTED_OBSTACLE:
          screen->show("OBSTACLE AHEAD", "Scanning...");
          safeServoWrite(SERVO_LEFT_LIM);
          transitionTo(SCANNING_LEFT, SERVO_DELAY);
          break;

        case SCANNING_LEFT:
          if (timerExpired()) {
            distLeft = sonar->read();
            safeServoWrite(SERVO_RIGHT_LIM);
            transitionTo(SCANNING_RIGHT, SERVO_DELAY);
          }
          break;

        case SCANNING_RIGHT:
          if (timerExpired()) {
            distRight = sonar->read();
            safeServoWrite(90);
            transitionTo(RESETTING_HEAD, SERVO_DELAY);
          }
          break;

        case RESETTING_HEAD:
          if (timerExpired()) {
            currentState = DECIDING_PATH;
          }
          break;

        case DECIDING_PATH:
          screen->show("THINKING...", "L:" + String((int)distLeft) + " R:" + String((int)distRight));
          makeDecision();
          break;

        case EXECUTING_TURN:
          if (timerExpired()) {
            motors->set(HALT);
            transitionTo(CRUISING, 100);
          }
          break;

        case EXECUTING_ESCAPE:
          screen->show("! WARNING !", "TOO CLOSE!");
          if (timerExpired()) {
            motors->set(HALT);
            transitionTo(DETECTED_OBSTACLE, 100);
          }
          break;

        case CLIFF_ESCAPE:
          if (timerExpired()) {
            motors->set(TRN_L);
            transitionTo(EXECUTING_TURN, 800); // Spin away from cliff
          }
          break;
      }
    }

  private:
    void transitionTo(BrainState next, unsigned long ms) {
      currentState = next;
      timer = millis();
      actionDuration = ms;
    }

    bool timerExpired() {
      return (millis() - timer >= actionDuration);
    }

    void makeDecision() {
      // Anti-loop guard
      if (consecutiveTurns >= 3) {
        motors->set(TRN_L);
        screen->show("DECISION:", "Loop Escape");
        transitionTo(EXECUTING_TURN, 900);
        consecutiveTurns = 0;
        return;
      }

      // Dead-end (Both blocked)
      if (distLeft < SAFE_DIST && distRight < SAFE_DIST) {
        motors->set(REV);
        screen->show("DECISION:", "Dead End (Rev)");
        transitionTo(EXECUTING_ESCAPE, 400);
        return;
      }

      // Adaptive turn time calculation
      consecutiveTurns++;
      float diff = fabs(distLeft - distRight); 
      float ratio = constrain(diff / 100.0f, 0.0f, 1.0f);
      int adaptiveTime = (int) round(250.0f + ratio * (600.0f - 250.0f));
      adaptiveTime = constrain(adaptiveTime, 200, 1200);

      if (distLeft > distRight) {
        motors->set(TRN_L);
        screen->show("DECISION:", "Turning Left");
      } else {
        motors->set(TRN_R);
        screen->show("DECISION:", "Turning Right");
      }
      transitionTo(EXECUTING_TURN, adaptiveTime);
    }

    void safeServoWrite(int angle) {
      int a = constrain(angle, SERVO_RIGHT_LIM, SERVO_LEFT_LIM);
      headServo.write(a);
    }
};

// ==========================================
// 6. GLOBAL INSTANCES & MAIN LOOP
// ==========================================
Sonar mySonar(TRIG_PIN, ECHO_PIN);
DriveTrain myMotors(IN1, IN2, IN3, IN4);
Dashboard myScreen;
RobotBrain myRobot(&mySonar, &myMotors, &myScreen);

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Start I2C bus first
  delay(50);

  // Initialize Hardware
  myScreen.begin();
  mySonar.begin();
  myMotors.begin();
  myRobot.begin();

  Serial.println("System Ready: Eng-Grade v3.3 (Corrected Pins)");
}

void loop() {
  myRobot.update();
  delay(1); // Yield to CPU
}