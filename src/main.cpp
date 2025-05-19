/*

Control Scheme

Right Trigger = Accelerate
Left Trigger  = Decelerate
Right Bumper  = Rotate Right
Left Bumper   = Rotate Left
Right Stick   = Steering

*/

// TODO tinker with the steering value
// TODO maybe make the R2 and L2 priotiy system based on the throttle and brake values?
// TODO optimize int pwm to byte and other data types that might need optimization
// TODO optimize so that handleacceleration handles the default setspeed instead of handlesteering

#include <Arduino.h>
#include <L298NX2.h>
#include <Bluepad32.h>

#define IDLE_ROTATE_PWM_SPEED 255
#define LS_DEADZONE 25
#define RS_DEADZONE 25
#define MAPPED_PWM_DEADZONE 5
#define PWM_UPPER_LIMIT 255

// Left DC Motor
int ENA = 26;
int IN1 = 27; 
int IN2 = 14;

// Right DC Motor
int ENB = 25;
int IN3 = 32; 
int IN4 = 33; 

L298NX2 DCMotors(ENA,IN1,IN2,ENB,IN3,IN4);
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl);
void onDisconnectedController(ControllerPtr ctl);
void processControllers();
void processGamepad(ControllerPtr ctl);
void handleSteering(ControllerPtr ctl, int* pwm, int* axisX_position, int* steerStrength);
void handleRotation(ControllerPtr ctl, int* pwm);
void handleAcceleration(ControllerPtr ctl);

void setup() {
  Serial.begin(115200);
  Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
  const uint8_t* addr = BP32.localBdAddress();
  Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

  // Setup the Bluepad32 callbacks
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.forgetBluetoothKeys();
  BP32.enableVirtualDevice(false);
}

void loop() {
  bool dataUpdated = BP32.update();
  if (dataUpdated){
    processControllers();
    Serial.printf(
      "PWM Left DC Motor: %d\t||\tPWM Right DC Motor: %d\t||\tFW/BW Direction: %s\t||\tL/R Direction: %s\n",
      DCMotors.getSpeedA(),
      DCMotors.getSpeedB(),
      (DCMotors.getDirectionA() == 1 && DCMotors.getDirectionB() == 1) ? "Backward" : "Forward",
      (DCMotors.getSpeedA() > DCMotors.getSpeedB() || (DCMotors.getDirectionA() == 0 && DCMotors.getDirectionB() == 1)) ? "Right" : "Left"
    );
  }

  vTaskDelay(10 / portTICK_PERIOD_MS);  
}

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id,
                      properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not found empty slot");
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  bool foundController = false;

  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      foundController = true;
      break;
    }
  }

  if (!foundController) {
    Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
  }
}

void processControllers() {
  for (auto myController : myControllers) {
    if (myController && myController->isConnected() && myController->hasData()) {
      if (myController->isGamepad()) {
          processGamepad(myController);
      } else {
        Serial.println("Unsupported controller");
      }
    }
  }
}

void processGamepad(ControllerPtr ctl) {
  // Calculates the pwm based on the throttle strength from the trigger buttons
  int throttleStrength = abs(ctl->throttle() - ctl->brake());
  int pwm = map(throttleStrength,0,1023,0,PWM_UPPER_LIMIT);

  // Calculates the steering strength based on the X axis of the left stick
  int axisX_position = ctl -> axisX(); 
  int steerStrength = map(abs(axisX_position), LS_DEADZONE, 512, 0, PWM_UPPER_LIMIT); 

  /* Right
    Left
  Steer */
  // Set default pwm if the steer is at default position/within the deadzone
  handleSteering(ctl, &pwm, &axisX_position, &steerStrength);

  /* Right
    Left
  Rotation */
  handleRotation(ctl, &pwm);

  /* Acceleration
    and
  Deceleration */
  // Since L1 and R1 (Rotation) is prioritized over R2 and L2 (Acceleration/Deceleration), acceleration is only handled when L1 and R1 are not pressed
  if (!(ctl->r1() || ctl->l1())) handleAcceleration(ctl);
}

void handleSteering(ControllerPtr ctl, int* pwm, int* axisX_position, int* steerStrength){
  // Set default pwm if the steer is at default position/within the deadzone
  if (*axisX_position > -LS_DEADZONE && *axisX_position < LS_DEADZONE)
  {
    // Optimized so that it only sets the pwm if there's been a change
    static int prevPwm = -1;
    if (prevPwm != *pwm || DCMotors.getSpeedA() != *pwm || DCMotors.getSpeedB() != *pwm){
      DCMotors.setSpeed(*pwm);
      prevPwm = *pwm;
    }
  }
  // Steer to the right by reducing the left motor's pwm value
  else if (*axisX_position >= LS_DEADZONE && *pwm > MAPPED_PWM_DEADZONE)
  {
    DCMotors.setSpeedA(*pwm);
    DCMotors.setSpeedB(constrain(*pwm - *steerStrength,MAPPED_PWM_DEADZONE,255));
  }
  // Steer to the left by reducing the right motor's pwm value
  else if (*axisX_position <= -LS_DEADZONE && *pwm > MAPPED_PWM_DEADZONE)
  {
    DCMotors.setSpeedB(*pwm);
    DCMotors.setSpeedA(constrain(*pwm - *steerStrength,MAPPED_PWM_DEADZONE,255));
  }
}

void handleRotation(ControllerPtr ctl, int* pwm){
  bool wasRotating_flag = false;
  // Rotate to the left, will use the default rotate pwm speed if the current pwm is within the mapped PWM deadzone (both triggers are at the default position)
  if (ctl -> l1() && !ctl->r1())
  {
    if (*pwm <= MAPPED_PWM_DEADZONE) DCMotors.setSpeed(IDLE_ROTATE_PWM_SPEED);
    DCMotors.forwardB();
    DCMotors.backwardA();
    wasRotating_flag = true;
    return;
  }
  // Rotate to the right, will use the default rotate pwm speed if the current pwm is within the mapped PWM deadzone (both triggers are at the default position)
  else if (ctl -> r1() && !ctl->l1())
  {
    if (*pwm <= MAPPED_PWM_DEADZONE) DCMotors.setSpeed(IDLE_ROTATE_PWM_SPEED);
    DCMotors.forwardA();
    DCMotors.backwardB();
    wasRotating_flag = true;
    return;
  }
  // If both L1 and R1 are pressed, stops the motors.
  else if (ctl -> r1() && ctl->l1()) {
    DCMotors.setSpeed(0);
    wasRotating_flag = true;
    return;
  }
  // Reset when both are released
  if  (wasRotating_flag && !ctl->r1() && !ctl->l1()){
    DCMotors.setSpeed(0);
    wasRotating_flag = false;
  }
}

void handleAcceleration(ControllerPtr ctl){
  static bool actionInProgress_flag = false;
  static unsigned long l2PressedTime = 0;
  static unsigned long r2PressedTime = 0;
  // Start the timer after either R2 or L2 is pressed
  if (ctl->r2() && !actionInProgress_flag) {
    if (r2PressedTime == 0) r2PressedTime = millis();
  }
  if (ctl->l2() && !actionInProgress_flag) {
    if (l2PressedTime == 0) l2PressedTime = millis();
  }
  // When both R2 and L2 are pressed, prioritize the button that is pressed first. If both buttons are pressed at the exact same time, R2 gets prioritized. 
  if (ctl->l2() && ctl->r2() && !actionInProgress_flag)
  {
    if (l2PressedTime < r2PressedTime) DCMotors.backward();
    else DCMotors.forward();
    actionInProgress_flag = true;
  }
  // If only R2 is pressed, then it will accelerate
  if (ctl -> r2()  && !ctl->l2() && !actionInProgress_flag)
  {
    DCMotors.forward();
    actionInProgress_flag = true;
  }
  // If only L2 is pressed, then it will decelerate
  if (ctl -> l2()  && !ctl->r2() && !actionInProgress_flag)
  {
    DCMotors.backward();
    actionInProgress_flag = true;
  }
  // Reset when both are released
  if (!ctl->r2() && !ctl->l2()) {
    r2PressedTime = 0;
    l2PressedTime = 0;
    actionInProgress_flag = false;
  }
}