// === Настройки ===
const int MOTOR_DIR_PIN = 12;
const int MOTOR_PWM_PIN = 10;
const int POSITION_SENSOR_PIN = A0;
const int PRESSURE_SENSOR_PIN = A1;

const int WINDOW_SIZE = 10;       // Размер окна скользящего среднего

float KP = 0.09;
float KI = 0.003;
float KD = 0.9;
const float MIN_COEFFICIENT = 0;
const float MAX_COEFFICIENT = 50;

const int MOTOR_MIN_SPEED = 0;
const int MOTOR_MAX_SPEED = 110;
const float MAX_VOLTAGE = 5;
const float MAX_PRESSURE_VOLTAGE = 4.5;
const float MIN_PRESSURE_VOLTAGE = 0.5;
const float MAX_PRESSURE_KPA = 6.0;
const int POSITION_TOLERANCE = 10;

int MIN_POSITION = 101;
int MAX_POSITION = 907;

const float KF = 0.2;

const unsigned long LOOP_INTERVAL_MS = 30;

// === Внутренние переменные ===
int targetPosition = 500;
float filteredPositionEncoder = 0;
float filteredPressureEncoder = 0;
float filteredPosition = 0;
float filteredPressure = 0;

bool motorEnabled = false;
int motorSpeed = 10;

float error = 0;
float previousError = 0;
float integral = 0;
float voltage = 0;

unsigned long lastLoopTime = 0;

// === Serial-команда ===
String inputString = "";
bool commandComplete = false;

// === Инициализация движения ===
bool initing = false;
int initStep = 0;
int stableCounter = 0;
int lastPosition = -1;
const int STABLE_THRESHOLD = 10;

// === Setup ===
void setup() {
  Serial.begin(9600);
  handleSerial();
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
}

// === Главный цикл ===
void loop() {
  if (millis() - lastLoopTime >= LOOP_INTERVAL_MS) {
    lastLoopTime = millis();
    
    handleSerial();

    if (initing) {
      handleInitialization();
      return;
    }

    filteredPosition = getFilteredPosition();
    filteredPressure = getFilteredPressure();
    sendTelemetry(filteredPosition);
    float speed = computePID();
    move(speed);
    if (!motorEnabled || abs(error) < POSITION_TOLERANCE) {
      stop();
    }
  }
}

// === Получение сглаженных значений ===
float getFilteredPosition() {
  int encoder = analogRead(POSITION_SENSOR_PIN);
  filteredPositionEncoder += (encoder - filteredPositionEncoder) * KF;
  return filteredPositionEncoder;
}

float getFilteredPressure() {
  float encoder = analogRead(PRESSURE_SENSOR_PIN);
  filteredPressureEncoder += (encoder - filteredPressureEncoder) * KF;
  float voltage = filteredPressureEncoder * (MAX_VOLTAGE / 1023.0);
  float pressure_kPa = (voltage - MIN_PRESSURE_VOLTAGE) * (MAX_PRESSURE_KPA / (MAX_PRESSURE_VOLTAGE - MIN_PRESSURE_VOLTAGE));
  return pressure_kPa * 0.01;
}

// === PID ===
float computePID() {
  error = targetPosition - filteredPosition;

  if (!motorEnabled || abs(error) < POSITION_TOLERANCE) {
    return 0;
  }

  integral += error;
  integral = constrain(integral, -10000, 10000);
  float derivative = error - previousError;
  previousError = error;

  float output = KP * error + KI * integral + KD * derivative;
  return output;
}

// === Управление мотором ===
void move(int speed) {
  motorSpeed = constrain(abs((int)speed), MOTOR_MIN_SPEED, MOTOR_MAX_SPEED);
  digitalWrite(MOTOR_DIR_PIN, speed > 0 ? HIGH : LOW);
  analogWrite(MOTOR_PWM_PIN, motorSpeed);
}

void stop() {
  analogWrite(MOTOR_PWM_PIN, 0);
}

// === Отправка телеметрии ===
void sendTelemetry(int currentPosition) {
  static int counter = 0;
  if (++counter >= 20) {
    Serial.print("Target: ");
    Serial.print(targetPosition);
    Serial.print(" | Current: ");
    Serial.print(currentPosition);
    Serial.print(" | Error: ");
    Serial.print(error);
    Serial.print(" | Integral: ");
    Serial.print(integral);
    Serial.print(" | Enabled: ");
    Serial.print(motorEnabled ? "YES" : "NO");
    Serial.print(" | Pressure(Bar): ");
    Serial.print(filteredPressure, 6);
    Serial.print(" | KP: ");
    Serial.print(KP, 6);
    Serial.print(" | KD: ");
    Serial.print(KD, 6);
    Serial.print(" | KI: ");
    Serial.println(KI, 6);
    counter = 0;
  }
}

// === Serial обработка ===
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      commandComplete = true;
    }
  }
}

void handleSerial() {
  if (!commandComplete) return;

  inputString.trim();
  if (inputString.startsWith("set ")) {
    int value = inputString.substring(4).toInt();
    value = constrain(value, MIN_POSITION, MAX_POSITION);
    targetPosition = value;
    Serial.print("Target set to ");
    Serial.println(targetPosition);
  } else if (inputString.startsWith("kp ")) {
    float value = inputString.substring(3).toFloat();
    value = constrain(value, MIN_COEFFICIENT, MAX_COEFFICIENT);
    KP = value;
  } else if (inputString.startsWith("ki ")) {
    float value = inputString.substring(3).toFloat();
    value = constrain(value, MIN_COEFFICIENT, MAX_COEFFICIENT);
    KI = value;
  } else if (inputString.startsWith("kd ")) {
    float value = inputString.substring(3).toFloat();
    value = constrain(value, MIN_COEFFICIENT, MAX_COEFFICIENT);
    KD = value;
  } else if (inputString.equals("enable")) {
    motorEnabled = true;
    Serial.println("Motor enabled");
  } else if (inputString.equals("disable")) {
    motorEnabled = false;
    Serial.println("Motor disabled");
  } else if (inputString.equals("start")) {
    Serial.println("Initialization started");
    initing = true;
    initStep = 0;
    stableCounter = 0;
    lastPosition = -1;
  } else if (inputString.equals("get")) {
    Serial.print("Target: ");
    Serial.print(targetPosition);
    Serial.print(" | Position: ");
    Serial.print(filteredPosition);
    Serial.print(" | Error: ");
    Serial.println(error);
  } else {
    Serial.println("Unknown command");
  }

  inputString = "";
  commandComplete = false;
}

void handleInitialization() {
  filteredPosition = getFilteredPosition();

  switch (initStep) {
    case 0: //вправо
      digitalWrite(MOTOR_DIR_PIN, HIGH);
      analogWrite(MOTOR_PWM_PIN, MOTOR_MAX_SPEED / 3);
      if (abs(filteredPosition - lastPosition) < 2) {
        stableCounter++;
      } else {
        stableCounter = 0;
        lastPosition = filteredPosition;
      }
      if (stableCounter >= STABLE_THRESHOLD) {
        MAX_POSITION = filteredPosition;
        Serial.print("MAX_POSITION: ");
        Serial.println(MAX_POSITION);
        initStep = 1;
        stableCounter = 0;
        lastPosition = -1;
        delay(500);
      }
      break;

    case 1: //влево
      digitalWrite(MOTOR_DIR_PIN, LOW);
      analogWrite(MOTOR_PWM_PIN, MOTOR_MAX_SPEED / 3);
      if (abs(filteredPosition - lastPosition) < 2) {
        stableCounter++;
      } else {
        stableCounter = 0;
        lastPosition = filteredPosition;
      }
      if (stableCounter >= STABLE_THRESHOLD) {
        MIN_POSITION = filteredPosition;
        Serial.print("MIN_POSITION: ");
        Serial.println(MIN_POSITION);
        initStep = 2;
        stableCounter = 0;
        lastPosition = -1;
        delay(500);
      }
      break;

    case 2: // Завершение
      stop();
      targetPosition = (MIN_POSITION + MAX_POSITION) / 2;
      Serial.print("Center position set to: ");
      Serial.println(targetPosition);
      initing = false;
      initStep = 0;
      break;
  }
}
