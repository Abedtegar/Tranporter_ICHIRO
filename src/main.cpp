/*ESP32 Transporter Control System with WiFi Web Server
 * Combined sensor reading (GY-25 gyroscope, TCS34725 color sensor)
 * Analog sensor reading with multiplexer (16 sensors)
 * Servo control
 * Motor control with encoder feedback
 * WiFi web server for monitoring and control
 */
#include <Adafruit_TCS34725.h>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

// WiFi credentials - ganti dengan WiFi Anda
const char *ssid = "Robotika@test";
const char *password = "12345678";

// Web server instance
WebServer server(80);

// Analog sensor multiplexer pins and configuration
const int ANALOG_PINS[] = {36, 39, 34, 35}; // 4 analog input pins
const int IN1_PIN = 33;                     // Multiplexer control pin 1
const int IN2_PIN = 32;                     // Multiplexer control pin 2
const int NUM_ANALOG_PINS = 4;
const int NUM_MUX_CHANNELS = 4;
const int TOTAL_SENSORS = 16;

// Sensor mapping for multiplexer
const int SENSOR_MAP[4][4] = {
    {6, 8, 14,
     3}, // Channel 0 (00): analog1=S6, analog2=S8, analog3=S14, analog4=S3
    {5, 11, 13,
     0}, // Channel 1 (01): analog1=S5, analog2=S11, analog3=S13, analog4=S0
    {4, 9, 12,
     2}, // Channel 2 (10): analog1=S4, analog2=S9, analog3=S12, analog4=S2
    {7, 10, 15,
     1} // Channel 3 (11): analog1=S7, analog2=S10, analog3=S15, analog4=S1
};

// Servo pins and objects
const int SERVO1_PIN = 13; // Servo 1 pin
const int SERVO2_PIN = 12; // Servo 2 pin
Servo servo1;
Servo servo2;

// Analog sensor values storage
int sensorValues[16] = {0};

// Servo control variables
int servo1Angle = 90; // Default center position
int servo2Angle = 90; // Default center position

// Manual motor control variables
bool manualMode = false;
int manualMotor1Speed = 0; // -100 to 100
int manualMotor2Speed = 0; // -100 to 100

int enc_a1 = 14;
int enc_b1 = 27;
int enc1 = 0;
int dir1 = 0;

int enc_a2 = 26;
int enc_b2 = 25;
int enc2 = 0;
int dir2 = 0;

int en1 = 23;
int en2 = 19;
int in1 = 18;
int in2 = 5;
int in3 = 4;
int in4 = 2;
// PWM / motor control settings (ESP32 LEDC)
const int LEDC_CHANNEL_A = 0;
const int LEDC_CHANNEL_B = 1;
const int LEDC_FREQ = 5000;    // 5 kHz
const int LEDC_RESOLUTION = 8; // 8-bit (0-255)

// Motor ramping state
int pwmPercent = 0; // 0..100 for active motor
bool rampUp = true;
bool dirA_forward = true; // motor1 direction
bool dirB_forward = true; // motor2 direction
unsigned long lastRampMillis = 0;
const unsigned long RAMP_STEP_MS = 50; // ms per step

// Sequential motor control state
enum MotorState {
  MOTOR1_CW_UP,    // Motor1 CW 0->100
  MOTOR1_CW_DOWN,  // Motor1 CW 100->0
  MOTOR1_CCW_UP,   // Motor1 CCW 0->100
  MOTOR1_CCW_DOWN, // Motor1 CCW 100->0
  MOTOR2_CW_UP,    // Motor2 CW 0->100
  MOTOR2_CW_DOWN,  // Motor2 CW 100->0
  MOTOR2_CCW_UP,   // Motor2 CCW 0->100
  MOTOR2_CCW_DOWN  // Motor2 CCW 100->0
};
MotorState currentState = MOTOR1_CW_UP;

void IRAM_ATTR handleEncoderA1_Rising() {

  if (digitalRead(enc_b1) == LOW) {
    // Clockwise rotation
    enc1++;
    dir1 = 1;
  }
  // else
  // {
  //   dir = -1;
  //   enc--;
  // }
}

void IRAM_ATTR handleEncoderB1_Rising() {

  if (digitalRead(enc_a1) == LOW) {
    // Clockwise rotation
    enc1--;
    dir1 = -1;
  }
  // else
  // {
  //   enc++;
  //   dir = 1;
  // }
}

void IRAM_ATTR handleEncoderA2_Rising() {

  if (digitalRead(enc_b2) == LOW) {
    // Clockwise rotation
    enc2++;
    dir2 = 1;
  }
  // else
  // {
  //   dir2 = -1;
  //   enc2--;
  // }
}

void IRAM_ATTR handleEncoderB2_Rising() {

  if (digitalRead(enc_a2) == LOW) {
    // Clockwise rotation
    enc2--;
    dir2 = -1;
  }
  // else
  // {
  //   dir = -1;
  //   enc--;
  // }
}

// On ESP32 use the built-in Serial2 (HardwareSerial) for RX2/TX2
// Update these pins if your board uses different pins for Serial2
static const int RXPin = 16; // RX2
static const int TXPin = 17; // TX2
float Roll, Pitch, Yaw;
unsigned char Re_buf[8];
uint8_t counter = 0;
unsigned long lastQueryMillis = 0;
const unsigned long QUERY_INTERVAL = 200; // ms between queries

// TCS34725 object: use default integration time and gain
Adafruit_TCS34725 tcs =
    Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
bool tcs_ok = false;
unsigned long lastColorMillis = 0;
const unsigned long COLOR_INTERVAL = 250; // ms between color reads

// Web server handler functions
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Transporter Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body { font-family: Arial; margin: 20px; background: #f0f0f0; }
        .container { max-width: 800px; margin: 0 auto; background: white; padding: 20px; border-radius: 10px; }
        .section { margin: 20px 0; padding: 15px; border: 1px solid #ddd; border-radius: 5px; }
        .sensor-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 10px; }
        .sensor-item { padding: 10px; background: #e8f4f8; border-radius: 5px; }
        .motor-control { display: flex; flex-direction: column; gap: 10px; }
        .slider-container { display: flex; align-items: center; gap: 10px; }
        .slider { width: 200px; }
        button { padding: 10px 20px; background: #007bff; color: white; border: none; border-radius: 5px; cursor: pointer; }
        button:hover { background: #0056b3; }
        .status { padding: 10px; background: #d4edda; border-radius: 5px; margin: 10px 0; }
        .emergency { background: #f8d7da !important; }
        #data { font-family: monospace; }
    </style>
</head>
<body>
    <div class="container">
        <h1>ESP32 Transporter Control System</h1>
        
        <div class="section">
            <h2>System Status</h2>
            <div id="status" class="status">Connecting...</div>
            <button onclick="toggleMode()">Toggle Auto/Manual Mode</button>
            <button onclick="emergencyStop()">EMERGENCY STOP</button>
        </div>

        <div class="section">
            <h2>Sensor Data</h2>
            <div id="data" class="sensor-grid">
                <div class="sensor-item">Roll: <span id="roll">--</span>°</div>
                <div class="sensor-item">Pitch: <span id="pitch">--</span>°</div>
                <div class="sensor-item">Yaw: <span id="yaw">--</span>°</div>
                <div class="sensor-item">Color R: <span id="colorR">--</span></div>
                <div class="sensor-item">Color G: <span id="colorG">--</span></div>
                <div class="sensor-item">Color B: <span id="colorB">--</span></div>
                <div class="sensor-item">Lux: <span id="lux">--</span></div>
                <div class="sensor-item">Encoder 1: <span id="enc1">--</span></div>
                <div class="sensor-item">Encoder 2: <span id="enc2">--</span></div>
                <div class="sensor-item">Motor State: <span id="motorState">--</span></div>
            </div>
        </div>

        <div class="section">
            <h2>Analog Sensors (16 channels)</h2>
            <div id="analogSensors" class="sensor-grid">
                <!-- Analog sensors will be populated by JavaScript -->
            </div>
        </div>

        <div class="section" id="manualControl" style="display:none;">
            <h2>Manual Motor Control</h2>
            <div class="motor-control">
                <div class="slider-container">
                    <label>Motor 1:</label>
                    <input type="range" id="motor1" class="slider" min="-100" max="100" value="0" onchange="updateMotor(1, this.value)">
                    <span id="motor1Value">0</span>%
                </div>
                <div class="slider-container">
                    <label>Motor 2:</label>
                    <input type="range" id="motor2" class="slider" min="-100" max="100" value="0" onchange="updateMotor(2, this.value)">
                    <span id="motor2Value">0</span>%
                </div>
            </div>
        </div>

        <div class="section">
            <h2>Servo Control</h2>
            <div class="motor-control">
                <div class="slider-container">
                    <label>Servo 1 Angle:</label>
                    <input type="range" id="servo1" class="slider" min="0" max="180" value="90" onchange="updateServo(1, this.value)">
                    <span id="servo1Value">90</span>°
                </div>
                <div class="slider-container">
                    <label>Servo 2 Angle:</label>
                    <input type="range" id="servo2" class="slider" min="0" max="180" value="90" onchange="updateServo(2, this.value)">
                    <span id="servo2Value">90</span>°
                </div>
            </div>
        </div>
    </div>

    <script>
        let manualMode = false;
        
        function updateData() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('roll').textContent = data.roll.toFixed(2);
                    document.getElementById('pitch').textContent = data.pitch.toFixed(2);
                    document.getElementById('yaw').textContent = data.yaw.toFixed(2);
                    document.getElementById('colorR').textContent = data.colorR;
                    document.getElementById('colorG').textContent = data.colorG;
                    document.getElementById('colorB').textContent = data.colorB;
                    document.getElementById('lux').textContent = data.lux.toFixed(2);
                    document.getElementById('enc1').textContent = data.enc1;
                    document.getElementById('enc2').textContent = data.enc2;
                    document.getElementById('motorState').textContent = data.motorState;
                    
                    // Update analog sensors
                    const analogContainer = document.getElementById('analogSensors');
                    if (data.analogSensors) {
                        let analogHTML = '';
                        for (let i = 0; i < data.analogSensors.length; i++) {
                            analogHTML += `<div class="sensor-item">S${i}: ${data.analogSensors[i]}</div>`;
                        }
                        analogContainer.innerHTML = analogHTML;
                    }
                    
                    // Update servo positions
                    if (data.servo1Angle !== undefined) {
                        document.getElementById('servo1').value = data.servo1Angle;
                        document.getElementById('servo1Value').textContent = data.servo1Angle;
                    }
                    if (data.servo2Angle !== undefined) {
                        document.getElementById('servo2').value = data.servo2Angle;
                        document.getElementById('servo2Value').textContent = data.servo2Angle;
                    }
                    
                    const status = document.getElementById('status');
                    status.textContent = `Mode: ${data.mode} | Connected`;
                    status.className = 'status';
                })
                .catch(error => {
                    const status = document.getElementById('status');
                    status.textContent = 'Connection Error';
                    status.className = 'status emergency';
                });
        }

        function toggleMode() {
            fetch('/toggleMode', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    manualMode = data.manual;
                    document.getElementById('manualControl').style.display = manualMode ? 'block' : 'none';
                });
        }

        function updateMotor(motor, value) {
            document.getElementById(`motor${motor}Value`).textContent = value;
            fetch('/setMotor', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({motor: motor, speed: parseInt(value)})
            });
        }

        function updateServo(servo, value) {
            document.getElementById(`servo${servo}Value`).textContent = value;
            fetch('/setServo', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({servo: servo, angle: parseInt(value)})
            });
        }

        function emergencyStop() {
            fetch('/emergencyStop', {method: 'POST'});
            document.getElementById('motor1').value = 0;
            document.getElementById('motor2').value = 0;
            document.getElementById('motor1Value').textContent = '0';
            document.getElementById('motor2Value').textContent = '0';
        }

        // Update data every 100ms
        setInterval(updateData, 100);
        updateData();
    </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}

void handleData() {
  // Read current color sensor data
  uint16_t r = 0, g = 0, b = 0, c = 0;
  float lux = 0;
  if (tcs_ok) {
    tcs.getRawData(&r, &g, &b, &c);
    lux = tcs.calculateLux(r, g, b);
  }

  // Get current motor state string
  String stateStr;
  switch (currentState) {
  case MOTOR1_CW_UP:
    stateStr = "M1_CW_UP";
    break;
  case MOTOR1_CW_DOWN:
    stateStr = "M1_CW_DOWN";
    break;
  case MOTOR1_CCW_UP:
    stateStr = "M1_CCW_UP";
    break;
  case MOTOR1_CCW_DOWN:
    stateStr = "M1_CCW_DOWN";
    break;
  case MOTOR2_CW_UP:
    stateStr = "M2_CW_UP";
    break;
  case MOTOR2_CW_DOWN:
    stateStr = "M2_CW_DOWN";
    break;
  case MOTOR2_CCW_UP:
    stateStr = "M2_CCW_UP";
    break;
  case MOTOR2_CCW_DOWN:
    stateStr = "M2_CCW_DOWN";
    break;
  }

  DynamicJsonDocument doc(2048); // Increased size for analog sensor data
  doc["roll"] = Roll;
  doc["pitch"] = Pitch;
  doc["yaw"] = Yaw;
  doc["colorR"] = r;
  doc["colorG"] = g;
  doc["colorB"] = b;
  doc["colorC"] = c;
  doc["lux"] = lux;
  doc["enc1"] = enc1;
  doc["enc2"] = enc2;
  doc["dir1"] = (dir1 == 1 ? "CW" : (dir1 == -1 ? "CCW" : "STOP"));
  doc["dir2"] = (dir2 == 1 ? "CW" : (dir2 == -1 ? "CCW" : "STOP"));
  doc["motorState"] = stateStr;
  doc["pwmPercent"] = pwmPercent;
  doc["mode"] = manualMode ? "Manual" : "Auto";
  doc["motor1Speed"] = manualMotor1Speed;
  doc["motor2Speed"] = manualMotor2Speed;

  // Add analog sensor data
  JsonArray sensors = doc.createNestedArray("analogSensors");
  for (int i = 0; i < 16; i++) {
    sensors.add(sensorValues[i]);
  }

  // Add servo data
  doc["servo1Angle"] = servo1Angle;
  doc["servo2Angle"] = servo2Angle;

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleToggleMode() {
  manualMode = !manualMode;
  if (!manualMode) {
    // Reset to auto mode
    manualMotor1Speed = 0;
    manualMotor2Speed = 0;
  }

  DynamicJsonDocument doc(256);
  doc["manual"] = manualMode;
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleSetMotor() {
  if (!manualMode) {
    server.send(400, "text/plain", "Not in manual mode");
    return;
  }

  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(256);
    deserializeJson(doc, server.arg("plain"));

    int motor = doc["motor"];
    int speed = doc["speed"];

    if (motor == 1) {
      manualMotor1Speed = constrain(speed, -100, 100);
    } else if (motor == 2) {
      manualMotor2Speed = constrain(speed, -100, 100);
    }

    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Invalid request");
  }
}

void handleSetServo() {
  if (server.hasArg("plain")) {
    DynamicJsonDocument doc(256);
    deserializeJson(doc, server.arg("plain"));

    int servo = doc["servo"];
    int angle = doc["angle"];

    // Constrain angle to valid range
    angle = constrain(angle, 0, 180);

    if (servo == 1) {
      servo1Angle = angle;
      servo1.write(servo1Angle);
    } else if (servo == 2) {
      servo2Angle = angle;
      servo2.write(servo2Angle);
    }

    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Invalid request");
  }
}

void handleEmergencyStop() {
  manualMotor1Speed = 0;
  manualMotor2Speed = 0;

  // Stop all motors immediately
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  ledcWrite(LEDC_CHANNEL_A, 0);
  ledcWrite(LEDC_CHANNEL_B, 0);

  server.send(200, "text/plain", "Emergency stop activated");
}

// Analog sensor and servo functions
void setMuxChannel(int channel) {
  switch (channel) {
  case 0: // 00
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
    break;
  case 1: // 01
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
    break;
  case 2: // 10
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
    break;
  case 3: // 11
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, HIGH);
    break;
  }
  delayMicroseconds(100); // Small delay for multiplexer switching
}

void readAllSensors() {
  // Read all sensors and store in array
  for (int muxChannel = 0; muxChannel < NUM_MUX_CHANNELS; muxChannel++) {
    setMuxChannel(muxChannel);
    delay(10); // Allow multiplexer to switch

    for (int pin = 0; pin < NUM_ANALOG_PINS; pin++) {
      int sensorValue = analogRead(ANALOG_PINS[pin]);
      int sensorNumber = SENSOR_MAP[muxChannel][pin]; // Use mapping table
      sensorValues[sensorNumber] = sensorValue;       // Store by sensor number
    }
  }
}

void setupWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi connected! IP address: ");
  Serial.println(WiFi.localIP());
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/toggleMode", HTTP_POST, handleToggleMode);
  server.on("/setMotor", HTTP_POST, handleSetMotor);
  server.on("/setServo", HTTP_POST, handleSetServo);
  server.on("/emergencyStop", HTTP_POST, handleEmergencyStop);

  server.begin();
  Serial.println("Web server started");
}

void updateMotorsManual() {
  if (!manualMode)
    return;

  // Motor 1 control
  uint32_t dutyA = 0;
  if (manualMotor1Speed > 0) {
    // Forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    dutyA = (uint32_t)((abs(manualMotor1Speed) * 255) / 100);
  } else if (manualMotor1Speed < 0) {
    // Reverse
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    dutyA = (uint32_t)((abs(manualMotor1Speed) * 255) / 100);
  } else {
    // Stop
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    dutyA = 0;
  }

  // Motor 2 control
  uint32_t dutyB = 0;
  if (manualMotor2Speed > 0) {
    // Forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    dutyB = (uint32_t)((abs(manualMotor2Speed) * 255) / 100);
  } else if (manualMotor2Speed < 0) {
    // Reverse
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    dutyB = (uint32_t)((abs(manualMotor2Speed) * 255) / 100);
  } else {
    // Stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    dutyB = 0;
  }

  ledcWrite(LEDC_CHANNEL_A, dutyA);
  ledcWrite(LEDC_CHANNEL_B, dutyB);
}

void setup() {
  // Debug serial
  Serial.begin(115200);
  delay(100);

  Serial.println("ESP32 Transporter Control System Starting...");

  // Initialize motor control pins and PWM
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Setup LEDC PWM channels
  ledcSetup(LEDC_CHANNEL_A, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttachPin(en1, LEDC_CHANNEL_A);
  ledcSetup(LEDC_CHANNEL_B, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttachPin(en2, LEDC_CHANNEL_B);

  pinMode(enc_a1, INPUT_PULLUP);
  pinMode(enc_b1, INPUT_PULLUP);
  pinMode(enc_a2, INPUT_PULLUP);
  pinMode(enc_b2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enc_a1), handleEncoderA1_Rising,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(enc_b1), handleEncoderB1_Rising,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(enc_a2), handleEncoderA2_Rising,
                  RISING);
  attachInterrupt(digitalPinToInterrupt(enc_b2), handleEncoderB2_Rising,
                  RISING);

  // Initialize Serial2 for GY-25 on RX2/TX2 pins
  Serial2.begin(115200, SERIAL_8N1, RXPin, TXPin);
  delay(200);

  // Initialize I2C (uses default SDA/SCL pins on ESP32)
  Wire.begin();

  // Initialize TCS34725
  tcs_ok = tcs.begin();
  if (tcs_ok) {
    Serial.println("TCS34725 initialized");
  } else {
    Serial.println("TCS34725 not found. Check wiring.");
  }

  // Initialize analog sensor multiplexer pins
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);

  // Initialize servos
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(90); // Center position
  servo2.write(90); // Center position

  Serial.println("Analog sensors and servos initialized");

  // Put sensor into correction mode briefly, then into query mode
  Serial2.write(0xA5);
  Serial2.write(0x54); // correction mode
  delay(200);
  Serial2.write(0xA5);
  Serial2.write(0x51); // query mode
  delay(200);

  counter = 0;
  lastQueryMillis = millis();

  // Initialize ramping state
  pwmPercent = 0;
  rampUp = true;
  dirA_forward = true;
  dirB_forward = true;
  currentState = MOTOR1_CW_UP;
  lastRampMillis = millis();

  // Initialize WiFi and web server
  setupWiFi();
  setupWebServer();

  Serial.println("System initialization complete!");
  Serial.println("Access web interface at: http://" +
                 WiFi.localIP().toString());
}
void loop() {
  unsigned long now = millis();

  // Handle web server clients
  server.handleClient();

  // Periodically request data (query mode): send 0xA5 0x51
  if (now - lastQueryMillis >= QUERY_INTERVAL) {
    Serial2.write(0xA5);
    Serial2.write(0x51);
    lastQueryMillis = now;
  }

  // Read incoming bytes from Serial2 and assemble frames
  while (Serial2.available()) {
    uint8_t b = (uint8_t)Serial2.read();
    // If we're expecting the start byte and this isn't it, skip
    if (counter == 0 && b != 0xAA) {
      // skip until start byte
      continue;
    }
    Re_buf[counter++] = b;
    if (counter >= 8) {
      // got a full packet
      counter = 0;
      if (Re_buf[0] == 0xAA && Re_buf[7] == 0x55) {
        int16_t yaw_raw = (int16_t)((Re_buf[1] << 8) | Re_buf[2]);
        int16_t pitch_raw = (int16_t)((Re_buf[3] << 8) | Re_buf[4]);
        int16_t roll_raw = (int16_t)((Re_buf[5] << 8) | Re_buf[6]);
        Yaw = yaw_raw / 100.0f;
        Pitch = pitch_raw / 100.0f;
        Roll = roll_raw / 100.0f;
      }
    }
  }

  // Periodically read color sensor
  if (tcs_ok && (now - lastColorMillis >= COLOR_INTERVAL)) {
    uint16_t r, g, b, c;
    float lux;
    tcs.getRawData(&r, &g, &b, &c);
    lux = tcs.calculateLux(r, g, b);
    lastColorMillis = now;
  }

  // Read analog sensors periodically
  static unsigned long lastAnalogRead = 0;
  if (now - lastAnalogRead >= 100) { // Read analog sensors every 100ms
    readAllSensors();
    lastAnalogRead = now;
  }

  // Consolidated serial output - print all data at once
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 50) {
    // GY-25 data
    Serial.print("Roll:");
    Serial.print(Roll);
    Serial.print(" Pitch:");
    Serial.print(Pitch);
    Serial.print(" Yaw:");
    Serial.print(Yaw);

    // TCS34725 color data
    if (tcs_ok) {
      uint16_t r, g, b, c;
      float lux;
      tcs.getRawData(&r, &g, &b, &c);
      lux = tcs.calculateLux(r, g, b);
      Serial.print(" | Color C:");
      Serial.print(c);
      Serial.print(" R:");
      Serial.print(r);
      Serial.print(" G:");
      Serial.print(g);
      Serial.print(" B:");
      Serial.print(b);
      Serial.print(" Lux:");
      Serial.print(lux);
    }

    // Encoder data
    Serial.print(" | Enc1:");
    Serial.print(enc1);
    Serial.print(" Dir:");
    Serial.print(dir1 == 1 ? "CW" : (dir1 == -1 ? "CCW" : "STOP"));
    Serial.print(" Enc2:");
    Serial.print(enc2);
    Serial.print(" Dir:");
    Serial.print(dir2 == 1 ? "CW" : (dir2 == -1 ? "CCW" : "STOP"));

    // Analog sensor data (condensed - show first few sensors)
    Serial.print(" | Sensors S0:");
    Serial.print(sensorValues[0]);
    Serial.print(" S1:");
    Serial.print(sensorValues[1]);
    Serial.print(" S2:");
    Serial.print(sensorValues[2]);
    Serial.print(" S3:");
    Serial.print(sensorValues[3]);

    // Servo positions
    Serial.print(" | Servo1:");
    Serial.print(servo1Angle);
    Serial.print("° Servo2:");
    Serial.print(servo2Angle);
    Serial.print("°");

    // Motor state
    Serial.print(" | Mode:");
    Serial.print(manualMode ? "MANUAL" : "AUTO");
    if (manualMode) {
      Serial.print(" M1:");
      Serial.print(manualMotor1Speed);
      Serial.print("% M2:");
      Serial.print(manualMotor2Speed);
      Serial.print("%");
    } else {
      Serial.print(" State:");
      switch (currentState) {
      case MOTOR1_CW_UP:
        Serial.print("M1_CW_UP");
        break;
      case MOTOR1_CW_DOWN:
        Serial.print("M1_CW_DOWN");
        break;
      case MOTOR1_CCW_UP:
        Serial.print("M1_CCW_UP");
        break;
      case MOTOR1_CCW_DOWN:
        Serial.print("M1_CCW_DOWN");
        break;
      case MOTOR2_CW_UP:
        Serial.print("M2_CW_UP");
        break;
      case MOTOR2_CW_DOWN:
        Serial.print("M2_CW_DOWN");
        break;
      case MOTOR2_CCW_UP:
        Serial.print("M2_CCW_UP");
        break;
      case MOTOR2_CCW_DOWN:
        Serial.print("M2_CCW_DOWN");
        break;
      }
      Serial.print(" PWM:");
      Serial.print(pwmPercent);
      Serial.print("%");
    }
    Serial.println();

    lastPrint = now;
  }

  // Motor control - either manual or automatic
  if (manualMode) {
    updateMotorsManual();
  } else {
    // Original automatic motor ramping logic
    if (now - lastRampMillis >= RAMP_STEP_MS) {
      lastRampMillis = now;

      // Step pwmPercent up or down based on current state
      if (rampUp) {
        pwmPercent++;
        if (pwmPercent >= 100) {
          pwmPercent = 100;
          rampUp = false;
        }
      } else {
        pwmPercent--;
        if (pwmPercent <= 0) {
          pwmPercent = 0;
          rampUp = true;
          // Advance to next state when cycle completes
          switch (currentState) {
          case MOTOR1_CW_UP:
            currentState = MOTOR1_CW_DOWN;
            break;
          case MOTOR1_CW_DOWN:
            currentState = MOTOR1_CCW_UP;
            break;
          case MOTOR1_CCW_UP:
            currentState = MOTOR1_CCW_DOWN;
            break;
          case MOTOR1_CCW_DOWN:
            currentState = MOTOR2_CW_UP;
            break;
          case MOTOR2_CW_UP:
            currentState = MOTOR2_CW_DOWN;
            break;
          case MOTOR2_CW_DOWN:
            currentState = MOTOR2_CCW_UP;
            break;
          case MOTOR2_CCW_UP:
            currentState = MOTOR2_CCW_DOWN;
            break;
          case MOTOR2_CCW_DOWN:
            currentState = MOTOR1_CW_UP; // Loop back to start
            break;
          }
        }
      }

      // Set motor directions and PWM based on current state
      uint32_t dutyA = 0, dutyB = 0;

      switch (currentState) {
      case MOTOR1_CW_UP:
      case MOTOR1_CW_DOWN:
        // Motor1 CW
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW); // Motor2 off
        digitalWrite(in4, LOW);
        dutyA = (uint32_t)((pwmPercent * 255) / 100);
        dutyB = 0;
        break;

      case MOTOR1_CCW_UP:
      case MOTOR1_CCW_DOWN:
        // Motor1 CCW
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
        digitalWrite(in3, LOW); // Motor2 off
        digitalWrite(in4, LOW);
        dutyA = (uint32_t)((pwmPercent * 255) / 100);
        dutyB = 0;
        break;

      case MOTOR2_CW_UP:
      case MOTOR2_CW_DOWN:
        // Motor2 CW
        digitalWrite(in1, LOW); // Motor1 off
        digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH);
        digitalWrite(in4, LOW);
        dutyA = 0;
        dutyB = (uint32_t)((pwmPercent * 255) / 100);
        break;

      case MOTOR2_CCW_UP:
      case MOTOR2_CCW_DOWN:
        // Motor2 CCW
        digitalWrite(in1, LOW); // Motor1 off
        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);
        digitalWrite(in4, HIGH);
        dutyA = 0;
        dutyB = (uint32_t)((pwmPercent * 255) / 100);
        break;
      }

      ledcWrite(LEDC_CHANNEL_A, dutyA);
      ledcWrite(LEDC_CHANNEL_B, dutyB);
    }
  }
}
