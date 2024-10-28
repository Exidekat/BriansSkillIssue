#include <CodeCell.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

CodeCell myCodeCell;
Preferences preferences;
WebServer server(80);

const char* ssid = "GORDAN FREEMAN";
const char* password = "crowbars";

unsigned long previousMillis = 0;
unsigned long lastRunMillis = 0;
unsigned long lastPrintMillis = 0;
unsigned long lastBackupMillis = 0;
const long sensorInterval = 10; // 10ms interval for sensor reads
const long runInterval = 100;   // 100ms for myCodeCell.Run()
const long printInterval = 5000; // Print sensor data every 5000ms interval
const long backupInterval = 10000; // Backup data every 10000ms

int cycle = 0;
float gx, gy, gz;
float ax, ay, az;
float raw_ax, raw_ay, raw_az;
float mx, my, mz;
float roll, pitch, yaw;
float vx = 0, vy = 0, vz = 0;
float px = 0, py = 0, pz = 0;
float theta = 0; // Angle with respect to global north

// Kalman Filter Variables
float kalman_px = 0, kalman_py = 0, kalman_pz = 0;
float kalman_vx = 0, kalman_vy = 0, kalman_vz = 0;
float kalman_ax = 0, kalman_ay = 0, kalman_az = 0;
float p_estimate = 1.0;
float process_noise = 0.01;
float measurement_noise = 0.1;
float kalman_gain;

// Zero-Velocity Update (ZUPT) and Drift Correction Variables
bool isStationary = false;
float stationaryThreshold = 0.5; // Increased threshold to better detect stationary state
float gravityMagnitude = 9.8;

// Smoothing with Moving Average
const int smoothingWindow = 5;
float ax_history[smoothingWindow] = {0}, ay_history[smoothingWindow] = {0}, az_history[smoothingWindow] = {0};
int smoothingIndex = 0;

int motionState = 0;
int motionActivity = 0;

// Pushup variables
bool pushupPosition = false;
int pushupCount = 0;

// Run variables
short unsigned int stepCount = 0;
int stepCountComplement = 0; // used for reset
float timeSpentRunning = 0.0;

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>Fitness Data</h1>";
  html += "<p>Pushup Count: " + String(pushupCount) + "</p>";
  html += "<p>Step Count: " + String(stepCount) + "</p>";
  html += "<p>Time Spent Running: " + String(timeSpentRunning) + " seconds</p>";
  html += "<form action=\"/reset\" method=\"POST\"><button type=\"submit\">Reset</button></form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleReset() {
  pushupCount = 0;
  stepCountComplement = stepCount;
  stepCount = 0;
  timeSpentRunning = 0.0;
  preferences.begin("fitnessData", false);
  preferences.putInt("pushupCount", pushupCount);
  preferences.putUInt("stepCount", stepCount);
  preferences.putFloat("timeRunning", timeSpentRunning);
  preferences.end();
  server.send(200, "text/html", "<html><body><h1>Data Reset</h1><a href=\"/\">Go Back</a></body></html>");
}

void setup() {
  Serial.begin(115200); // Set Serial baud rate to 115200
  myCodeCell.Init(LIGHT + MOTION_ROTATION + MOTION_ACCELEROMETER + MOTION_GRAVITY + MOTION_GYRO + MOTION_MAGNETOMETER + MOTION_STEP_COUNTER + MOTION_STATE + MOTION_TAP_DETECTOR);
  myCodeCell.Run();

  // Load data from persistence
  preferences.begin("fitnessData", false);
  pushupCount = preferences.getInt("pushupCount", 0);
  stepCount = preferences.getUInt("stepCount", 0);
  timeSpentRunning = preferences.getFloat("timeRunning", 0.0);
  preferences.end();

  Serial.println("Loaded data from persistence:");
  Serial.printf("Pushup Count: %d\n", pushupCount);
  Serial.printf("Step Count: %d\n", stepCount);
  Serial.printf("Time Spent Running: %.2f seconds\n", timeSpentRunning);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi...");
  int wifiFailureCount = 0;
  while (WiFi.status() != WL_CONNECTED and wifiFailureCount < 50) {
    delay(500);
    Serial.print(".");
    wifiFailureCount++;
  }
  Serial.println("Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Set up web server routes
  server.on("/", handleRoot);
  server.on("/reset", HTTP_POST, handleReset);
  server.begin();
  Serial.println("Web server started");
}

void loop() {
  unsigned long currentMillis = millis();

  // Call myCodeCell.Run() every 100ms for power and battery management
  if (currentMillis - lastRunMillis >= runInterval) {
    lastRunMillis = currentMillis;
    //myCodeCell.Run();
  }

  // Read sensors every 10ms
  if (currentMillis - previousMillis >= sensorInterval) {
    previousMillis = currentMillis;
    
    // Read accelerometer and gravity data
    myCodeCell.Motion_AccelerometerRead(raw_ax, raw_ay, raw_az); // Outputs linear acceleration in m/s/s
    myCodeCell.Motion_GravityRead(gx, gy, gz); // Outputs gravity vector of magnitude 9.8 m/s/s

    // Calculate intermediate local frame acceleration by removing gravity effect
    float acc_local_x = raw_ax - gx;
    float acc_local_y = raw_ay - gy;
    float acc_local_z = raw_az - gz;

    // Calculate rotation angles based on gravity vector to obtain the orientation in global frame
    float pitch_angle = atan2(gx, sqrt(gy * gy + gz * gz));
    float roll_angle = atan2(gy, gz);

    // Convert accelerations from local frame to global frame using the rotation matrix
    float acc_global_x = acc_local_x * cos(pitch_angle) + acc_local_z * sin(pitch_angle);
    float acc_global_y = acc_local_x * sin(roll_angle) * sin(pitch_angle) + acc_local_y * cos(roll_angle) - acc_local_z * sin(roll_angle) * cos(pitch_angle);
    float acc_global_z = -acc_local_x * cos(roll_angle) * sin(pitch_angle) + acc_local_y * sin(roll_angle) + acc_local_z * cos(roll_angle) * cos(pitch_angle);

    // Ensure that acceleration values are within a realistic range
    acc_global_x = (fabs(acc_global_x) < 50) ? acc_global_x : 0;
    acc_global_y = (fabs(acc_global_y) < 50) ? acc_global_y : 0;
    acc_global_z = (fabs(acc_global_z) < 50) ? acc_global_z : 0;

    // Apply Moving Average for Smoothing
    ax_history[smoothingIndex] = acc_global_x;
    ay_history[smoothingIndex] = acc_global_y;
    az_history[smoothingIndex] = acc_global_z;
    smoothingIndex = (smoothingIndex + 1) % smoothingWindow;
    float acc_smoothed_x = 0, acc_smoothed_y = 0, acc_smoothed_z = 0;
    for (int i = 0; i < smoothingWindow; i++) {
      acc_smoothed_x += ax_history[i];
      acc_smoothed_y += ay_history[i];
      acc_smoothed_z += az_history[i];
    }
    acc_smoothed_x /= smoothingWindow;
    acc_smoothed_y /= smoothingWindow;
    acc_smoothed_z /= smoothingWindow;

    // Apply Kalman filter for acceleration
    kalman_gain = p_estimate / (p_estimate + measurement_noise);
    kalman_ax = kalman_ax + kalman_gain * (acc_smoothed_x - kalman_ax);
    kalman_ay = kalman_ay + kalman_gain * (acc_smoothed_y - kalman_ay);
    kalman_az = kalman_az + kalman_gain * (acc_smoothed_z - kalman_az);
    p_estimate = (1 - kalman_gain) * p_estimate + process_noise;

    // Check for stationary condition using Zero-Velocity Update (ZUPT)
    float accelerationMagnitude = sqrt(kalman_ax * kalman_ax + kalman_ay * kalman_ay + kalman_az * kalman_az);
    isStationary = fabs(accelerationMagnitude - gravityMagnitude) < stationaryThreshold;

    // If stationary, set velocities to zero
    if (isStationary) {
      kalman_vx = 0;
      kalman_vy = 0;
      kalman_vz = 0;
    } else {
      // Adjust velocity based on corrected acceleration in the global frame
      kalman_vx = kalman_vx * 0.9 + kalman_ax * sensorInterval / 1000;
      kalman_vy = kalman_vy * 0.9 + kalman_ay * sensorInterval / 1000;
      kalman_vz = kalman_vz * 0.9 + kalman_az * sensorInterval / 1000;
    }

    // Update position based on velocity
    kalman_px += kalman_vx * sensorInterval / 1000;
    kalman_py += kalman_vy * sensorInterval / 1000;
    kalman_pz += kalman_vz * sensorInterval / 1000;

    // Read magnetometer data for orientation
    myCodeCell.Motion_MagnetometerRead(mx, my, mz);
    theta = atan2(my, mx) * 180 / M_PI; // Calculate angle with respect to global north

    // Check if user is in pushup position.
    pushupPosition = (gz > 7);

    if (pushupPosition) {
      pushupCount++;
    }
    
    myCodeCell.Motion_StepCounterRead(stepCount);
    stepCount = stepCount - stepCountComplement;
                                 
    motionState = myCodeCell.Motion_StateRead();        // Reads the current state (On Table = 1, Stationary = 2, Stable = 3, Motion = 4)
    motionActivity = myCodeCell.Motion_ActivityRead();  // Reads the current activity (Driving = 1, Cycling = 2, Walking = 3/6, Still = 4, Tilting = 5, Running = 7, Climbing Stairs = 8)

    if (motionActivity >= 7) {
      timeSpentRunning += sensorInterval / 1000.0;
    }

  }

  // Print data every second
  if (currentMillis - lastPrintMillis >= printInterval) {
    lastPrintMillis = currentMillis;

    // Count cycles
    cycle++;

    // Print cycle count and sensor data in a more readable format
    Serial.println("====================================================");
    Serial.print("Cycle: "); Serial.println(cycle);
    Serial.println("----------------------------------------------------");
    Serial.printf("Gravity Vector (Gx, Gy, Gz): %.2f, %.2f, %.2f m/s^2\n", gx, gy, gz);
    Serial.printf("Raw Acceleration (Ax, Ay, Az): %.2f, %.2f, %.2f m/s^2\n", raw_ax, raw_ay, raw_az);
    Serial.printf("Smoothed Acceleration (Acc_Global_X, Acc_Global_Y, Acc_Global_Z): %.2f, %.2f, %.2f m/s^2\n", kalman_ax, kalman_ay, kalman_az);
    Serial.printf("Velocity (Vx, Vy, Vz): %.2f, %.2f, %.2f m/s\n", kalman_vx, kalman_vy, kalman_vz);
    Serial.printf("Position (Px, Py, Pz): %.2f, %.2f, %.2f m\n", kalman_px, kalman_py, kalman_pz);
    Serial.printf("Orientation (Theta): %.2f degrees\n", theta);
    Serial.printf("Stationary: %s\n", isStationary ? "Yes" : "No");
    Serial.println("----------------------------------------------------");

    Serial.printf("Pushup Position  : %s\n", pushupPosition ? "Yes" : "No");
    Serial.printf("Pushup Count     : %d\n", pushupCount);
    Serial.println("----------------------------------------------------");
    Serial.printf("Step Count       : %d\n", stepCount);
    Serial.printf("Time Running     : %.2f seconds\n", timeSpentRunning);
    Serial.println("----------------------------------------------------");
    Serial.printf("Motion State     : %d\n", motionState);
    Serial.printf("Motion Activity  : %d\n", motionActivity);

    Serial.println("====================================================\n");

  }

  // Backup data every 10 seconds
  if (currentMillis - lastBackupMillis >= backupInterval) {
    lastBackupMillis = currentMillis;

    // Save data to persistence
    preferences.begin("fitnessData", false);
    preferences.putInt("pushupCount", pushupCount);
    preferences.putUInt("stepCount", stepCount);
    preferences.putFloat("timeRunning", timeSpentRunning);
    preferences.end();

    Serial.println("Data saved to persistence.");
  }

  // Handle web server requests
  server.handleClient();
}
