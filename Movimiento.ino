#include "main.h"
#include "webinterface.h"
#include "walking.h"


// -------------------- WIFI / SERVER --------------------
const char* ssid     = "Humanoide_ESP8266";
const char* password = "12345678";
ESP8266WebServer server(80);
// IP: 192.168.4.1

// -------------------- SERVOS --------------------
Servo hipL, kneeL, ankleL;
Servo hipR, kneeR, ankleR;


// -------------------- STATE --------------------
RobotConfig robotConfig;
LegAngles currentLeftLeg = {robotConfig.hipLOffset, robotConfig.kneeLOffset, robotConfig.ankleLOffset};
LegAngles currentRightLeg = {robotConfig.hipROffset, robotConfig.kneeROffset, robotConfig.ankleROffset};

volatile bool shouldWalkForward = false;  // set by /forward handler



// -------------------- HANDLERS --------------------
void handleRoot() { server.send(200, "text/html", htmlPage(currentLeftLeg, currentRightLeg)); }
void handleKinematics() { server.send(200, "text/html", htmlKinematicsPage()); }

void handleSetServos() {
  Serial.println("Actualizando posiciones de servos:");
  if (server.hasArg("hipL"))   { currentLeftLeg.hip   = clampServoAngle(server.arg("hipL").toInt());   hipL.write(currentLeftLeg.hip);   Serial.println("Left Hip -> "   + server.arg("hipL")); }
  if (server.hasArg("kneeL"))  { currentLeftLeg.knee  = clampServoAngle(server.arg("kneeL").toInt());  kneeL.write(currentLeftLeg.knee);  Serial.println("Left Knee -> "  + server.arg("kneeL")); }
  if (server.hasArg("ankleL")) { currentLeftLeg.ankle = clampServoAngle(server.arg("ankleL").toInt()); ankleL.write(currentLeftLeg.ankle);Serial.println("Left Ankle -> " + server.arg("ankleL")); }
  if (server.hasArg("hipR"))   { currentRightLeg.hip   = clampServoAngle(server.arg("hipR").toInt());   hipR.write(currentRightLeg.hip);   Serial.println("Right Hip -> "   + server.arg("hipR")); }
  if (server.hasArg("kneeR"))  { currentRightLeg.knee  = clampServoAngle(server.arg("kneeR").toInt());  kneeR.write(currentRightLeg.knee);  Serial.println("Right Knee -> "  + server.arg("kneeR")); }
  if (server.hasArg("ankleR")) { currentRightLeg.ankle = clampServoAngle(server.arg("ankleR").toInt()); ankleR.write(currentRightLeg.ankle);Serial.println("Right Ankle -> " + server.arg("ankleR")); }

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleSetConfig() {
  Serial.println("Actualizando configuración del robot:");
  
  // Update offsets
  if (server.hasArg("hipLOffset"))   { robotConfig.hipLOffset = clampServoAngle(server.arg("hipLOffset").toInt());   Serial.println("Hip L Offset -> " + server.arg("hipLOffset")); }
  if (server.hasArg("kneeLOffset"))  { robotConfig.kneeLOffset = clampServoAngle(server.arg("kneeLOffset").toInt()); Serial.println("Knee L Offset -> " + server.arg("kneeLOffset")); }
  if (server.hasArg("ankleLOffset")) { robotConfig.ankleLOffset = clampServoAngle(server.arg("ankleLOffset").toInt()); Serial.println("Ankle L Offset -> " + server.arg("ankleLOffset")); }
  if (server.hasArg("hipROffset"))   { robotConfig.hipROffset = clampServoAngle(server.arg("hipROffset").toInt());   Serial.println("Hip R Offset -> " + server.arg("hipROffset")); }
  if (server.hasArg("kneeROffset"))  { robotConfig.kneeROffset = clampServoAngle(server.arg("kneeROffset").toInt()); Serial.println("Knee R Offset -> " + server.arg("kneeROffset")); }
  if (server.hasArg("ankleROffset")) { robotConfig.ankleROffset = clampServoAngle(server.arg("ankleROffset").toInt()); Serial.println("Ankle R Offset -> " + server.arg("ankleROffset")); }
  
  // Update leg geometry
  if (server.hasArg("l1")) { robotConfig.l1 = server.arg("l1").toFloat(); Serial.println("L1 -> " + server.arg("l1")); }
  if (server.hasArg("l2")) { robotConfig.l2 = server.arg("l2").toFloat(); Serial.println("L2 -> " + server.arg("l2")); }
  
  // Update step parameters
  if (server.hasArg("stepClearance")) { robotConfig.stepClearance = server.arg("stepClearance").toFloat(); Serial.println("Step Clearance -> " + server.arg("stepClearance")); }
  if (server.hasArg("stepHeight")) { robotConfig.stepHeight = server.arg("stepHeight").toFloat(); Serial.println("Step Height -> " + server.arg("stepHeight")); }
  if (server.hasArg("stepLength")) { robotConfig.stepLength = server.arg("stepLength").toFloat(); Serial.println("Step Length -> " + server.arg("stepLength")); }
  if (server.hasArg("stepVelocity")) { robotConfig.stepVelocity = server.arg("stepVelocity").toInt(); Serial.println("Step Velocity -> " + server.arg("stepVelocity")); }
  if (server.hasArg("stepOffset")) { robotConfig.stepOffset = server.arg("stepOffset").toFloat(); Serial.println("Step Offset -> " + server.arg("stepOffset")); }
  if (server.hasArg("numSteps")) { robotConfig.numSteps = server.arg("numSteps").toInt(); Serial.println("Number of Steps -> " + server.arg("numSteps")); }

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleForward()  { Serial.println("UI: Adelante"); shouldWalkForward = true;  server.sendHeader("Location", "/"); server.send(303); }
void handleBackward() { Serial.println("UI: Atras");                       server.sendHeader("Location", "/"); server.send(303); }
void handleLeft()     { Serial.println("UI: Izquierda");                   server.sendHeader("Location", "/"); server.send(303); }
void handleRight()    { Serial.println("UI: Derecha");                     server.sendHeader("Location", "/"); server.send(303); }

// -------------------- INITIALIZATION --------------------
void initializeWiFi() {
  Serial.println("\nIniciando Access Point...");
  WiFi.softAP(ssid, password);
  Serial.print("IP del AP: "); 
  Serial.println(WiFi.softAPIP());
  Serial.println("IP: 192.168.4.1");
}

void initializeServos() {
  // Attach servos to pins
  hipR.attach(HIP_R_PIN); 
  kneeR.attach(KNEE_R_PIN); 
  ankleR.attach(ANKLE_R_PIN);
  hipL.attach(HIP_L_PIN); 
  kneeL.attach(KNEE_L_PIN); 
  ankleL.attach(ANKLE_L_PIN);

  // Initialize servos to standing position
  hipL.write(robotConfig.hipLOffset); 
  kneeL.write(robotConfig.kneeLOffset); 
  ankleL.write(robotConfig.ankleLOffset);
  hipR.write(robotConfig.hipROffset); 
  kneeR.write(robotConfig.kneeROffset); 
  ankleR.write(robotConfig.ankleROffset);
  
  // Initialize current leg positions
  currentLeftLeg = {robotConfig.hipLOffset, robotConfig.kneeLOffset, robotConfig.ankleLOffset};
  currentRightLeg = {robotConfig.hipROffset, robotConfig.kneeROffset, robotConfig.ankleROffset};
  
  delay(300); // Allow servos to reach initial position
  Serial.println("Servos inicializados en posición de pie");
}

void initializeWebServer() {
  server.on("/", handleRoot);
  server.on("/kinematics", handleKinematics);
  server.on("/setservos", handleSetServos);
  server.on("/setconfig", handleSetConfig);
  server.on("/forward", handleForward);
  server.on("/backward", handleBackward);
  server.on("/left", handleLeft);
  server.on("/right", handleRight);

  server.begin();
  Serial.println("Servidor web iniciado.");
}

void initializeSystem() {
  Serial.begin(115200);
  
  initializeWiFi();
  initializeServos();
  initializeWebServer();
  
  Serial.println("Sistema inicializado completamente");
}

// -------------------- SETUP --------------------
void setup() {
  initializeSystem();
}

// -------------------- LOOP --------------------
void loop() {
  server.handleClient();

  // run any requested action non-blocking from the main loop
  if (shouldWalkForward) {
    Serial.println("== WALK FORWARD ==");
    Serial.print("Taking ");
    Serial.print(robotConfig.numSteps);
    Serial.println(" step(s)");
    
    for (int i = 0; i < robotConfig.numSteps; i++) {
      Serial.print("Step ");
      Serial.print(i + 1);
      Serial.print(" of ");
      Serial.println(robotConfig.numSteps);
      takeStep(robotConfig.stepLength, robotConfig.stepVelocity);
    }
    
    // moveToStandingPosition();
    shouldWalkForward = false;
    Serial.println("== DONE ==");
  }
}
