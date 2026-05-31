#include <PRIZM.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define rxPin 2
#define txPin 9

SoftwareSerial espSerial(rxPin, txPin);
PRIZM prizm;

// ==============================
// Robot identity / status
// ==============================
const char *ROBOT_ID = "robot_A";
const char *robot_state = "idle";
int current_path_id = -1;
int current_waypoint_index = -1;

// ==============================
// Robot geometry
// ==============================
const float WHEEL_DIAMETER_CM = 10.16; // 4 inches
const float WHEEL_BASE_CM = 26.035;    // 10.25 inches
const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const int GRIPPER_SERVO_ID = 1;
const int GRIPPER_OPEN_DEG = 100;
const int GRIPPER_CLOSED_DEG = 35;
const int GRIPPER_SERVO_SPEED_PERCENT = 75;
// Re-assert the servo target for this long after a toggle so a dropped or
// interrupted I2C write still reaches the controller and the servo settles.
const unsigned long GRIPPER_SETTLE_MS = 700;

// ==============================
// Pose state
// ==============================
float x_cm = 200.0;
float y_cm = 100.0;
float theta_rad = PI / 2.0;

long prevLeftDeg = 0;
long prevRightDeg = 0;

// ==============================
// Path storage
// ==============================
const int MAX_WAYPOINTS = 12;
int16_t waypoint_xs[MAX_WAYPOINTS];
int16_t waypoint_ys[MAX_WAYPOINTS];
int waypoint_count = 0;

// ==============================
// Motion / execution state
// ==============================
enum PrimitiveType
{
    PRIM_NONE,
    PRIM_TURN,
    PRIM_DRIVE,
    PRIM_CONT_DRIVE
};

PrimitiveType active_primitive = PRIM_NONE;
bool path_loaded = false;
bool path_paused = false;
bool path_started_sent = false;
bool gripper_closed = false;
int gripper_target_deg = GRIPPER_OPEN_DEG;
unsigned long gripper_settle_until_ms = 0;

// Wall-scan state machine (see beginObstacleScan/updateScan).
bool scanning = false;
int scan_index = 0;
bool scan_recentering = false;
bool scan_backing = false;

const float POSITION_TOLERANCE_CM = 2.0;
const float HEADING_TOLERANCE_DEG = 4.0;

int turn_speed_deg_per_sec = 150;
int drive_speed_deg_per_sec = 200;

// ==============================
// Serial receive buffer
// ==============================
// The PRIZM is an ATmega328P with only 2 KB of SRAM, so this buffer stays at
// 512 bytes -- growing it crashes the board. The arbiter caps each
// path_assignment (MAX_WAYPOINTS_PER_ASSIGNMENT) so messages stay well under
// this size. The length counter MUST be wide enough to index the whole buffer:
// a uint8_t wraps at 256 and silently corrupts any message longer than 255
// bytes, which is what was dropping most dispatch commands.
const int SERIAL_LINE_BUFFER_SIZE = 512;
char serialLineBuffer[SERIAL_LINE_BUFFER_SIZE];
uint16_t serialLineLength = 0;
bool serialLineOverflow = false;

// ==============================
// Timing
// ==============================
unsigned long lastTelemetrySendMs = 0;
const unsigned long TELEMETRY_PERIOD_MS = 250;
unsigned long lastSensorReadMs = 0;
// Read the forward ultrasonics at 25 Hz. With OBSTACLE_CONFIRM_SAMPLES=2 this
// puts the worst-case time from an obstacle appearing to the wheels stopping at
// ~80 ms (was ~160 ms at 80 ms/sample), so the robot reacts before it hits.
const unsigned long SENSOR_READ_PERIOD_MS = 40;
const unsigned long CLAW_SENSOR_READ_PERIOD_MS = 500;
const unsigned long SONIC_TIMEOUT_US = 8000;
// Obstacle confirmation: a forward sensor must read within its clearance for
// this many consecutive samples before we treat it as a real obstacle. This
// rejects single-sample dropouts/spikes from the ultrasonics.
const int OBSTACLE_CONFIRM_SAMPLES = 2;
// If a sensor returns no echo, hold its last valid reading for this long
// before declaring the reading stale (-1).
const unsigned long SENSOR_VALID_HOLD_MS = 500;
int cached_left_ultrasonic_cm = -1;
int cached_right_ultrasonic_cm = -1;
int cached_front_ultrasonic_cm = -1;
int cached_claw_ultrasonic_cm = -1;
const bool USE_ULTRASONIC_SENSORS = true;
const int RIGHT_FORWARD_ULTRASONIC_PIN = 3;
const int CLAW_ULTRASONIC_PIN = 4;
const int LEFT_FORWARD_ULTRASONIC_PIN = 5;
// Stop clearances measured at the two forward ultrasonics, in cm.
// Each side has its own target gap, and the gap shrinks when the claw is
// closed (the claw extends the robot's reach, so it can approach closer).
const float LEFT_STOP_CLAW_OPEN_CM = 18.5;
const float LEFT_STOP_CLAW_CLOSED_CM = 13.0;
const float RIGHT_STOP_CLAW_OPEN_CM = 15.5;
const float RIGHT_STOP_CLAW_CLOSED_CM = 10.0;
const int DEFAULT_CONTINUOUS_MOTOR_POWER = 35;
// Wall-scan sweep: on hitting a wall mid-path, the robot pivots from
// -HALF_SWEEP to +HALF_SWEEP, pausing to take SCAN_SAMPLES readings.
// Sweep a 90 deg cone (+/-45) so the arbiter can see where the wall ends, but
// take only 3 samples (left, center, right) so the scan turns smoothly instead
// of stop-starting through many tiny steps.
const float SCAN_HALF_SWEEP_DEG = 45.0;
const int SCAN_SAMPLES = 3;
const float SCAN_STEP_DEG = (2.0 * SCAN_HALF_SWEEP_DEG) / (SCAN_SAMPLES - 1);
// If we stop this close to a wall, reverse a little first so there is room to
// pivot and re-approach at an angle instead of grinding nose-against-wall.
const int SCAN_BACKUP_TRIGGER_CM = 18;
const float SCAN_BACKUP_DISTANCE_CM = 8.0;
int cached_front_ir_raw = -1;
int cached_left_ir_raw = -1;
int cached_front_ir_cm = -1;
int cached_left_ir_cm = -1;
unsigned long lastClawSensorReadMs = 0;
// Forward ultrasonic filter state (per side).
unsigned long leftUltraValidMs = 0;
unsigned long rightUltraValidMs = 0;
int leftCloseStreak = 0;
int rightCloseStreak = 0;

// ==============================
// Helpers
// ==============================
float normalizeAngle(float angle)
{
    while (angle > PI)
        angle -= 2.0 * PI;
    while (angle < -PI)
        angle += 2.0 * PI;
    return angle;
}

float radToDeg(float angle_rad)
{
    return angle_rad * 180.0 / PI;
}

float degToRad(float angle_deg)
{
    return angle_deg * PI / 180.0;
}

int cmToMotorDegrees(float distance_cm)
{
    return (int)round((distance_cm / WHEEL_CIRCUMFERENCE_CM) * 360.0);
}

int robotTurnDegToMotorDegrees(float robot_turn_deg)
{
    float robot_turn_rad = robot_turn_deg * PI / 180.0;
    float wheel_travel_cm = (robot_turn_rad * WHEEL_BASE_CM) / 2.0;
    return cmToMotorDegrees(wheel_travel_cm);
}

void setRobotState(const char *new_state)
{
    robot_state = new_state;
}

void resetEncoderTracking()
{
    prizm.resetEncoders();
    prevLeftDeg = 0;
    prevRightDeg = 0;
}

float distanceToWaypoint(float target_x, float target_y)
{
    float dx = target_x - x_cm;
    float dy = target_y - y_cm;
    return sqrt(dx * dx + dy * dy);
}

float headingToWaypointRad(float target_x, float target_y)
{
    float dx = target_x - x_cm;
    float dy = target_y - y_cm;
    return atan2(dy, dx);
}

float headingErrorDeg(float target_heading_rad)
{
    float err = normalizeAngle(target_heading_rad - theta_rad);
    return radToDeg(err);
}

// BroadCast Helpers
void broadcastPrint(const __FlashStringHelper *msg)
{
    Serial.print(msg);
    espSerial.print(msg);
}

void broadcastPrint(const char *msg)
{
    Serial.print(msg);
    espSerial.print(msg);
}

void broadcastPrintFloat(float val, int decimals)
{
    Serial.print(val, decimals);
    espSerial.print(val, decimals);
}

void broadcastPrintInt(int val)
{
    Serial.print(val);
    espSerial.print(val);
}

void broadcastPrintLn(const __FlashStringHelper *msg)
{
    Serial.println(msg);
    espSerial.println(msg);
}

void broadcastPrintULong(unsigned long val)
{
    Serial.print(val);
    espSerial.print(val);
}

// ==============================
// Odometry
// ==============================
void updatePoseFromEncoders(long leftDeg, long rightDeg)
{
    long deltaLeftDeg = leftDeg - prevLeftDeg;
    long deltaRightDeg = rightDeg - prevRightDeg;

    prevLeftDeg = leftDeg;
    prevRightDeg = rightDeg;

    // Convert motor degrees to wheel travel in cm
    float dL = ((float)deltaLeftDeg / 360.0) * WHEEL_CIRCUMFERENCE_CM;
    float dR = ((float)deltaRightDeg / 360.0) * WHEEL_CIRCUMFERENCE_CM;

    // Motor 2 positive means left wheel backward, so flip left side
    dL = -dL;

    float dCenter = (dL + dR) / 2.0;
    float dTheta = (dR - dL) / WHEEL_BASE_CM;

    float thetaMid = theta_rad + dTheta / 2.0;

    x_cm += dCenter * cos(thetaMid);
    y_cm += dCenter * sin(thetaMid);
    theta_rad = normalizeAngle(theta_rad + dTheta);
}

void updateOdometry()
{
    long leftDeg = prizm.readEncoderDegrees(2);  // motor 2 = left wheel
    long rightDeg = prizm.readEncoderDegrees(1); // motor 1 = right wheel
    updatePoseFromEncoders(leftDeg, rightDeg);
}

// ==============================
// Telemetry
// ==============================
int closestValidDistance(int a, int b)
{
    if (a > 0 && b > 0)
        return min(a, b);
    if (a > 0)
        return a;
    if (b > 0)
        return b;
    return -1;
}

int readSonicSensorCMFast(int pin)
{
    delayMicroseconds(300);
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);

    unsigned long duration = pulseIn(pin, HIGH, SONIC_TIMEOUT_US);
    if (duration == 0)
        return -1;
    return (int)(duration / 29 / 2);
}

// Fold one raw ultrasonic sample into the filtered state for one side.
// Holds the last valid reading across brief dropouts and requires a streak of
// in-clearance samples before the obstacle is considered confirmed.
void updateForwardFilter(int raw, int &cached, unsigned long &lastValidMs,
                         int &closeStreak, float clearanceCm, unsigned long now)
{
    if (raw > 0)
    {
        cached = raw;
        lastValidMs = now;
        if ((float)raw <= clearanceCm)
        {
            if (closeStreak < OBSTACLE_CONFIRM_SAMPLES)
                closeStreak++;
        }
        else
        {
            closeStreak = 0;
        }
    }
    else if (now - lastValidMs > SENSOR_VALID_HOLD_MS)
    {
        cached = -1;
        closeStreak = 0;
    }
}

void maybeUpdateSensors()
{
    unsigned long now = millis();
    if (now - lastSensorReadMs < SENSOR_READ_PERIOD_MS)
        return;

    // Only pay the blocking pulseIn cost while actually driving forward. When
    // idle/turning the loop stays fast so GUI commands are acted on promptly.
    bool active_drive =
        (active_primitive == PRIM_DRIVE || active_primitive == PRIM_CONT_DRIVE);

    if (USE_ULTRASONIC_SENSORS && active_drive)
    {
        int raw_right = readSonicSensorCMFast(RIGHT_FORWARD_ULTRASONIC_PIN);
        int raw_left = readSonicSensorCMFast(LEFT_FORWARD_ULTRASONIC_PIN);
        updateForwardFilter(raw_right, cached_right_ultrasonic_cm, rightUltraValidMs,
                            rightCloseStreak, rightStopClearanceCm(), now);
        updateForwardFilter(raw_left, cached_left_ultrasonic_cm, leftUltraValidMs,
                            leftCloseStreak, leftStopClearanceCm(), now);
        if (now - lastClawSensorReadMs >= CLAW_SENSOR_READ_PERIOD_MS)
        {
            cached_claw_ultrasonic_cm = readSonicSensorCMFast(CLAW_ULTRASONIC_PIN);
            lastClawSensorReadMs = now;
        }
        cached_front_ultrasonic_cm = closestValidDistance(cached_right_ultrasonic_cm, cached_left_ultrasonic_cm);
    }
    else
    {
        cached_left_ultrasonic_cm = -1;
        cached_right_ultrasonic_cm = -1;
        cached_front_ultrasonic_cm = -1;
        cached_claw_ultrasonic_cm = -1;
        leftCloseStreak = 0;
        rightCloseStreak = 0;
    }
    cached_front_ir_raw = -1;
    cached_left_ir_raw = -1;
    cached_front_ir_cm = -1;
    cached_left_ir_cm = -1;
    lastSensorReadMs = now;
}

void printPoseJSON()
{
    unsigned long t_ms = millis();
    float theta_deg = radToDeg(theta_rad);

    broadcastPrint(F("{\"type\":\"telemetry\""));
    broadcastPrint(F(",\"robot_id\":\""));
    broadcastPrint(ROBOT_ID);

    broadcastPrint(F("\""));
    broadcastPrint(F(",\"state\":\""));
    broadcastPrint(robot_state);

    broadcastPrint(F("\""));
    broadcastPrint(F(",\"path_id\":"));
    broadcastPrintInt(current_path_id);

    broadcastPrint(F(",\"waypoint_index\":"));
    broadcastPrintInt(current_waypoint_index);

    broadcastPrint(F(",\"t_ms\":"));
    broadcastPrintULong(t_ms);

    broadcastPrint(F(",\"x_cm\":"));
    broadcastPrintFloat(x_cm, 2);

    broadcastPrint(F(",\"y_cm\":"));
    broadcastPrintFloat(y_cm, 2);

    broadcastPrint(F(",\"theta_deg\":"));
    broadcastPrintFloat(theta_deg, 2);

    broadcastPrint(F(",\"front_ultrasonic_cm\":"));
    broadcastPrintInt(cached_front_ultrasonic_cm);

    broadcastPrint(F(",\"left_ultrasonic_cm\":"));
    broadcastPrintInt(cached_left_ultrasonic_cm);

    broadcastPrint(F(",\"right_ultrasonic_cm\":"));
    broadcastPrintInt(cached_right_ultrasonic_cm);

    broadcastPrint(F(",\"claw_ultrasonic_cm\":"));
    broadcastPrintInt(cached_claw_ultrasonic_cm);

    broadcastPrint(F(",\"front_ir_raw\":"));
    broadcastPrintInt(cached_front_ir_raw);

    broadcastPrint(F(",\"left_ir_raw\":"));
    broadcastPrintInt(cached_left_ir_raw);

    broadcastPrint(F(",\"front_ir_cm\":"));
    broadcastPrintInt(cached_front_ir_cm);

    broadcastPrint(F(",\"left_ir_cm\":"));
    broadcastPrintInt(cached_left_ir_cm);

    broadcastPrintLn(F("}"));
}

void maybeSendTelemetry()
{
    // Don't transmit if there's incoming data waiting
    if (espSerial.available() > 0)
    {
        lastTelemetrySendMs = millis(); // reset timer, try again next cycle
        return;
    }

    unsigned long now = millis();
    if (now - lastTelemetrySendMs >= TELEMETRY_PERIOD_MS)
    {
        printPoseJSON();
        lastTelemetrySendMs = now;
    }
}

// ==============================
// Low-level motion primitives
// ==============================
void stopMotorsNow()
{
    // Explicitly zero any in-flight motor-degree command before cutting power.
    prizm.setMotorDegrees(0, 0, 0, 0);
    prizm.setMotorPower(1, 0);
    prizm.setMotorPower(2, 0);
    active_primitive = PRIM_NONE;
}

void interruptActivePrimitive()
{
    if (active_primitive != PRIM_NONE)
    {
        updateOdometry();
    }

    stopMotorsNow();
    resetEncoderTracking();
    scanning = false;
    scan_recentering = false;
    scan_backing = false;
    scan_index = 0;
}

void startDriveStraight(float distance_cm)
{
    int motor_deg = cmToMotorDegrees(distance_cm);

    resetEncoderTracking();
    prizm.setMotorDegrees(drive_speed_deg_per_sec, motor_deg,
                          drive_speed_deg_per_sec, -motor_deg);

    active_primitive = PRIM_DRIVE;
    setRobotState("executing_path");
}

void startContinuousDriveForward(int motor_power)
{
    interruptActivePrimitive();
    clearCurrentPath();
    motor_power = constrain(abs(motor_power), 15, 70);
    resetEncoderTracking();
    prizm.setMotorPower(1, motor_power);
    prizm.setMotorPower(2, -motor_power);
    active_primitive = PRIM_CONT_DRIVE;
    setRobotState("manual_forward");
}

void startTurnInPlace(float robot_turn_deg)
{
    int motor_deg = robotTurnDegToMotorDegrees(fabs(robot_turn_deg));

    resetEncoderTracking();

    if (robot_turn_deg > 0)
    {
        prizm.setMotorDegrees(turn_speed_deg_per_sec, motor_deg,
                              turn_speed_deg_per_sec, motor_deg);
    }
    else
    {
        prizm.setMotorDegrees(turn_speed_deg_per_sec, -motor_deg,
                              turn_speed_deg_per_sec, -motor_deg);
    }

    active_primitive = PRIM_TURN;
    setRobotState("executing_path");
}

bool motorsBusy()
{
    return (prizm.readMotorBusy(1) == 1 || prizm.readMotorBusy(2) == 1);
}

float leftStopClearanceCm()
{
    return gripper_closed ? LEFT_STOP_CLAW_CLOSED_CM : LEFT_STOP_CLAW_OPEN_CM;
}

float rightStopClearanceCm()
{
    return gripper_closed ? RIGHT_STOP_CLAW_CLOSED_CM : RIGHT_STOP_CLAW_OPEN_CM;
}

bool leftObstacleDetected()
{
    return leftCloseStreak >= OBSTACLE_CONFIRM_SAMPLES;
}

bool rightObstacleDetected()
{
    return rightCloseStreak >= OBSTACLE_CONFIRM_SAMPLES;
}

bool frontObstacleDetected()
{
    return leftObstacleDetected() || rightObstacleDetected();
}

// Sweep the two forward ultrasonics across the wall so the arbiter learns its
// extent, then ask for a replan. The robot pivots to one side, takes a reading
// at each of SCAN_SAMPLES headings, and recenters before reporting needs_replan.
void beginObstacleScan()
{
    interruptActivePrimitive();
    path_paused = true;
    scanning = true;
    scan_index = 0;
    scan_recentering = false;
    scan_backing = false;
    setRobotState("scanning");
    sendStatus("scanning", "wall_scan_start");

    // Reverse first if we are pinned against the wall, then sweep.
    if (cached_front_ultrasonic_cm > 0 && cached_front_ultrasonic_cm < SCAN_BACKUP_TRIGGER_CM)
    {
        scan_backing = true;
        startDriveStraight(-SCAN_BACKUP_DISTANCE_CM);
    }
    else
    {
        startTurnInPlace(-SCAN_HALF_SWEEP_DEG);
    }
}

void captureScanSample()
{
    cached_right_ultrasonic_cm = readSonicSensorCMFast(RIGHT_FORWARD_ULTRASONIC_PIN);
    cached_left_ultrasonic_cm = readSonicSensorCMFast(LEFT_FORWARD_ULTRASONIC_PIN);
    cached_front_ultrasonic_cm =
        closestValidDistance(cached_right_ultrasonic_cm, cached_left_ultrasonic_cm);
    printPoseJSON();
}

void updateScan()
{
    updateOdometry();
    if (motorsBusy())
        return;

    if (scan_backing)
    {
        // Finished reversing; now we have room to sweep.
        scan_backing = false;
        startTurnInPlace(-SCAN_HALF_SWEEP_DEG);
        return;
    }

    if (scan_recentering)
    {
        scanning = false;
        active_primitive = PRIM_NONE;
        leftCloseStreak = 0;
        rightCloseStreak = 0;
        setRobotState("needs_replan");
        sendStatus("needs_replan", "wall_scan_complete");
        printPoseJSON();
        return;
    }

    captureScanSample();
    scan_index++;
    if (scan_index < SCAN_SAMPLES)
    {
        startTurnInPlace(SCAN_STEP_DEG);
    }
    else
    {
        scan_recentering = true;
        startTurnInPlace(-SCAN_HALF_SWEEP_DEG);
    }
}

// A confirmed forward obstacle is a hard stop in both drive modes. Manual
// continuous driving simply halts and reports "blocked". Autonomous path
// driving scans the wall and asks the arbiter to replan around it.
void maybeReactiveAvoidance()
{
    if (scanning)
        return;
    if (active_primitive != PRIM_DRIVE && active_primitive != PRIM_CONT_DRIVE)
        return;
    if (!frontObstacleDetected())
        return;

    if (active_primitive == PRIM_CONT_DRIVE)
    {
        interruptActivePrimitive();
        setRobotState("blocked");
        sendStatus("blocked", "continuous_drive_obstacle");
        printPoseJSON();
        return;
    }

    beginObstacleScan();
}

// ==============================
// Simple JSON field parsing
// ==============================
const char *findFieldValueStart(const char *json, const char *key)
{
    static char pattern[32];
    snprintf(pattern, sizeof(pattern), "\"%s\":", key);

    const char *start = strstr(json, pattern);
    if (start == NULL)
        return NULL;

    return start + strlen(pattern);
}

bool extractStringField(const char *json, const char *key, char *outVal, size_t outSize)
{
    const char *valueStart = findFieldValueStart(json, key);
    if (valueStart == NULL || *valueStart != '"' || outSize == 0)
        return false;

    valueStart++;
    const char *valueEnd = strchr(valueStart, '"');
    if (valueEnd == NULL)
        return false;

    size_t copyLen = valueEnd - valueStart;
    if (copyLen >= outSize)
    {
        copyLen = outSize - 1;
    }

    memcpy(outVal, valueStart, copyLen);
    outVal[copyLen] = '\0';
    return true;
}

bool jsonHasType(const char *json, const char *typeValue)
{
    char actualType[20];
    return extractStringField(json, "type", actualType, sizeof(actualType)) && strcmp(actualType, typeValue) == 0;
}

bool jsonTargetsThisRobot(const char *json)
{
    char targetRobot[20];
    return extractStringField(json, "robot_id", targetRobot, sizeof(targetRobot)) && strcmp(targetRobot, ROBOT_ID) == 0;
}

bool extractIntField(const char *json, const char *key, int &outVal)
{
    const char *valueStart = findFieldValueStart(json, key);
    if (valueStart == NULL)
        return false;

    char *valueEnd = NULL;
    long parsed = strtol(valueStart, &valueEnd, 10);
    if (valueEnd == valueStart)
        return false;

    outVal = (int)parsed;
    return true;
}

bool extractFloatField(const char *json, const char *key, float &outVal)
{
    const char *valueStart = findFieldValueStart(json, key);
    if (valueStart == NULL)
        return false;

    char *valueEnd = NULL;
    float parsed = (float)strtod(valueStart, &valueEnd);
    if (valueEnd == valueStart)
        return false;

    outVal = parsed;
    return true;
}

bool extractBoolField(const char *json, const char *key, bool &outVal)
{
    const char *valueStart = findFieldValueStart(json, key);
    if (valueStart == NULL)
        return false;

    if (strncmp(valueStart, "true", 4) == 0)
    {
        outVal = true;
        return true;
    }

    if (strncmp(valueStart, "false", 5) == 0)
    {
        outVal = false;
        return true;
    }

    return false;
}

int extractWaypoints(const char *json)
{
    int count = 0;
    const char *searchPos = json;

    while (count < MAX_WAYPOINTS)
    {
        const char *xKey = strstr(searchPos, "\"x_cm\":");
        if (xKey == NULL)
            break;
        xKey += 7;

        char *xEnd = NULL;
        waypoint_xs[count] = (int16_t)lround(strtod(xKey, &xEnd));
        if (xEnd == xKey)
            break;

        const char *yKey = strstr(xEnd, "\"y_cm\":");
        if (yKey == NULL)
            break;
        yKey += 7;

        char *yEnd = NULL;
        waypoint_ys[count] = (int16_t)lround(strtod(yKey, &yEnd));
        if (yEnd == yKey)
            break;

        count++;
        searchPos = yEnd;
    }

    return count;
}

// ==============================
// Command handling
// ==============================
void clearCurrentPath()
{
    waypoint_count = 0;
    current_waypoint_index = -1;
    current_path_id = -1;
    path_loaded = false;
    path_started_sent = false;
    active_primitive = PRIM_NONE;
}

void sendAck(const char *forType)
{
    broadcastPrint(F("{\"type\":\"ack\""));
    broadcastPrint(F(",\"robot_id\":\""));
    broadcastPrint(ROBOT_ID);
    broadcastPrint(F("\""));
    broadcastPrint(F(",\"for\":\""));
    broadcastPrint(forType);
    broadcastPrint(F("\""));
    broadcastPrint(F(",\"path_id\":"));
    broadcastPrintInt(current_path_id);
    broadcastPrint(F(",\"t_ms\":"));
    broadcastPrintULong(millis());
    broadcastPrintLn(F("}"));
}

void sendStatus(const char *state, const char *reason)
{
    broadcastPrint(F("{\"type\":\"status\""));
    broadcastPrint(F(",\"robot_id\":\""));
    broadcastPrint(ROBOT_ID);
    broadcastPrint(F("\""));
    broadcastPrint(F(",\"state\":\""));
    broadcastPrint(state);
    broadcastPrint(F("\""));
    broadcastPrint(F(",\"path_id\":"));
    broadcastPrintInt(current_path_id);
    broadcastPrint(F(",\"waypoint_index\":"));
    broadcastPrintInt(current_waypoint_index);
    broadcastPrint(F(",\"reason\":\""));
    broadcastPrint(reason);
    broadcastPrint(F("\""));
    broadcastPrint(F(",\"t_ms\":"));
    broadcastPrintULong(millis());
    broadcastPrintLn(F("}"));
}

void sendPathStarted()
{
    broadcastPrint(F("{\"type\":\"path_started\""));
    broadcastPrint(F(",\"robot_id\":\""));
    broadcastPrint(ROBOT_ID);
    broadcastPrint(F("\""));
    broadcastPrint(F(",\"path_id\":"));
    broadcastPrintInt(current_path_id);
    broadcastPrint(F(",\"t_ms\":"));
    broadcastPrintULong(millis());
    broadcastPrintLn(F("}"));
}

void sendWaypointReached()
{
    broadcastPrint(F("{\"type\":\"waypoint_reached\""));
    broadcastPrint(F(",\"robot_id\":\""));
    broadcastPrint(ROBOT_ID);
    broadcastPrint(F("\""));
    broadcastPrint(F(",\"path_id\":"));
    broadcastPrintInt(current_path_id);
    broadcastPrint(F(",\"waypoint_index\":"));
    broadcastPrintInt(current_waypoint_index);
    broadcastPrint(F(",\"t_ms\":"));
    broadcastPrintULong(millis());
    broadcastPrint(F(",\"x_cm\":"));
    broadcastPrintFloat(x_cm, 2);
    broadcastPrint(F(",\"y_cm\":"));
    broadcastPrintFloat(y_cm, 2);
    broadcastPrint(F(",\"theta_deg\":"));
    broadcastPrintFloat(radToDeg(theta_rad), 2);
    broadcastPrintLn(F("}"));
}

void sendPathComplete()
{
    broadcastPrint(F("{\"type\":\"path_complete\""));
    broadcastPrint(F(",\"robot_id\":\""));
    broadcastPrint(ROBOT_ID);
    broadcastPrint(F("\""));
    broadcastPrint(F(",\"path_id\":"));
    broadcastPrintInt(current_path_id);
    broadcastPrint(F(",\"t_ms\":"));
    broadcastPrintULong(millis());
    broadcastPrint(F(",\"x_cm\":"));
    broadcastPrintFloat(x_cm, 2);
    broadcastPrint(F(",\"y_cm\":"));
    broadcastPrintFloat(y_cm, 2);
    broadcastPrint(F(",\"theta_deg\":"));
    broadcastPrintFloat(radToDeg(theta_rad), 2);
    broadcastPrintLn(F("}"));
}

void handlePathAssignment(const char *json)
{
    if (!jsonTargetsThisRobot(json))
        return;

    int newPathId = -1;
    extractIntField(json, "path_id", newPathId);

    int newTurnSpeed = turn_speed_deg_per_sec;
    int newDriveSpeed = drive_speed_deg_per_sec;

    if (!extractIntField(json, "turn_speed_deg_per_sec", newTurnSpeed))
    {
        const char *motionBlock = strstr(json, "\"motion\"");
        if (motionBlock)
        {
            extractIntField(motionBlock, "turn_speed_deg_per_sec", newTurnSpeed);
        }
    }
    if (!extractIntField(json, "drive_speed_deg_per_sec", newDriveSpeed))
    {
        const char *motionBlock = strstr(json, "\"motion\"");
        if (motionBlock)
        {
            extractIntField(motionBlock, "drive_speed_deg_per_sec", newDriveSpeed);
        }
    }

    bool replaceExisting = true;
    extractBoolField(json, "replace_existing", replaceExisting);

    if (!replaceExisting && (path_loaded || active_primitive != PRIM_NONE))
    {
        sendStatus("blocked", "path_rejected_busy");
        return;
    }

    int newWaypointCount = extractWaypoints(json);
    if (newWaypointCount <= 0)
    {
        sendStatus("error", "bad_path_assignment");
        return;
    }

    if (replaceExisting && (path_loaded || active_primitive != PRIM_NONE))
    {
        interruptActivePrimitive();
    }

    waypoint_count = newWaypointCount;
    current_path_id = newPathId;
    current_waypoint_index = 0;
    turn_speed_deg_per_sec = newTurnSpeed;
    drive_speed_deg_per_sec = newDriveSpeed;

    path_loaded = true;
    path_paused = false;
    path_started_sent = false;
    active_primitive = PRIM_NONE;
    setRobotState("idle");

    sendAck("path_assignment");
    sendStatus("idle", "path_loaded");
    printPoseJSON();
}

void performPause()
{
    if (path_loaded || active_primitive != PRIM_NONE)
    {
        interruptActivePrimitive();
    }

    path_paused = true;
    setRobotState("paused");
    sendAck("pause");
    sendStatus("paused", "pause_requested");
    printPoseJSON();
}

void handlePause(const char *json)
{
    if (!jsonTargetsThisRobot(json))
        return;
    performPause();
}

void performResume()
{
    path_paused = false;
    if (path_loaded)
    {
        setRobotState("idle");
    }
    sendAck("resume");
    sendStatus(robot_state, "resume_requested");
}

void handleResume(const char *json)
{
    if (!jsonTargetsThisRobot(json))
        return;
    performResume();
}

void performStop()
{
    interruptActivePrimitive();
    clearCurrentPath();
    path_paused = false;
    setRobotState("idle");

    sendAck("stop");
    sendStatus("idle", "stop_requested");
    printPoseJSON();
}

void performToggleGripper()
{
    gripper_closed = !gripper_closed;
    gripper_target_deg = gripper_closed ? GRIPPER_CLOSED_DEG : GRIPPER_OPEN_DEG;
    prizm.setServoSpeed(GRIPPER_SERVO_ID, GRIPPER_SERVO_SPEED_PERCENT);
    prizm.setServoPosition(GRIPPER_SERVO_ID, gripper_target_deg);
    gripper_settle_until_ms = millis() + GRIPPER_SETTLE_MS;

    sendAck("toggle_gripper");
    sendStatus(robot_state, gripper_closed ? "gripper_closed" : "gripper_opened");
}

// Re-assert the latest gripper target for a short window so an I2C write that
// was dropped or cut short by a busy loop still reaches the servo controller.
void maybeHoldGripper()
{
    if (gripper_settle_until_ms == 0)
        return;
    if ((long)(millis() - gripper_settle_until_ms) >= 0)
    {
        gripper_settle_until_ms = 0;
        return;
    }
    prizm.setServoPosition(GRIPPER_SERVO_ID, gripper_target_deg);
}

void handleStop(const char *json)
{
    if (!jsonTargetsThisRobot(json))
        return;
    performStop();
}

void handleContinuousDrive(const char *json)
{
    if (!jsonTargetsThisRobot(json))
        return;

    int motorPower = DEFAULT_CONTINUOUS_MOTOR_POWER;
    if (!extractIntField(json, "motor_power", motorPower))
    {
        int driveSpeed = drive_speed_deg_per_sec;
        if (extractIntField(json, "drive_speed_deg_per_sec", driveSpeed))
            motorPower = driveSpeed / 6;
    }

    startContinuousDriveForward(motorPower);
    sendAck("continuous_drive");
    sendStatus("manual_forward", "continuous_drive_started");
    printPoseJSON();
}

void handleToggleGripper(const char *json)
{
    if (!jsonTargetsThisRobot(json))
        return;
    performToggleGripper();
}

void handleControlOpcode(char opcode)
{
    if (opcode == 'P')
    {
        performPause();
    }
    else if (opcode == 'R')
    {
        performResume();
    }
    else if (opcode == 'S')
    {
        performStop();
    }
    else if (opcode == 'G'){
        performToggleGripper();
    }
}

void handleIncomingJson(const char *json)
{
    if (jsonHasType(json, "path_assignment"))
    {
        handlePathAssignment(json);
    }
    else if (jsonHasType(json, "pause"))
    {
        handlePause(json);
    }
    else if (jsonHasType(json, "resume"))
    {
        handleResume(json);
    }
    else if (jsonHasType(json, "stop"))
    {
        handleStop(json);
    }
    else if (jsonHasType(json, "continuous_drive"))
    {
        handleContinuousDrive(json);
    }
    else if (jsonHasType(json, "toggle_gripper"))
    {
        handleToggleGripper(json);
    }
}

void readFromStream(Stream &stream)
{
    while (stream.available() > 0)
    {
        char c = (char)stream.read();
        if (c == '\r')
            continue;
        if (c == '\n')
        {
            if (serialLineOverflow)
            {
                // This line overran the buffer; discard it entirely so we
                // never feed a truncated fragment to the JSON parser.
                serialLineOverflow = false;
                serialLineLength = 0;
            }
            else if (serialLineLength > 0)
            {
                serialLineBuffer[serialLineLength] = '\0';
                if (serialLineLength == 1 &&
                    (serialLineBuffer[0] == 'P' ||
                     serialLineBuffer[0] == 'R' ||
                     serialLineBuffer[0] == 'S' ||
                     serialLineBuffer[0] == 'G'))
                {
                    handleControlOpcode(serialLineBuffer[0]);
                }
                else
                {
                    handleIncomingJson(serialLineBuffer);
                }
                serialLineLength = 0;
            }
        }
        else if (!serialLineOverflow)
        {
            if (serialLineLength < SERIAL_LINE_BUFFER_SIZE - 1)
            {
                serialLineBuffer[serialLineLength++] = c;
            }
            else
            {
                // Buffer full before newline: drop the rest of this line.
                serialLineOverflow = true;
            }
        }
    }
}

// Read from USB Serial and Software Serial
void readSerialCommands()
{
    readFromStream(Serial);
    readFromStream(espSerial);
}
// ==============================
// Path execution state machine
// ==============================
void updateActivePrimitive()
{
    if (scanning)
    {
        updateScan();
        return;
    }
    if (active_primitive == PRIM_NONE)
        return;

    updateOdometry();

    if (active_primitive == PRIM_CONT_DRIVE)
        return;

    if (!motorsBusy())
    {
        if (active_primitive == PRIM_DRIVE)
        {
            sendWaypointReached();
            current_waypoint_index++;
        }

        active_primitive = PRIM_NONE;

        if (path_paused)
        {
            setRobotState("paused");
        }
        else
        {
            setRobotState("idle");
        }

        printPoseJSON();
    }
}

void maybeStartNextPrimitive()
{
    if (scanning)
        return;
    if (!path_loaded)
        return;
    if (path_paused)
        return;
    if (active_primitive != PRIM_NONE)
        return;

    if (!path_started_sent)
    {
        sendPathStarted();
        path_started_sent = true;
    }

    if (current_waypoint_index < 0 || current_waypoint_index >= waypoint_count)
    {
        sendPathComplete();
        clearCurrentPath();
        setRobotState("idle");
        printPoseJSON();
        return;
    }

    float target_x = (float)waypoint_xs[current_waypoint_index];
    float target_y = (float)waypoint_ys[current_waypoint_index];

    float dist = distanceToWaypoint(target_x, target_y);

    if (dist <= POSITION_TOLERANCE_CM)
    {
        sendWaypointReached();
        current_waypoint_index++;

        if (current_waypoint_index >= waypoint_count)
        {
            sendPathComplete();
            clearCurrentPath();
            setRobotState("idle");
            printPoseJSON();
        }
        return;
    }

    float target_heading = headingToWaypointRad(target_x, target_y);
    float heading_err_deg = headingErrorDeg(target_heading);

    if (fabs(heading_err_deg) > HEADING_TOLERANCE_DEG)
    {
        startTurnInPlace(heading_err_deg);
    }
    else
    {
        startDriveStraight(dist);
    }
}

// ==============================
// Setup / loop
// ==============================
void setup()
{
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);

    prizm.PrizmBegin();
    Serial.begin(115200);  // USB SERIAL
    espSerial.begin(38400); // ESP COMMUNICATION SERIAL (ESP NEEDS TO LOOK AT THE SAME BAUDRATE)
    prizm.setServoSpeed(GRIPPER_SERVO_ID, GRIPPER_SERVO_SPEED_PERCENT);
    gripper_closed = false;
    gripper_target_deg = GRIPPER_OPEN_DEG;
    prizm.setServoPosition(GRIPPER_SERVO_ID, gripper_target_deg);

    delay(1500);

    setRobotState("ready");
    maybeUpdateSensors();
    printPoseJSON();
}

void loop()
{
    // 1. Read commands from Serial (USB or Wifi)
    readSerialCommands();
    readSerialCommands();

    // 2. Refresh sensors early so obstacle reaction uses current readings
    maybeUpdateSensors();
    maybeReactiveAvoidance();
    readSerialCommands();

    // 3. Update ongoing motion / odometry
    updateActivePrimitive();
    readSerialCommands();

    // 4. Start next primitive if needed
    maybeStartNextPrimitive();
    readSerialCommands();

    // 5. Refresh sensors only when the serial input is quiet
    maybeUpdateSensors();

    // 6. Keep re-asserting a recent gripper command until the servo settles
    maybeHoldGripper();

    // 7. Periodic telemetry
    maybeSendTelemetry();
}
