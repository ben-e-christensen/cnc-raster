// Stage 4: full inverse kinematics. Target platform pose -> six crank
// angles -> six step targets, with a simple serial command interface.
//
// Builds on the confirmed-working stage 3 firmware (UART bring-up, toff,
// current, all six motors spinning). This stage replaces the "just spin"
// test loop with real target-seeking motion driven by kinematics.h.
//
// Everything about the geometry (base pivots, shaft directions, platform
// points, the confirmed motor<->hole pairing, the 60-degree assembly twist)
// lives in kinematics.h - see that file's header comment for the full
// derivation trail.
//
// SERIAL COMMANDS (115200 baud):
//   Z                        - home/zero. Physically position the platform
//                              level, centered, at the neutral height
//                              (~171.74mm) BEFORE sending this. It tells the
//                              firmware "this is the neutral pose," and
//                              everything after is tracked relative to it.
//   P x y z roll pitch yaw   - command a new target pose. Position in mm
//                              (relative to neutral position, so P 0 0 0 0 0 0
//                              means "go back to neutral"), angles in degrees.
//   W tilt_deg [period_sec]  - orbital tilt: holds a constant tilt magnitude
//                              (tilt_deg) while continuously rotating the
//                              tilt DIRECTION around a full circle, like
//                              swirling a marble around the rim of a bowl.
//                              period_sec is how long one full revolution
//                              takes (default 4s if omitted). Workspace
//                              sweep confirms 12.0 deg is the largest radius
//                              reachable at every point around the circle -
//                              go higher and it'll stutter at some phases.
//   J amplitude_deg [period_sec] - joint-space traveling wave: drives raw
//                              crank angles directly (no IK at all). Each
//                              motor rises to amplitude_deg, HOLDS there
//                              for one full segment, falls, then rests -
//                              phase-shifted 1/6 of a cycle per motor index
//                              so consecutive pairs (0,1),(1,2),(2,3),
//                              (3,4),(4,5),(5,0) are each briefly BOTH held
//                              at full amplitude simultaneously as the
//                              active pair steps around the ring.
//   S                        - stop any active orbital tilt or joint wave
//                              and immediately return to the neutral pose.
//
// MOTION NOTE: this stage moves each leg toward its target at a fixed max
// step rate - no acceleration profile yet. Fine for a slow-moving balancing
// platform; worth revisiting if you ever want fast, smooth pose changes.

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>
#include "kinematics.h"

constexpr uint8_t NUM_MOTORS = 6;

// Indexed 0-5, matching Driver0-Driver5 sockets.
constexpr uint8_t STEP_PINS[NUM_MOTORS]   = { PF13, PG0,  PF11, PG4, PF9,  PC13 };
constexpr uint8_t DIR_PINS[NUM_MOTORS]    = { PF12, PG1,  PG3,  PC1, PF10, PF0  };
constexpr uint8_t ENABLE_PINS[NUM_MOTORS] = { PF14, PF15, PG5,  PA0, PG2,  PF1  }; // all active-low

constexpr float R_SENSE = 0.11f;
constexpr uint8_t DRIVER_ADDRESS = 0b00;
constexpr uint16_t TEST_CURRENT_MA = 600; // still conservative - raise once confirmed reliable

SoftwareSerial uartM0(PC4, PC4);
SoftwareSerial uartM1(PD11, PD11);
SoftwareSerial uartM2(PC6, PC6);
SoftwareSerial uartM3(PC7, PC7);
SoftwareSerial uartM4(PF2, PF2);
SoftwareSerial uartM5(PE4, PE4);

TMC2209Stepper driver0(&uartM0, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver1(&uartM1, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver2(&uartM2, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver3(&uartM3, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver4(&uartM4, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver5(&uartM5, R_SENSE, DRIVER_ADDRESS);

TMC2209Stepper* drivers[NUM_MOTORS] = { &driver0, &driver1, &driver2,
                                         &driver3, &driver4, &driver5 };
SoftwareSerial*  uarts[NUM_MOTORS]  = { &uartM0, &uartM1, &uartM2,
                                         &uartM3, &uartM4, &uartM5 };

// --- Motion / kinematics state ---
constexpr uint16_t MICROSTEPS = 16;
constexpr float STEP_ANGLE_DEG = 1.8f;
constexpr float STEPS_PER_REV = (360.0f / STEP_ANGLE_DEG) * MICROSTEPS; // 3200
constexpr float STEPS_PER_RAD = STEPS_PER_REV / (2.0f * PI);

constexpr uint32_t STEP_PULSE_US = 20;
constexpr uint32_t MAX_STEP_INTERVAL_US = 2000; // max step rate per leg (500 steps/sec)

float neutral_angle[NUM_LEGS];      // crank angle at the homed neutral pose
float current_angle[NUM_LEGS];      // best-known current angle (from step count)
int32_t current_step_pos[NUM_LEGS] = {0};
int32_t target_step_pos[NUM_LEGS]  = {0};
uint32_t last_step_time[NUM_LEGS]  = {0};
bool homed = false;

// --- Orbital tilt ("W") state ---
bool orbit_active = false;
float orbit_tilt_deg = 0;
float orbit_period_ms = 4000;
uint32_t orbit_start_millis = 0;
uint32_t last_orbit_update = 0;
constexpr uint32_t ORBIT_UPDATE_INTERVAL_MS = 20; // ~50Hz pose recompute rate

// --- Joint-space traveling wave ("J") state ---
bool jointwave_active = false;
float jointwave_amplitude_deg = 0;
float jointwave_period_ms = 4000;
uint32_t jointwave_start_millis = 0;
uint32_t last_jointwave_update = 0;
constexpr uint32_t JOINTWAVE_UPDATE_INTERVAL_MS = 20; // ~50Hz

// Shared by set_target_pose() and update_orbit(): solves all six legs for
// a given position+rotation and, if reachable, converts to step targets.
// If unreachable, just leaves the existing target_step_pos[] alone (holds
// position) rather than erroring - callers decide whether to warn.
bool commit_pose(const Vec3& pos, const Mat3& rot) {
    uint8_t fail_mask = solve_all_legs(pos, rot, current_angle);
    if (fail_mask) return false;
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        float delta_from_neutral = current_angle[i] - neutral_angle[i];
        target_step_pos[i] = (int32_t) roundf(delta_from_neutral * STEPS_PER_RAD);
    }
    return true;
}

bool set_target_pose(float x, float y, float z, float roll, float pitch, float yaw) {
    Vec3 pos(x, y, z + NEUTRAL_Z);
    Mat3 rot = rotation_from_rpy_deg(roll, pitch, yaw);
    if (!commit_pose(pos, rot)) {
        Serial.println("WARNING: pose unreachable for at least one leg.");
        return false;
    }
    return true;
}

// Builds a rotation for tilting by angle_deg around a horizontal axis that
// itself points in the rotating compass direction phase_rad - i.e. a true
// single-axis "conical" tilt, not two composed Euler rotations. This is
// what update_orbit() needs: composing roll=A*cos(phase) and
// pitch=A*sin(phase) as sequential Euler angles doesn't commute, and the
// resulting tilt axis wobbles off a clean circle (visible as a
// back-and-forth rock) once the angle isn't tiny. Built via Rodrigues'
// rotation formula around axis (sin(phase), -cos(phase), 0).
Mat3 rotation_conical_tilt(float angle_deg, float phase_rad) {
    float theta = radians(angle_deg);
    float ax = sinf(phase_rad);
    float ay = -cosf(phase_rad);
    float s = sinf(theta), c = cosf(theta);
    Mat3 out;
    out.m[0][0] = c + ax*ax*(1-c);
    out.m[0][1] = ax*ay*(1-c);
    out.m[0][2] = ay*s;
    out.m[1][0] = ay*ax*(1-c);
    out.m[1][1] = c + ay*ay*(1-c);
    out.m[1][2] = -ax*s;
    out.m[2][0] = -ay*s;
    out.m[2][1] = ax*s;
    out.m[2][2] = c;
    return out;
}

// Called at ~50Hz while an orbital tilt is active. Advances the phase and
// recomputes the pose for a constant-magnitude, rotating-direction tilt -
// a genuine single-axis conical rotation now, not composed roll+pitch.
// If a particular phase happens to be unreachable (only possible above the
// ~12 deg full-circle-safe radius the sweep tool found), it just holds the
// previous target for that one tick - the sweep moves on a moment later.
void update_orbit() {
    float elapsed_ms = (float)(millis() - orbit_start_millis);
    float phase = fmodf(elapsed_ms / orbit_period_ms, 1.0f) * 2.0f * PI;
    Mat3 rot = rotation_conical_tilt(orbit_tilt_deg, phase);
    Vec3 pos(0, 0, NEUTRAL_Z);
    commit_pose(pos, rot); // silently keeps old target if this phase is unreachable
}

// Shape for one motor's traveling-wave cycle, as a function of that motor's
// own local phase (wrapped into [0,1)). Segments, in order:
//   [0,   1/6): falling from D to 0
//   [1/6, 4/6): holding at 0 (three segments - half the period)
//   [4/6, 5/6): rising from 0 to D
//   [5/6, 1  ): holding at D
// Each motor is phase-shifted by 1/6 of the cycle relative to the next, so
// consecutive motors (i, i+1) are BOTH in their hold-at-D segment at the
// same moment - i falling out of it just as i+1 settles into it - giving
// the six overlapping pairs (0,1),(1,2),(2,3),(3,4),(4,5),(5,0) in turn.
float trapezoid_wave(float local_phase) {
    if (local_phase < 1.0f/6.0f) {
        return 1.0f - local_phase * 6.0f; // falling
    } else if (local_phase < 4.0f/6.0f) {
        return 0.0f; // holding at 0
    } else if (local_phase < 5.0f/6.0f) {
        return (local_phase - 4.0f/6.0f) * 6.0f; // rising
    } else {
        return 1.0f; // holding at D
    }
}

// Called at ~50Hz while a joint-space wave is active. Directly sets each
// motor's target crank angle from the traveling trapezoidal wave - no IK,
// no platform pose involved at all, since this is defined purely in terms
// of each motor's own angle relative to its homed neutral.
void update_jointwave() {
    float elapsed_ms = (float)(millis() - jointwave_start_millis);
    float global_phase = fmodf(elapsed_ms / jointwave_period_ms, 1.0f);
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        float local_phase = global_phase - (float)i / 6.0f;
        local_phase -= floorf(local_phase); // wrap into [0,1)
        float value = trapezoid_wave(local_phase);
        float delta_deg = jointwave_amplitude_deg * value;
        target_step_pos[i] = (int32_t) roundf(radians(delta_deg) * STEPS_PER_RAD);
    }
}

void do_homing() {
    orbit_active = false;
    jointwave_active = false;
    Vec3 neutral_pos(0, 0, NEUTRAL_Z);
    Mat3 identity = rotation_from_rpy_deg(0, 0, 0);
    for (uint8_t i = 0; i < NUM_LEGS; i++) current_angle[i] = 0; // no prior reference yet
    uint8_t fail_mask = solve_all_legs(neutral_pos, identity, current_angle);
    if (fail_mask) {
        Serial.println("ERROR: neutral pose itself failed to solve - check geometry constants.");
        return;
    }
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        neutral_angle[i] = current_angle[i];
        current_step_pos[i] = 0;
        target_step_pos[i] = 0;
    }
    homed = true;
    Serial.println("Homed. Neutral angles (deg):");
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        Serial.print("  leg "); Serial.print(i); Serial.print(": ");
        Serial.println(degrees(neutral_angle[i]));
    }
}

char cmd_buf[64];
uint8_t cmd_len = 0;

void process_command(char* line) {
    if (line[0] == 'Z' || line[0] == 'z') {
        do_homing();
        return;
    }
    if (line[0] == 'P' || line[0] == 'p') {
        float x, y, z, roll, pitch, yaw;
        int n = sscanf(line + 1, "%f %f %f %f %f %f", &x, &y, &z, &roll, &pitch, &yaw);
        if (n != 6) {
            Serial.println("Parse error. Usage: P x y z roll pitch yaw");
            return;
        }
        if (!homed) {
            Serial.println("Not homed yet - send Z first.");
            return;
        }
        orbit_active = false; // a direct pose command overrides any running orbit
        jointwave_active = false;
        if (set_target_pose(x, y, z, roll, pitch, yaw)) {
            Serial.println("Target pose accepted.");
        }
        return;
    }
    if (line[0] == 'W' || line[0] == 'w') {
        float tilt_deg, period_sec;
        int n = sscanf(line + 1, "%f %f", &tilt_deg, &period_sec);
        if (n < 1) {
            Serial.println("Parse error. Usage: W tilt_deg [period_sec]");
            return;
        }
        if (!homed) {
            Serial.println("Not homed yet - send Z first.");
            return;
        }
        jointwave_active = false;
        orbit_tilt_deg = tilt_deg;
        orbit_period_ms = (n == 2) ? (period_sec * 1000.0f) : 4000.0f;
        orbit_start_millis = millis();
        orbit_active = true;
        Serial.print("Orbit started: tilt=");
        Serial.print(orbit_tilt_deg);
        Serial.print(" deg, period=");
        Serial.print(orbit_period_ms / 1000.0f);
        Serial.println(" sec. Send S to stop.");
        return;
    }
    if (line[0] == 'J' || line[0] == 'j') {
        float amp_deg, period_sec;
        int n = sscanf(line + 1, "%f %f", &amp_deg, &period_sec);
        if (n < 1) {
            Serial.println("Parse error. Usage: J amplitude_deg [period_sec]");
            return;
        }
        if (!homed) {
            Serial.println("Not homed yet - send Z first.");
            return;
        }
        orbit_active = false;
        jointwave_amplitude_deg = amp_deg;
        jointwave_period_ms = (n == 2) ? (period_sec * 1000.0f) : 4000.0f;
        jointwave_start_millis = millis();
        jointwave_active = true;
        Serial.print("Joint wave started: amplitude=");
        Serial.print(jointwave_amplitude_deg);
        Serial.print(" deg, period=");
        Serial.print(jointwave_period_ms / 1000.0f);
        Serial.println(" sec. Send S to stop.");
        return;
    }
    if (line[0] == 'S' || line[0] == 's') {
        orbit_active = false;
        jointwave_active = false;
        if (homed) {
            set_target_pose(0, 0, 0, 0, 0, 0);
            Serial.println("Stopped. Returning to neutral pose.");
        } else {
            Serial.println("Stopped, holding position.");
        }
        return;
    }
    Serial.println("Unknown command. Use Z (home), P x y z roll pitch yaw (move), W tilt_deg [period_sec] (orbit), J amplitude_deg [period_sec] (joint wave), or S (stop).");
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    init_crank_bases();

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(STEP_PINS[i], OUTPUT);
        pinMode(DIR_PINS[i], OUTPUT);
        pinMode(ENABLE_PINS[i], OUTPUT);
        digitalWrite(ENABLE_PINS[i], LOW);
        digitalWrite(DIR_PINS[i], LOW);
        digitalWrite(STEP_PINS[i], LOW);

        uarts[i]->begin(115200);
        drivers[i]->begin();
        drivers[i]->toff(4);
        drivers[i]->rms_current(TEST_CURRENT_MA);
        drivers[i]->microsteps(MICROSTEPS);
        drivers[i]->pwm_autoscale(true);

        uint8_t result = drivers[i]->test_connection();
        Serial.print("Motor "); Serial.print(i);
        Serial.print(" UART test_connection(): "); Serial.println(result);
    }

    Serial.println("Ready. Send Z to home, then P x y z roll pitch yaw to move.");
}

void loop() {
    // --- serial command intake ---
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                process_command(cmd_buf);
                cmd_len = 0;
            }
        } else if (cmd_len < sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = c;
        }
    }

    // --- orbital tilt: recompute target pose at ~50Hz while active ---
    if (orbit_active && (millis() - last_orbit_update >= ORBIT_UPDATE_INTERVAL_MS)) {
        last_orbit_update = millis();
        update_orbit();
    }

    // --- joint-space wave: recompute crank angles directly at ~50Hz while active ---
    if (jointwave_active && (millis() - last_jointwave_update >= JOINTWAVE_UPDATE_INTERVAL_MS)) {
        last_jointwave_update = millis();
        update_jointwave();
    }

    // --- per-leg target-seeking motion ---
    uint32_t now = micros();
    for (uint8_t i = 0; i < NUM_LEGS; i++) {
        if (current_step_pos[i] == target_step_pos[i]) continue;
        if (now - last_step_time[i] < MAX_STEP_INTERVAL_US) continue;

        bool forward = target_step_pos[i] > current_step_pos[i];
        digitalWrite(DIR_PINS[i], forward ? HIGH : LOW);
        digitalWrite(STEP_PINS[i], HIGH);
        delayMicroseconds(STEP_PULSE_US);
        digitalWrite(STEP_PINS[i], LOW);

        current_step_pos[i] += forward ? 1 : -1;
        last_step_time[i] = now;
    }
}