// ESP32 version
#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ========================= IMU (BNO08x) SETUP =========================

// I2C IMU (Adafruit BNO08x breakout, addr 0x4A on ESP32 default I2C)
Adafruit_BNO08x bno(55);
sh2_SensorValue_t imuVal;

// Change for your location: east = +, west = - (Toronto ≈ -10)
const float DECLINATION_DEG = -10.0f;

// Wrap [0,360)
static inline float wrap360(float d) {
  while (d < 0.0f)    d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return d;
}

// Wrap to [-180,180)
static inline float wrap180(float d) {
  d = wrap360(d);      // first ensure [0,360)
  if (d >= 180.0f) {
    d -= 360.0f;
  }
  return d;
}

// Global heading (TRUE, i.e., magnetic + declination)
float g_true_heading = 0.0f;
bool  g_have_heading = false;
// Filtered heading (for smoothing)
float g_filt_heading = 0.0f;
bool  g_filt_init    = false;


// Initialize IMU
void initIMU() {
  Serial.println("[DEBUG] initIMU() start");
  Wire.begin(21, 22);  // SDA, SCL for ESP32
  Serial.println("Starting BNO08x...");
  if (!bno.begin_I2C(0x4A, &Wire)) {
    Serial.println("BNO08x not found");
    while (1) { delay(10); }
  }

  Serial.println("BNO08x found!");

  // Optional: set to Game Rotation Vector for headset-like orientation
  bno.enableReport(SH2_ROTATION_VECTOR, 100000);  // 10 ms = 100 Hz
  // Or use 100000 for 10 Hz, etc.

  // Small delay for stability
  delay(100);
}

// Call this often (e.g., once per loop) to refresh heading if a new IMU
// sample is available. It does NOT block.
void updateHeadingFromIMU() {
  // Serial.println("[DEBUG] updateHeadingFromIMU() start");
  if (bno.getSensorEvent(&imuVal) &&
      imuVal.sensorId == SH2_ROTATION_VECTOR) {

    // Serial.println("in the update");
    
    float r = imuVal.un.rotationVector.real;
    float i = imuVal.un.rotationVector.i;
    float j = imuVal.un.rotationVector.j;
    float k = imuVal.un.rotationVector.k;

    float ysqr = j * j;

    // Yaw (Z axis rotation)
    float t3   = +2.0f * (r * k + i * j);
    float t4   = +1.0f - 2.0f * (ysqr + k * k);
    float yaw  = atan2f(t3, t4) * 180.0f / PI;   // in degrees

    // Convert to [0,360)
    yaw = wrap360(yaw);

    // Convert MAGNETIC heading to TRUE using local declination
    float trueDeg = yaw + DECLINATION_DEG;
    trueDeg = wrap360(trueDeg);

    // ---- low-pass filter on a circle ----
    if (!g_filt_init) {
      g_filt_heading = trueDeg;
      g_filt_init    = true;
    }

    const float alpha = 0.1f;   // smoothing factor (0<alpha<=1), smaller = smoother
    // Filter the heading error (handles wrap-around properly)
    float err = wrap180(trueDeg - g_filt_heading);
    g_filt_heading = wrap360(g_filt_heading + alpha * err);

    // Expose filtered heading to the rest of the code
    g_true_heading = g_filt_heading;
    g_have_heading = true;

    Serial.print("Raw yaw: ");
    Serial.print(yaw);
    Serial.print(" deg, TRUE filtered heading: ");
    Serial.print(g_true_heading);
    Serial.println(" deg");
  }
}

// ========================= SERVO / GAIT CODE =========================



// Pick ONLY output-capable, non-strap GPIOs for PWM.
// Avoid 0, 2, 12, 15; avoid input-only 34–39.
// Adjust this list to your wiring.
// int servoPins[NUM_SERVOS] = {
//   13, 14, 16, 17, 18, 19, 21, 22, 23, 25
// };

constexpr int NUM_SERVOS = 6;

int servoPins[NUM_SERVOS] = {
  13, 14, 16, 17, 18, 19
};


Servo myServos[NUM_SERVOS];

float pi = 3.1415926f;
int  TotalNum = NUM_SERVOS;

// Attach all servos
void attachAllServos() {
  Serial.println("[DEBUG] attachAllServos() start");
  for (int i = 0; i < NUM_SERVOS; i++) {
    myServos[i].attach(servoPins[i], 500, 2400);
    myServos[i].write(90);
  }
}

// center all servos at `val` (default 90)
void centerAll(int val = 90, int d = 15) {
  Serial.println("[DEBUG] centerAll() start");
  for (int i = 0; i < TotalNum; i++) {
    myServos[i].write(val);
    delay(d);
  }
}

// Straight snake
void straightline() {
  Serial.println("[DEBUG] straightline() start");
  for (int i = 0; i < TotalNum; i++)
  {
    myServos[i].write(90);
    delay(15);
  }
}

struct SlitherState {
  int offset;
  int amplitude;
  int speed;
  float wavelengths;

  int deg;                    // current degree (0..359)
  unsigned long lastUpdateMs; // last time we updated servos
};

SlitherState g_slither = {
  0, 35, 2, 1.5,   // default params
  0, 0
};

// Set current gait parameters
void setSlitherParams(int offset, int Amplitude, int Speed, float Wavelengths) {
  g_slither.offset      = offset;
  g_slither.amplitude   = Amplitude;
  g_slither.speed       = Speed;
  g_slither.wavelengths = Wavelengths;
}

// Do ONE small slither step (non-blocking)
void slitherStep() {
  const unsigned long now = millis();
  const unsigned long updatePeriodMs = 10;   // like your old delay(10)

  // Only update servos every 10 ms
  if (now - g_slither.lastUpdateMs < updatePeriodMs) {
    return;
  }
  g_slither.lastUpdateMs = now;

  float Shift = 2 * pi / NUM_SERVOS;
  float rads  = g_slither.deg * pi / 180.0f;

  for (int j = 0; j < NUM_SERVOS; j++) {
    float theta = 90
                + g_slither.offset
                + g_slither.amplitude *
                  sin(g_slither.speed * rads + j * g_slither.wavelengths * Shift);
    int Position = int(theta);
    myServos[j].write(Position);
  }

  g_slither.deg++;
  if (g_slither.deg >= 360) {
    g_slither.deg = 0;
  }
}


// ========================= HEADING-BASED GAIT =========================

// Basic turn parameters (tweak to taste)
const int   TURN_OFFSET_LEFT  = +30;  // offset added to base angle when turning left
const int   TURN_OFFSET_RIGHT = -30;  // offset added to base angle when turning right
const int   TURN_AMPLITUDE    = 35;
const int   TURN_SPEED        = 4;
const float TURN_WAVELENGTHS  = 1.5f;
// Heading control mode with hysteresis to avoid chattering
enum HeadingMode {
  HEADING_STRAIGHT,
  HEADING_TURN_LEFT,
  HEADING_TURN_RIGHT
};

HeadingMode g_heading_mode = HEADING_STRAIGHT;


// Move so that TRUE heading tends toward target_true_deg
void moveTowardsHeading(float target_true_deg, float tol_deg = 5.0f) {
  // Re-evaluate heading control at a slower rate than servo update
  static unsigned long lastCtrlMs = 0;
  const unsigned long CTRL_PERIOD_MS = 200;   // 5 Hz control

  unsigned long now = millis();
  if (now - lastCtrlMs < CTRL_PERIOD_MS) {
    return;   // keep previous gait
  }
  lastCtrlMs = now;

  // Decide gait, do NOT block
  if (!g_have_heading) {
    // No heading yet, just go straight-ish
    setSlitherParams(0, 35, 2, 1.5);  // straightline-ish params
    return;
  }

  target_true_deg = wrap360(target_true_deg);

  float current = g_true_heading;   // filtered heading
  float error   = wrap180(target_true_deg - current);

  // Interpret tol_deg as the OUTER tolerance; inner is half of that.
  if (tol_deg <= 0.0f) {
    tol_deg = 15.0f; // default outer tolerance if caller passes 0
  }
  float tol_outer = tol_deg;
  float tol_inner = tol_deg * 0.5f;  // inner hysteresis band
  if (tol_inner > tol_outer) {
    tol_inner = tol_outer;
  }

  Serial.print("Target ");
  Serial.print(target_true_deg);
  Serial.print("°, current ");
  Serial.print(current);
  Serial.print("°, error ");
  Serial.println(error);

  // --- decide mode with hysteresis ---
  switch (g_heading_mode) {
    case HEADING_STRAIGHT:
      if (error >  tol_outer) {
        g_heading_mode = HEADING_TURN_LEFT;
      } else if (error < -tol_outer) {
        g_heading_mode = HEADING_TURN_RIGHT;
      }
      break;

    case HEADING_TURN_LEFT:
      if (fabs(error) < tol_inner) {
        g_heading_mode = HEADING_STRAIGHT;
      }
      break;

    case HEADING_TURN_RIGHT:
      if (fabs(error) < tol_inner) {
        g_heading_mode = HEADING_STRAIGHT;
      }
      break;
  }

  // --- choose gait from mode ---
  if (g_heading_mode == HEADING_STRAIGHT) {
    // Roughly facing the right way -> forward gait
    setSlitherParams(0, 35, 3, 1.5);
  } else if (g_heading_mode == HEADING_TURN_LEFT) {
    // Need to increase heading (empirically "left") -> one biased gait
    setSlitherParams(TURN_OFFSET_RIGHT, TURN_AMPLITUDE, TURN_SPEED, TURN_WAVELENGTHS);
  } else { // HEADING_TURN_RIGHT
    // Need to decrease heading -> opposite biased gait
    setSlitherParams(TURN_OFFSET_LEFT, TURN_AMPLITUDE, TURN_SPEED, TURN_WAVELENGTHS);
  }
}
// ========================= SETUP / LOOP =========================

void setup() {
  Serial.println("[DEBUG] setup() start");
  Serial.begin(115200);
  delay(1500);

  initIMU();
  attachAllServos();
  straightline();

  Serial.print("Delay Complete");
}

void loop() {
  // 1. Get latest IMU sample (non-blocking)
  updateHeadingFromIMU();
  Serial.println("[DEBUG] loop() start");
  // Grab any new IMU sample (non-blocking)
  Serial.println("Start Update");
  // updateHeadingFromIMU();
  Serial.println("Finish Update");

  if (g_have_heading) {
    Serial.print("TRUE heading: ");
    Serial.print(g_true_heading);
    Serial.println(" deg");
  } else {
    Serial.println("Waiting for IMU heading...");
  }

  // Keep moving while trying to face 90° TRUE
  moveTowardsHeading(355, 50.0f);

  // 3. Advance the gait one tiny step
  slitherStep();
}
