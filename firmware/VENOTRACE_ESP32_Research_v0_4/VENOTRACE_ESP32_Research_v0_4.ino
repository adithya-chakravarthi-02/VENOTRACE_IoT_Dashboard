/*
  VENOTRACE - Wearable Optical Sensing and Motion-Aware Signal Analysis
  ESP32 DevKit V1 + MAX30102 + MPU6050
  RESEARCH PROTOTYPE ONLY - NOT A DVT DIAGNOSTIC DEVICE
  Libs: SparkFun MAX3010x (MAX30105.h), Adafruit MPU6050, Adafruit Unified Sensor

  OUTPUT (115200 baud), every line starts with a tag:
    #...  metadata / diagnostics / legends / command acks (dashboard: log or ignore)
    D,... 25 Hz sample row   (schema printed at boot as #SCHEMA,D,...)
    W,... 5 s window features (#SCHEMA,W,...)
    P,... research-model interface record (#SCHEMA,P,...); NO_MODEL by default

  SERIAL COMMANDS (send with newline):
    subj <n>    subject id           site <n>   body site
    phase <n>   protocol phase       label <n>  -1 unlabelled, 0 control, 1 reference-positive
    mark        increment event marker           cal   recalibrate IMU (keep still)
    rest        clear resting IR variation reference           status / help

  q bitmask: 1 MAX30102 down | 2 MPU6050 down | 4 no contact | 8 saturation
             16 motion | 32 baseline warm-up.  q==0 -> passes these firmware quality gates; not clinical validation.
*/

#include <Wire.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include "MAX30105.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================ USER SETTINGS ============================
#define SERIAL_BAUD        115200
#define I2C_SDA            21
#define I2C_SCL            22
#define I2C_CLOCK_HZ       100000  // conservative clock for initial hardware debugging

// MAX30102 (effective rate = SAMPLE_RATE / SAMPLE_AVG)
#define LED_BRIGHTNESS     80        // 0-255; raise if IR stays low on skin
#define SAMPLE_AVG         4
#define LED_MODE           2         // Red + IR
#define SAMPLE_RATE        100
#define PULSE_WIDTH        411
#define ADC_RANGE          16384
#define FS_HZ              (SAMPLE_RATE / SAMPLE_AVG)   // 25 Hz

// Quality gates
#define IR_CONTACT_MIN     20000UL
#define SATURATION_LEVEL   260000UL
#define BASELINE_ALPHA     0.002f
#define WARMUP_SAMPLES     (FS_HZ * 10)
#define ACC_DEV_G_MAX      0.15f
#define GYRO_DPS_MAX       30.0f

// Windowing
#define WINDOW_SAMPLES     (FS_HZ * 5)   // 5 s windows
#define MIN_VALID_FRAC     0.80f

// ---- Future research-model slot. Keep disabled until externally validated. ----
// Feature order: [ir_variation_pct, red_variation_pct, red_ir_variation_ratio, relative_change_from_rest_pct]
#define MODEL_LOADED       0
#define MODEL_NAME         "none"
#define N_FEAT             4
const float MODEL_MEAN[N_FEAT] = {0, 0, 0, 0};
const float MODEL_STD[N_FEAT]  = {1, 1, 1, 1};
const float MODEL_W[N_FEAT]    = {0, 0, 0, 0};
const float MODEL_B            = 0;
const float MODEL_THRESHOLD    = 0.5f;
// =======================================================================

#define RAD2DEG            57.2957795f
#define G_MS2              9.80665f
#define RETRY_INTERVAL_MS  2000

MAX30105 optical;
Adafruit_MPU6050 imu;

bool opticalOk = false, imuOk = false;
uint8_t imuAddr = 0;
uint32_t seq = 0, winId = 0, lastRetryMs = 0;

// Session labels (stamped on every row)
struct Session { int16_t subj; int8_t site; int8_t phase; int8_t label; uint16_t mark; };
Session S = {0, 0, 0, -1, 0};

// Optical baseline
float redBase = 0, irBase = 0;
uint32_t baseCount = 0;

// IMU state + calibration
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, tempC = 0;
float gyroBias[3] = {0, 0, 0};
float accScale = 1.0f;

// Window accumulators (Welford, valid samples only)
struct Win {
  uint16_t total, n, cNoC, cMot, cOth;
  double mIR, m2IR, mR, m2R;
};
Win W = {};
float restIrVarPct = NAN;   // resting IR variation reference (phases 0/1)

// ============================ HELPERS ==================================
static const char* fn(char* buf, size_t n, float v, int d) {
  if (isnan(v) || isinf(v)) snprintf(buf, n, "NA");
  else snprintf(buf, n, "%.*f", d, v);
  return buf;
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void i2cScan() {
  Serial.println("# I2C scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) {
      Serial.printf("#   0x%02X%s\n", a,
        a == 0x57 ? "  (MAX30102)" : (a == 0x68 || a == 0x69) ? "  (MPU6050)" : "");
      found++;
    }
  }
  if (!found) Serial.println("#   none found: check VCC/GND/SDA/SCL");
}

void resetWindow() { memset(&W, 0, sizeof(W)); }

void resetSubjectSiteReferences() {
  redBase = 0;
  irBase = 0;
  baseCount = 0;
  restIrVarPct = NAN;
  resetWindow();
}

// ============================ SENSOR INIT ==============================
bool initOptical() {
  if (!i2cPresent(0x57)) { Serial.println("# MAX30102: no ACK at 0x57"); return false; }
  if (!optical.begin(Wire, I2C_SPEED_STANDARD)) { Serial.println("# MAX30102: begin() failed"); return false; }
  optical.setup(LED_BRIGHTNESS, SAMPLE_AVG, LED_MODE, SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);
  optical.clearFIFO();
  baseCount = 0;
  Serial.printf("# MAX30102: OK part_id=0x%02X fs=%d Hz LED=%d\n",
                optical.readPartID(), FS_HZ, LED_BRIGHTNESS);
  return true;
}

bool initImu() {
  const uint8_t addrs[2] = {0x68, 0x69};
  for (uint8_t i = 0; i < 2; i++) {
    if (i2cPresent(addrs[i]) && imu.begin(addrs[i], &Wire)) {
      imuAddr = addrs[i];
      imu.setAccelerometerRange(MPU6050_RANGE_4_G);
      imu.setGyroRange(MPU6050_RANGE_500_DEG);
      imu.setFilterBandwidth(MPU6050_BAND_21_HZ);
      Serial.printf("# MPU6050: OK addr=0x%02X range=+-4g/+-500dps\n", imuAddr);
      return true;
    }
  }
  Serial.println("# MPU6050: not found at 0x68/0x69");
  return false;
}

// Keep device still. Estimates gyro bias and accelerometer 1 g scale.
bool calibrateImu() {
  if (!imuOk) { Serial.println("# CAL: IMU not available"); return false; }
  const int N = 60;
  double sg[3] = {0, 0, 0}, sa = 0;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    if (!imu.getEvent(&a, &g, &t)) { Serial.println("# CAL: read failed"); return false; }
    sg[0] += g.gyro.x; sg[1] += g.gyro.y; sg[2] += g.gyro.z;
    sa += sqrt(a.acceleration.x * a.acceleration.x +
               a.acceleration.y * a.acceleration.y +
               a.acceleration.z * a.acceleration.z);
    delay(10);
  }
  float b[3];
  for (int k = 0; k < 3; k++) b[k] = (float)(sg[k] / N) * RAD2DEG;
  if (fabsf(b[0]) > 30 || fabsf(b[1]) > 30 || fabsf(b[2]) > 30) {
    Serial.println("# CAL: rejected (device moving). Keep still and send: cal");
    return false;
  }
  for (int k = 0; k < 3; k++) gyroBias[k] = b[k];
  accScale = G_MS2 / (float)(sa / N);
  Serial.printf("# CAL: gyro_bias=[%.2f,%.2f,%.2f] dps acc_scale=%.4f\n",
                gyroBias[0], gyroBias[1], gyroBias[2], accScale);
  return true;
}

void readImu() {
  sensors_event_t a, g, t;
  if (imu.getEvent(&a, &g, &t)) {
    ax = a.acceleration.x * accScale;
    ay = a.acceleration.y * accScale;
    az = a.acceleration.z * accScale;
    gx = g.gyro.x * RAD2DEG - gyroBias[0];
    gy = g.gyro.y * RAD2DEG - gyroBias[1];
    gz = g.gyro.z * RAD2DEG - gyroBias[2];
    tempC = t.temperature;
  } else {
    imuOk = false;
    Serial.println("# MPU6050: read failed, will retry");
  }
}

// ============================ OUTPUT ===================================
void printLegends() {
  Serial.println("#LEGEND,phase,0=rest_supine,1=rest_sitting,2=leg_elevated,3=calf_pump,4=standing,5=walking,6=recovery");
  Serial.println("#LEGEND,site,0=calf_L,1=calf_R,2=ankle_L,3=ankle_R,4=thigh_L,5=thigh_R");
  Serial.println("#LEGEND,label,-1=unlabelled,0=control(reference-negative),1=reference-positive");
  Serial.println("#LEGEND,q,1=max_down,2=imu_down,4=no_contact,8=saturated,16=motion,32=warmup,0=passed_configured_quality_gates");
  Serial.println("#SCHEMA,D,t_ms,seq,subj,site,phase,label,mark,red,ir,red_base,ir_base,red_dev_pct,ir_dev_pct,"
                 "ax_ms2,ay_ms2,az_ms2,gx_dps,gy_dps,gz_dps,pitch_deg,roll_deg,acc_dev_g,gyro_dps,temp_c,motion,q");
  Serial.println("#SCHEMA,W,win,t_ms,subj,site,phase,label,n_valid,valid_frac,ir_mean,red_mean,"
                 "ir_variation_pct,red_variation_pct,red_ir_variation_ratio,relative_change_from_rest_pct,status");
  Serial.println("#SCHEMA,P,win,t_ms,model,status,score,flag,note");
}

void printStatus() {
  Serial.printf("# STATUS optical=%s imu=%s subj=%d site=%d phase=%d label=%d mark=%u model=%s rest_ir_variation_reference=%s\n",
    opticalOk ? "OK" : "FAIL", imuOk ? "OK" : "FAIL", S.subj, S.site, S.phase, S.label, S.mark,
    MODEL_LOADED ? MODEL_NAME : "none", isnan(restIrVarPct) ? "NA" : "set");
}

void printHelp() {
  Serial.println("# CMDS: subj <n> | site <n> | phase <n> | label <-1|0|1> | mark | cal | rest | status | help");
  printLegends();
}

void emitD(uint32_t red, uint32_t ir, float redDev, float irDev,
           float pitch, float roll, float accDevG, float gyroDps, bool motion, uint8_t q) {
  char buf[320];
  snprintf(buf, sizeof(buf),
    "D,%lu,%lu,%d,%d,%d,%d,%u,%lu,%lu,%.1f,%.1f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.1f,%.1f,%.3f,%.2f,%.1f,%d,%u",
    (unsigned long)millis(), (unsigned long)seq, S.subj, S.site, S.phase, S.label, S.mark,
    (unsigned long)red, (unsigned long)ir, redBase, irBase, redDev, irDev,
    ax, ay, az, gx, gy, gz, pitch, roll, accDevG, gyroDps, tempC, motion ? 1 : 0, q);
  Serial.println(buf);
}

// ============================ PREDICTION ===============================
void emitPrediction(uint32_t id, bool windowOk, float f[N_FEAT]) {
  char sc[12] = "NA", fl[4] = "NA";
  const char* status;
  if (!windowOk)                 status = "INVALID_WINDOW";
  else if (!MODEL_LOADED)        status = "NO_MODEL";
  else if (isnan(f[3]))          status = "NO_REST_BASELINE";
  else {
    float z = MODEL_B;
    for (int i = 0; i < N_FEAT; i++) z += MODEL_W[i] * ((f[i] - MODEL_MEAN[i]) / MODEL_STD[i]);
    float p = 1.0f / (1.0f + expf(-z));
    snprintf(sc, sizeof(sc), "%.3f", p);
    snprintf(fl, sizeof(fl), "%d", p >= MODEL_THRESHOLD ? 1 : 0);
    status = "OK";
  }
  Serial.printf("P,%lu,%lu,%s,%s,%s,%s,RESEARCH_ONLY_NOT_DIAGNOSTIC\n",
    (unsigned long)id, (unsigned long)millis(), MODEL_LOADED ? MODEL_NAME : "none", status, sc, fl);
}

// ============================ WINDOWING ================================
void finalizeWindow() {
  char b1[14], b2[14], b3[14], b4[14], b5[14], b6[14], b7[14];
  float valid = W.total ? (float)W.n / W.total : 0;
  float dcIR = NAN, dcR = NAN, irVarPct = NAN, redVarPct = NAN, redIrRatio = NAN, relativeChangePct = NAN;
  const char* status = "OK";
  bool ok = (valid >= MIN_VALID_FRAC && W.n >= 10);

  if (!ok) {
    if (W.cNoC >= W.cMot && W.cNoC >= W.cOth) status = "NO_CONTACT";
    else if (W.cMot >= W.cOth)                status = "MOTION";
    else                                      status = "LOW_QUALITY";
  } else {
    dcIR = (float)W.mIR; dcR = (float)W.mR;
    float sdIR = sqrtf((float)(W.m2IR / (W.n - 1)));
    float sdR  = sqrtf((float)(W.m2R  / (W.n - 1)));
    irVarPct = 100.0f * sdIR / dcIR;
    redVarPct  = 100.0f * sdR / dcR;
    if (irVarPct > 0) redIrRatio = redVarPct / irVarPct;
    if (S.phase == 0 || S.phase == 1)
      restIrVarPct = isnan(restIrVarPct) ? irVarPct : 0.7f * restIrVarPct + 0.3f * irVarPct;
    if (!isnan(restIrVarPct) && restIrVarPct > 0) relativeChangePct = 100.0f * (irVarPct - restIrVarPct) / restIrVarPct;
  }

  Serial.printf("W,%lu,%lu,%d,%d,%d,%d,%u,%.2f,%s,%s,%s,%s,%s,%s,%s\n",
    (unsigned long)winId, (unsigned long)millis(), S.subj, S.site, S.phase, S.label,
    W.n, valid,
    fn(b1, sizeof(b1), dcIR, 1), fn(b2, sizeof(b2), dcR, 1),
    fn(b3, sizeof(b3), irVarPct, 3), fn(b4, sizeof(b4), redVarPct, 3),
    fn(b5, sizeof(b5), redIrRatio, 3),  fn(b6, sizeof(b6), relativeChangePct, 1), status);

  float f[N_FEAT] = {irVarPct, redVarPct, redIrRatio, relativeChangePct};
  emitPrediction(winId, ok, f);
  winId++;
}

void windowAdd(uint32_t red, uint32_t ir, uint8_t q) {
  W.total++;
  if (q == 0) {
    W.n++;
    double d = (double)ir - W.mIR;  W.mIR += d / W.n;  W.m2IR += d * ((double)ir - W.mIR);
    d = (double)red - W.mR;         W.mR  += d / W.n;  W.m2R  += d * ((double)red - W.mR);
  } else if (q & 4)  W.cNoC++;
  else if (q & 16)   W.cMot++;
  else               W.cOth++;
  if (W.total >= WINDOW_SAMPLES) { finalizeWindow(); resetWindow(); }
}

// ============================ SAMPLE PIPELINE ==========================
void processSample(uint32_t red, uint32_t ir) {
  bool contact = (ir >= IR_CONTACT_MIN);

  // Baseline updates only while in contact
  if (contact) {
    if (baseCount == 0) { redBase = red; irBase = ir; }
    else {
      redBase += BASELINE_ALPHA * (red - redBase);
      irBase  += BASELINE_ALPHA * (ir  - irBase);
    }
    baseCount++;
  }
  float redDev = (contact && redBase > 0) ? 100.0f * (red - redBase) / redBase : 0;
  float irDev  = (contact && irBase  > 0) ? 100.0f * (ir  - irBase)  / irBase  : 0;

  // IMU
  float accDevG = 0, gyroDps = 0, pitch = 0, roll = 0;
  bool motion = false;
  if (imuOk) {
    readImu();
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    accDevG = fabsf(mag / G_MS2 - 1.0f);
    gyroDps = sqrtf(gx * gx + gy * gy + gz * gz);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;  // mounting-dependent
    roll  = atan2f(ay, az) * RAD2DEG;
    motion = (accDevG > ACC_DEV_G_MAX) || (gyroDps > GYRO_DPS_MAX);
  }

  uint8_t q = 0;
  if (!opticalOk)                                       q |= 1;
  if (!imuOk)                                           q |= 2;
  if (!contact)                                         q |= 4;
  if (ir > SATURATION_LEVEL || red > SATURATION_LEVEL)  q |= 8;
  if (motion)                                           q |= 16;
  if (baseCount < WARMUP_SAMPLES)                       q |= 32;

  emitD(red, ir, redDev, irDev, pitch, roll, accDevG, gyroDps, motion, q);
  windowAdd(red, ir, q);
  seq++;
}

// ============================ COMMANDS =================================
void ack() {
  Serial.printf("# OK subj=%d site=%d phase=%d label=%d mark=%u\n", S.subj, S.site, S.phase, S.label, S.mark);
}

void handleLine(char* line) {
  char cmd[16] = {0};
  long v = 0;
  int n = sscanf(line, "%15s %ld", cmd, &v);
  if (n < 1) return;

  if (!strcasecmp(cmd, "subj") && n == 2 && v >= 0 && v <= 9999) { S.subj = (int16_t)v; resetSubjectSiteReferences(); ack(); }
  else if (!strcasecmp(cmd, "site") && n == 2 && v >= 0 && v <= 9)  { S.site = (int8_t)v; resetSubjectSiteReferences(); ack(); }
  else if (!strcasecmp(cmd, "phase") && n == 2 && v >= 0 && v <= 9) { S.phase = (int8_t)v; resetWindow(); ack(); }
  else if (!strcasecmp(cmd, "label") && n == 2 && v >= -1 && v <= 1){ S.label = (int8_t)v; resetWindow(); ack(); }
  else if (!strcasecmp(cmd, "mark"))   { S.mark++; ack(); }
  else if (!strcasecmp(cmd, "cal"))    { Serial.println("# CAL: keep still..."); calibrateImu(); }
  else if (!strcasecmp(cmd, "rest"))   { restIrVarPct = NAN; Serial.println("# OK resting IR variation reference cleared"); }
  else if (!strcasecmp(cmd, "status")) { printStatus(); }
  else if (!strcasecmp(cmd, "help"))   { printHelp(); }
  else Serial.printf("# ERR unknown or invalid command: %s\n", line);
}

void pollSerial() {
  static char buf[48];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) { buf[len] = 0; handleLine(buf); len = 0; }
    } else if (len < sizeof(buf) - 1) buf[len++] = c;
  }
}

// ============================ SETUP / LOOP =============================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  Serial.println("# VENOTRACE - Wearable Optical Sensing and Motion-Aware Signal Analysis");
  Serial.println("# Research prototype only; not a DVT diagnostic device.");
  Serial.println("# Firmware version: 0.4");
  Serial.printf("# built %s %s\n", __DATE__, __TIME__);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  i2cScan();
  opticalOk = initOptical();
  imuOk = initImu();
  Serial.println("# keep device still for IMU calibration...");
  calibrateImu();
  printStatus();
  printLegends();
  Serial.println("# MODEL STATUS: NO_MODEL (research classifier disabled).");
  Serial.println("# type 'help' for commands");
}

void loop() {
  pollSerial();

  if ((!opticalOk || !imuOk) && millis() - lastRetryMs > RETRY_INTERVAL_MS) {
    lastRetryMs = millis();
    if (!opticalOk) opticalOk = initOptical();
    if (!imuOk) { imuOk = initImu(); if (imuOk) calibrateImu(); }
  }

  if (!opticalOk) {   // heartbeat rows so the stream never goes silent
    static uint32_t lastBeat = 0;
    if (millis() - lastBeat >= 1000 / FS_HZ) { lastBeat = millis(); processSample(0, 0); }
    return;
  }

  optical.check();
  while (optical.available()) {
    uint32_t red = optical.getFIFORed();
    uint32_t ir  = optical.getFIFOIR();
    optical.nextSample();
    processSample(red, ir);
  }
}
