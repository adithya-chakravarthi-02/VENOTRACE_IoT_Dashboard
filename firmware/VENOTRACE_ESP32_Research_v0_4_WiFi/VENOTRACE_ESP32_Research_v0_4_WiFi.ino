/*
  =============================================================================
  VENOTRACE - Wearable Optical Sensing & Motion-Aware Live Wi-Fi Streamer
  Target Hardware: ESP32 DevKit V1 + MAX30102 + MPU6050
  Mode: Direct Wi-Fi Access Point (ESP32 broadcasts its own Wi-Fi)
  =============================================================================

  HOW IT WORKS:
  1. The ESP32 starts its OWN Wi-Fi network (Hotspot / SoftAP):
       SSID:     VENOTRACE_ESP32
       Password: (None - Open network by default for instant connection)
       IP:       192.168.4.1
  2. You connect your laptop directly to the "VENOTRACE_ESP32" Wi-Fi.
  3. The ESP32 blasts live data in TWO ways:
       - Direct in browser: Open http://192.168.4.1 (live waveforms & stats)
       - TCP Stream: Port 8888 (streams raw 25 Hz tagged rows for Python Dashboard)
       - Serial (USB) 115200 baud remains simultaneously active.

  NO THIRD-PARTY WI-FI LIBRARIES NEEDED:
  Uses only native ESP32 core libraries (<WiFi.h>, <WebServer.h>, <WiFiServer.h>)
  along with SparkFun MAX3010x and Adafruit MPU6050.
*/

#include <WiFi.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include <strings.h>
#include "MAX30105.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ============================ USER CONFIGURATION ============================
#define SERIAL_BAUD        115200
#define I2C_SDA            21
#define I2C_SCL            22
#define I2C_CLOCK_HZ       100000

// Wi-Fi Access Point (Broadcast from ESP32)
#define WIFI_SSID          "VENOTRACE_ESP32"
#define WIFI_PASS          ""              // "" = Open (No password). Set to "12345678" if password desired.
#define TCP_STREAM_PORT    8888

// MAX30102 Optical Sensor
#define LED_BRIGHTNESS     80              // 0-255 (Adjust if signal count is too low/high)
#define SAMPLE_AVG         4
#define LED_MODE           2               // Red + IR
#define SAMPLE_RATE        100
#define PULSE_WIDTH        411
#define ADC_RANGE          16384
#define FS_HZ              (SAMPLE_RATE / SAMPLE_AVG)   // 25 Hz

// Signal Quality Gates
#define IR_CONTACT_MIN     20000UL
#define SATURATION_LEVEL   260000UL
#define BASELINE_ALPHA     0.002f
#define WARMUP_SAMPLES     (FS_HZ * 10)
#define ACC_DEV_G_MAX      0.15f
#define GYRO_DPS_MAX       30.0f

// Feature Windowing (5 seconds = 125 samples at 25 Hz)
#define WINDOW_SAMPLES     (FS_HZ * 5)
#define MIN_VALID_FRAC     0.80f

// Research Model Slot (Disabled by default)
#define MODEL_LOADED       0
#define MODEL_NAME         "none"
#define N_FEAT             4
const float MODEL_MEAN[N_FEAT] = {0, 0, 0, 0};
const float MODEL_STD[N_FEAT]  = {1, 1, 1, 1};
const float MODEL_W[N_FEAT]    = {0, 0, 0, 0};
const float MODEL_B            = 0;
const float MODEL_THRESHOLD    = 0.5f;

#define RAD2DEG            57.2957795f
#define G_MS2              9.80665f
#define RETRY_INTERVAL_MS  2000

MAX30105 optical;
Adafruit_MPU6050 imu;

// Wi-Fi Servers
WiFiServer tcpServer(TCP_STREAM_PORT);
WiFiClient tcpClient;
WebServer webServer(80);

bool opticalOk = false, imuOk = false;
uint8_t imuAddr = 0;
uint32_t seq = 0, winId = 0, lastRetryMs = 0;

// Session labels
struct Session { int16_t subj; int8_t site; int8_t phase; int8_t label; uint16_t mark; };
Session S = {0, 0, 0, -1, 0};

// Optical baseline
float redBase = 0, irBase = 0;
uint32_t baseCount = 0;

// IMU state & calibration
float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, tempC = 0;
float gyroBias[3] = {0, 0, 0};
float accScale = 1.0f;

// Window accumulators
struct Win {
  uint16_t total, n, cNoC, cMot, cOth;
  double mIR, m2IR, mR, m2R;
};
Win W = {};
float restIrVarPct = NAN;

// Live cached reading for the Web Monitor
struct LatestSample {
  uint32_t red = 0;
  uint32_t ir = 0;
  float redDev = 0;
  float irDev = 0;
  float accDev = 0;
  float gyroDps = 0;
  float pitch = 0;
  float roll = 0;
  bool motion = false;
  uint8_t q = 0;
  uint32_t timestamp = 0;
} live;

// Forward declaration
void handleLine(char* line);

// ============================ BROADCAST HELPER =========================
void broadcastLine(const char* line) {
  Serial.println(line);
  if (tcpClient && tcpClient.connected()) {
    tcpClient.println(line);
  }
}

// ============================ ON-CHIP WEB MONITOR UI ====================
const char PAGE_INDEX[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>VENOTRACE | ESP32 Live Telemetry</title>
<style>
:root{--bg:#070d19;--card:#0f172a;--border:#1e293b;--text:#f8fafc;--muted:#94a3b8;--cyan:#38bdf8;--orange:#fb923c;--green:#4ade80;--red:#f87171;--blue:#818cf8}
*{box-sizing:border-box;margin:0;padding:0}
body{background:radial-gradient(1200px 600px at 20% -10%,#1e1b4b 0%,transparent 60%),var(--bg);color:var(--text);font-family:Inter,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;padding:24px 16px;line-height:1.5}
.container{max-width:1100px;margin:0 auto}
header{display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:16px;margin-bottom:20px;padding-bottom:16px;border-bottom:1px solid var(--border)}
.brand{display:flex;align-items:center;gap:14px}
.logo{width:46px;height:46px;background:linear-gradient(135deg,#38bdf8,#818cf8);color:#030712;display:grid;place-items:center;border-radius:14px;font-weight:900;font-size:16px;box-shadow:0 8px 24px rgba(56,189,248,.3)}
h1{font-size:24px;letter-spacing:-.03em}
.badge{background:#1e293b;border:1px solid #334155;border-radius:999px;padding:6px 14px;font-size:12px;color:#cbd5e1;display:flex;align-items:center;gap:8px}
.dot{width:8px;height:8px;border-radius:50%;background:var(--green);box-shadow:0 0 10px var(--green);animation:pulse 1.8s infinite}
@keyframes pulse{0%{opacity:1;transform:scale(1)}50%{opacity:.4;transform:scale(.85)}100%{opacity:1;transform:scale(1)}}
.notice{background:#1e1a0f;border:1px solid #78350f;color:#fef3c7;padding:14px 18px;border-radius:14px;font-size:12px;margin-bottom:20px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(230px,1fr));gap:14px;margin-bottom:18px}
.card{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:18px;box-shadow:0 10px 30px rgba(0,0,0,.3)}
.label{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:.08em;font-weight:700}
.val{font-size:32px;font-weight:800;margin:8px 0 4px;letter-spacing:-.03em;font-variant-numeric:tabular-nums}
.sub{font-size:11px;color:var(--muted)}
.chart-box{background:var(--card);border:1px solid var(--border);border-radius:16px;padding:18px;box-shadow:0 10px 30px rgba(0,0,0,.3);margin-bottom:18px}
.chart-head{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;flex-wrap:wrap;gap:8px}
canvas{width:100%;height:240px;background:#030712;border:1px solid #1e293b;border-radius:12px;display:block}
.row{display:flex;gap:10px;flex-wrap:wrap;margin-top:16px}
button{background:#1e293b;border:1px solid #334155;color:#f8fafc;padding:9px 16px;border-radius:10px;cursor:pointer;font-weight:600;font-size:12px;transition:.15s}
button:hover{filter:brightness(1.2)}
button.primary{background:var(--cyan);color:#030712;border-color:transparent;font-weight:700}
footer{display:flex;justify-content:space-between;flex-wrap:wrap;gap:10px;color:var(--muted);font-size:11px;margin-top:24px;padding:16px 4px 0;border-top:1px solid var(--border)}
</style>
</head>
<body>
<div class="container">
  <header>
    <div class="brand">
      <div class="logo">VT</div>
      <div>
        <h1>VENOTRACE Wi-Fi Live Stream</h1>
        <div style="font-size:12px;color:var(--muted)">Direct wireless broadcast from ESP32 Access Point</div>
      </div>
    </div>
    <div class="badge">
      <span class="dot"></span>
      <span>Wi-Fi SSID: <strong>VENOTRACE_ESP32</strong> · IP: <strong>192.168.4.1</strong></span>
    </div>
  </header>

  <div class="notice">
    <strong>Live ESP32 Transmission:</strong> MAX30102 optical PPG & MPU6050 motion readings are being streamed in real time. For long-term SQLite database recording, 5-second window statistical analysis, and CSV exports, open the VENOTRACE Python Dashboard on your laptop at <code>http://127.0.0.1:8000</code>.
  </div>

  <div class="grid">
    <div class="card">
      <div class="label">Optical Infrared (IR)</div>
      <div id="vIr" class="val" style="color:var(--cyan)">—</div>
      <div id="vIrDev" class="sub">Baseline deviation: —</div>
    </div>
    <div class="card">
      <div class="label">Optical Red</div>
      <div id="vRed" class="val" style="color:var(--orange)">—</div>
      <div id="vRedDev" class="sub">Baseline deviation: —</div>
    </div>
    <div class="card">
      <div class="label">Acceleration Deviation</div>
      <div id="vAcc" class="val" style="color:var(--blue)">—</div>
      <div id="vMotion" class="sub">Motion filter: —</div>
    </div>
    <div class="card">
      <div class="label">Quality Flag (q)</div>
      <div id="vQ" class="val" style="color:var(--green)">WAITING</div>
      <div id="vStatus" class="sub">Sensor check…</div>
    </div>
  </div>

  <div class="chart-box">
    <div class="chart-head">
      <div>
        <div class="label">Live Optical Oscilloscope (25 Hz Stream)</div>
        <div style="font-size:11px;color:var(--muted);margin-top:2px">Real-time Red and Infrared reflection counts from MAX30102</div>
      </div>
      <div style="font-size:12px;display:flex;gap:14px">
        <span style="color:var(--cyan)">● Infrared (IR)</span>
        <span style="color:var(--orange)">● Red</span>
      </div>
    </div>
    <canvas id="scope"></canvas>
    <div class="row">
      <button class="primary" onclick="sendCmd('mark')">Add Event Marker</button>
      <button onclick="sendCmd('cal')">Calibrate IMU</button>
      <button onclick="sendCmd('rest')">Clear Resting Baseline</button>
      <button onclick="sendCmd('status')">Device Status</button>
    </div>
  </div>

  <footer>
    <span>ESP32 Wi-Fi Hotspot: <code>VENOTRACE_ESP32</code> (192.168.4.1)</span>
    <span>TCP Stream Server: <code>192.168.4.1:8888</code></span>
  </footer>
</div>

<script>
const irWave = [], redWave = [];
const MAX_SAMPLES = 120;
const canvas = document.getElementById('scope');
const ctx = canvas.getContext('2d');

function fitCanvas() {
  canvas.width = canvas.parentElement.clientWidth - 36;
  canvas.height = 240;
}
window.addEventListener('resize', fitCanvas);
fitCanvas();

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if (irWave.length < 2) return;

  const all = [...irWave, ...redWave];
  let min = Math.min(...all), max = Math.max(...all);
  if (min === max) { min -= 10; max += 10; }
  const pad = (max - min) * 0.08;
  min -= pad; max += pad;

  // Grid lines
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1;
  for (let i = 1; i <= 3; i++) {
    const y = (canvas.height / 4) * i;
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(canvas.width, y);
    ctx.stroke();
  }

  function plotLine(data, color) {
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.2;
    ctx.lineJoin = 'round';
    data.forEach((val, i) => {
      const x = (i / (MAX_SAMPLES - 1)) * canvas.width;
      const y = canvas.height - ((val - min) / (max - min)) * canvas.height;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    });
    ctx.stroke();
  }

  plotLine(irWave, '#38bdf8');
  plotLine(redWave, '#fb923c');
}

let inFlight = false;
async function fetchStream() {
  if (inFlight) return;
  inFlight = true;
  try {
    const res = await fetch('/api/latest');
    if (!res.ok) return;
    const d = await res.json();
    document.getElementById('vIr').textContent = Number(d.ir).toLocaleString();
    document.getElementById('vRed').textContent = Number(d.red).toLocaleString();
    document.getElementById('vIrDev').textContent = 'Baseline dev: ' + (d.ir_dev || 0).toFixed(2) + '%';
    document.getElementById('vRedDev').textContent = 'Baseline dev: ' + (d.red_dev || 0).toFixed(2) + '%';
    document.getElementById('vAcc').textContent = (d.acc_dev || 0).toFixed(3) + ' g';
    document.getElementById('vMotion').textContent = d.motion ? 'Motion detected' : 'Sensor stable (low accel dev)';

    let qText = 'USABLE';
    let qColor = 'var(--green)';
    if (d.q & 4) { qText = 'NO CONTACT'; qColor = 'var(--red)'; }
    else if (d.q & 16) { qText = 'MOTION'; qColor = 'var(--orange)'; }
    else if (d.q & 32) { qText = 'WARMUP'; qColor = 'var(--orange)'; }
    else if (d.q & 8) { qText = 'SATURATED'; qColor = 'var(--red)'; }
    else if (d.q & 1) { qText = 'OPTICAL SENSOR DOWN'; qColor = 'var(--red)'; }
    else if (d.q & 2) { qText = 'IMU DOWN'; qColor = 'var(--red)'; }
    
    const qEl = document.getElementById('vQ');
    qEl.textContent = qText;
    qEl.style.color = qColor;
    document.getElementById('vStatus').textContent = 'Gyro: ' + (d.gyro_dps || 0).toFixed(1) + ' dps · Temp: ' + (d.temp || 0).toFixed(1) + ' °C';

    irWave.push(d.ir);
    redWave.push(d.red);
    if (irWave.length > MAX_SAMPLES) irWave.shift();
    if (redWave.length > MAX_SAMPLES) redWave.shift();
    draw();
  } catch (e) {
  } finally {
    inFlight = false;
  }
}

async function sendCmd(cmd) {
  try {
    await fetch('/api/command?cmd=' + encodeURIComponent(cmd), { method: 'POST' });
  } catch(e) {}
}

setInterval(fetchStream, 50); // 20 updates per second
</script>
</body>
</html>
)rawliteral";

// ============================ WEB SERVER HANDLERS ======================
void handleRoot() {
  webServer.send_P(200, "text/html", PAGE_INDEX);
}

void handleApiLatest() {
  char json[380];
  snprintf(json, sizeof(json),
    "{\"seq\":%lu,\"t_ms\":%lu,\"red\":%lu,\"ir\":%lu,\"red_base\":%.1f,\"ir_base\":%.1f,"
    "\"red_dev\":%.2f,\"ir_dev\":%.2f,\"acc_dev\":%.3f,\"gyro_dps\":%.2f,"
    "\"pitch\":%.1f,\"roll\":%.1f,\"temp\":%.1f,\"motion\":%s,\"q\":%u}",
    (unsigned long)seq, (unsigned long)live.timestamp,
    (unsigned long)live.red, (unsigned long)live.ir, redBase, irBase,
    live.redDev, live.irDev, live.accDev, live.gyroDps,
    live.pitch, live.roll, tempC, live.motion ? "true" : "false", live.q);
  webServer.send(200, "application/json", json);
}

void handleApiCommand() {
  if (webServer.hasArg("cmd")) {
    String cmd = webServer.arg("cmd");
    char buf[48];
    cmd.toCharArray(buf, sizeof(buf));
    handleLine(buf);
    webServer.send(200, "application/json", "{\"ok\":true}");
  } else {
    webServer.send(400, "application/json", "{\"error\":\"Missing cmd\"}");
  }
}

// ============================ SENSOR HELPERS ===========================
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
  broadcastLine("# I2C scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (i2cPresent(a)) {
      char buf[64];
      snprintf(buf, sizeof(buf), "#   0x%02X%s", a,
        a == 0x57 ? "  (MAX30102)" : (a == 0x68 || a == 0x69) ? "  (MPU6050)" : "");
      broadcastLine(buf);
      found++;
    }
  }
  if (!found) broadcastLine("#   none found: check VCC/GND/SDA/SCL");
}

void resetWindow() { memset(&W, 0, sizeof(W)); }

void resetSubjectSiteReferences() {
  redBase = 0;
  irBase = 0;
  baseCount = 0;
  restIrVarPct = NAN;
  resetWindow();
}

// ============================ SENSOR INITIALIZATION ====================
bool initOptical() {
  if (!i2cPresent(0x57)) { broadcastLine("# MAX30102: no ACK at 0x57"); return false; }
  if (!optical.begin(Wire, I2C_SPEED_STANDARD)) { broadcastLine("# MAX30102: begin() failed"); return false; }
  optical.setup(LED_BRIGHTNESS, SAMPLE_AVG, LED_MODE, SAMPLE_RATE, PULSE_WIDTH, ADC_RANGE);
  optical.clearFIFO();
  baseCount = 0;
  char buf[80];
  snprintf(buf, sizeof(buf), "# MAX30102: OK part_id=0x%02X fs=%d Hz LED=%d",
           optical.readPartID(), FS_HZ, LED_BRIGHTNESS);
  broadcastLine(buf);
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
      char buf[80];
      snprintf(buf, sizeof(buf), "# MPU6050: OK addr=0x%02X range=+-4g/+-500dps", imuAddr);
      broadcastLine(buf);
      return true;
    }
  }
  broadcastLine("# MPU6050: not found at 0x68/0x69");
  return false;
}

bool calibrateImu() {
  if (!imuOk) { broadcastLine("# CAL: IMU not available"); return false; }
  const int N = 60;
  double sg[3] = {0, 0, 0}, sa = 0;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, t;
    if (!imu.getEvent(&a, &g, &t)) { broadcastLine("# CAL: read failed"); return false; }
    sg[0] += g.gyro.x; sg[1] += g.gyro.y; sg[2] += g.gyro.z;
    sa += sqrt(a.acceleration.x * a.acceleration.x +
               a.acceleration.y * a.acceleration.y +
               a.acceleration.z * a.acceleration.z);
    delay(10);
  }
  float b[3];
  for (int k = 0; k < 3; k++) b[k] = (float)(sg[k] / N) * RAD2DEG;
  if (fabsf(b[0]) > 30 || fabsf(b[1]) > 30 || fabsf(b[2]) > 30) {
    broadcastLine("# CAL: rejected (device moving). Keep still and send: cal");
    return false;
  }
  for (int k = 0; k < 3; k++) gyroBias[k] = b[k];
  accScale = G_MS2 / (float)(sa / N);
  char buf[90];
  snprintf(buf, sizeof(buf), "# CAL: gyro_bias=[%.2f,%.2f,%.2f] dps acc_scale=%.4f",
           gyroBias[0], gyroBias[1], gyroBias[2], accScale);
  broadcastLine(buf);
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
    broadcastLine("# MPU6050: read failed, will retry");
  }
}

// ============================ PROTOCOL OUTPUT ==========================
void printLegends() {
  broadcastLine("#LEGEND,phase,0=rest_supine,1=rest_sitting,2=leg_elevated,3=calf_pump,4=standing,5=walking,6=recovery");
  broadcastLine("#LEGEND,site,0=calf_L,1=calf_R,2=ankle_L,3=ankle_R,4=thigh_L,5=thigh_R");
  broadcastLine("#LEGEND,label,-1=unlabelled,0=control(reference-negative),1=reference-positive");
  broadcastLine("#LEGEND,q,1=max_down,2=imu_down,4=no_contact,8=saturated,16=motion,32=warmup,0=passed_configured_quality_gates");
  broadcastLine("#SCHEMA,D,t_ms,seq,subj,site,phase,label,mark,red,ir,red_base,ir_base,red_dev_pct,ir_dev_pct,"
                 "ax_ms2,ay_ms2,az_ms2,gx_dps,gy_dps,gz_dps,pitch_deg,roll_deg,acc_dev_g,gyro_dps,temp_c,motion,q");
  broadcastLine("#SCHEMA,W,win,t_ms,subj,site,phase,label,n_valid,valid_frac,ir_mean,red_mean,"
                 "ir_variation_pct,red_variation_pct,red_ir_variation_ratio,relative_change_from_rest_pct,status");
  broadcastLine("#SCHEMA,P,win,t_ms,model,status,score,flag,note");
}

void printStatus() {
  char buf[160];
  snprintf(buf, sizeof(buf),
    "# STATUS optical=%s imu=%s subj=%d site=%d phase=%d label=%d mark=%u model=%s rest_ir_var_ref=%s",
    opticalOk ? "OK" : "FAIL", imuOk ? "OK" : "FAIL", S.subj, S.site, S.phase, S.label, S.mark,
    MODEL_LOADED ? MODEL_NAME : "none", isnan(restIrVarPct) ? "NA" : "set");
  broadcastLine(buf);
}

void printHelp() {
  broadcastLine("# CMDS: subj <n> | site <n> | phase <n> | label <-1|0|1> | mark | cal | rest | status | help");
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
  broadcastLine(buf);
}

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
  char buf[128];
  snprintf(buf, sizeof(buf), "P,%lu,%lu,%s,%s,%s,%s,RESEARCH_ONLY_NOT_DIAGNOSTIC",
    (unsigned long)id, (unsigned long)millis(), MODEL_LOADED ? MODEL_NAME : "none", status, sc, fl);
  broadcastLine(buf);
}

void finalizeWindow() {
  char b1[14], b2[14], b3[14], b4[14], b5[14], b6[14];
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

  char buf[200];
  snprintf(buf, sizeof(buf), "W,%lu,%lu,%d,%d,%d,%d,%u,%.2f,%s,%s,%s,%s,%s,%s,%s",
    (unsigned long)winId, (unsigned long)millis(), S.subj, S.site, S.phase, S.label,
    W.n, valid,
    fn(b1, sizeof(b1), dcIR, 1), fn(b2, sizeof(b2), dcR, 1),
    fn(b3, sizeof(b3), irVarPct, 3), fn(b4, sizeof(b4), redVarPct, 3),
    fn(b5, sizeof(b5), redIrRatio, 3),  fn(b6, sizeof(b6), relativeChangePct, 1), status);
  broadcastLine(buf);

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

void processSample(uint32_t red, uint32_t ir) {
  bool contact = (ir >= IR_CONTACT_MIN);

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

  float accDevG = 0, gyroDps = 0, pitch = 0, roll = 0;
  bool motion = false;
  if (imuOk) {
    readImu();
    float mag = sqrtf(ax * ax + ay * ay + az * az);
    accDevG = fabsf(mag / G_MS2 - 1.0f);
    gyroDps = sqrtf(gx * gx + gy * gy + gz * gz);
    pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD2DEG;
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

  // Cache sample for web monitor
  live.red = red;
  live.ir = ir;
  live.redDev = redDev;
  live.irDev = irDev;
  live.accDev = accDevG;
  live.gyroDps = gyroDps;
  live.pitch = pitch;
  live.roll = roll;
  live.motion = motion;
  live.q = q;
  live.timestamp = millis();

  emitD(red, ir, redDev, irDev, pitch, roll, accDevG, gyroDps, motion, q);
  windowAdd(red, ir, q);
  seq++;
}

// ============================ COMMAND PROCESSING =======================
void ack() {
  char buf[80];
  snprintf(buf, sizeof(buf), "# OK subj=%d site=%d phase=%d label=%d mark=%u", S.subj, S.site, S.phase, S.label, S.mark);
  broadcastLine(buf);
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
  else if (!strcasecmp(cmd, "cal"))    { broadcastLine("# CAL: keep still..."); calibrateImu(); }
  else if (!strcasecmp(cmd, "rest"))   { restIrVarPct = NAN; broadcastLine("# OK resting IR variation reference cleared"); }
  else if (!strcasecmp(cmd, "status")) { printStatus(); }
  else if (!strcasecmp(cmd, "help"))   { printHelp(); }
  else {
    char buf[80];
    snprintf(buf, sizeof(buf), "# ERR unknown or invalid command: %s", line);
    broadcastLine(buf);
  }
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

void pollTcpClient() {
  if (tcpServer.hasClient()) {
    if (!tcpClient || !tcpClient.connected()) {
      tcpClient = tcpServer.available();
      broadcastLine("# TCP client connected: Live telemetry streaming active");
      printLegends();
      printStatus();
    } else {
      WiFiClient rejected = tcpServer.available();
      rejected.stop();
    }
  }

  if (tcpClient && tcpClient.connected()) {
    static char tcpBuf[48];
    static uint8_t tcpLen = 0;
    while (tcpClient.available()) {
      char c = (char)tcpClient.read();
      if (c == '\n' || c == '\r') {
        if (tcpLen) { tcpBuf[tcpLen] = 0; handleLine(tcpBuf); tcpLen = 0; }
      } else if (tcpLen < sizeof(tcpBuf) - 1) tcpBuf[tcpLen++] = c;
    }
  }
}

// ============================ SETUP & MAIN LOOP ========================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println("\n# =============================================================");
  Serial.println("# VENOTRACE Wearable Optical & Motion Live Wi-Fi Platform");
  Serial.println("# =============================================================");

  // 1. Initialize Wi-Fi SoftAP (Broadcast from ESP32)
  WiFi.mode(WIFI_AP);
  if (strlen(WIFI_PASS) >= 8) {
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
  } else {
    WiFi.softAP(WIFI_SSID); // Open network
  }
  delay(100);

  IPAddress apIP = WiFi.softAPIP();
  Serial.printf("# ESP32 Wi-Fi Hotspot ACTIVE:\n");
  Serial.printf("#   Network Name (SSID): %s\n", WIFI_SSID);
  Serial.printf("#   Password:            %s\n", strlen(WIFI_PASS) >= 8 ? WIFI_PASS : "None (Open Network)");
  Serial.printf("#   ESP32 Web Monitor:   http://%s\n", apIP.toString().c_str());
  Serial.printf("#   TCP Stream Port:     %s:%d\n", apIP.toString().c_str(), TCP_STREAM_PORT);

  // 2. Start Services
  tcpServer.begin();
  tcpServer.setNoDelay(true);

  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/api/latest", HTTP_GET, handleApiLatest);
  webServer.on("/api/command", HTTP_POST, handleApiCommand);
  webServer.begin();

  // 3. Sensor I2C setup
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);

  i2cScan();
  opticalOk = initOptical();
  imuOk = initImu();
  Serial.println("# Keep device still for IMU calibration...");
  calibrateImu();
  printStatus();
  printLegends();
}

void loop() {
  webServer.handleClient();
  pollTcpClient();
  pollSerial();

  if ((!opticalOk || !imuOk) && millis() - lastRetryMs > RETRY_INTERVAL_MS) {
    lastRetryMs = millis();
    if (!opticalOk) opticalOk = initOptical();
    if (!imuOk) { imuOk = initImu(); if (imuOk) calibrateImu(); }
  }

  if (!opticalOk) {
    static uint32_t lastBeat = 0;
    if (millis() - lastBeat >= 1000 / FS_HZ) {
      lastBeat = millis();
      processSample(0, 0);
    }
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
