/*
 * ESP32-CAM (AI-Thinker, OV2640) silhouette component counter  -- v3
 * ------------------------------------------------------------------
 *  - Grayscale QQVGA (160x120) capture, binary threshold, flood-fill blob count
 *  - Serial output @115200 + web UI (count, live binary view, threshold slider)
 *  - Laser toggle (GPIO 2)
 *  - 4 vibrator motors via NPN transistors (GPIO 12 / 13 / 14 / 15)
 *         + master "all on / all off" button
 *  - Servo "button pusher" (GPIO 16) with 4-state tracking machine:
 *         State 0 (Off) -> State 1 (Level 1) -> State 2 (Level 2) -> State 3 (Level 3) -> State 0
 *
 * HTTP API (all GET):
 *   /              web page
 *   /frame         binary frame + status (used by the page)
 *   /set?thr=&min=&max=
 *   /laser?state=1|0|toggle
 *   /vib?id=0..3&state=1|0|toggle        0=UpLeft 1=UpRight 2=DownLeft 3=DownRight
 *   /vib?all=1|0|toggle                  toggle: any running -> all off, else all on
 *   /servo?target=0..3                   move light pad to target state (0=Off, 1..3=Level 1..3)
 *   /servo?sync=0..3                     tell the ESP the light's real state (no motion)
 */

#include "esp_camera.h"
#include "esp_http_server.h"
#include "driver/ledc.h"
#include <WiFi.h>

// ====================== USER SETTINGS ======================
#define USE_STA_MODE 0                       // 0 = SoftAP, 1 = join existing Wi-Fi
const char* STA_SSID = "Hokman_X10";
const char* STA_PASS = "07901474661";
const char* AP_SSID  = "ESP32CAM-Counter";
const char* AP_PASS  = "12345678";           // min 8 chars

#define DEFAULT_THRESHOLD 120                // pixel < threshold  => object
#define DEFAULT_MIN_AREA  15                 // ignore dust smaller than this
#define DEFAULT_MAX_AREA  3000               // ignore blobs larger than this
#define IGNORE_EDGE_BLOBS false              // true = skip blobs touching the border

// ---- Output pins ----
#define LASER_PIN        2                   // Updated: Laser output on GPIO 2
#define VIB_UL_PIN      12                   // Up Left    (NPN base, active HIGH)
#define VIB_UR_PIN      13                   // Up Right
#define VIB_DL_PIN      14                   // Down Left
#define VIB_DR_PIN      15                   // Down Right
#define SERVO_PIN       4                   // Requires PSRAM Disabled on AI-Thinker ESP32-CAM

// ---- Servo tuning ----
#define SERVO_HOME_ANGLE     165             // resting position (degrees)
#define SERVO_PUSH_ANGLE     180             // target pressing angle
#define SERVO_PUSH_HOLD_MS   250             // how long the button is held down (200-300ms)
#define SERVO_SETTLE_MS      300             // time to return to resting angle
#define SERVO_INTER_PRESS_MS 150             // extra pause between consecutive presses
#define SERVO_MIN_US         500             // pulse width at 0 degrees
#define SERVO_MAX_US        2500             // pulse width at 180 degrees

#define VIB_STAGGER_MS        30             // delay between motors on "all on" (limits inrush)
// ===========================================================

// ---- AI-Thinker ESP32-CAM pin map ----
#define PWDN_GPIO_NUM   32
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM   26
#define SIOC_GPIO_NUM   27
#define Y9_GPIO_NUM     35
#define Y8_GPIO_NUM     34
#define Y7_GPIO_NUM     39
#define Y6_GPIO_NUM     36
#define Y5_GPIO_NUM     21
#define Y4_GPIO_NUM     19
#define Y3_GPIO_NUM     18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM  25
#define HREF_GPIO_NUM   23
#define PCLK_GPIO_NUM   22

// ---- Image geometry ----
static const int W = 160;
static const int H = 120;
static const int NPIX = W * H;
static const int PACKED_BYTES = NPIX / 8;    // 2400
static const int MAX_BOXES = 64;
static const int HEADER_BYTES = 10;

// ---- Working buffers ----
static uint8_t*  bin   = nullptr;            // 0 bg, 1 object, 2 visited
static uint16_t* queue = nullptr;            // BFS queue (pixel indices)

// ---- Shared vision state (guarded by gMutex) ----
static SemaphoreHandle_t gMutex;
static volatile uint8_t  gThreshold = DEFAULT_THRESHOLD;
static volatile uint16_t gMinArea   = DEFAULT_MIN_AREA;
static volatile uint16_t gMaxArea   = DEFAULT_MAX_AREA;

struct Box { uint8_t x0, y0, x1, y1; };
static uint16_t gCount = 0;
static uint8_t  gNumBoxes = 0;
static Box      gBoxes[MAX_BOXES];
static uint8_t  gPacked[PACKED_BYTES];
static uint8_t  gOut[HEADER_BYTES + MAX_BOXES * 4 + PACKED_BYTES];   // HTTP response scratch

// ---- Output & Light Pad State ----
#define IO_LASER     0x01
#define IO_VIB_MASK  0x1E                    // bit (i+1) = vibrator i
static const uint8_t VIB_PINS[4] = { VIB_UL_PIN, VIB_UR_PIN, VIB_DL_PIN, VIB_DR_PIN };
static const char*   VIB_NAMES[4] = { "Up Left", "Up Right", "Down Left", "Down Right" };
static volatile uint8_t gIO = 0;

static TaskHandle_t     servoTaskHandle    = nullptr;
static bool             servoReady         = false;
static volatile bool    servoBusy          = false;
static volatile uint8_t currentLightLevel  = 0;  // 0=Off, 1=Level 1, 2=Level 2, 3=Level 3
static volatile uint8_t targetLightLevel   = 0;  // requested target state (0..3)

// ============================ WEB PAGE ============================
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Component Counter</title>
<style>
 body{font-family:system-ui,Arial,sans-serif;background:#111;color:#eee;margin:0;padding:16px;text-align:center}
 h1{font-size:1.1rem;font-weight:500;color:#aaa;margin:0}
 h2{font-size:.95rem;font-weight:600;color:#9ca3af;margin:0 0 8px 0;text-transform:uppercase;letter-spacing:.05em}
 #count{font-size:7rem;font-weight:700;line-height:1.1;color:#4ade80}
 canvas{width:min(100%,640px);image-rendering:pixelated;background:#fff;border:2px solid #444;border-radius:6px}
 .panel{max-width:640px;margin:14px auto;text-align:left;background:#1c1c1c;padding:12px 16px;border-radius:8px}
 .row{display:flex;align-items:center;gap:10px;margin:8px 0}
 .row label{width:130px;color:#bbb}
 input[type=range]{flex:1}
 input[type=number],select{background:#000;color:#fff;border:1px solid #555;padding:4px;border-radius:4px}
 input[type=number]{width:90px}
 .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
 .btn{background:#2a2a2a;color:#eee;border:1px solid #555;border-radius:8px;padding:14px 10px;font-size:1rem;cursor:pointer;touch-action:manipulation}
 .btn:active{filter:brightness(1.3)}
 .btn.on{background:#16a34a;border-color:#22c55e;color:#fff}
 .btn.laser.on{background:#dc2626;border-color:#f87171}
 .btn:disabled{opacity:.45;cursor:not-allowed}
 .wide{width:100%;margin-top:10px}
 #status{font-size:.8rem;color:#777}
 .note{font-size:.8rem;color:#777}
</style></head><body>
<h1>COMPONENTS DETECTED</h1>
<div id="count">--</div>
<canvas id="cv" width="640" height="480"></canvas>

<div class="panel">
 <h2>Detection</h2>
 <div class="row"><label>Threshold: <b id="tv">--</b></label>
  <input id="thr" type="range" min="0" max="255" value="120"></div>
 <div class="row"><label>Min blob area</label><input id="mn" type="number" min="1" max="19200"></div>
 <div class="row"><label>Max blob area</label><input id="mx" type="number" min="1" max="19200"></div>
</div>

<div class="panel">
 <h2>Laser</h2>
 <button id="laser" class="btn laser wide" style="margin-top:0">Laser: --</button>
</div>

<div class="panel">
 <h2>Vibrators</h2>
 <div class="grid">
  <button id="v0" class="btn" data-name="Up Left">Up Left</button>
  <button id="v1" class="btn" data-name="Up Right">Up Right</button>
  <button id="v2" class="btn" data-name="Down Left">Down Left</button>
  <button id="v3" class="btn" data-name="Down Right">Down Right</button>
 </div>
 <button id="vall" class="btn wide">All vibrators</button>
</div>

<div class="panel">
 <h2>Light Pad Control</h2>
 <div class="grid">
  <button id="l1" class="btn" data-lvl="1">Level 1</button>
  <button id="l2" class="btn" data-lvl="2">Level 2</button>
  <button id="l3" class="btn" data-lvl="3">Level 3</button>
  <button id="l0" class="btn" data-lvl="0">Off</button>
 </div>
 <div class="row" style="margin-top:12px">
  <span>Current State: <b id="lvl">--</b></span>
  <span style="flex:1"></span>
  <label for="sync" style="width:auto" class="note">Sync actual:</label>
  <select id="sync"><option value="0">Off</option><option value="1">Level 1</option>
   <option value="2">Level 2</option><option value="3">Level 3</option></select>
 </div>
 <div class="note">Cycle: Off &rarr; Level 1 &rarr; Level 2 &rarr; Level 3 &rarr; Off. Use "Sync actual" if the light pad was adjusted manually.</div>
</div>

<div class="panel"><div id="status">connecting...</div></div>

<script>
const W=160,H=120,S=4;
const $=id=>document.getElementById(id);
const cv=$('cv'),ctx=cv.getContext('2d');
const off=document.createElement('canvas');off.width=W;off.height=H;
const octx=off.getContext('2d');const img=octx.createImageData(W,H);
const thr=$('thr'),tv=$('tv'),mn=$('mn'),mx=$('mx');
const lvlNames=['Off','Level 1','Level 2','Level 3'];
const lBtns=[$('l0'),$('l1'),$('l2'),$('l3')];
let inited=false,dragging=false;

function act(path){fetch(path,{cache:'no-store'}).catch(()=>{});}
thr.addEventListener('input',()=>{dragging=true;tv.textContent=thr.value;});
thr.addEventListener('change',()=>{act('/set?thr='+thr.value);dragging=false;});
mn.addEventListener('change',()=>act('/set?min='+mn.value));
mx.addEventListener('change',()=>act('/set?max='+mx.value));

$('laser').addEventListener('click',()=>act('/laser?state=toggle'));
for(let i=0;i<4;i++) $('v'+i).addEventListener('click',()=>act('/vib?id='+i+'&state=toggle'));
$('vall').addEventListener('click',()=>act('/vib?all=toggle'));

lBtns.forEach(btn=>{
  btn.addEventListener('click',()=>act('/servo?target='+btn.dataset.lvl));
});
$('sync').addEventListener('change',e=>act('/servo?sync='+e.target.value));

function setBtn(el,on,onTxt,offTxt){el.classList.toggle('on',!!on);el.textContent=on?onTxt:offTxt;}

async function tick(){
 try{
  const r=await fetch('/frame',{cache:'no-store'});
  const b=new Uint8Array(await r.arrayBuffer());
  const count=b[0]|(b[1]<<8),nb=b[2],t=b[3],mina=b[4]|(b[5]<<8),maxa=b[6]|(b[7]<<8);
  const io=b[8],st=b[9];
  if(!inited){thr.value=t;tv.textContent=t;mn.value=mina;mx.value=maxa;inited=true;}
  else if(!dragging){thr.value=t;tv.textContent=t;}
  $('count').textContent=count;

  // outputs
  setBtn($('laser'),io&1,'Laser: ON','Laser: OFF');
  let anyVib=false;
  for(let i=0;i<4;i++){
    const on=(io>>(i+1))&1; if(on)anyVib=true;
    const el=$('v'+i); el.classList.toggle('on',!!on);
    el.textContent=el.dataset.name+(on?' - ON':'');
  }
  setBtn($('vall'),anyVib,'ALL VIBRATORS OFF','ALL VIBRATORS ON');

  // servo & light pad state
  const curLvl=st&3, busy=(st>>2)&1, avail=(st>>3)&1;
  lBtns.forEach((btn,idx)=>{
    btn.disabled=busy||!avail;
    btn.classList.toggle('on', curLvl===idx);
  });
  $('sync').disabled=busy;
  $('lvl').textContent=!avail?'Servo unavailable':(busy?lvlNames[curLvl]+' (Moving...)':lvlNames[curLvl]);
  if(document.activeElement!==$('sync'))$('sync').value=curLvl;

  // image
  const boxOff=10,bmOff=10+nb*4,d=img.data;   // 10-byte header
  for(let i=0;i<W*H;i++){
    const on=(b[bmOff+(i>>3)]>>(i&7))&1;
    const v=on?40:255;
    d[i*4]=v;d[i*4+1]=v;d[i*4+2]=v;d[i*4+3]=255;
  }
  octx.putImageData(img,0,0);
  ctx.imageSmoothingEnabled=false;
  ctx.drawImage(off,0,0,W*S,H*S);
  ctx.strokeStyle='#ff2d2d';ctx.lineWidth=2;ctx.fillStyle='#ff2d2d';ctx.font='bold 16px sans-serif';
  for(let k=0;k<nb;k++){
    const x0=b[boxOff+k*4],y0=b[boxOff+k*4+1],x1=b[boxOff+k*4+2],y1=b[boxOff+k*4+3];
    ctx.strokeRect(x0*S,y0*S,(x1-x0+1)*S,(y1-y0+1)*S);
    ctx.fillText(k+1,x0*S+2,Math.max(14,y0*S-3));
  }
  $('status').textContent='live - '+new Date().toLocaleTimeString();
 }catch(e){$('status').textContent='connection lost, retrying...';}
 setTimeout(tick,150);
}
tick();
</script></body></html>
)rawliteral";

// ============================ OUTPUT HELPERS ============================
static void setLaser(bool on) {
  digitalWrite(LASER_PIN, on ? HIGH : LOW);
  gIO = on ? (gIO | IO_LASER) : (gIO & ~IO_LASER);
}

static void setVib(int i, bool on) {
  digitalWrite(VIB_PINS[i], on ? HIGH : LOW);
  uint8_t bit = 1 << (i + 1);
  gIO = on ? (gIO | bit) : (gIO & ~bit);
}

// ============================ SERVO (IDF LEDC, low-speed mode) ============================
#define SERVO_LEDC_MODE   LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER  LEDC_TIMER_1
#define SERVO_LEDC_CH     LEDC_CHANNEL_1
#define SERVO_RES_BITS    14                 // 16384 counts per 20 ms period

static uint32_t servoAngleToDuty(int angle) {
  angle = constrain(angle, 0, 180);
  uint32_t us = SERVO_MIN_US + (uint32_t)angle * (SERVO_MAX_US - SERVO_MIN_US) / 180;
  return (us * (1UL << SERVO_RES_BITS)) / 20000UL;
}

static void servoWriteAngle(int angle) {
  ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CH, servoAngleToDuty(angle));
  ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CH);
}

static void servoStop() {                    // stop pulses: no jitter, no holding current
  ledc_stop(SERVO_LEDC_MODE, SERVO_LEDC_CH, 0);
}

// Single physical "button press" action helper:
// Moves from resting angle to pressing angle, holds briefly, and returns to resting angle.
static void pressButtonOnce() {
  servoWriteAngle(SERVO_PUSH_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(SERVO_PUSH_HOLD_MS));
  servoWriteAngle(SERVO_HOME_ANGLE);
  vTaskDelay(pdMS_TO_TICKS(SERVO_SETTLE_MS));
}

static void servoTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // wait for a target level request

    uint8_t target  = targetLightLevel & 3;
    uint8_t current = currentLightLevel & 3;
    uint8_t pressesNeeded = (target - current + 4) % 4;

    if (pressesNeeded > 0) {
      Serial.printf("Servo: transitioning from state %u to %u (%u press%s)\n",
                    current, target, pressesNeeded, pressesNeeded == 1 ? "" : "es");

      for (uint8_t i = 0; i < pressesNeeded; i++) {
        pressButtonOnce();
        // Short delay before the next press if multiple presses are required
        if (i + 1 < pressesNeeded) {
          vTaskDelay(pdMS_TO_TICKS(SERVO_INTER_PRESS_MS));
        }
      }
      servoStop();
      currentLightLevel = target;
      Serial.printf("Servo: complete, currentLightLevel = %u\n", (unsigned)currentLightLevel);
    }

    servoBusy = false;
  }
}

static void servoInit() {
#if SERVO_PIN == 16
  if (psramFound()) {
    Serial.println("Servo DISABLED: GPIO 16 is the PSRAM chip-select and PSRAM is enabled.");
    Serial.println("  -> Arduino IDE: Tools > PSRAM > Disabled, then re-upload (or change SERVO_PIN).");
    return;
  }
#endif
  ledc_timer_config_t t = {};
  t.speed_mode      = SERVO_LEDC_MODE;
  t.duty_resolution = LEDC_TIMER_14_BIT;
  t.timer_num       = SERVO_LEDC_TIMER;
  t.freq_hz         = 50;
  t.clk_cfg         = LEDC_AUTO_CLK;
  if (ledc_timer_config(&t) != ESP_OK) {
    Serial.println("Servo: LEDC timer config failed");
    return;
  }
  ledc_channel_config_t ch = {};
  ch.gpio_num   = SERVO_PIN;
  ch.speed_mode = SERVO_LEDC_MODE;
  ch.channel    = SERVO_LEDC_CH;
  ch.intr_type  = LEDC_INTR_DISABLE;
  ch.timer_sel  = SERVO_LEDC_TIMER;
  ch.duty       = servoAngleToDuty(SERVO_HOME_ANGLE);
  ch.hpoint     = 0;
  if (ledc_channel_config(&ch) != ESP_OK) {
    Serial.println("Servo: LEDC channel config failed");
    return;
  }
  delay(500);                                          // let it reach the home position
  servoStop();
  xTaskCreatePinnedToCore(servoTask, "servo", 3072, nullptr, 2, &servoTaskHandle, 0);
  servoReady = (servoTaskHandle != nullptr);
  Serial.println(servoReady ? "Servo ready on GPIO 16." : "Servo task creation failed.");
}

// ============================ HTTP HANDLERS ============================
static bool queryParam(httpd_req_t* req, const char* key, char* out, size_t outLen) {
  char q[96];
  if (httpd_req_get_url_query_str(req, q, sizeof(q)) != ESP_OK) return false;
  return httpd_query_key_value(q, key, out, outLen) == ESP_OK;
}

// 1 = on, 0 = off, -1 = toggle / unspecified
static int parseState(const char* s) {
  if (!strcmp(s, "1") || !strcmp(s, "on"))  return 1;
  if (!strcmp(s, "0") || !strcmp(s, "off")) return 0;
  return -1;
}

static esp_err_t sendText(httpd_req_t* req, const char* txt) {
  httpd_resp_set_type(req, "text/plain");
  return httpd_resp_send(req, txt, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, PAGE, HTTPD_RESP_USE_STRLEN);
}

// Binary layout:
// [0..1] count (LE)  [2] numBoxes  [3] threshold  [4..5] minArea  [6..7] maxArea
// [8]  outputs: bit0 laser, bit1..4 vibrators 0..3
// [9]  servo:   bits0-1 currentLightLevel (0=Off..3), bit2 busy, bit3 servo available
// [10 .. 10+4*nb)     boxes x0,y0,x1,y1
// [then 2400 bytes]   1-bit bitmap, pixel i -> byte i>>3, bit (i&7), 1 = object
static esp_err_t frameHandler(httpd_req_t* req) {
  size_t len = 0;
  xSemaphoreTake(gMutex, portMAX_DELAY);
  gOut[0] = gCount & 0xFF;
  gOut[1] = gCount >> 8;
  gOut[2] = gNumBoxes;
  gOut[3] = gThreshold;
  gOut[4] = gMinArea & 0xFF;
  gOut[5] = gMinArea >> 8;
  gOut[6] = gMaxArea & 0xFF;
  gOut[7] = gMaxArea >> 8;
  gOut[8] = gIO;
  gOut[9] = (currentLightLevel & 3) | (servoBusy ? 0x04 : 0) | (servoReady ? 0x08 : 0);
  len = HEADER_BYTES;
  for (int i = 0; i < gNumBoxes; i++) {
    gOut[len++] = gBoxes[i].x0;
    gOut[len++] = gBoxes[i].y0;
    gOut[len++] = gBoxes[i].x1;
    gOut[len++] = gBoxes[i].y1;
  }
  memcpy(gOut + len, gPacked, PACKED_BYTES);
  len += PACKED_BYTES;
  xSemaphoreGive(gMutex);

  httpd_resp_set_type(req, "application/octet-stream");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, (const char*)gOut, len);
}

static esp_err_t setHandler(httpd_req_t* req) {
  char val[12];
  xSemaphoreTake(gMutex, portMAX_DELAY);
  if (queryParam(req, "thr", val, sizeof(val)))
    gThreshold = (uint8_t)constrain(atoi(val), 0, 255);
  if (queryParam(req, "min", val, sizeof(val)))
    gMinArea = (uint16_t)constrain(atoi(val), 1, NPIX);
  if (queryParam(req, "max", val, sizeof(val)))
    gMaxArea = (uint16_t)constrain(atoi(val), 1, NPIX);
  xSemaphoreGive(gMutex);
  return sendText(req, "ok");
}

static esp_err_t laserHandler(httpd_req_t* req) {
  char v[8];
  int st = -1;
  if (queryParam(req, "state", v, sizeof(v))) st = parseState(v);
  bool on = (st < 0) ? !(gIO & IO_LASER) : (st == 1);
  setLaser(on);
  Serial.printf("Laser (GPIO %u) %s\n", LASER_PIN, on ? "ON" : "OFF");
  return sendText(req, on ? "laser=1" : "laser=0");
}

static esp_err_t vibHandler(httpd_req_t* req) {
  char v[8];
  if (queryParam(req, "all", v, sizeof(v))) {
    int st = parseState(v);
    bool anyOn = (gIO & IO_VIB_MASK) != 0;
    bool on = (st < 0) ? !anyOn : (st == 1);
    for (int i = 0; i < 4; i++) {
      setVib(i, on);
      if (on && i < 3) vTaskDelay(pdMS_TO_TICKS(VIB_STAGGER_MS));
    }
    Serial.printf("All vibrators %s\n", on ? "ON" : "OFF");
    return sendText(req, on ? "all=1" : "all=0");
  }
  if (queryParam(req, "id", v, sizeof(v))) {
    int id = atoi(v);
    if (id < 0 || id > 3) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "id must be 0..3");
    int st = -1;
    char s[8];
    if (queryParam(req, "state", s, sizeof(s))) st = parseState(s);
    bool cur = (gIO & (1 << (id + 1))) != 0;
    bool on = (st < 0) ? !cur : (st == 1);
    setVib(id, on);
    Serial.printf("Vibrator %s (GPIO %u) %s\n", VIB_NAMES[id], VIB_PINS[id], on ? "ON" : "OFF");
    return sendText(req, on ? "vib=1" : "vib=0");
  }
  return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "use ?id=0..3 or ?all=");
}

static esp_err_t servoHandler(httpd_req_t* req) {
  char v[8];

  // Ignore incoming commands while the servo is actively moving to prevent desynchronization
  if (servoBusy) {
    return sendText(req, "busy");
  }

  // Manual state sync (without moving the servo)
  if (queryParam(req, "sync", v, sizeof(v))) {
    currentLightLevel = (uint8_t)constrain(atoi(v), 0, 3);
    Serial.printf("Light state manually synced to %u\n", (unsigned)currentLightLevel);
    return sendText(req, "synced");
  }

  if (!servoReady) {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return sendText(req, "servo unavailable");
  }

  // Target state control (0 = Off, 1 = Level 1, 2 = Level 2, 3 = Level 3)
  if (queryParam(req, "target", v, sizeof(v))) {
    uint8_t targetLevel = (uint8_t)constrain(atoi(v), 0, 3);
    uint8_t pressesNeeded = (targetLevel - currentLightLevel + 4) % 4;

    if (pressesNeeded == 0) {
      return sendText(req, "already at target");
    }

    targetLightLevel = targetLevel;
    servoBusy = true;                        // cleared by servoTask when all presses finish
    xTaskNotifyGive(servoTaskHandle);
    return sendText(req, "moving");
  }

  return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "use ?target=0..3 or ?sync=0..3");
}

static void startWebServer() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.stack_size = 8192;
  cfg.max_uri_handlers = 10;
  httpd_handle_t srv = nullptr;
  if (httpd_start(&srv, &cfg) != ESP_OK) {
    Serial.println("HTTP server failed to start");
    return;
  }
  httpd_uri_t uIndex = { "/",      HTTP_GET, indexHandler, nullptr };
  httpd_uri_t uFrame = { "/frame", HTTP_GET, frameHandler, nullptr };
  httpd_uri_t uSet   = { "/set",   HTTP_GET, setHandler,   nullptr };
  httpd_uri_t uLaser = { "/laser", HTTP_GET, laserHandler, nullptr };
  httpd_uri_t uVib   = { "/vib",   HTTP_GET, vibHandler,   nullptr };
  httpd_uri_t uServo = { "/servo", HTTP_GET, servoHandler, nullptr };
  httpd_register_uri_handler(srv, &uIndex);
  httpd_register_uri_handler(srv, &uFrame);
  httpd_register_uri_handler(srv, &uSet);
  httpd_register_uri_handler(srv, &uLaser);
  httpd_register_uri_handler(srv, &uVib);
  httpd_register_uri_handler(srv, &uServo);
}

// ============================ CAMERA ============================
static bool initCamera() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_GRAYSCALE;
  c.frame_size   = FRAMESIZE_QQVGA;          // 160x120
  c.jpeg_quality = 12;                       // unused for grayscale

  if (psramFound()) {
    c.fb_count    = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode   = CAMERA_GRAB_LATEST;
  } else {
    c.fb_count    = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;
    c.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_whitebal(s, 0);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
  }
  return true;
}

// ============================ BLOB DETECTION ============================
static void processFrame(const uint8_t* gray, uint8_t thr, uint16_t minA, uint16_t maxA,
                         uint16_t& outCount, uint8_t& outNb, Box* outBoxes, uint8_t* outPacked) {
  memset(outPacked, 0, PACKED_BYTES);
  for (int i = 0; i < NPIX; i++) {
    bin[i] = (gray[i] < thr) ? 1 : 0;          // dark = object
  }

  uint16_t count = 0;
  uint8_t nb = 0;

  for (int start = 0; start < NPIX; start++) {
    if (bin[start] != 1) continue;

    int head = 0, tail = 0;
    queue[tail++] = start;
    bin[start] = 2;

    int area = 0;
    int minX = W, minY = H, maxX = 0, maxY = 0;

    while (head < tail) {
      int idx = queue[head++];
      int x = idx % W;
      int y = idx / W;
      area++;
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;

      for (int dy = -1; dy <= 1; dy++) {
        int ny = y + dy;
        if (ny < 0 || ny >= H) continue;
        for (int dx = -1; dx <= 1; dx++) {
          if (dx == 0 && dy == 0) continue;
          int nx = x + dx;
          if (nx < 0 || nx >= W) continue;
          int n = ny * W + nx;
          if (bin[n] == 1) {
            bin[n] = 2;
            queue[tail++] = n;
          }
        }
      }
    }

    bool touchesEdge = (minX == 0 || minY == 0 || maxX == W - 1 || maxY == H - 1);
    bool valid = (area >= minA) && (area <= maxA) && !(IGNORE_EDGE_BLOBS && touchesEdge);
    if (!valid) continue;

    count++;
    if (nb < MAX_BOXES) {
      outBoxes[nb++] = { (uint8_t)minX, (uint8_t)minY, (uint8_t)maxX, (uint8_t)maxY };
    }
  }

  for (int i = 0; i < NPIX; i++) {
    if (bin[i]) outPacked[i >> 3] |= (1 << (i & 7));
  }

  outCount = count;
  outNb = nb;
}

// ============================ SETUP / LOOP ============================
void setup() {
  // Drive every output LOW first thing (GPIO 12 is a boot-strapping pin and must stay low at boot).
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  for (int i = 0; i < 4; i++) {
    pinMode(VIB_PINS[i], OUTPUT);
    digitalWrite(VIB_PINS[i], LOW);
  }

  Serial.begin(115200);
  Serial.println();
  Serial.println("ESP32-CAM component counter v3 starting...");

  gMutex = xSemaphoreCreateMutex();

  bin   = (uint8_t*)malloc(NPIX);
  queue = (uint16_t*)malloc(NPIX * sizeof(uint16_t));
  if (!bin || !queue) {
    Serial.println("Buffer allocation failed, halting.");
    while (true) delay(1000);
  }

  if (!initCamera()) {
    Serial.println("Camera error, halting. Check the ribbon cable and power (use 5V, >=500mA).");
    while (true) delay(1000);
  }

  servoInit();                                 // after the camera, so XCLK is already running

#if USE_STA_MODE
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("Connecting to Wi-Fi");
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(500);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Open http://");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Wi-Fi failed, falling back to SoftAP.");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("Open http://");
    Serial.println(WiFi.softAPIP());
  }
#else
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.printf("Join Wi-Fi \"%s\" (password %s), then open http://", AP_SSID, AP_PASS);
  Serial.println(WiFi.softAPIP());
#endif

  startWebServer();
  Serial.println("Web server ready.");
}

void loop() {
  static uint16_t lastPrinted = 0xFFFF;
  static uint32_t lastPrintMs = 0;

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Frame capture failed");
    delay(100);
    return;
  }

  if (fb->width != W || fb->height != H || fb->format != PIXFORMAT_GRAYSCALE) {
    Serial.printf("Unexpected frame %ux%u fmt=%d\n", fb->width, fb->height, fb->format);
    esp_camera_fb_return(fb);
    delay(100);
    return;
  }

  xSemaphoreTake(gMutex, portMAX_DELAY);
  uint8_t  thr  = gThreshold;
  uint16_t minA = gMinArea;
  uint16_t maxA = gMaxArea;
  xSemaphoreGive(gMutex);

  static Box     boxes[MAX_BOXES];
  static uint8_t packed[PACKED_BYTES];
  uint16_t count = 0;
  uint8_t nb = 0;

  processFrame(fb->buf, thr, minA, maxA, count, nb, boxes, packed);

  esp_camera_fb_return(fb);
  fb = nullptr;

  xSemaphoreTake(gMutex, portMAX_DELAY);
  gCount = count;
  gNumBoxes = nb;
  memcpy(gBoxes, boxes, nb * sizeof(Box));
  memcpy(gPacked, packed, PACKED_BYTES);
  xSemaphoreGive(gMutex);

  uint32_t now = millis();
  if (count != lastPrinted || now - lastPrintMs >= 1000) {
    Serial.printf("Count: %u  (thr=%u, min=%u, max=%u)\n", count, thr, minA, maxA);
    lastPrinted = count;
    lastPrintMs = now;
  }

  delay(20);
}