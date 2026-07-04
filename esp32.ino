#include <WiFi.h>
#include <esp_wifi.h>           // esp_wifi_set_ps(), esp_wifi_set_bandwidth()
#include <WebServer.h>          // built-in ESP32 HTTP server
#include <WebSocketsServer.h>   // Markus Sattler — install "WebSockets"
#include <NewPing.h>

// ── Motor Pins ───────────────────────────────────────────
#define MOTOR_A_IN1  27
#define MOTOR_A_IN2  26
#define MOTOR_B_IN3  13
#define MOTOR_B_IN4  14
#define MOTOR_A_EN   25
#define MOTOR_B_EN   12

// ── IR Sensor Pins ───────────────────────────────────────
#define IR_LEFT_PIN    32
#define IR_CENTER_PIN  35
#define IR_RIGHT_PIN   34

// ── Ultrasonic Pins ──────────────────────────────────────
#define US_LEFT_TRIG   33
#define US_LEFT_ECHO   15
#define US_CENTER_TRIG 16
#define US_CENTER_ECHO 17
#define US_RIGHT_TRIG  18
#define US_RIGHT_ECHO  19

// ── IR-mode Speed Constants (Increased Turn Speeds by 8) ─
#define BASE_SPEED        110
#define SLOW_INNER_SPEED   48
#define FAST_OUTER_SPEED  128
#define FAST_REV_SPEED     75

// ── Wall-mode Speed Constants ────────────────────────────
#define WALL_BASE_SPEED  130
#define TURN_FWD         80
#define TURN_REV          60

// ── Manual D-Pad Speed Constants ─────────────────────────
#define MANUAL_SPEED        140 // BASE_SPEED + 20
#define MANUAL_TURN_FWD     120 // TURN_FWD + 20
#define MANUAL_TURN_REV      80 // TURN_REV + 20

// ── Thresholds ───────────────────────────────────────────
#define WALL_DETECT      30   // cm — any sensor < this → wall mode
#define WALL_THRESHOLD   17   // cm — too close, must steer
#define MAX_DISTANCE    100   // limits US sensor blocking time

// ── Auto-mode Timing ─────────────────────────────────────
#define WALL_CHECK_INTERVAL   500
#define WALL_CLEAR_TIMEOUT   1000

// ── Sensor Broadcast Timing ───────────────────────────────
#define BROADCAST_INTERVAL   150   // ms — web task broadcasts sensors

// ── WiFi Settings ────────────────────────────────────────
#define WIFI_SSID    "RobotControl"
#define WIFI_PASS    "BMD12345"
#define WIFI_CH       6
#define WIFI_MAX_CON  1

// ════════════════════════════════════════════════════════
//  SENSOR OBJECTS
// ════════════════════════════════════════════════════════

NewPing sonarLeft  (US_LEFT_TRIG,   US_LEFT_ECHO,   MAX_DISTANCE);
NewPing sonarCenter(US_CENTER_TRIG, US_CENTER_ECHO, MAX_DISTANCE);
NewPing sonarRight (US_RIGHT_TRIG,  US_RIGHT_ECHO,  MAX_DISTANCE);

// ════════════════════════════════════════════════════════
//  WEB SERVER + WEBSOCKET  (owned by Core 0 web task)
// ════════════════════════════════════════════════════════

WebServer        httpServer(80);
WebSocketsServer wsServer(81);

// ════════════════════════════════════════════════════════
//  SHARED STATE  — accessed from both cores
//  Protected by a portMUX spinlock (safe in task context)
// ════════════════════════════════════════════════════════

portMUX_TYPE sharedMux = portMUX_INITIALIZER_UNLOCKED;

// ── Robot / auto mode ────────────────────────────────────
enum RobotMode   { LINE_FOLLOW, WALL_FOLLOW };
enum ControlMode { AUTO_MODE,   MANUAL_MODE };

volatile RobotMode   currentMode = LINE_FOLLOW;   // written Core 1
volatile ControlMode controlMode = AUTO_MODE;      // written Core 0 WS handler

// ── Manual command (written Core 0, read Core 1) ─────────
struct ManualCmd {
  bool  fwd, bwd, left, right;
  float jsX, jsY;
  bool  joystickActive;
};
ManualCmd manCmd = {};   // protected by sharedMux

// ── Sensor cache (written Core 1, read Core 0) ───────────
struct SensorCache {
  int  usL, usC, usR;
  bool irL, irC, irR;
  bool isWall;           // true = currentMode is WALL_FOLLOW
};
SensorCache sensorCache = { MAX_DISTANCE, MAX_DISTANCE, MAX_DISTANCE,
                             false, false, false, false };

// ════════════════════════════════════════════════════════
//  PROGMEM HTML PAGE  (no CDN, fully self-contained)
//  WebSocket on port 81: ws://192.168.4.1:81
// ════════════════════════════════════════════════════════

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Robot Control</title>
<style>
*{margin:0;padding:0;box-sizing:border-box;-webkit-tap-highlight-color:transparent}
:root{
  --bg:#060c1a;--glass:rgba(255,255,255,.04);--glass2:rgba(0,0,0,.35);
  --border:rgba(255,255,255,.09);--cyan:#00e5ff;--green:#00ff88;
  --amber:#ffb300;--red:#ff4466;--purple:#7b8cff;--text:#dce8ff;--dim:#4a5a7a;--r:18px;
}
html,body{height:100%;background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,sans-serif;overflow-x:hidden}
body{background:radial-gradient(ellipse 100% 50% at 50% -10%,rgba(0,80,160,.25) 0%,transparent 70%),radial-gradient(ellipse 60% 40% at 80% 80%,rgba(0,20,60,.4) 0%,transparent 70%),#060c1a;min-height:100vh;padding:10px}
.card{background:var(--glass);border:1px solid var(--border);border-radius:var(--r);padding:16px;margin-bottom:12px;backdrop-filter:blur(14px);-webkit-backdrop-filter:blur(14px);transition:opacity .35s}
.card.hidden{opacity:0;pointer-events:none;height:0!important;padding:0!important;margin:0!important;overflow:hidden;border:none}
.card-title{font-size:.6rem;font-weight:700;letter-spacing:.15em;text-transform:uppercase;color:var(--dim);margin-bottom:14px}
.header{display:flex;align-items:center;justify-content:space-between;padding:12px 16px;margin-bottom:12px;background:var(--glass);border:1px solid var(--border);border-radius:var(--r);backdrop-filter:blur(14px)}
.logo{font-size:1rem;font-weight:800;letter-spacing:.05em;background:linear-gradient(90deg,var(--cyan),var(--purple));-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.header-right{display:flex;align-items:center;gap:14px}
.ws-dot{width:9px;height:9px;border-radius:50%;background:#1a2230;border:1px solid #2a3a50;transition:all .4s}
.ws-dot.on{background:var(--green);border-color:var(--green);box-shadow:0 0 6px var(--green),0 0 14px rgba(0,255,136,.35)}
.ws-dot.err{background:var(--red);border-color:var(--red);box-shadow:0 0 6px var(--red)}
.tog-row{display:flex;align-items:center;gap:8px}
.tog-lbl{font-size:.65rem;font-weight:700;letter-spacing:.08em;color:var(--dim);transition:color .3s;user-select:none}
.tog-lbl.on{color:var(--text)}
.tog{position:relative;width:52px;height:26px;cursor:pointer;flex-shrink:0}
.tog input{opacity:0;width:0;height:0;position:absolute}
.tog-track{position:absolute;inset:0;border-radius:13px;background:#0a1428;border:1px solid var(--border)}
.tog-knob{position:absolute;top:3px;left:3px;width:20px;height:20px;border-radius:50%;background:var(--cyan);transition:transform .3s cubic-bezier(.4,0,.2,1);box-shadow:0 0 8px var(--cyan)}
.tog input:checked~.tog-knob{transform:translateX(26px);background:var(--purple);box-shadow:0 0 8px var(--purple)}
.rbadge{display:inline-flex;align-items:center;gap:6px;padding:4px 11px;border-radius:30px;font-size:.62rem;font-weight:700;letter-spacing:.1em;border:1px solid;transition:all .4s;margin-bottom:14px}
.rbadge.line{color:var(--cyan);border-color:rgba(0,229,255,.4);background:rgba(0,229,255,.07);box-shadow:0 0 12px rgba(0,229,255,.12)}
.rbadge.wall{color:var(--amber);border-color:rgba(255,179,0,.4);background:rgba(255,179,0,.07);box-shadow:0 0 12px rgba(255,179,0,.12)}
.bdot{width:6px;height:6px;border-radius:50%;background:currentColor}
.sec-lbl{font-size:.58rem;font-weight:700;letter-spacing:.12em;text-transform:uppercase;color:var(--dim);margin-bottom:10px}
.ir-row{display:flex;justify-content:center;gap:28px;margin-bottom:18px}
.ir-item{display:flex;flex-direction:column;align-items:center;gap:7px}
.ir-led{width:38px;height:38px;border-radius:50%;background:#080f1e;border:2px solid #141f35;position:relative;transition:all .2s}
.ir-led::after{content:'';position:absolute;inset:5px;border-radius:50%;background:radial-gradient(circle at 35% 30%,#111f3a,#060d1e);transition:all .2s}
.ir-led.on{background:#001810;border-color:var(--green);box-shadow:0 0 8px var(--green),0 0 20px rgba(0,255,136,.25)}
.ir-led.on::after{background:radial-gradient(circle at 35% 30%,var(--green),#009950)}
.ir-lbl{font-size:.58rem;font-weight:700;letter-spacing:.12em;color:var(--dim);text-transform:uppercase}
.us-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.us-cell{background:var(--glass2);border-radius:13px;border:1px solid var(--border);padding:11px 8px;text-align:center}
.us-val{font-size:1.35rem;font-weight:800;letter-spacing:-.02em;line-height:1;background:linear-gradient(170deg,#fff,var(--cyan));-webkit-background-clip:text;-webkit-text-fill-color:transparent;transition:all .25s}
.us-val.warn{background:linear-gradient(170deg,#fff,var(--amber));-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.us-val.alrt{background:linear-gradient(170deg,#fff,var(--red));-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.us-unit{font-size:.55rem;color:var(--dim);margin-top:3px}
.us-track{height:3px;background:rgba(255,255,255,.06);border-radius:2px;margin-top:8px}
.us-fill{height:100%;border-radius:2px;background:var(--cyan);transition:width .15s linear}
.us-fill.warn{background:var(--amber)}.us-fill.alrt{background:var(--red)}
.us-name{font-size:.55rem;color:var(--dim);margin-top:5px;font-weight:700;letter-spacing:.12em;text-transform:uppercase}

.man-grid{display:grid;grid-template-columns:1fr 1fr;gap:14px;align-items:start;padding:10px 0}
.ctrl-title{font-size:.6rem;font-weight:700;letter-spacing:.12em;text-transform:uppercase;color:var(--dim)}

/* Larger Joystick Styles */
.joy-wrap{display:flex;flex-direction:column;align-items:center;gap:8px}
.joystick{width:200px;height:200px;border-radius:50%;background:radial-gradient(circle at 50% 50%,#0d1f40,#060d1f);border:2px solid rgba(0,229,255,.2);position:relative;cursor:crosshair;box-shadow:inset 0 0 30px rgba(0,229,255,.04),0 0 16px rgba(0,229,255,.06);touch-action:none;user-select:none}
.joy-ring{position:absolute;inset:16px;border-radius:50%;border:1px dashed rgba(0,229,255,.12);pointer-events:none}
.joy-ch{position:absolute;background:rgba(0,229,255,.07);pointer-events:none}
.joy-ch.h{top:50%;left:8%;width:84%;height:1px;transform:translateY(-50%)}
.joy-ch.v{left:50%;top:8%;height:84%;width:1px;transform:translateX(-50%)}
.joy-knob{position:absolute;top:50%;left:50%;width:56px;height:56px;border-radius:50%;transform:translate(-50%,-50%);background:radial-gradient(circle at 35% 30%,#60d8ff,var(--cyan));box-shadow:0 0 12px var(--cyan),0 2px 8px rgba(0,0,0,.4);pointer-events:none;transition:box-shadow .15s}
.joystick.drag .joy-knob{box-shadow:0 0 26px var(--cyan),0 0 50px rgba(0,229,255,.28),0 2px 8px rgba(0,0,0,.4)}
.spd-row{display:flex;align-items:center;gap:6px;font-size:.65rem;color:var(--dim);width:200px}
.spd-track{flex:1;height:4px;background:rgba(255,255,255,.07);border-radius:2px}
.spd-fill{height:100%;border-radius:2px;background:var(--cyan);width:0;transition:width .08s}
.joy-sub{font-size:.6rem;color:var(--dim);text-align:center}

/* D-Pad Styles */
.dp-wrap{display:flex;flex-direction:column;align-items:center;gap:8px}
.dpad{display:grid;grid-template-columns:52px 52px 52px;grid-template-rows:52px 52px 52px;gap:4px}
.dpb{border-radius:11px;background:rgba(255,255,255,.05);border:1px solid var(--border);cursor:pointer;display:flex;align-items:center;justify-content:center;font-size:1.25rem;color:var(--text);touch-action:none;user-select:none;transition:all .1s;-webkit-user-select:none}
.dpb:active,.dpb.pr{background:rgba(0,229,255,.14);border-color:rgba(0,229,255,.5);box-shadow:0 0 10px rgba(0,229,255,.18);color:var(--cyan);transform:scale(.94)}
.dp-blank{background:transparent!important;border:none!important;pointer-events:none}
.dp-stop-lbl{font-size:.62rem;font-weight:800;letter-spacing:.06em}
.dp-sub{font-size:.55rem;color:var(--dim);text-align:center;margin-top:4px}
</style>
</head>
<body>

<div class="header">
  <span class="logo">🤖 ROBOT UI</span>
  <div class="header-right">
    <div class="ws-dot" id="wsDot"></div>
    <div class="tog-row">
      <span class="tog-lbl on" id="lblA">AUTO</span>
      <label class="tog">
        <input type="checkbox" id="modeTog" onchange="onToggle()">
        <div class="tog-track"></div>
        <div class="tog-knob"></div>
      </label>
      <span class="tog-lbl" id="lblM">MANUAL</span>
    </div>
  </div>
</div>

<div class="card" id="autoPanel">
  <div class="card-title">Autonomous Sensor Feed</div>
  <div class="rbadge line" id="rbadge"><div class="bdot"></div><span id="rbadgeTxt">LINE FOLLOW</span></div>
  <div class="sec-lbl">IR Line Sensors</div>
  <div class="ir-row">
    <div class="ir-item"><div class="ir-led" id="irL"></div><div class="ir-lbl">Left</div></div>
    <div class="ir-item"><div class="ir-led" id="irC"></div><div class="ir-lbl">Center</div></div>
    <div class="ir-item"><div class="ir-led" id="irR"></div><div class="ir-lbl">Right</div></div>
  </div>
  <div class="sec-lbl">Ultrasonic Distance</div>
  <div class="us-grid">
    <div class="us-cell"><div class="us-val" id="uslv">—</div><div class="us-unit">cm</div><div class="us-track"><div class="us-fill" id="uslb"></div></div><div class="us-name">Left</div></div>
    <div class="us-cell"><div class="us-val" id="uscv">—</div><div class="us-unit">cm</div><div class="us-track"><div class="us-fill" id="uscb"></div></div><div class="us-name">Center</div></div>
    <div class="us-cell"><div class="us-val" id="usrv">—</div><div class="us-unit">cm</div><div class="us-track"><div class="us-fill" id="usrb"></div></div><div class="us-name">Right</div></div>
  </div>
</div>

<div class="card hidden" id="manPanel">
  <div class="card-title">Manual Control</div>
  <div class="man-grid">
    
    <div class="joy-wrap">
      <div class="ctrl-title">Joystick</div>
      <div class="joystick" id="joy">
        <div class="joy-ring"></div><div class="joy-ch h"></div><div class="joy-ch v"></div>
        <div class="joy-knob" id="joyKnob"></div>
      </div>
      <div class="joy-sub">Smooth &amp; Analog</div>
      <div class="spd-row"><span>SPD</span><div class="spd-track"><div class="spd-fill" id="spdFill"></div></div><span id="spdPct">0%</span></div>
    </div>

    <div class="dp-wrap">
      <div class="ctrl-title">D-Pad</div>
      <div class="dpad">
        <div class="dpb dp-blank"></div>
        <div class="dpb" id="dpU" data-dir="fwd">▲</div>
        <div class="dpb dp-blank"></div>
        <div class="dpb" id="dpL" data-dir="left">◄</div>
        <div class="dpb dp-stop-lbl" id="dpS" data-dir="stop">■</div>
        <div class="dpb" id="dpR" data-dir="right">►</div>
        <div class="dpb dp-blank"></div>
        <div class="dpb" id="dpD" data-dir="bwd">▼</div>
        <div class="dpb dp-blank"></div>
      </div>
      <div class="dp-sub">Fast &amp; Precise</div>
    </div>

  </div>
</div>

<script>
var ws, reconn, missedPings = 0;
function connect() {
  ws = new WebSocket('ws://' + location.hostname + ':81/');
  ws.onopen = function() {
    missedPings = 0;
    dot('on');
    clearTimeout(reconn);
    send(document.getElementById('modeTog').checked ? 'manual' : 'auto');
  };
  ws.onclose = function() { dot(''); reconn = setTimeout(connect, 1500); };
  ws.onerror = function() { dot('err'); ws.close(); };
  ws.onmessage = function(e) { try { update(JSON.parse(e.data)); } catch(x){} };
}
function dot(cls) {
  var d = document.getElementById('wsDot');
  d.className = 'ws-dot' + (cls ? ' '+cls : '');
}
function send(cmd, extra) {
  if (!ws || ws.readyState !== 1) return;
  var msg = {cmd:cmd};
  if (extra) { msg.x=extra.x; msg.y=extra.y; msg.dir=extra.dir; }
  ws.send(JSON.stringify(msg));
}
connect();

function update(d) {
  led('irL',d.irL); led('irC',d.irC); led('irR',d.irR);
  usDist('l',d.usL); usDist('c',d.usC); usDist('r',d.usR);
  var b=document.getElementById('rbadge'), t=document.getElementById('rbadgeTxt');
  if(d.robot==='WALL'){b.className='rbadge wall';t.textContent='WALL FOLLOW';}
  else{b.className='rbadge line';t.textContent='LINE FOLLOW';}
}
function led(id,val){var e=document.getElementById(id);val?e.classList.add('on'):e.classList.remove('on');}
function usDist(side,raw){
  var v=parseInt(raw)||0;
  var ve=document.getElementById('us'+side+'v'), be=document.getElementById('us'+side+'b');
  var pct=Math.max(0,Math.min(100,(1-v/100)*100));
  ve.textContent=v>=100?'100+':v;
  be.style.width=pct+'%';
  var c=v<15?'alrt':v<30?'warn':'';
  ve.className='us-val'+(c?' '+c:''); be.className='us-fill'+(c?' '+c:'');
}
function onToggle(){
  var m=document.getElementById('modeTog').checked;
  document.getElementById('lblA').classList.toggle('on',!m);
  document.getElementById('lblM').classList.toggle('on',m);
  document.getElementById('autoPanel').classList.toggle('hidden',m);
  document.getElementById('manPanel').classList.toggle('hidden',!m);
  send(m?'manual':'auto');
  if(!m) { resetJoy(); send('dpad',{dir:'stop'}); }
}

// Joystick Logic
var joy=document.getElementById('joy'),joyKnob=document.getElementById('joyKnob');
var spdFill=document.getElementById('spdFill'),spdPct=document.getElementById('spdPct');
var joyOn=false, MAX_R=72; // Increased max travel radius for the larger joystick
joy.addEventListener('pointerdown',function(e){joyOn=true;joy.classList.add('drag');joy.setPointerCapture(e.pointerId);moveJoy(e);});
joy.addEventListener('pointermove',function(e){if(joyOn)moveJoy(e);});
joy.addEventListener('pointerup',resetJoy);
joy.addEventListener('pointercancel',resetJoy);
function moveJoy(e){
  var r=joy.getBoundingClientRect(),cx=r.left+r.width/2,cy=r.top+r.height/2;
  var dx=e.clientX-cx,dy=e.clientY-cy,d=Math.sqrt(dx*dx+dy*dy);
  if(d>MAX_R){dx=dx/d*MAX_R;dy=dy/d*MAX_R;}
  joyKnob.style.transform='translate(calc(-50% + '+dx+'px),calc(-50% + '+dy+'px))';
  var nx=+(dx/MAX_R).toFixed(2),ny=+(-dy/MAX_R).toFixed(2);
  var spd=Math.min(100,Math.round(Math.sqrt(nx*nx+ny*ny)*100));
  spdFill.style.width=spd+'%';spdPct.textContent=spd+'%';
  send('joystick',{x:nx,y:ny});
}
function resetJoy(){
  joyOn=false;joy.classList.remove('drag');
  joyKnob.style.transform='translate(-50%,-50%)';
  spdFill.style.width='0%';spdPct.textContent='0%';
  send('joystick',{x:0,y:0});
}

// D-Pad Logic
document.querySelectorAll('.dpb:not(.dp-blank)').forEach(function(btn){
  btn.addEventListener('pointerdown',function(e){e.preventDefault();btn.classList.add('pr');btn.setPointerCapture(e.pointerId);send('dpad',{dir:btn.dataset.dir});});
  btn.addEventListener('pointerup',function(){btn.classList.remove('pr');if(btn.dataset.dir!=='stop')send('dpad',{dir:'stop'});});
  btn.addEventListener('pointercancel',function(){btn.classList.remove('pr');send('dpad',{dir:'stop'});});
  btn.addEventListener('contextmenu',function(e){e.preventDefault();});
});
</script>
</body>
</html>
)rawliteral";

// ════════════════════════════════════════════════════════
//  MOTOR FUNCTIONS  (unchanged from original sketch)
// ════════════════════════════════════════════════════════

void stopMotors() {
  digitalWrite(MOTOR_A_IN1, LOW); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, 0);    analogWrite(MOTOR_B_EN, 0);
}
void stop() { stopMotors(); }

void forward(int spd = BASE_SPEED) {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, spd); analogWrite(MOTOR_B_EN, spd);
}
void backward() {
  digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, LOW);  digitalWrite(MOTOR_B_IN4, HIGH);
  analogWrite(MOTOR_A_EN, (int)(200 ));
  analogWrite(MOTOR_B_EN, (int)(BASE_SPEED / 1.2));
}
void slowLeft() {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, FAST_OUTER_SPEED);
  analogWrite(MOTOR_B_EN, SLOW_INNER_SPEED);
}
void slowRight() {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, SLOW_INNER_SPEED);
  analogWrite(MOTOR_B_EN, FAST_OUTER_SPEED);
}
void fastLeft() {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);  digitalWrite(MOTOR_B_IN4, HIGH);
  analogWrite(MOTOR_A_EN, FAST_OUTER_SPEED);
  analogWrite(MOTOR_B_EN, FAST_REV_SPEED);
}
void fastRight() {
  digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, FAST_REV_SPEED);
  analogWrite(MOTOR_B_EN, FAST_OUTER_SPEED);
}
void wallForward() {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, WALL_BASE_SPEED);
  analogWrite(MOTOR_B_EN, WALL_BASE_SPEED);
}
void wallBackward() {
  digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, LOW);  digitalWrite(MOTOR_B_IN4, HIGH);
  analogWrite(MOTOR_A_EN, 110);
  analogWrite(MOTOR_B_EN, WALL_BASE_SPEED);
}
void turnRight() {
  digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, HIGH);
  digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  analogWrite(MOTOR_A_EN, TURN_REV);
  analogWrite(MOTOR_B_EN, TURN_FWD);
}
void turnLeft() {
  digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
  digitalWrite(MOTOR_B_IN3, LOW);  digitalWrite(MOTOR_B_IN4, HIGH);
  analogWrite(MOTOR_A_EN, TURN_FWD);
  analogWrite(MOTOR_B_EN, TURN_REV);
}

// ════════════════════════════════════════════════════════
//  SENSOR READS  (Core 1 only)
// ════════════════════════════════════════════════════════

int readLeft()   { int d = sonarLeft.ping_cm();   return d == 0 ? MAX_DISTANCE : d; }
int readCenter() { int d = sonarCenter.ping_cm(); return d == 0 ? MAX_DISTANCE : d; }
int readRight()  { int d = sonarRight.ping_cm();  return d == 0 ? MAX_DISTANCE : d; }

// ════════════════════════════════════════════════════════
//  MANUAL DRIVE  (Core 1 only — reads manCmd under lock)
// ════════════════════════════════════════════════════════

void applyJoystick(float x, float y, bool active) {
  if (!active) { stopMotors(); return; }
  int base     = (int)(fabsf(y) * 255.0f);
  int leftSpd  = constrain((int)(base * (1.0f + x)), 0, 255);
  int rightSpd = constrain((int)(base * (1.0f - x)), 0, 255);
  
  if (y >= 0.0f) {
    digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN3, LOW);  digitalWrite(MOTOR_B_IN4, HIGH);
  }
  analogWrite(MOTOR_A_EN, leftSpd);
  analogWrite(MOTOR_B_EN, rightSpd);
}

void applyDpad(bool fwd, bool bwd, bool left, bool right) {
  if (fwd) {
    digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
    analogWrite(MOTOR_A_EN, MANUAL_SPEED);
    analogWrite(MOTOR_B_EN, MANUAL_SPEED);
  } else if (bwd) {
    digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN3, LOW);  digitalWrite(MOTOR_B_IN4, HIGH);
    analogWrite(MOTOR_A_EN, MANUAL_SPEED);
    analogWrite(MOTOR_B_EN, MANUAL_SPEED);
  } else if (left) {
    digitalWrite(MOTOR_A_IN1, HIGH); digitalWrite(MOTOR_A_IN2, LOW);
    digitalWrite(MOTOR_B_IN3, LOW);  digitalWrite(MOTOR_B_IN4, HIGH);
    analogWrite(MOTOR_A_EN, MANUAL_TURN_FWD);
    analogWrite(MOTOR_B_EN, MANUAL_TURN_REV);
  } else if (right) {
    digitalWrite(MOTOR_A_IN1, LOW);  digitalWrite(MOTOR_A_IN2, HIGH);
    digitalWrite(MOTOR_B_IN3, HIGH); digitalWrite(MOTOR_B_IN4, LOW);
    analogWrite(MOTOR_A_EN, MANUAL_TURN_REV);
    analogWrite(MOTOR_B_EN, MANUAL_TURN_FWD);
  } else {
    stopMotors();
  }
}

// ════════════════════════════════════════════════════════
//  WEBSOCKET EVENT HANDLER  (runs on Core 0 web task)
// ════════════════════════════════════════════════════════

void wsEventHandler(uint8_t clientId, WStype_t type,
                    uint8_t *payload, size_t length) {
  switch (type) {

    case WStype_CONNECTED:
      Serial.printf(">> WS client %u connected\n", clientId);
      break;

    case WStype_DISCONNECTED:
      Serial.printf(">> WS client %u disconnected\n", clientId);
      // Safety stop if the browser drops in manual mode
      portENTER_CRITICAL(&sharedMux);
      if (controlMode == MANUAL_MODE) {
        memset(&manCmd, 0, sizeof(manCmd));
      }
      portEXIT_CRITICAL(&sharedMux);
      stopMotors();
      break;

    case WStype_TEXT: {
      char buf[160];
      size_t n = (length < sizeof(buf)-1) ? length : sizeof(buf)-1;
      memcpy(buf, payload, n);
      buf[n] = '\0';
      String msg(buf);

      if (msg.indexOf("\"auto\"") > 0) {
        portENTER_CRITICAL(&sharedMux);
        controlMode = AUTO_MODE;
        memset(&manCmd, 0, sizeof(manCmd));
        portEXIT_CRITICAL(&sharedMux);
        stopMotors();
        Serial.println(">> WEB -> AUTO");

      } else if (msg.indexOf("\"manual\"") > 0) {
        portENTER_CRITICAL(&sharedMux);
        controlMode = MANUAL_MODE;
        memset(&manCmd, 0, sizeof(manCmd));
        portEXIT_CRITICAL(&sharedMux);
        stopMotors();
        Serial.println(">> WEB -> MANUAL");

      } else if (msg.indexOf("\"joystick\"") > 0) {
        int xi = msg.indexOf("\"x\":") + 4;
        int yi = msg.indexOf("\"y\":") + 4;
        if (xi >= 4 && yi >= 4) {
          float nx = msg.substring(xi).toFloat();
          float ny = msg.substring(yi).toFloat();
          portENTER_CRITICAL(&sharedMux);
          manCmd.jsX = nx; manCmd.jsY = ny;
          manCmd.joystickActive = (fabsf(nx) > 0.03f || fabsf(ny) > 0.03f);
          if (manCmd.joystickActive)
            manCmd.fwd = manCmd.bwd = manCmd.left = manCmd.right = false;
          portEXIT_CRITICAL(&sharedMux);
        }

      } else if (msg.indexOf("\"dpad\"") > 0) {
        portENTER_CRITICAL(&sharedMux);
        manCmd.joystickActive = false;
        manCmd.jsX = 0; manCmd.jsY = 0;
        manCmd.fwd   = (msg.indexOf("\"fwd\"")   > 0);
        manCmd.bwd   = (msg.indexOf("\"bwd\"")   > 0);
        manCmd.left  = (msg.indexOf("\"left\"")  > 0);
        manCmd.right = (msg.indexOf("\"right\"") > 0);
        portEXIT_CRITICAL(&sharedMux);
      }
      break;
    }

    default: break;
  }
}

// ════════════════════════════════════════════════════════
//  WEB SERVER TASK  — pinned to Core 0 (WiFi core)
//  Runs entirely independently of the robot loop on Core 1.
//  Calls wsServer.loop() every ~1 ms → WebSocket heartbeat
//  is always serviced, regardless of sonar blocking on Core 1.
// ════════════════════════════════════════════════════════

unsigned long lastBroadcast = 0;

void webTask(void *param) {
  for (;;) {
    httpServer.handleClient();
    wsServer.loop();

    // Broadcast sensor data at BROADCAST_INTERVAL ms
    unsigned long now = millis();
    if (now - lastBroadcast >= BROADCAST_INTERVAL) {
      lastBroadcast = now;

      // Read sensor cache under lock (written by Core 1)
      SensorCache sc;
      portENTER_CRITICAL(&sharedMux);
      sc = sensorCache;
      portEXIT_CRITICAL(&sharedMux);

      char json[180];
      snprintf(json, sizeof(json),
        "{\"robot\":\"%s\","
        "\"irL\":%d,\"irC\":%d,\"irR\":%d,"
        "\"usL\":%d,\"usC\":%d,\"usR\":%d}",
        sc.isWall ? "WALL" : "LINE",
        sc.irL?1:0, sc.irC?1:0, sc.irR?1:0,
        sc.usL, sc.usC, sc.usR
      );
      wsServer.broadcastTXT(json);
    }

    vTaskDelay(1); // yield 1 ms — keeps WDT happy
  }
}

// ════════════════════════════════════════════════════════
//  SETUP
// ════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(MOTOR_A_IN1, OUTPUT); pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN3, OUTPUT); pinMode(MOTOR_B_IN4, OUTPUT);
  pinMode(MOTOR_A_EN,  OUTPUT); pinMode(MOTOR_B_EN,  OUTPUT);

  // IR sensor pins
  pinMode(IR_LEFT_PIN,   INPUT);
  pinMode(IR_CENTER_PIN, INPUT);
  pinMode(IR_RIGHT_PIN,  INPUT);

  stopMotors();

  // ── WiFi AP ───────────────────────────────────────────
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, WIFI_CH, 0, WIFI_MAX_CON);

  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // low power, less interference
  WiFi.setSleep(false);                  // disable WiFi modem sleep
  esp_wifi_set_ps(WIFI_PS_NONE);         // belt-and-suspenders: no power save
  esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20); // 20 MHz, less noise

  Serial.print(">> AP  : "); Serial.println(WIFI_SSID);
  Serial.print(">> IP  : "); Serial.println(WiFi.softAPIP());
  Serial.printf(">> Ch  : %d  |  TX: 8.5 dBm  |  BW: HT20\n", WIFI_CH);

  // ── HTTP + WebSocket servers ──────────────────────────
  httpServer.on("/", HTTP_GET, []() {
    httpServer.send_P(200, "text/html", INDEX_HTML);
  });
  httpServer.begin();

  wsServer.begin();
  wsServer.onEvent(wsEventHandler);

  Serial.println(">> Web UI : http://192.168.4.1");
  Serial.println(">> WS     : ws://192.168.4.1:81");

  // ── Launch web task on Core 0 ─────────────────────────
  // Stack 8 KB is sufficient for HTTP + WS + JSON formatting.
  // Priority 1 = just above idle; robot loop runs at priority 1 too
  // but on Core 1, so they never contend.
  xTaskCreatePinnedToCore(
    webTask,     // task function
    "WebTask",   // name (for debugging)
    8192,        // stack size in bytes
    NULL,        // parameter
    1,           // priority
    NULL,        // task handle (not needed)
    0            // Core 0 — the WiFi core
  );

  delay(1000);
  Serial.println(">> Startup — LINE MODE  (robot loop on Core 1)");
}

// ════════════════════════════════════════════════════════
//  MAIN LOOP  — Core 1 (robot logic only)
//  Never calls httpServer or wsServer directly.
// ════════════════════════════════════════════════════════

// Auto-mode timing (Core 1 private — no sharing needed)
unsigned long lastWallCheck      = 0;
unsigned long wallClearSince     = 0;
bool          wallWasClear       = false;
unsigned long wallActionStart    = 0;
unsigned long wallActionDuration = 0;

void loop() {
  unsigned long now = millis();

  // ── US poll: mode-switch decision ────────────────────
  if (now - lastWallCheck >= WALL_CHECK_INTERVAL) {
    lastWallCheck = now;

    int dL = readLeft();
    int dC = readCenter();
    int dR = readRight();

    // Read IR while we're here — update cache together
    bool irL = (digitalRead(IR_LEFT_PIN)   == LOW);
    bool irC = (digitalRead(IR_CENTER_PIN) == LOW);
    bool irR = (digitalRead(IR_RIGHT_PIN)  == LOW);

    bool wallNear = (dL < WALL_DETECT || dC < WALL_DETECT || dR < WALL_DETECT);

    // Read controlMode safely
    ControlMode cm;
    portENTER_CRITICAL(&sharedMux);
    cm = controlMode;
    portEXIT_CRITICAL(&sharedMux);

    if (cm == AUTO_MODE) {
      if (wallNear) {
        wallWasClear = false; wallClearSince = 0;
        if (currentMode != WALL_FOLLOW) {
          currentMode = WALL_FOLLOW;
          stopMotors();
          wallActionStart = 0; wallActionDuration = 0;
          Serial.println(">> WALL MODE");
        }
      } else {
        if (currentMode == WALL_FOLLOW) {
          if (!wallWasClear) { wallWasClear = true; wallClearSince = now; }
          else if (now - wallClearSince >= WALL_CLEAR_TIMEOUT) {
            currentMode  = LINE_FOLLOW;
            wallWasClear = false;
            stopMotors();
            Serial.println(">> LINE MODE");
          }
        }
      }
    }

    // Update sensor cache for web task to broadcast
    portENTER_CRITICAL(&sharedMux);
    sensorCache.usL    = dL;  sensorCache.usC = dC;  sensorCache.usR = dR;
    sensorCache.irL    = irL; sensorCache.irC = irC; sensorCache.irR = irR;
    sensorCache.isWall = (currentMode == WALL_FOLLOW);
    portEXIT_CRITICAL(&sharedMux);
  }

  // ── Get current control mode safely ──────────────────
  ControlMode cm;
  portENTER_CRITICAL(&sharedMux);
  cm = controlMode;
  portEXIT_CRITICAL(&sharedMux);

  // ════════════════════════════════════════════════════
  //  AUTO MODE
  // ════════════════════════════════════════════════════
  if (cm == AUTO_MODE) {

    // ─ LINE FOLLOW ─────────────────────────────────────
    if (currentMode == LINE_FOLLOW) {
      bool L = (digitalRead(IR_LEFT_PIN)   == LOW);
      bool C = (digitalRead(IR_CENTER_PIN) == LOW);
      bool R = (digitalRead(IR_RIGHT_PIN)  == LOW);

      if      ( C && !L && !R) forward();
      else if ( L &&  C && !R) fastLeft();
      else if ( R &&  C && !L) fastRight();
      else if ( L &&  R      ) forward();
      else if ( L && !C && !R) slowLeft();
      else if ( R && !C && !L) slowRight();
      else                       backward();
    }

    // ─ WALL FOLLOW (non-blocking millis) ───────────────
    else {
      if (now - wallActionStart >= wallActionDuration) {
        int dL = readLeft();
        int dC = readCenter();
        int dR = readRight();

        // Keep cache fresh during active wall-follow steering
        portENTER_CRITICAL(&sharedMux);
        sensorCache.usL = dL; sensorCache.usC = dC; sensorCache.usR = dR;
        sensorCache.isWall = true;
        portEXIT_CRITICAL(&sharedMux);

        if (dC < WALL_THRESHOLD) {
          if (dL >= dR) turnLeft(); else turnRight();
          wallActionStart = now; wallActionDuration = 300;
        } else if (dL < WALL_THRESHOLD) {
          turnRight();
          wallActionStart = now; wallActionDuration = 200;
        } else if (dR < WALL_THRESHOLD) {
          turnLeft();
          wallActionStart = now; wallActionDuration = 200;
        } else {
          wallForward();
          wallActionDuration = 0;
        }
      }
    }
  }

  // ════════════════════════════════════════════════════
  //  MANUAL MODE — read command snapshot under lock
  // ════════════════════════════════════════════════════
  else {
    ManualCmd cmd;
    portENTER_CRITICAL(&sharedMux);
    cmd = manCmd;
    portEXIT_CRITICAL(&sharedMux);

    if (cmd.joystickActive) applyJoystick(cmd.jsX, cmd.jsY, true);
    else                    applyDpad(cmd.fwd, cmd.bwd, cmd.left, cmd.right);
  }
}
