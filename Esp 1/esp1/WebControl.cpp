#include "WebControl.h"

#include <WebServer.h>

namespace app {
namespace {

WebServer webServer(80);
CommandExecutor commandExecutor = nullptr;
StatusJsonProvider statusJsonProvider = nullptr;

const char kControlPage[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>ESP32 Drive Control</title>
  <style>
    :root {
      --bg: #f2efe8;
      --panel: #fffaf2;
      --ink: #1f2a30;
      --muted: #66757f;
      --accent: #0e7c86;
      --accent-2: #d96f32;
      --danger: #b93232;
      --border: #d9d0c0;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "Trebuchet MS", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, #f9d9a7 0, transparent 28%),
        radial-gradient(circle at bottom right, #b7dfe0 0, transparent 30%),
        var(--bg);
    }
    header {
      background: var(--ink);
      color: #fff;
      padding: 14px 20px;
      display: flex;
      align-items: center;
      justify-content: space-between;
    }
    header h1 {
      margin: 0;
      font-size: 1.3rem;
      font-family: Georgia, serif;
    }
    #stopBtn {
      padding: 10px 22px;
      border: 0;
      border-radius: 10px;
      background: var(--danger);
      color: #fff;
      font-weight: 700;
      font-size: 1rem;
      cursor: pointer;
    }
    main {
      max-width: 1200px;
      margin: 0 auto;
      padding: 20px;
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 16px;
    }
    .panel {
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 16px;
      box-shadow: 0 10px 30px rgba(31,42,48,0.08);
    }
    h2 {
      margin: 0 0 12px;
      font-family: Georgia, "Times New Roman", serif;
      font-size: 1.05rem;
      letter-spacing: 0.02em;
    }
    .row {
      display: flex;
      gap: 8px;
      align-items: flex-end;
      margin-bottom: 10px;
    }
    .row label {
      flex: 1;
      font-size: 0.82rem;
      color: var(--muted);
    }
    .row input {
      width: 100%;
      margin-top: 4px;
      padding: 8px 10px;
      border-radius: 8px;
      border: 1px solid var(--border);
      font-size: 0.95rem;
      background: #fff;
    }
    .row button, button.full-btn {
      padding: 9px 14px;
      border: 0;
      border-radius: 10px;
      background: var(--accent);
      color: #fff;
      font-weight: 700;
      cursor: pointer;
      white-space: nowrap;
      font-size: 0.9rem;
    }
    button.full-btn {
      width: 100%;
      padding: 11px;
      margin-top: 6px;
    }
    button.alt  { background: var(--accent-2); }
    button.ghost{ background: #33464f; }
    .btn-row {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px;
      margin-top: 6px;
    }
    .btn-row button {
      padding: 10px 6px;
      border: 0;
      border-radius: 10px;
      background: var(--accent);
      color: #fff;
      font-weight: 700;
      cursor: pointer;
      font-size: 0.88rem;
    }
    .btn-row button.alt   { background: var(--accent-2); }
    .btn-row button.ghost { background: #33464f; }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 8px 12px;
    }
    .si span { display: block; color: var(--muted); font-size: 0.78rem; }
    .si strong { font-size: 0.95rem; }
    .wheel-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 12px;
    }
    .wheel-box {
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 10px;
      background: #faf6ee;
    }
    .wheel-box h3 {
      margin: 0 0 8px;
      font-size: 0.88rem;
      color: var(--muted);
    }
    pre {
      margin: 0;
      padding: 12px;
      border-radius: 12px;
      background: #182126;
      color: #d9f0ef;
      min-height: 60px;
      overflow: auto;
      white-space: pre-wrap;
      font-size: 0.82rem;
    }
    .full { grid-column: 1 / -1; }
    .notice { font-size: 0.78rem; color: var(--muted); margin-top: 4px; }
  </style>
</head>
<body>

<header>
  <h1>ESP32 Drive Tuning Dashboard</h1>
  <button id="stopBtn" onclick="sendCommand('STOP')">&#9724; STOP</button>
</header>

<main>

  <!-- A) Motion Commands -->
  <section class="panel">
    <h2>Motion Commands</h2>
    <div class="row">
      <label>Distance (m)<input id="moveDistance" type="number" step="0.01" value="0.30"></label>
      <label>Heading (deg)<input id="moveHeading" type="number" step="1" value="0"></label>
      <button onclick="sendMove()">MOVE</button>
    </div>
    <div class="row">
      <label>Heading (deg)<input id="rotateHeading" type="number" step="1" value="90"></label>
      <button onclick="sendRotate()">ROTATE</button>
    </div>
    <div class="row">
      <label>Raw command<input id="rawCommand" type="text" placeholder="e.g. STATUS"></label>
      <button class="ghost" onclick="sendRaw()">SEND</button>
    </div>
    <p class="notice">STOP is always in the top-right corner. Motion commands are rejected while another move is active.</p>
  </section>

  <!-- F) Live Status -->
  <section class="panel">
    <h2>Live Status <span style="font-size:0.75rem;color:var(--muted)">(300 ms refresh)</span></h2>
    <div class="status-grid">
      <div class="si"><span>Mode</span><strong id="mode">-</strong></div>
      <div class="si"><span>Active</span><strong id="active">-</strong></div>
      <div class="si"><span>Yaw (deg)</span><strong id="yawDeg">-</strong></div>
      <div class="si"><span>Target Yaw</span><strong id="targetYawDeg">-</strong></div>
      <div class="si"><span>Heading Err</span><strong id="headingErrorDeg">-</strong></div>
      <div class="si"><span>Turn RPM</span><strong id="turnCorrectionRPM">-</strong></div>
      <div class="si"><span>Distance (m)</span><strong id="currentDistanceM">-</strong></div>
      <div class="si"><span>Target Dist</span><strong id="targetDistanceM">-</strong></div>
      <div class="si"><span>Dist Error</span><strong id="distanceErrorM">-</strong></div>
      <div class="si"><span>Base RPM</span><strong id="baseForwardRpm">-</strong></div>
      <div class="si"><span>Fault</span><strong id="fault">-</strong></div>
      <div class="si"><span>Kp pos</span><strong id="Kp_pos">-</strong></div>
      <div class="si"><span>Ki pos</span><strong id="Ki_pos">-</strong></div>
      <div class="si"><span>Kp head</span><strong id="Kp_heading_rpm">-</strong></div>
      <div class="si"><span>Ki head</span><strong id="Ki_heading_rpm_per_rad_s">-</strong></div>
      <div class="si"><span>Kp rot</span><strong id="Kp_rotate_rpm">-</strong></div>
      <div class="si"><span>Rot Tol</span><strong id="HEADING_TOLERANCE_DEG">-</strong></div>
    </div>
  </section>

  <!-- B) Position Loop Tuning -->
  <section class="panel">
    <h2>Position Loop Tuning</h2>
    <div class="row">
      <label>PKP (Kp_pos)<input id="pkp" type="number" step="0.1" value="0.8"></label>
      <button onclick="sendValueCommand('PKP','pkp')">SET</button>
    </div>
    <div class="row">
      <label>PKI (Ki_pos)<input id="pki" type="number" step="0.1" value="0"></label>
      <button onclick="sendValueCommand('PKI','pki')">SET</button>
    </div>
  </section>

  <!-- C) Heading Hold During MOVE -->
  <section class="panel">
    <h2>Heading Hold (MOVE)</h2>
    <div class="row">
      <label>HKP<input id="hkp" type="number" step="1" value="60"></label>
      <button onclick="sendValueCommand('HKP','hkp')">SET</button>
    </div>
    <div class="row">
      <label>HKI<input id="hki" type="number" step="0.1" value="2"></label>
      <button onclick="sendValueCommand('HKI','hki')">SET</button>
    </div>
    <div class="btn-row">
      <button onclick="sendCommand('HEADING ON')">HEADING ON</button>
      <button onclick="sendCommand('HEADING OFF')">HEADING OFF</button>
      <button class="ghost" onclick="sendCommand('HINVERT')">HINVERT</button>
    </div>
  </section>

  <!-- D) Rotate Loop Tuning -->
  <section class="panel">
    <h2>Rotate Loop Tuning</h2>
    <div class="row">
      <label>RKP<input id="rkp" type="number" step="1" value="10"></label>
      <button onclick="sendValueCommand('RKP','rkp')">SET</button>
    </div>
    <div class="row">
      <label>RTOL (deg)<input id="rtol" type="number" step="0.5" value="1"></label>
      <button onclick="sendValueCommand('RTOL','rtol')">SET</button>
    </div>
    <div class="btn-row">
      <button class="ghost" onclick="sendCommand('RINVERT')">RINVERT</button>
    </div>
  </section>

  <!-- E) Wheel Velocity PI Tuning -->
  <section class="panel full">
    <h2>Wheel Velocity PI Tuning</h2>
    <div class="wheel-grid">
      <div class="wheel-box">
        <h3>Wheel 0 (R1)</h3>
        <div class="row">
          <label>VKP<input id="vkp0" type="number" step="0.1" value="5.0"></label>
          <button onclick="sendIndexedCommand('VKP',0,'vkp0')">SET</button>
        </div>
        <div class="row">
          <label>VKI<input id="vki0" type="number" step="0.01" value="1.15"></label>
          <button onclick="sendIndexedCommand('VKI',0,'vki0')">SET</button>
        </div>
      </div>
      <div class="wheel-box">
        <h3>Wheel 1 (R2)</h3>
        <div class="row">
          <label>VKP<input id="vkp1" type="number" step="0.1" value="5.5"></label>
          <button onclick="sendIndexedCommand('VKP',1,'vkp1')">SET</button>
        </div>
        <div class="row">
          <label>VKI<input id="vki1" type="number" step="0.01" value="1.15"></label>
          <button onclick="sendIndexedCommand('VKI',1,'vki1')">SET</button>
        </div>
      </div>
      <div class="wheel-box">
        <h3>Wheel 2 (F1)</h3>
        <div class="row">
          <label>VKP<input id="vkp2" type="number" step="0.1" value="4.5"></label>
          <button onclick="sendIndexedCommand('VKP',2,'vkp2')">SET</button>
        </div>
        <div class="row">
          <label>VKI<input id="vki2" type="number" step="0.01" value="1.15"></label>
          <button onclick="sendIndexedCommand('VKI',2,'vki2')">SET</button>
        </div>
      </div>
      <div class="wheel-box">
        <h3>Wheel 3 (F2)</h3>
        <div class="row">
          <label>VKP<input id="vkp3" type="number" step="0.1" value="5.5"></label>
          <button onclick="sendIndexedCommand('VKP',3,'vkp3')">SET</button>
        </div>
        <div class="row">
          <label>VKI<input id="vki3" type="number" step="0.01" value="1.30"></label>
          <button onclick="sendIndexedCommand('VKI',3,'vki3')">SET</button>
        </div>
      </div>
    </div>
    <div style="margin-top:14px">
      <div class="row">
        <label>VKPALL (all wheels Kp)<input id="vkpall" type="number" step="0.1" value="2"></label>
        <button onclick="sendValueCommand('VKPALL','vkpall')">SET ALL</button>
      </div>
      <div class="row">
        <label>VKIALL (all wheels Ki)<input id="vkiall" type="number" step="0.01" value="0.1"></label>
        <button onclick="sendValueCommand('VKIALL','vkiall')">SET ALL</button>
      </div>
    </div>
    <div style="margin-top:12px;font-size:0.82rem;color:var(--muted)">
      Live kpVel:
      <strong id="kpVel_0">-</strong>,
      <strong id="kpVel_1">-</strong>,
      <strong id="kpVel_2">-</strong>,
      <strong id="kpVel_3">-</strong>
      &nbsp;|&nbsp; kiVel:
      <strong id="kiVel_0">-</strong>,
      <strong id="kiVel_1">-</strong>,
      <strong id="kiVel_2">-</strong>,
      <strong id="kiVel_3">-</strong>
    </div>
  </section>

  <!-- Last response log -->
  <section class="panel full">
    <h2>Last Response</h2>
    <pre id="responseBox">Waiting for first command...</pre>
    <div style="margin-top:8px;font-size:0.8rem;color:var(--muted)" id="lastCmdStatus"></div>
  </section>

</main>

<script>
  async function getText(url) {
    const r = await fetch(url);
    return r.text();
  }

  async function sendCommand(command) {
    const text = await getText('/cmd?line=' + encodeURIComponent(command));
    document.getElementById('responseBox').textContent = '> ' + command + '\n' + text;
    refreshStatus();
  }

  function sendMove() {
    const d = document.getElementById('moveDistance').value;
    const h = document.getElementById('moveHeading').value;
    sendCommand('MOVE ' + d + ' ' + h);
  }

  function sendRotate() {
    sendCommand('ROTATE ' + document.getElementById('rotateHeading').value);
  }

  function sendRaw() {
    const v = document.getElementById('rawCommand').value.trim();
    if (v) sendCommand(v);
  }

  function sendValueCommand(prefix, inputId) {
    sendCommand(prefix + ' ' + document.getElementById(inputId).value);
  }

  function sendIndexedCommand(prefix, index, inputId) {
    sendCommand(prefix + ' ' + index + ' ' + document.getElementById(inputId).value);
  }

  function setInputFromStatus(inputId, statusValue) {
    if (statusValue === undefined || statusValue === null) {
      return;
    }
    const input = document.getElementById(inputId);
    if (!input) {
      return;
    }
    if (document.activeElement === input) {
      return;
    }
    input.value = statusValue;
  }

  async function refreshStatus() {
    try {
      const r = await fetch('/status');
      const data = await r.json();
      Object.keys(data).forEach(key => {
        const el = document.getElementById(key);
        if (el) el.textContent = data[key];
      });

      // Keep tuning inputs synced with controller state, but never interrupt typing.
      setInputFromStatus('pkp', data.Kp_pos);
      setInputFromStatus('pki', data.Ki_pos);
      setInputFromStatus('hkp', data.Kp_heading_rpm);
      setInputFromStatus('hki', data.Ki_heading_rpm_per_rad_s);
      setInputFromStatus('rkp', data.Kp_rotate_rpm);
      setInputFromStatus('rtol', data.HEADING_TOLERANCE_DEG);
      setInputFromStatus('vkp0', data.kpVel_0);
      setInputFromStatus('vkp1', data.kpVel_1);
      setInputFromStatus('vkp2', data.kpVel_2);
      setInputFromStatus('vkp3', data.kpVel_3);
      setInputFromStatus('vki0', data.kiVel_0);
      setInputFromStatus('vki1', data.kiVel_1);
      setInputFromStatus('vki2', data.kiVel_2);
      setInputFromStatus('vki3', data.kiVel_3);

      if (data.lastCommandResponse !== undefined) {
        document.getElementById('lastCmdStatus').textContent =
          'Last cmd result: ' + data.lastCommandResponse;
      }
    } catch (e) {
      document.getElementById('responseBox').textContent = 'STATUS ERROR\n' + e;
    }
  }

  refreshStatus();
  setInterval(refreshStatus, 300);
</script>
</body>
</html>
)HTML";

void sendPlainText(const String& text) {
  webServer.send(200, "text/plain", text);
}

String execute(const String& commandLine) {
  String response;
  if (commandExecutor == nullptr) {
    response = "ERR WEB_NOT_READY";
    return response;
  }
  commandExecutor(commandLine, response);
  return response;
}

void handleRoot() {
  webServer.send_P(200, "text/html", kControlPage);
}

void handleStatus() {
  if (statusJsonProvider == nullptr) {
    webServer.send(500, "application/json", "{\"error\":\"STATUS_PROVIDER_NOT_READY\"}");
    return;
  }
  webServer.send(200, "application/json", statusJsonProvider());
}

void handleCommand() {
  if (!webServer.hasArg("line")) {
    webServer.send(400, "text/plain", "ERR MISSING_LINE");
    return;
  }
  sendPlainText(execute(webServer.arg("line")));
}

void handleMove() {
  if (!webServer.hasArg("distance") || !webServer.hasArg("heading")) {
    webServer.send(400, "text/plain", "ERR FORMAT /move?distance=<m>&heading=<deg>");
    return;
  }
  sendPlainText(execute(String("MOVE ") + webServer.arg("distance") + " " + webServer.arg("heading")));
}

void handleRotate() {
  if (!webServer.hasArg("heading")) {
    webServer.send(400, "text/plain", "ERR FORMAT /rotate?heading=<deg>");
    return;
  }
  sendPlainText(execute(String("ROTATE ") + webServer.arg("heading")));
}

void handleStop() {
  sendPlainText(execute("STOP"));
}

}  // namespace

void configureWebControl(CommandExecutor executor, StatusJsonProvider statusProvider) {
  commandExecutor = executor;
  statusJsonProvider = statusProvider;
}

void beginWebControl() {
  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/status", HTTP_GET, handleStatus);
  webServer.on("/cmd", HTTP_GET, handleCommand);
  webServer.on("/move", HTTP_GET, handleMove);
  webServer.on("/rotate", HTTP_GET, handleRotate);
  webServer.on("/stop", HTTP_GET, handleStop);
  webServer.begin();
}

void pollWebControl() {
  webServer.handleClient();
}

}  // namespace app