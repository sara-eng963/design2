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
      box-shadow: 0 10px 30px rgba(31, 42, 48, 0.08);
    }
    h1, h2 {
      margin: 0 0 12px;
      font-family: Georgia, "Times New Roman", serif;
      letter-spacing: 0.02em;
    }
    h1 {
      grid-column: 1 / -1;
      font-size: 2rem;
    }
    h2 {
      font-size: 1.1rem;
    }
    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px 12px;
    }
    .status-item span {
      display: block;
      color: var(--muted);
      font-size: 0.82rem;
    }
    .status-item strong {
      font-size: 1rem;
    }
    .buttons {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    button {
      width: 100%;
      padding: 12px 10px;
      border: 0;
      border-radius: 12px;
      background: var(--accent);
      color: #fff;
      font-weight: 700;
      cursor: pointer;
    }
    button.alt {
      background: var(--accent-2);
    }
    button.ghost {
      background: #33464f;
    }
    label {
      display: block;
      margin-bottom: 10px;
      font-size: 0.9rem;
      color: var(--muted);
    }
    input {
      width: 100%;
      margin-top: 6px;
      padding: 10px 12px;
      border-radius: 10px;
      border: 1px solid var(--border);
      font-size: 1rem;
      background: #fff;
    }
    pre {
      margin: 0;
      padding: 12px;
      border-radius: 12px;
      background: #182126;
      color: #d9f0ef;
      min-height: 110px;
      overflow: auto;
      white-space: pre-wrap;
    }
    .full { grid-column: 1 / -1; }
  </style>
</head>
<body>
  <main>
    <h1>ESP32 Drive Web Control</h1>

    <section class="panel">
      <h2>Live Status</h2>
      <div class="status-grid">
        <div class="status-item"><span>Mode</span><strong id="mode">-</strong></div>
        <div class="status-item"><span>Active</span><strong id="active">-</strong></div>
        <div class="status-item"><span>Yaw</span><strong id="yawDeg">-</strong></div>
        <div class="status-item"><span>Target Yaw</span><strong id="targetYawDeg">-</strong></div>
        <div class="status-item"><span>Yaw Error</span><strong id="headingErrorDeg">-</strong></div>
        <div class="status-item"><span>Turn RPM</span><strong id="turnCorrectionRPM">-</strong></div>
        <div class="status-item"><span>Distance</span><strong id="currentDistanceM">-</strong></div>
        <div class="status-item"><span>Target Dist</span><strong id="targetDistanceM">-</strong></div>
        <div class="status-item"><span>Dist Error</span><strong id="distanceErrorM">-</strong></div>
        <div class="status-item"><span>Base RPM</span><strong id="baseForwardRpm">-</strong></div>
        <div class="status-item"><span>Last Result</span><strong id="lastMotionResult">-</strong></div>
        <div class="status-item"><span>Fault</span><strong id="fault">-</strong></div>
      </div>
    </section>

    <section class="panel">
      <h2>Quick Actions</h2>
      <div class="buttons">
        <button onclick="sendCommand('STATUS')">STATUS</button>
        <button class="alt" onclick="sendCommand('STOP')">STOP</button>
        <button onclick="sendCommand('ROTATE 0')">ROTATE 0</button>
        <button onclick="sendCommand('ROTATE 90')">ROTATE 90</button>
        <button onclick="sendCommand('ROTATE 180')">ROTATE 180</button>
        <button onclick="sendCommand('ROTATE -90')">ROTATE -90</button>
        <button onclick="sendCommand('MOVE 0.2 0')">MOVE 0.2 @ 0</button>
        <button onclick="sendCommand('MOVE 0.2 90')">MOVE 0.2 @ 90</button>
        <button onclick="sendCommand('MOVE 0.2 180')">MOVE 0.2 @ 180</button>
        <button onclick="sendCommand('MOVE 0.2 -90')">MOVE 0.2 @ -90</button>
      </div>
    </section>

    <section class="panel">
      <h2>Custom Move</h2>
      <label>Distance (m)<input id="moveDistance" type="number" step="0.01" value="0.30"></label>
      <label>Heading (deg)<input id="moveHeading" type="number" step="1" value="0"></label>
      <button onclick="sendMove()">SEND MOVE</button>
    </section>

    <section class="panel">
      <h2>Custom Rotate</h2>
      <label>Heading (deg)<input id="rotateHeading" type="number" step="1" value="90"></label>
      <button onclick="sendRotate()">SEND ROTATE</button>
    </section>

    <section class="panel">
      <h2>Raw Command</h2>
      <label>Command<input id="rawCommand" type="text" value="STATUS"></label>
      <button class="ghost" onclick="sendRaw()">SEND RAW</button>
    </section>

    <section class="panel">
      <h2>Rotate Tuning</h2>
      <label>RKP<input id="rkp" type="number" step="1" value="20"></label>
      <label>RMAX<input id="rmax" type="number" step="1" value="80"></label>
      <label>RMIN<input id="rmin" type="number" step="1" value="45"></label>
      <label>RTOL<input id="rtol" type="number" step="1" value="10"></label>
      <div class="buttons">
        <button onclick="sendValueCommand('RKP', 'rkp')">SET RKP</button>
        <button onclick="sendValueCommand('RMAX', 'rmax')">SET RMAX</button>
        <button onclick="sendValueCommand('RMIN', 'rmin')">SET RMIN</button>
        <button onclick="sendValueCommand('RTOL', 'rtol')">SET RTOL</button>
        <button class="ghost" onclick="sendCommand('RINVERT')">RINVERT</button>
      </div>
    </section>

    <section class="panel">
      <h2>Heading Tuning</h2>
      <label>HKP<input id="hkp" type="number" step="1" value="60"></label>
      <label>HKI<input id="hki" type="number" step="0.1" value="2"></label>
      <label>HMAX<input id="hmax" type="number" step="1" value="30"></label>
      <div class="buttons">
        <button onclick="sendValueCommand('HKP', 'hkp')">SET HKP</button>
        <button onclick="sendValueCommand('HKI', 'hki')">SET HKI</button>
        <button onclick="sendValueCommand('HMAX', 'hmax')">SET HMAX</button>
        <button class="ghost" onclick="sendCommand('HINVERT')">HINVERT</button>
      </div>
    </section>

    <section class="panel full">
      <h2>Last Response</h2>
      <pre id="responseBox">Waiting for status...</pre>
    </section>
  </main>

  <script>
    async function getText(url) {
      const response = await fetch(url);
      return response.text();
    }

    async function sendCommand(command) {
      const text = await getText('/cmd?line=' + encodeURIComponent(command));
      document.getElementById('responseBox').textContent = command + '\n' + text;
      refreshStatus();
    }

    function sendMove() {
      const distance = document.getElementById('moveDistance').value;
      const heading = document.getElementById('moveHeading').value;
      sendCommand('MOVE ' + distance + ' ' + heading);
    }

    function sendRotate() {
      const heading = document.getElementById('rotateHeading').value;
      sendCommand('ROTATE ' + heading);
    }

    function sendRaw() {
      sendCommand(document.getElementById('rawCommand').value);
    }

    function sendValueCommand(prefix, inputId) {
      sendCommand(prefix + ' ' + document.getElementById(inputId).value);
    }

    async function refreshStatus() {
      try {
        const response = await fetch('/status');
        const data = await response.json();
        Object.keys(data).forEach((key) => {
          const element = document.getElementById(key);
          if (element) {
            element.textContent = data[key];
          }
        });
      } catch (error) {
        document.getElementById('responseBox').textContent = 'STATUS ERROR\n' + error;
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