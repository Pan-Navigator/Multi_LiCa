import json

with open("pcd_data_p03v2.json") as f:
    pcd = json.load(f)

SENSOR_META = [
    {"name": "front",    "joint": "joint_rslidarfront",    "frame_id": "rslidarfront",    "color": "#4da6ff", "glColor": [0.302, 0.651, 1.0, 1]},
    {"name": "back",     "joint": "joint_rslidarback",     "frame_id": "rslidarback",     "color": "#ff7a45", "glColor": [1.0, 0.478, 0.271, 1]},
    {"name": "left",     "joint": "joint_rslidarleft",     "frame_id": "rslidarleft",     "color": "#4fd18f", "glColor": [0.31, 0.82, 0.561, 1]},
    {"name": "right",    "joint": "joint_rslidarright",    "frame_id": "rslidarright",    "color": "#f5d033", "glColor": [0.961, 0.816, 0.2, 1]},
    {"name": "fronttop", "joint": "joint_rslidarfront_top","frame_id": "rslidarfronttop", "color": "#b073ff", "glColor": [0.69, 0.451, 1.0, 1]},
    {"name": "backtop",  "joint": "joint_rslidarback_top", "frame_id": "rslidarbacktop",  "color": "#ff5fa8", "glColor": [1.0, 0.373, 0.659, 1]},
]

tuner_config = {
    "header": {
        "main": "Van Noord P3v2 six-lidar extrinsics tuner",
        "sub": "6 lidars · back/right/backtop calibrated via chained GICP, front/left held at measured priors (no usable overlap) · clouds from calibration_20260731_110805 · frame base_link"
    },
    "sensors": SENSOR_META,
    "target": "front",
}

HEAD_AND_BODY = r'''<!--
  Multi_LiCa LiDAR Extrinsics Tuner instance.
  Config-driven: sensor list/joints/colors live in the "tunerConfig" JSON block,
  seed transforms + per-sensor point clouds live in the "pcdData" JSON block.
  Point clouds are stored raw in sensor frame (int16, cm precision); the seed/tuned
  extrinsic is applied at render time, so dragging a slider re-transforms the cloud live.
-->
<html>
<head>
<meta charset="utf8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Van Noord P3v2 LiDAR Extrinsics Tuner</title>
<style>
  :root {
    --bg: #0b0f14;
    --panel: rgba(19, 26, 34, 0.94);
    --card: #121a23;
    --line: #232e3a;
    --text: #d7dee6;
    --muted: #7d8b99;
    --accent: #8fb6d9;
    --good: #4fc98a;
    --bad: #ff5a47;
  }
  * { margin: 0; padding: 0; box-sizing: border-box; }
  html, body { height: 100%; overflow: hidden; background: var(--bg); }
  body { font: 13px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif; color: var(--text); }
  canvas#gl { display: block; width: 100vw; height: 100vh; cursor: grab; }
  canvas#gl:active { cursor: grabbing; }

  .panel {
    position: fixed; top: 0; right: 0; bottom: 0; width: 340px;
    background: var(--panel); border-left: 1px solid var(--line);
    backdrop-filter: blur(6px);
    overflow-y: auto; padding: 14px; display: flex; flex-direction: column; gap: 12px;
  }
  .panel h1 { font-size: 14px; font-weight: 600; }
  .panel h1 small { display: block; color: var(--muted); font-weight: 400; font-size: 11.5px; margin-top: 2px; }

  .card { background: var(--card); border: 1px solid var(--line); border-radius: 8px; padding: 10px 12px; }
  .card header { display: flex; align-items: center; gap: 8px; margin-bottom: 8px; }
  .card header .dot { width: 10px; height: 10px; border-radius: 3px; }
  .card header .name { font-weight: 600; }
  .card header .spacer { flex: 1; }
  .card header button, .ab button, .copy, select {
    background: transparent; border: 1px solid var(--line); border-radius: 5px;
    color: var(--muted); font: inherit; font-size: 11.5px; padding: 3px 9px; cursor: pointer;
  }
  select { color: var(--text); background: #0d131a; }
  .card header button:hover, .ab button:hover, .copy:hover { color: var(--text); border-color: var(--accent); }
  button:focus-visible, input:focus-visible { outline: 2px solid var(--accent); outline-offset: -1px; }

  .row { display: grid; grid-template-columns: 40px 1fr 76px; align-items: center; gap: 8px; margin: 5px 0; }
  .row label { color: var(--muted); font-size: 11.5px; }
  .row input[type=range] { width: 100%; accent-color: var(--accent); height: 18px; }
  .row input[type=number] {
    width: 100%; background: #0d131a; border: 1px solid var(--line); border-radius: 4px;
    color: var(--text); font: 11.5px ui-monospace, Menlo, Consolas, monospace;
    padding: 3px 5px; font-variant-numeric: tabular-nums;
  }
  .row.dirty label { color: var(--accent); }
  .baseline { color: var(--muted); font-size: 10.5px; font-family: ui-monospace, Menlo, Consolas, monospace; margin-top: 6px; line-height: 1.6; font-variant-numeric: tabular-nums; }
  .baseline b { color: var(--text); font-weight: 600; }
  .baseline .now { color: var(--accent); }
  .urdf pre.flash { border-color: var(--accent); }

  .ab { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
  .ab button.active { background: #1d2836; color: var(--text); border-color: var(--accent); }
  .ab .hint { color: var(--muted); font-size: 11px; }

  .metric { display: flex; align-items: baseline; gap: 8px; font-family: ui-monospace, Menlo, Consolas, monospace; }
  .metric .val { font-size: 20px; font-weight: 600; color: var(--good); font-variant-numeric: tabular-nums; }
  .metric .lbl { color: var(--muted); font-size: 11px; }
  .ovsel { display: flex; align-items: center; gap: 6px; margin-top: 8px; color: var(--muted); font-size: 11.5px; }

  .urdf { position: relative; }
  .urdf pre {
    background: #0d131a; border: 1px solid var(--line); border-radius: 6px;
    font: 10.5px/1.7 ui-monospace, Menlo, Consolas, monospace; color: var(--text);
    padding: 9px 10px; overflow-x: auto; white-space: pre;
  }
  .urdf .copy { position: absolute; top: 6px; right: 6px; background: var(--card); }
  .chips { display: flex; gap: 8px; flex-wrap: wrap; }
  .chip {
    display: flex; align-items: center; gap: 7px; background: transparent;
    border: 1px solid var(--line); border-radius: 6px; color: var(--text);
    font: inherit; font-size: 12px; padding: 4px 11px; cursor: pointer;
  }
  .chip .cdot { width: 9px; height: 9px; border-radius: 2px; }
  .chip.off { color: var(--muted); }
  .chip.off .cdot { opacity: 0.25; }
  .size { display: flex; align-items: center; gap: 8px; color: var(--muted); font-size: 12px; }
  .size input { flex: 1; accent-color: var(--accent); }

  .hud {
    position: fixed; left: 12px; bottom: 12px; padding: 7px 11px;
    background: var(--panel); border: 1px solid var(--line); border-radius: 6px;
    font: 11.5px ui-monospace, Menlo, Consolas, monospace; color: var(--muted);
  }

  .map2d {
    position: fixed; top: 12px; left: 12px; width: 400px;
    background: var(--panel); border: 1px solid var(--line); border-radius: 8px;
    padding: 10px; display: none; flex-direction: column; gap: 8px; z-index: 5;
  }
  .map2d.on { display: flex; }
  .map2d .mhead { font-size: 11.5px; color: var(--muted); }
  .map2d .mhead b { color: var(--text); font-weight: 600; }
  .map2d canvas {
    width: 100%; aspect-ratio: 1; image-rendering: pixelated;
    background: #fff; border: 1px solid var(--line); border-radius: 4px;
  }
  .map2d .zband { display: flex; align-items: center; gap: 6px; font-size: 11.5px; color: var(--muted); }
  .map2d .zband input {
    width: 54px; background: #0d131a; border: 1px solid var(--line); border-radius: 4px;
    color: var(--text); font: 11.5px ui-monospace, Menlo, Consolas, monospace; padding: 2px 5px;
  }
</style>
</head>
<body>
<canvas id="gl"></canvas>
<div class="hud">drag rotate &middot; shift/right-drag pan &middot; wheel zoom &middot; grid 1 m / 10 m to ±100 m &middot; hold <b>B</b> = baseline</div>

<div class="map2d" id="map2d">
  <div class="mhead"><b>2D occupancy</b> &middot; top-down &middot; x&uarr; fwd &middot; y&larr; left &middot; view: <b id="mapView">tuned</b></div>
  <canvas id="mapCanvas"></canvas>
  <div class="zband">z&#8209;slice
    <input type="number" id="zLo" value="0.2" step="0.1"> to
    <input type="number" id="zHi" value="3.0" step="0.1"> m
    <span style="flex:1"></span><span id="mapInfo"></span>
  </div>
</div>

<div class="panel">
  <h1 id="title"></h1>

  <div class="ab">
    <button id="abTuned" class="active">Tuned</button>
    <button id="abBase">Baseline (URDF)</button>
    <span class="hint">or hold B</span>
    <button id="map2dBtn">2D map</button>
  </div>

  <div class="card metric-card">
    <div class="metric"><span class="val" id="overlap">&ndash;</span><span class="lbl" id="overlapLbl"></span></div>
    <div class="ovsel">overlap
      <select id="ovA"></select> vs <select id="ovB"></select>
      <span style="flex:1"></span>within 0.3 m
    </div>
  </div>

  <div id="cards"></div>
  <div class="chips" id="chips"></div>
  <label class="size">point size <input type="range" id="ptSize" min="1" max="5" step="0.5" value="2"></label>

  <div class="urdf">
    <pre id="urdfOut"></pre>
    <button class="copy" id="copyBtn">copy</button>
  </div>
</div>

'''

MAIN_SCRIPT = r'''<script>
"use strict";
const CFG = JSON.parse(document.getElementById("tunerConfig").textContent);
const DATA = JSON.parse(document.getElementById("pcdData").textContent);

// Config-driven: sensor list, joint names, and colors all come from the preset.
const SENSORS = CFG.sensors.map(s => s.name);
const JOINTS = Object.fromEntries(CFG.sensors.map(s => [s.name, s.joint]));
const CSSCOLOR = Object.fromEntries(CFG.sensors.map(s => [s.name, s.color]));      // hex string
const GLCOLOR = Object.fromEntries(CFG.sensors.map(s => [s.name, s.glColor]));     // [r,g,b,1]
const PARAMS = ["x", "y", "z", "roll", "pitch", "yaw"];
const D2R = Math.PI / 180, R2D = 180 / Math.PI;

// header
{
  const h = document.getElementById("title");
  h.textContent = CFG.header.main;
  if (CFG.header.sub) {
    const s = document.createElement("small");
    s.textContent = CFG.header.sub;
    h.appendChild(s);
  }
}

// baseline (URDF seed) in m / degrees for UI
const baseline = {}, tuned = {};
for (const s of SENSORS) {
  const seed = DATA.seeds[s];
  baseline[s] = {
    x: seed.xyz[0], y: seed.xyz[1], z: seed.xyz[2],
    roll: seed.rpy[0] * R2D, pitch: seed.rpy[1] * R2D, yaw: seed.rpy[2] * R2D,
  };
  tuned[s] = { ...baseline[s] };
}

function decode(entry) {
  const bin = atob(entry.b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  const q = new Int16Array(bytes.buffer);
  const f = new Float32Array(q.length);
  for (let i = 0; i < q.length; i++) f[i] = q[i] / 100;
  return f;
}
const rawPts = {};
for (const s of SENSORS) rawPts[s] = decode(DATA.sensors[s]);

// --- WebGL ---
const canvas = document.getElementById("gl");
const gl = canvas.getContext("webgl", { antialias: true });
const VS = `
attribute vec3 aPos;
uniform mat4 uMVP;
uniform float uSize;
void main() { gl_Position = uMVP * vec4(aPos, 1.0); gl_PointSize = uSize; }`;
const FS = `
precision mediump float;
uniform vec4 uColor;
void main() { gl_FragColor = uColor; }`;
const prog = gl.createProgram();
for (const [t, src] of [[gl.VERTEX_SHADER, VS], [gl.FRAGMENT_SHADER, FS]]) {
  const sh = gl.createShader(t); gl.shaderSource(sh, src); gl.compileShader(sh); gl.attachShader(prog, sh);
}
gl.linkProgram(prog);
const locPos = gl.getAttribLocation(prog, "aPos");
const locMVP = gl.getUniformLocation(prog, "uMVP");
const locSize = gl.getUniformLocation(prog, "uSize");
const locColor = gl.getUniformLocation(prog, "uColor");

function makeBuffer(arr) {
  const b = gl.createBuffer();
  gl.bindBuffer(gl.ARRAY_BUFFER, b);
  gl.bufferData(gl.ARRAY_BUFFER, arr, gl.STATIC_DRAW);
  return { buf: b, n: arr.length / 3 };
}
const cloudBuf = {};
for (const s of SENSORS) cloudBuf[s] = makeBuffer(rawPts[s]);
function gridLines(step, extent) {
  const v = [];
  for (let i = -extent; i <= extent; i += step)
    v.push(i, -extent, 0, i, extent, 0, -extent, i, 0, extent, i, 0);
  return new Float32Array(v);
}
// Large-area ground reference at base_link (horizontal plane, z=0): 1 m fine grid to
// ±20 m, 10 m coarse grid out to ±100 m, so the P3v2 outdoor/greenhouse layout
// reads at scale, not just the immediate lidar overlap zone.
const grid1 = makeBuffer(gridLines(1, 20)), grid10 = makeBuffer(gridLines(10, 100));
const axes = [
  { b: makeBuffer(new Float32Array([0,0,0, 2,0,0])), c: [1.0, 0.42, 0.38, 1] },
  { b: makeBuffer(new Float32Array([0,0,0, 0,2,0])), c: [0.45, 0.85, 0.45, 1] },
  { b: makeBuffer(new Float32Array([0,0,0, 0,0,2])), c: [0.42, 0.66, 1.0, 1] },
];

// --- mat4 helpers (column-major) ---
function perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2), nf = 1 / (near - far);
  return [f/aspect,0,0,0, 0,f,0,0, 0,0,(far+near)*nf,-1, 0,0,2*far*near*nf,0];
}
function lookAt(eye, at, up) {
  const sub = (a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
  const norm = a=>{const l=Math.hypot(...a);return [a[0]/l,a[1]/l,a[2]/l];};
  const cross = (a,b)=>[a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
  const dot = (a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
  const z = norm(sub(eye, at)), x = norm(cross(up, z)), y = cross(z, x);
  return [x[0],y[0],z[0],0, x[1],y[1],z[1],0, x[2],y[2],z[2],0,
          -dot(x,eye),-dot(y,eye),-dot(z,eye),1];
}
function mul(a, b) {
  const o = new Array(16);
  for (let c = 0; c < 4; c++) for (let r = 0; r < 4; r++) {
    let s = 0;
    for (let k = 0; k < 4; k++) s += a[k*4+r] * b[c*4+k];
    o[c*4+r] = s;
  }
  return o;
}
// URDF rpy -> model matrix: R = Rz(yaw)*Ry(pitch)*Rx(roll), then translate
function modelMatrix(p) {
  const r = p.roll * D2R, pt = p.pitch * D2R, y = p.yaw * D2R;
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(pt), sp = Math.sin(pt);
  const cy = Math.cos(y), sy = Math.sin(y);
  const R = [
    cy*cp, sy*cp, -sp,
    cy*sp*sr - sy*cr, sy*sp*sr + cy*cr, cp*sr,
    cy*sp*cr + sy*sr, sy*sp*cr - cy*sr, cp*cr,
  ];
  return [R[0],R[1],R[2],0, R[3],R[4],R[5],0, R[6],R[7],R[8],0, p.x,p.y,p.z,1];
}

// --- camera ---
const cam = { az: -2.3, el: 0.9, r: 45, target: [0, 0, 1] };
const show = {};
for (const s of SENSORS) show[s] = true;
let ptSize = 2, viewBaseline = false;

function activeParams(s) { return viewBaseline ? baseline[s] : tuned[s]; }

function draw() {
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  const w = canvas.clientWidth * dpr, h = canvas.clientHeight * dpr;
  if (canvas.width !== w || canvas.height !== h) { canvas.width = w; canvas.height = h; }
  gl.viewport(0, 0, w, h);
  gl.clearColor(0.043, 0.059, 0.078, 1);
  gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
  gl.enable(gl.DEPTH_TEST);

  const ce = Math.cos(cam.el), t = cam.target;
  const eye = [t[0] + cam.r * ce * Math.cos(cam.az),
               t[1] + cam.r * ce * Math.sin(cam.az),
               t[2] + cam.r * Math.sin(cam.el)];
  const PV = mul(perspective(0.9, w / h, 0.1, 500), lookAt(eye, t, [0, 0, 1]));

  gl.useProgram(prog);
  gl.enableVertexAttribArray(locPos);
  const bind = o => { gl.bindBuffer(gl.ARRAY_BUFFER, o.buf); gl.vertexAttribPointer(locPos, 3, gl.FLOAT, false, 0, 0); };
  const setMVP = m => gl.uniformMatrix4fv(locMVP, false, new Float32Array(m));

  setMVP(PV);
  gl.uniform1f(locSize, 1);
  gl.uniform4f(locColor, 0.13, 0.17, 0.22, 1); bind(grid1); gl.drawArrays(gl.LINES, 0, grid1.n);
  gl.uniform4f(locColor, 0.20, 0.26, 0.33, 1); bind(grid10); gl.drawArrays(gl.LINES, 0, grid10.n);
  for (const a of axes) { gl.uniform4f(locColor, ...a.c); bind(a.b); gl.drawArrays(gl.LINES, 0, 2); }

  gl.uniform1f(locSize, ptSize * dpr);
  for (const s of SENSORS) {
    if (!show[s]) continue;
    setMVP(mul(PV, modelMatrix(activeParams(s))));
    gl.uniform4f(locColor, ...GLCOLOR[s]);
    bind(cloudBuf[s]);
    gl.drawArrays(gl.POINTS, 0, cloudBuf[s].n);
  }
}
function requestDraw() { requestAnimationFrame(draw); }

// --- overlap metric (voxel hash, debounced, selectable pair) ---
const VOX = 0.3;
function key(x, y, z) {
  return ((Math.round(x / VOX) + 512) * 2048 + (Math.round(y / VOX) + 512)) * 2048 + (Math.round(z / VOX) + 512);
}
function applyM(m, pts, stride) {
  const out = [];
  for (let i = 0; i < pts.length; i += 3 * stride) {
    const x = pts[i], y = pts[i+1], z = pts[i+2];
    out.push(m[0]*x + m[4]*y + m[8]*z + m[12],
             m[1]*x + m[5]*y + m[9]*z + m[13],
             m[2]*x + m[6]*y + m[10]*z + m[14]);
  }
  return out;
}
let ovA = SENSORS[0], ovB = SENSORS[1] || SENSORS[0];
let overlapTimer = null;
function scheduleOverlap() {
  clearTimeout(overlapTimer);
  overlapTimer = setTimeout(() => {
    const a = applyM(modelMatrix(activeParams(ovA)), rawPts[ovA], 2);
    const set = new Set();
    for (let i = 0; i < a.length; i += 3) {
      for (const [dx, dy, dz] of [[0,0,0],[VOX,0,0],[-VOX,0,0],[0,VOX,0],[0,-VOX,0],[0,0,VOX],[0,0,-VOX]])
        set.add(key(a[i]+dx, a[i+1]+dy, a[i+2]+dz));
    }
    const b = applyM(modelMatrix(activeParams(ovB)), rawPts[ovB], 3);
    let hit = 0, tot = 0;
    for (let i = 0; i < b.length; i += 3, tot++)
      if (set.has(key(b[i], b[i+1], b[i+2]))) hit++;
    const pct = tot ? (100 * hit / tot) : 0;
    const el = document.getElementById("overlap");
    el.textContent = pct.toFixed(1) + "%";
    el.style.color = pct > 60 ? "var(--good)" : pct > 30 ? "#e0b04d" : "var(--bad)";
    document.getElementById("overlapLbl").textContent =
      `of ${ovB} points near a ${ovA} point (live)`;
  }, 250);
}

// --- panel UI ---
const RANGES = {
  x: [-3, 3, 0.005], y: [-3, 3, 0.005], z: [-1, 2, 0.005],
  roll: [-180, 180, 0.1], pitch: [-180, 180, 0.1], yaw: [-180, 180, 0.1],
};
const UNITS = { x: "m", y: "m", z: "m", roll: "°", pitch: "°", yaw: "°" };
const inputs = {};
for (const s of SENSORS) inputs[s] = {};

function buildCard(sensor) {
  const el = document.createElement("div");
  el.className = "card";
  const h = document.createElement("header");
  h.innerHTML = `<span class="dot" style="background:${CSSCOLOR[sensor]}"></span>
    <span class="name">${sensor}</span><span class="spacer"></span>`;
  const reset = document.createElement("button");
  reset.textContent = "reset to URDF";
  reset.onclick = () => { Object.assign(tuned[sensor], baseline[sensor]); syncInputs(sensor); onTune(); };
  h.appendChild(reset);
  el.appendChild(h);

  for (const p of PARAMS) {
    const [lo, hi, step] = RANGES[p];
    const row = document.createElement("div");
    row.className = "row";
    const lab = document.createElement("label");
    lab.textContent = p + " " + UNITS[p];
    const slider = document.createElement("input");
    slider.type = "range";
    slider.min = p.length === 1 ? baseline[sensor][p] + lo : lo;
    slider.max = p.length === 1 ? baseline[sensor][p] + hi : hi;
    slider.step = step;
    const num = document.createElement("input");
    num.type = "number"; num.step = step;
    slider.oninput = () => { tuned[sensor][p] = +slider.value; num.value = (+slider.value).toFixed(p.length === 1 ? 3 : 2); markDirty(row, sensor, p); onTune(); };
    num.onchange = () => { tuned[sensor][p] = +num.value; slider.value = num.value; markDirty(row, sensor, p); onTune(); };
    row.append(lab, slider, num);
    el.appendChild(row);
    inputs[sensor][p] = { slider, num, row };
  }
  const bl = document.createElement("div");
  bl.className = "baseline";
  const b = baseline[sensor];
  const now = document.createElement("div");
  now.className = "now";
  now.id = "now-" + sensor;
  const seed = document.createElement("div");
  seed.innerHTML = `<b>seed:</b> xyz ${b.x.toFixed(4)} ${b.y.toFixed(4)} ${b.z.toFixed(4)} · ` +
    `rpy° ${b.roll.toFixed(2)} ${b.pitch.toFixed(2)} ${b.yaw.toFixed(2)}`;
  bl.append(now, seed);
  el.appendChild(bl);
  document.getElementById("cards").appendChild(el);
}
function updateNowLine(sensor) {
  const p = tuned[sensor];
  document.getElementById("now-" + sensor).innerHTML =
    `<b>now:</b>&nbsp; xyz ${p.x.toFixed(4)} ${p.y.toFixed(4)} ${p.z.toFixed(4)} · ` +
    `rpy° ${p.roll.toFixed(2)} ${p.pitch.toFixed(2)} ${p.yaw.toFixed(2)}`;
}
function markDirty(row, sensor, p) {
  row.classList.toggle("dirty", Math.abs(tuned[sensor][p] - baseline[sensor][p]) > 1e-6);
}
function syncInputs(sensor) {
  for (const p of PARAMS) {
    const { slider, num, row } = inputs[sensor][p];
    slider.value = tuned[sensor][p];
    num.value = (+tuned[sensor][p]).toFixed(p.length === 1 ? 3 : 2);
    markDirty(row, sensor, p);
  }
}
for (const s of SENSORS) { buildCard(s); syncInputs(s); }

// chips + overlap-pair selectors
const chips = document.getElementById("chips");
for (const s of SENSORS) {
  const c = document.createElement("button");
  c.className = "chip";
  c.innerHTML = `<span class="cdot" style="background:${CSSCOLOR[s]}"></span>${s}`;
  c.onclick = function () { show[s] = !show[s]; this.classList.toggle("off", !show[s]); scheduleMap(); requestDraw(); };
  chips.appendChild(c);
}
const selA = document.getElementById("ovA"), selB = document.getElementById("ovB");
for (const s of SENSORS) {
  selA.appendChild(new Option(s, s)); selB.appendChild(new Option(s, s));
}
selA.value = ovA; selB.value = ovB;
selA.onchange = () => { ovA = selA.value; scheduleOverlap(); };
selB.onchange = () => { ovB = selB.value; scheduleOverlap(); };

function urdfText() {
  let out = "";
  for (const s of SENSORS) {
    const p = tuned[s];
    const rpy = [p.roll, p.pitch, p.yaw].map(v => +(v * D2R).toFixed(7));
    out += `<joint name="${JOINTS[s]}" type="fixed">\n` +
           `  <origin rpy="${rpy.join(" ")}"\n          xyz="${(+p.x.toFixed(4))} ${(+p.y.toFixed(4))} ${(+p.z.toFixed(4))}"/>\n` +
           `</joint>\n`;
  }
  return out;
}
let flashTimer = null;
function onTune() {
  const pre = document.getElementById("urdfOut");
  pre.textContent = urdfText();
  pre.classList.add("flash");
  clearTimeout(flashTimer);
  flashTimer = setTimeout(() => pre.classList.remove("flash"), 350);
  for (const s of SENSORS) updateNowLine(s);
  scheduleOverlap();
  scheduleMap();
  requestDraw();
}
document.getElementById("copyBtn").onclick = function () {
  navigator.clipboard.writeText(urdfText()).then(() => {
    this.textContent = "copied"; setTimeout(() => this.textContent = "copy", 1200);
  });
};

// A/B
const abT = document.getElementById("abTuned"), abB = document.getElementById("abBase");
function setView(b) {
  viewBaseline = b;
  abT.classList.toggle("active", !b);
  abB.classList.toggle("active", b);
  scheduleOverlap(); drawMap(); requestDraw();
}
abT.onclick = () => setView(false);
abB.onclick = () => setView(true);
window.addEventListener("keydown", e => {
  if (e.key.toLowerCase() === "b" && !e.repeat && document.activeElement.tagName !== "INPUT" && document.activeElement.tagName !== "SELECT") setView(true);
});
window.addEventListener("keyup", e => { if (e.key.toLowerCase() === "b") setView(false); });

document.getElementById("ptSize").oninput = function () { ptSize = +this.value; requestDraw(); };

// --- orbit input ---
let dragging = false, panning = false, lastX = 0, lastY = 0;
canvas.addEventListener("mousedown", e => { dragging = true; panning = e.button === 2 || e.shiftKey; lastX = e.clientX; lastY = e.clientY; });
window.addEventListener("mouseup", () => dragging = false);
window.addEventListener("mousemove", e => {
  if (!dragging) return;
  const dx = e.clientX - lastX, dy = e.clientY - lastY;
  lastX = e.clientX; lastY = e.clientY;
  if (panning) {
    const s = cam.r * 0.0016, ca = Math.cos(cam.az), sa = Math.sin(cam.az);
    cam.target[0] -= (dx * sa - dy * ca * Math.sin(cam.el)) * s;
    cam.target[1] -= (-dx * ca - dy * sa * Math.sin(cam.el)) * s;
    cam.target[2] += dy * Math.cos(cam.el) * s;
  } else {
    cam.az -= dx * 0.006;
    cam.el = Math.min(1.55, Math.max(-1.55, cam.el + dy * 0.006));
  }
  requestDraw();
});
canvas.addEventListener("contextmenu", e => e.preventDefault());
canvas.addEventListener("wheel", e => {
  e.preventDefault();
  cam.r = Math.min(300, Math.max(2, cam.r * (e.deltaY > 0 ? 1.12 : 0.89)));
  requestDraw();
}, { passive: false });
let touches = [];
canvas.addEventListener("touchstart", e => { touches = [...e.touches]; }, { passive: true });
canvas.addEventListener("touchmove", e => {
  e.preventDefault();
  const t = [...e.touches];
  if (t.length === 1 && touches.length === 1) {
    cam.az -= (t[0].clientX - touches[0].clientX) * 0.006;
    cam.el = Math.min(1.55, Math.max(-1.55, cam.el + (t[0].clientY - touches[0].clientY) * 0.006));
  } else if (t.length === 2 && touches.length === 2) {
    const d0 = Math.hypot(touches[0].clientX - touches[1].clientX, touches[0].clientY - touches[1].clientY);
    const d1 = Math.hypot(t[0].clientX - t[1].clientX, t[0].clientY - t[1].clientY);
    if (d1 > 0) cam.r = Math.min(300, Math.max(2, cam.r * d0 / d1));
  }
  touches = t; requestDraw();
}, { passive: false });
window.addEventListener("resize", requestDraw);

// --- 2D occupancy overlay (top-down slice built from the displayed clouds) ---
let map2dOn = false, mapR = 12, mapCell = 0.05, zLo = 0.2, zHi = 3.0, mapTimer = null;
const mapDim = Math.round(2 * mapR / mapCell);
const mapCanvas = document.getElementById("mapCanvas");
const mapCtx = mapCanvas.getContext("2d");
mapCanvas.width = mapDim; mapCanvas.height = mapDim;
function computeMap() {
  const counts = new Float32Array(mapDim * mapDim);
  let maxc = 0, tot = 0;
  for (const s of SENSORS) {
    if (!show[s]) continue;
    const m = modelMatrix(activeParams(s)), pts = rawPts[s];
    for (let i = 0; i < pts.length; i += 3) {
      const x = pts[i], y = pts[i + 1], z = pts[i + 2];
      const wz = m[2] * x + m[6] * y + m[10] * z + m[14];
      if (wz < zLo || wz > zHi) continue;
      const wx = m[0] * x + m[4] * y + m[8] * z + m[12];
      const wy = m[1] * x + m[5] * y + m[9] * z + m[13];
      const col = (mapR - wy) / mapCell | 0, row = (mapR - wx) / mapCell | 0;
      if (col < 0 || col >= mapDim || row < 0 || row >= mapDim) continue;
      const c = ++counts[row * mapDim + col];
      if (c > maxc) maxc = c;
      tot++;
    }
  }
  return { counts, maxc, tot };
}
function drawMap() {
  if (!map2dOn) return;
  const { counts, maxc, tot } = computeMap();
  const img = mapCtx.createImageData(mapDim, mapDim);
  const OCC = 1; // >=1 return per cell => occupied (binary occupancy grid)
  for (let i = 0; i < counts.length; i++) {
    const px = i * 4, v = counts[i] >= OCC ? 0 : 255; // occupied = black, free = white
    img.data[px] = v; img.data[px + 1] = v; img.data[px + 2] = v; img.data[px + 3] = 255;
  }
  mapCtx.putImageData(img, 0, 0);
  mapCtx.strokeStyle = "rgba(0,0,0,0.12)"; mapCtx.lineWidth = 1;
  for (let g = -mapR; g <= mapR; g += 1) {
    const p = Math.round((mapR - g) / mapCell) + 0.5;
    mapCtx.beginPath(); mapCtx.moveTo(0, p); mapCtx.lineTo(mapDim, p);
    mapCtx.moveTo(p, 0); mapCtx.lineTo(p, mapDim); mapCtx.stroke();
  }
  const oc = mapR / mapCell;
  mapCtx.strokeStyle = "#777"; mapCtx.lineWidth = 2;
  mapCtx.beginPath(); mapCtx.moveTo(oc, oc); mapCtx.lineTo(oc, oc - 15); mapCtx.stroke();
  mapCtx.fillStyle = "#777";
  mapCtx.beginPath(); mapCtx.arc(oc, oc, 3, 0, 7); mapCtx.fill();
  document.getElementById("mapInfo").textContent =
    tot.toLocaleString() + " pts · " + (zHi - zLo).toFixed(1) + " m band";
  document.getElementById("mapView").textContent = viewBaseline ? "baseline" : "tuned";
}
function scheduleMap() { if (!map2dOn) return; clearTimeout(mapTimer); mapTimer = setTimeout(drawMap, 120); }
document.getElementById("map2dBtn").onclick = function () {
  map2dOn = !map2dOn;
  document.getElementById("map2d").classList.toggle("on", map2dOn);
  this.classList.toggle("active", map2dOn);
  if (map2dOn) drawMap();
};
document.getElementById("zLo").onchange = function () { zLo = +this.value; drawMap(); };
document.getElementById("zHi").onchange = function () { zHi = +this.value; drawMap(); };

// No calibration has been run against this bag yet (this is the raw URDF/tf_static
// seed for all six sensors) -- unlike the Rijkzwaan P3 tuner, there is no baked
// GICP result to overlay here.
onTune(); draw();
</script>
'''

full = HEAD_AND_BODY
full += f'<script id="tunerConfig" type="application/json">{json.dumps(tuner_config)}</script>\n'
full += f'<script id="pcdData" type="application/json">{json.dumps(pcd)}</script>\n\n'
full += MAIN_SCRIPT

with open("van_noord_p03v2_lidar_tuner.html", "w") as f:
    f.write(full)

print("wrote", len(full), "chars")
