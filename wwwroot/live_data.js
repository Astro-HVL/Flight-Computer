// live_data.js - connects to SignalR telemetry hub and updates the live page
const conn = new signalR.HubConnectionBuilder().withUrl('/telemetry').withAutomaticReconnect().build();

// DOM elements (will be set after DOMContentLoaded)
let heightDisplay, speedDisplay, accelDisplay, circleAlt, circleVel, circleG;
let modeOn, modeReady, modePending;

// create small debug status box so users can see init state without opening console
let __statusBox = null;
function ensureStatusBox(){
  if (__statusBox) return __statusBox;
  __statusBox = document.createElement('div');
  __statusBox.id = '__live_data_status';
  Object.assign(__statusBox.style, { position:'fixed', right:'12px', bottom:'12px', padding:'8px 12px', background:'rgba(0,0,0,0.6)', color:'#fff', fontSize:'12px', borderRadius:'8px', zIndex:9999 });
  __statusBox.textContent = 'live_data: init';
  document.body.appendChild(__statusBox);
  return __statusBox;
}
function setStatus(text){ try { ensureStatusBox().textContent = 'live_data: ' + text; } catch(e){} }

// mode elements will be looked up after DOMContentLoaded

function setActiveMode(name){
  [modeOn, modeReady, modePending].forEach(el => { if(!el) return; el.classList.remove('active-mode'); });
  const map = { 'on': modeOn, 'ready': modeReady, 'pending': modePending };
  const el = map[name];
  if (el) el.classList.add('active-mode');
}

// altitude chart
let altChart;
document.addEventListener('DOMContentLoaded', () => {
  heightDisplay = document.getElementById('heightValue') || document.getElementById('heightDisplay');
  speedDisplay = document.getElementById('speedValue') || document.getElementById('speedDisplay');
  accelDisplay = document.getElementById('accelValue') || document.getElementById('accelDisplay');
  circleAlt = document.getElementById('circleAltValue');
  circleVel = document.getElementById('circleVelValue');
  circleG = document.getElementById('circleGValue');

  console.log('live_data elements', { heightDisplay, speedDisplay, accelDisplay, circleAlt, circleVel, circleG });

  // mode elements
  modeOn = document.getElementById('modeOn');
  modeReady = document.getElementById('modeReady');
  modePending = document.getElementById('modePending');

  // make mode boxes clickable for testing
  try {
    if (modeOn) modeOn.addEventListener('click', () => setActiveMode('on'));
    if (modeReady) modeReady.addEventListener('click', () => setActiveMode('ready'));
    if (modePending) modePending.addEventListener('click', () => setActiveMode('pending'));
  } catch(e) { }

  // ensure white circles are visible even if CSS didn't load properly
  try {
    const wrappers = document.querySelectorAll('.white-circle');
    wrappers.forEach(w => {
      w.style.display = 'flex';
      w.style.flexDirection = 'column';
      w.style.alignItems = 'center';
      w.style.justifyContent = 'center';
      w.style.background = '#f11414ff';
      w.style.color = '#000';
      w.style.width = w.style.width || '260px';
      w.style.height = w.style.height || '260px';
      w.style.borderRadius = '130px';
      w.style.border = w.style.border || '2px solid rgba(0,0,0,0.06)';
      w.style.boxSizing = 'border-box';
      w.style.zIndex = 5;
    });
  } catch (e) { console.warn('forcing circle styles failed', e); }

  createAltChart();
});

// Fallback 2D rocket renderer if rocket3d.js is not available
function createFallbackRocket() {
  const container = document.getElementById('rocket3dContainer');
  if (!container) return null;
  // create a canvas sized to container
  const canvas = document.createElement('canvas');
  canvas.width = container.clientWidth || 220;
  canvas.height = container.clientHeight || 220;
  canvas.style.width = '100%';
  canvas.style.height = '100%';
  container.innerHTML = '';
  container.appendChild(canvas);
  const ctx = canvas.getContext('2d');

  function draw(pitch = 0, yaw = 0, roll = 0) {
    ctx.clearRect(0,0,canvas.width,canvas.height);
    ctx.save();
    ctx.translate(canvas.width/2, canvas.height - 20);
    ctx.rotate((yaw||0) * Math.PI/180);
    // simple rocket
    ctx.fillStyle = '#fff'; ctx.strokeStyle='#000'; ctx.lineWidth=2;
    ctx.beginPath(); ctx.moveTo(0,-80); ctx.lineTo(12,-30); ctx.lineTo(6,-30); ctx.lineTo(6,0); ctx.lineTo(-6,0); ctx.lineTo(-6,-30); ctx.lineTo(-12,-30); ctx.closePath(); ctx.fill(); ctx.stroke();
    // axes
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(0,-50); ctx.strokeStyle='gold'; ctx.lineWidth=3; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(30,-30); ctx.strokeStyle='crimson'; ctx.lineWidth=3; ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0,0); ctx.lineTo(40,0); ctx.strokeStyle='limegreen'; ctx.lineWidth=3; ctx.stroke();
    ctx.restore();
  }
  return { draw };
}

// If rocket3d isn't present, set a fallback setter that draws to the 2D canvas
if (typeof initRocket3D !== 'function') {
  document.addEventListener('DOMContentLoaded', () => {
    const fallback = createFallbackRocket();
    if (fallback) {
      window.setRocket3DRotation = (pitch,yaw,roll) => { try { fallback.draw(pitch,yaw,roll); } catch(e){} };
      console.warn('Using 2D fallback rocket renderer');
    }
  });
}

function createAltChart(){
  const el = document.getElementById('altitudeChart');
  if (!el) {
    console.warn('altitudeChart canvas not found');
    return;
  }
  const ctx = el.getContext('2d');
  altChart = new Chart(ctx, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        {
          label: 'Altitude (m)',
          data: [],
          borderColor: 'Magenta',
          fill: false,
          yAxisID: 'y_alt',
          pointRadius: 1
        }
      ]
    },
    options: {
      animation: false,
      responsive: false,
      maintainAspectRatio: true,
      scales: {
        x: {
          display: true,
          min: 0,
          title: { display: true, text: 'Time (s)', font: { size: 20 }, color: '#fff' },
          ticks: { font: { size: 14 }, color: '#fff', callback: function(value, index) {
            const label = this.getLabelForValue(index);
            if (!label) return '';
            return parseFloat(label);
          } }
        },
        y_alt: {
          title: { display: true, text: 'Altitude (m)', font: { size: 20 }, color: '#fff' },
          ticks: { font: { size: 14 }, color: '#fff' }
        }
      },
      plugins: { legend: { labels: { font: { size: 20 }, color: '#fff' } } }
    }
  });
}

// helper to update metrics
function updateMetrics(alt, vel, ax, ay, az){
  if (heightDisplay) heightDisplay.textContent = (alt !== undefined ? Number(alt).toFixed(0) : '-');
  if (speedDisplay) speedDisplay.textContent = (vel !== undefined ? Number(vel).toFixed(1) : '-');
  if (accelDisplay) {
    if (ax===undefined||ay===undefined||az===undefined) accelDisplay.textContent='-';
    else accelDisplay.textContent = Math.sqrt(ax*ax+ay*ay+az*az).toFixed(2);
  }
  // update bottom-left white circles
  if (circleAlt) circleAlt.textContent = (alt !== undefined ? Number(alt).toFixed(0) : '-');
  if (circleVel) circleVel.textContent = (vel !== undefined ? Number(vel).toFixed(1) : '-');
  if (circleG) {
    if (ax===undefined||ay===undefined||az===undefined) circleG.textContent='-';
    else circleG.textContent = Math.sqrt(ax*ax+ay*ay+az*az).toFixed(2);
  }
}

// push altitude to chart
function pushAltitude(tSec, alt){
  if (!altChart) return;
  altChart.data.labels.push(tSec.toFixed(2));
  altChart.data.datasets[0].data.push(alt);
  if (altChart.data.labels.length>200) { altChart.data.labels.shift(); altChart.data.datasets[0].data.shift(); }
  altChart.update('none');
}

// receive telemetry
conn.on('telemetry', payload => {
  try {
    if (payload.type !== 'telemetry') return;
    const t = Number(payload.t)/1000.0; // seconds
    const alt = Number(payload.alt);
    const vel = Number(payload.vel);
    const ax = Number(payload.ax);
    const ay = Number(payload.ay);
    const az = Number(payload.az);
    const pitch = Number(payload.pitch);
    const roll = Number(payload.roll);
    const yaw = Number(payload.yaw);

    updateMetrics(alt, vel, ax, ay, az);
    pushAltitude(t, alt);

    // update 3D rocket via external function if provided by rocket3d.js
    if (typeof setRocket3DRotation === 'function') setRocket3DRotation(pitch, yaw, roll);

    // mode heuristic: use velocity and altitude to choose a mode for demo
    if (vel > 1) setActiveMode('on');
    else if (alt > 10) setActiveMode('ready');
    else setActiveMode('pending');
  } catch(e){ console.error('telemetry parse error', e); }
});

async function startConn(){
  try {
    await conn.start();
    console.log('live_data connected');
  }
  catch(e){ console.error('conn start failed', e); setTimeout(startConn,2000); }
}

// Initialize UI and 3D model once DOM is ready
document.addEventListener('DOMContentLoaded', () => {
  console.log('DOM ready - initializing live_data UI');
  // placeholders for circles
  try {
    if (circleAlt && circleAlt.textContent.trim() === '') circleAlt.textContent = '-';
    if (circleVel && circleVel.textContent.trim() === '') circleVel.textContent = '-';
    if (circleG && circleG.textContent.trim() === '') circleG.textContent = '-';
  } catch (e) { console.warn('placeholder init failed', e); }

  // initialize the shared 3D model container if available — run once after a short delay so layout settles
    if (typeof initRocket3D === 'function') {
      // only init once across potential multiple pages
      if (!window.__rocket3d_inited) {
        window.__rocket3d_inited = true;
        setTimeout(() => {
          try { initRocket3D(); }
          catch(e) { console.error('initRocket3D error', e); setStatus('3D init error'); }
        }, 120);
      } else {
        console.log('rocket3d already initialized elsewhere');
        setStatus('3D already init');
      }
    } else {
      console.warn('initRocket3D not found'); setStatus('3D not found');
    }

  // start SignalR connection
  startConn();

  // default mode
  setActiveMode('ready');
});
