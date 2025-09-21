const connection = new signalR.HubConnectionBuilder()
    .withUrl("/telemetry")
    .withAutomaticReconnect()
    .build();

const debug = document.getElementById('debug');
function dbg(s){
  debug.textContent = s + "\n" + debug.textContent;
  if(debug.textContent.length>8000) debug.textContent = debug.textContent.slice(0,8000);
}

// DOM elements
const valTime = document.getElementById('val_time');
const valSeq  = document.getElementById('val_seq');
const valAx   = document.getElementById('val_ax');
const valAy   = document.getElementById('val_ay');
const valAz   = document.getElementById('val_az');
const valPitch= document.getElementById('val_pitch');
const valRoll = document.getElementById('val_roll');
const valYaw  = document.getElementById('val_yaw');
const valTemp = document.getElementById('val_temp');
const valVel  = document.getElementById('val_vel');
const valPress= document.getElementById('val_press');
const valLat  = document.getElementById('val_lat');
const valLon  = document.getElementById('val_lon');
const valAlt  = document.getElementById('val_alt');

// charts
let velChart, altChart, accChart, orientChart, envChart;
function createCharts(){
  
  // velocity chart
  const ctx1 = document.getElementById('velChart').getContext('2d');
  velChart = new Chart(ctx1, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label:'Velocity (m/s)', data:[], borderColor:'goldenrod', fill:false, yAxisID:'y_vel', pointRadius:1 }
      ]
    },
    options: { animation:false, responsive:false, maintainAspectRatio: true, scales:{ x:{ display:true, title:{display:true, text:'Samples'} } } }
  });

  // alt chart
  const ctx2 = document.getElementById('altChart').getContext('2d');
  altChart = new Chart(ctx2, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label:'Altitude (m)', data:[], borderColor:'Magenta', fill:false, yAxisID:'y_alt', pointRadius:1 }
      ]
    },
      options: { animation: false, responsive: false, maintainAspectRatio: true, scales:{ x:{ display:true, title:{display:true, text:'Samples'} } } }
  });

  // accel chart
  const ctx3 = document.getElementById('accChart').getContext('2d');
  accChart = new Chart(ctx3, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label: 'ax (g)', data: [], borderColor:'red', fill:false, pointRadius:1 },
        { label: 'ay (g)', data: [], borderColor:'green', fill:false, pointRadius:1 },
        { label: 'az (g)', data: [], borderColor:'blue', fill:false, pointRadius:1 }
      ]
    },
      options: { animation: false, responsive: false, maintainAspectRatio: true, scales:{ x:{ display:true, title:{display:true, text:'Samples'} } } }
  });


  // orientation
  const ctx4 = document.getElementById('orientChart').getContext('2d');
  orientChart = new Chart(ctx4, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label:'Pitch (\u00B0)', data:[], borderColor:'orange', fill:false, pointRadius:1 },
        { label:'Roll (\u00B0)',  data:[], borderColor:'purple', fill:false, pointRadius:1 },
        { label:'Yaw (\u00B0)',   data:[], borderColor:'teal', fill:false, pointRadius:1 }
      ]
    },
      options: { animation: false, responsive: false, maintainAspectRatio: true }
  });

  // environment: temp, press
  const ctx5 = document.getElementById('envChart').getContext('2d');
  envChart = new Chart(ctx5, {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label:'Temperature (\u00B0C)', data:[], borderColor:'brown', fill:false, yAxisID:'y_temp', pointRadius:1 },
        { label:'Pressure (atm)', data:[], borderColor:'gray', fill:false, yAxisID:'y_press', pointRadius:1 }
      ]
    },
    options: {
      animation:false,
      responsive:false,
      scales: {
        y_temp: { type:'linear', position:'left', title:{display:true, text:'Temp (\u00B0C)'} },
        y_press:{ type:'linear', position:'right', title:{display:true, text:'Pressure (atm)'}, grid:{ drawOnChartArea:false } },
      }
    }
  });
}

function updateLatest(tMs, seq, ax, ay, az, pitch, roll, yaw, temp, vel, press, lat, lon, alt) {
  valTime.textContent = (Number(tMs)/1000).toFixed(3) + " s";
  valSeq.textContent = seq ?? '-';
  valAx.textContent = (ax !== undefined ? Number(ax).toFixed(3) : '-');
  valAy.textContent = (ay !== undefined ? Number(ay).toFixed(3) : '-');
  valAz.textContent = (az !== undefined ? Number(az).toFixed(3) : '-');
  valPitch.textContent = (pitch !== undefined ? Number(pitch).toFixed(2) : '-');
  valRoll.textContent = (roll !== undefined ? Number(roll).toFixed(2) : '-');
  valYaw.textContent = (yaw !== undefined ? Number(yaw).toFixed(2) : '-');
  valTemp.textContent = (temp !== undefined ? Number(temp).toFixed(2) : '-');
  valVel.textContent = (vel !== undefined ? Number(vel).toFixed(2) : '-');
  valPress.textContent = (press !== undefined ? Number(press).toFixed(1) : '-');
  valLat.textContent = (lat !== undefined ? (Number(lat)/1e6).toFixed(6) : '-');
  valLon.textContent = (lon !== undefined ? (Number(lon)/1e6).toFixed(6) : '-');
  valAlt.textContent = (alt !== undefined ? Number(alt).toFixed(0) : '-');
}

function pushToCharts(ax, ay, az, pitch, roll, yaw, temp, vel, press, alt) {
  const maxPoints = 250;
  // velocity
  velChart.data.labels.push('');
  velChart.data.datasets[0].data.push(vel);
  if (velChart.data.labels.length > maxPoints) { velChart.data.labels.shift(); velChart.data.datasets.forEach(ds => ds.data.shift()); }
  velChart.update('none');

  // alt
  altChart.data.labels.push('');
  altChart.data.datasets[0].data.push(alt);
  if (altChart.data.labels.length > maxPoints) { altChart.data.labels.shift(); altChart.data.datasets.forEach(ds => ds.data.shift()); }
  altChart.update('none');

  // acceleration
  accChart.data.labels.push('');
  accChart.data.datasets[0].data.push(ax);
  accChart.data.datasets[1].data.push(ay);
  accChart.data.datasets[2].data.push(az);
  if (accChart.data.labels.length > maxPoints) { accChart.data.labels.shift(); accChart.data.datasets.forEach(ds => ds.data.shift()); }
  accChart.update('none');

  // orientation
  orientChart.data.labels.push('');
  orientChart.data.datasets[0].data.push(pitch);
  orientChart.data.datasets[1].data.push(roll);
  orientChart.data.datasets[2].data.push(yaw);
  if (orientChart.data.labels.length > maxPoints) { orientChart.data.labels.shift(); orientChart.data.datasets.forEach(ds => ds.data.shift()); }
  orientChart.update('none');

  // env
  envChart.data.labels.push('');
  envChart.data.datasets[0].data.push(temp);
  envChart.data.datasets[1].data.push(press);
  if (envChart.data.labels.length > maxPoints) { envChart.data.labels.shift(); envChart.data.datasets.forEach(ds => ds.data.shift()); }
  envChart.update('none');
}

// SignalR handler
connection.on("telemetry", (payload) => {
  try {
    if (payload.type === 'telemetry') {
      const t = Number(payload.t);
      const seq = Number(payload.seq);
      const ax = Number(payload.ax);
      const ay = Number(payload.ay);
      const az = Number(payload.az);
      const pitch = Number(payload.pitch);
      const roll = Number(payload.roll);
      const yaw = Number(payload.yaw);
      const temp = Number(payload.temp);
      const vel = Number(payload.vel);
      const press = Number(payload.press);
      const lat = Number(payload.lat);
      const lon = Number(payload.lon);
      const alt = Number(payload.alt);

      updateLatest(t, seq, ax, ay, az, pitch, roll, yaw, temp, vel, press, lat, lon, alt);
      pushToCharts(ax, ay, az, pitch, roll, yaw, temp, vel, press, alt);
    } else {
      dbg("RAW: " + JSON.stringify(payload));
    }
  } catch (e) {
    dbg("Error parsing payload: " + e);
  }
});

async function start(){
  createCharts();
  try {
    await connection.start();
    dbg("Connected to SignalR hub");
  } catch (err) {
    dbg("SignalR start failed: " + err);
    setTimeout(start, 2000);
  }
}
start();