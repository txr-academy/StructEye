const connectBtn = document.getElementById('connect-btn');
const sessionInput = document.getElementById('session-input');
const setupPanel = document.getElementById('setup-panel');
const dashboardMain = document.getElementById('dashboard-main');
const sessionDisplay = document.getElementById('session-display');
const headerTime = document.getElementById('header-time');
const connError = document.getElementById('conn-error');

const summaryTbody = document.getElementById('summary-tbody');
const nodesGrid = document.getElementById('nodes-grid');
const tableRowTemplate = document.getElementById('table-row-template');
const nodeCardTemplate = document.getElementById('node-card-template');

let mqttClient = null;
const nodesData = {};

let totalRx = 0;
let totalDropped = 0;

function formatNum(num) {
    return (num >= 0 ? '+' : '') + num.toFixed(2);
}

function generateAsciiBar(val, maxVal) {
    const bars = 20;
    const filled = Math.min(Math.max(Math.floor((val / maxVal) * bars), 0), bars);
    let barStr = '';
    for (let i = 0; i < filled; i++) barStr += '█';
    for (let i = filled; i < bars; i++) barStr += '░';
    return barStr;
}

function updateNodeUI(data) {
    const nid = data.node_id;
    
    if (!nodesData[nid]) {
        nodesData[nid] = { lastSeen: new Date() };
        
        // Create table row
        const rowClone = tableRowTemplate.content.cloneNode(true);
        const row = rowClone.querySelector('tr');
        row.id = `row-${nid}`;
        row.querySelector('.node-name').textContent = `Node ${nid}`;
        summaryTbody.appendChild(row);

        // Create card
        const cardClone = nodeCardTemplate.content.cloneNode(true);
        const card = cardClone.querySelector('.node-card');
        card.id = `card-${nid}`;
        card.querySelector('.node-id').textContent = nid;
        nodesGrid.appendChild(card);
    }
    
    nodesData[nid].lastSeen = new Date();
    
    // Formatting
    const alertColor = data.alert_level === 0 ? 'green' : (data.alert_level === 1 ? 'yellow' : 'red');
    const alertIcon = data.alert_level === 0 ? '✅ OK' : (data.alert_level === 1 ? '⚠️ WARN' : '🚨 CRIT');
    const borderClass = data.alert_level === 0 ? 'ok-border' : (data.alert_level === 1 ? 'warn-border' : 'critical-border');
    
    const vibAscii = generateAsciiBar(data.vibration_m_s2, 10.0);
    const tiltAscii = generateAsciiBar(data.tilt_deg, 30.0);
    
    const accelStr = `${formatNum(data.accel.x)} / ${formatNum(data.accel.y)} / ${formatNum(data.accel.z)}`;
    const gyroStr = `${formatNum(data.gyro.x)} / ${formatNum(data.gyro.y)} / ${formatNum(data.gyro.z)}`;

    // Update Table Row
    const row = document.getElementById(`row-${nid}`);
    row.classList.remove('updated'); void row.offsetWidth; row.classList.add('updated');
    
    row.querySelector('.seq-val').textContent = data.sequence;
    row.querySelector('.vib-bar').textContent = vibAscii;
    row.querySelector('.vib-val').textContent = data.vibration_m_s2.toFixed(2);
    row.querySelector('.tilt-bar').textContent = tiltAscii;
    row.querySelector('.tilt-val').textContent = data.tilt_deg.toFixed(2);
    
    row.querySelector('.col-accel').textContent = accelStr;
    row.querySelector('.col-gyro').textContent = gyroStr;
    
    row.querySelector('.rx-val').textContent = data.packet_count;
    row.querySelector('.missed-val').textContent = data.missed_packets;
    
    const rowAlert = row.querySelector('.alert-badge');
    rowAlert.textContent = alertIcon;
    rowAlert.className = `col-alert alert-badge ${alertColor}`;
    
    // Update Card
    const card = document.getElementById(`card-${nid}`);
    card.classList.remove('updated'); void card.offsetWidth; card.classList.add('updated');
    card.className = `node-card ${borderClass} updated`;
    
    card.querySelectorAll('.alert-badge').forEach(b => {
        b.textContent = alertIcon;
        b.className = `alert-badge ${alertColor}`;
    });
    
    card.querySelector('.seq-val').textContent = data.sequence;
    card.querySelector('.rx-val').textContent = data.packet_count;
    card.querySelector('.missed-val').textContent = data.missed_packets;
    
    card.querySelector('.vib-bar').textContent = vibAscii;
    card.querySelector('.vib-val').textContent = data.vibration_m_s2.toFixed(2);
    card.querySelector('.tilt-bar').textContent = tiltAscii;
    card.querySelector('.tilt-val').textContent = data.tilt_deg.toFixed(2);
    
    card.querySelector('.accel-x').textContent = formatNum(data.accel.x) + ' m/s²';
    card.querySelector('.accel-y').textContent = formatNum(data.accel.y) + ' m/s²';
    card.querySelector('.accel-z').textContent = formatNum(data.accel.z) + ' m/s²';
    
    card.querySelector('.gyro-x').textContent = formatNum(data.gyro.x) + ' °/s';
    card.querySelector('.gyro-y').textContent = formatNum(data.gyro.y) + ' °/s';
    card.querySelector('.gyro-z').textContent = formatNum(data.gyro.z) + ' °/s';
    
    // Stats calc
    totalRx += 1;
    if (data.missed_packets > 0) totalDropped += data.missed_packets; // approximate
    updateStats();
}

function updateStats() {
    const active = Object.keys(nodesData).length;
    document.getElementById('stat-rx').textContent = totalRx;
    document.getElementById('stat-dropped').textContent = totalDropped;
    document.getElementById('stat-active').textContent = active;
    
    document.getElementById('stat-valid').textContent = totalRx;
    const rate = totalRx === 0 ? 0 : (totalDropped / (totalRx + totalDropped)) * 100;
    document.getElementById('stat-rate').textContent = rate.toFixed(1) + '%';
    document.getElementById('stat-health').textContent = rate > 5 ? 'POOR' : 'EXCELLENT';
}

// Clock & Timers
setInterval(() => {
    const now = new Date();
    headerTime.textContent = now.toISOString().replace('T', ' ').substring(0, 19);
    
    Object.keys(nodesData).forEach(nid => {
        const diff = Math.floor((now - nodesData[nid].lastSeen) / 1000);
        let text = diff < 60 ? `${diff}s ago` : `${Math.floor(diff/60)}m ago`;
        let color = diff < 15 ? 'green' : (diff < 35 ? 'yellow' : 'red');
        
        document.querySelectorAll(`#row-${nid} .last-seen, #card-${nid} .last-seen`).forEach(el => {
            el.textContent = text;
            el.className = `last-seen ${color}`;
        });
    });
}, 1000);

connectBtn.addEventListener('click', () => {
    const sessionId = sessionInput.value.trim();
    if (!sessionId) {
        connError.textContent = "Please enter a valid Session ID.";
        return;
    }
    connError.textContent = "";
    
    const clientId = "web_" + Math.random().toString(16).substr(2, 8);
    const brokerUrl = "wss://broker.emqx.io:8084/mqtt";
    
    sessionDisplay.textContent = "Connecting...";
    
    mqttClient = mqtt.connect(brokerUrl, {
        clientId: clientId,
        keepalive: 60,
        reconnectPeriod: 1000
    });

    mqttClient.on('connect', () => {
        sessionDisplay.textContent = sessionId;
        setupPanel.classList.add('hidden');
        dashboardMain.classList.remove('hidden');
        
        const topic = `${sessionId}/telemetry/+`;
        mqttClient.subscribe(topic);
    });

    mqttClient.on('message', (topic, message) => {
        try {
            const data = JSON.parse(message.toString());
            updateNodeUI(data);
        } catch (e) {
            console.error("JSON Error:", e);
        }
    });

    mqttClient.on('error', (err) => {
        connError.textContent = "Connection Error: " + err.message;
    });

    mqttClient.on('close', () => {
        sessionDisplay.textContent = "Disconnected";
    });
});
