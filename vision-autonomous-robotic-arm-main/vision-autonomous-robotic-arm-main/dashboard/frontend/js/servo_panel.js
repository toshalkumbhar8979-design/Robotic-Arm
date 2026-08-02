/* ==========================================================================
   SERVO CONTROL PANEL & ASSEMBLY HELPER (JS)
   ==========================================================================
   Project:  Vision-Based Autonomous Robotic Arm
   File:     servo_panel.js
   Location: dashboard/frontend/js/

   PURPOSE:
     Manages 6-DOF joint sliders, Lock 90° assembly mode, Home position button,
     Emergency Stop button, and Serial port connection management.

   RELATED DECISIONS:
     - Decision #5: Safety System (Emergency Stop)
     - Decision #6: Serial Protocol (Binary 6-byte streaming)
     - Decision #15: Arduino CLI Port Management
     - Decision #19: Active Physical Assembly Tool
     - Decision #20: Smooth S-Curve Trajectory Interpolation & Zero-Jerk Motion
   ========================================================================== */

const ServoPanel = {
  NUM_SERVOS: 6,
  sliders: [],
  valueDisplays: [],
  lastSendTime: 0,
  sendIntervalMs: 33, // ~30Hz send rate limit to prevent packet lag

  init() {
    this.cacheDOM();
    this.bindEvents();
    this.fetchPorts();
  },

  cacheDOM() {
    for (let i = 0; i < this.NUM_SERVOS; i++) {
      this.sliders[i] = document.getElementById(`sliderServo${i}`);
      this.valueDisplays[i] = document.getElementById(`valServo${i}`);
    }

    this.btnLock90 = document.getElementById('btnLock90');
    this.btnHome = document.getElementById('btnHome');
    this.btnSweep = document.getElementById('btnSweep');
    this.btnEstop = document.getElementById('btnEstop');
    this.btnResetEstop = document.getElementById('btnResetEstop');
    this.btnAutoConnect = document.getElementById('btnAutoConnect');
    this.btnConnect = document.getElementById('btnConnect');
    this.btnDisconnect = document.getElementById('btnDisconnect');
    this.portSelect = document.getElementById('portSelect');
  },

  bindEvents() {
    // Slider input listeners (manual drag throttled sending)
    for (let i = 0; i < this.NUM_SERVOS; i++) {
      if (this.sliders[i]) {
        this.sliders[i].addEventListener('input', () => {
          const val = this.sliders[i].value;
          if (this.valueDisplays[i]) {
            this.valueDisplays[i].textContent = `${val}°`;
          }
          this.throttledSendAngles();
        });
      }
    }

    // Lock All at 90° Button (Decision #19 & #20 — Smooth S-Curve Lock)
    if (this.btnLock90) {
      this.btnLock90.addEventListener('click', () => {
        App.sendWS('lock90');
        App.log('Action: Smooth transition to Lock All at 90°...');
      });
    }

    // Move to Home Position Button (Decision #20 — Smooth S-Curve Home)
    if (this.btnHome) {
      this.btnHome.addEventListener('click', () => {
        App.sendWS('home');
        App.log('Action: Smooth transition to Home Position...');
      });
    }

    // Run Joint Sweep Test Button (Decision #19 & #20 — Smooth S-Curve Sweep)
    if (this.btnSweep) {
      this.btnSweep.addEventListener('click', () => {
        App.sendWS('sweep');
        App.log('Action: Started Zero-Jerk Joint Sweep Test Routine...');
      });
    }

    // Emergency Stop Button (Decision #5)
    if (this.btnEstop) {
      this.btnEstop.addEventListener('click', () => {
        App.sendWS('estop');
        App.log('CRITICAL: EMERGENCY STOP ACTIVATED!');
      });
    }

    // Reset E-Stop Button
    if (this.btnResetEstop) {
      this.btnResetEstop.addEventListener('click', () => {
        App.sendWS('reset_estop');
        App.log('Action: Emergency Stop Reset.');
      });
    }

    // Serial Port Connection Management
    if (this.btnAutoConnect) {
      this.btnAutoConnect.addEventListener('click', () => this.autoConnectPort());
    }

    if (this.btnConnect) {
      this.btnConnect.addEventListener('click', () => this.connectSelectedPort());
    }

    if (this.btnDisconnect) {
      this.btnDisconnect.addEventListener('click', () => this.disconnectPort());
    }
  },

  /* Trigger single servo solo test (Decision #19) */
  testServo(index) {
    App.sendWS('test_servo', { index });
    App.log(`Action: Testing Servo ${index} in isolation...`);
  },

  /* Get current angles array from sliders */
  getAnglesFromSliders() {
    const angles = [];
    for (let i = 0; i < this.NUM_SERVOS; i++) {
      angles.push(parseInt(this.sliders[i] ? this.sliders[i].value : 90, 10));
    }
    return angles;
  },

  /* Set sliders from an array of angles */
  setSlidersFromAngles(angles) {
    for (let i = 0; i < Math.min(angles.length, this.NUM_SERVOS); i++) {
      if (this.sliders[i]) this.sliders[i].value = angles[i];
      if (this.valueDisplays[i]) this.valueDisplays[i].textContent = `${angles[i]}°`;
    }
  },

  /* Throttled WebSocket sender to maintain ~30Hz packet rate */
  throttledSendAngles() {
    const now = Date.now();
    if (now - this.lastSendTime >= this.sendIntervalMs) {
      this.lastSendTime = now;
      const angles = this.getAnglesFromSliders();
      App.sendWS('set_angles', { angles });
    }
  },

  /* Update sliders when backend sends status update */
  updateSlidersFromBackend(angles) {
    // Only update if user isn't actively dragging sliders
    if (document.activeElement && document.activeElement.type === 'range') {
      return;
    }
    this.setSlidersFromAngles(angles);
  },

  /* Fetch list of available serial ports via REST API */
  async fetchPorts() {
    try {
      const res = await fetch('/api/ports');
      if (!res.ok) return;
      const data = await res.json();
      
      if (this.portSelect) {
        this.portSelect.innerHTML = '';
        if (data.ports && data.ports.length > 0) {
          data.ports.forEach(p => {
            const opt = document.createElement('option');
            opt.value = p.port;
            opt.textContent = `${p.port} (${p.board_name})`;
            if (p.is_arduino) opt.selected = true;
            this.portSelect.appendChild(opt);
          });
        } else {
          this.portSelect.innerHTML = '<option value="">No Arduino ports detected</option>';
        }
      }
    } catch (e) {
      console.warn('Could not fetch serial ports:', e);
    }
  },

  async autoConnectPort() {
    try {
      App.log('Auto-detecting Arduino port...');
      const res = await fetch('/api/auto-connect', { method: 'POST' });
      const data = await res.json();
      if (res.ok) {
        App.log(`Success: ${data.message}`);
      } else {
        App.log(`Failed: ${data.detail || 'Could not auto-connect'}`);
      }
    } catch (e) {
      App.log(`Connection error: ${e.message}`);
    }
  },

  async connectSelectedPort() {
    const selectedPort = this.portSelect ? this.portSelect.value : '';
    if (!selectedPort) {
      App.log('Please select a serial port first.');
      return;
    }

    try {
      App.log(`Connecting to ${selectedPort}...`);
      const res = await fetch('/api/connect', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ port: selectedPort, baudrate: 115200 })
      });
      const data = await res.json();
      if (res.ok) {
        App.log(`Success: ${data.message}`);
      } else {
        App.log(`Error: ${data.detail || 'Connection failed'}`);
      }
    } catch (e) {
      App.log(`Connection error: ${e.message}`);
    }
  },

  async disconnectPort() {
    try {
      const res = await fetch('/api/disconnect', { method: 'POST' });
      const data = await res.json();
      App.log(`Disconnected: ${data.message}`);
    } catch (e) {
      App.log(`Disconnect error: ${e.message}`);
    }
  }
};

// Export to window scope so app.js can call ServoPanel.updateSlidersFromBackend
window.ServoPanel = ServoPanel;

document.addEventListener('DOMContentLoaded', () => {
  ServoPanel.init();
});
