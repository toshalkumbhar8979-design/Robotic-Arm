/* ==========================================================================
   TELEOPERATION PANEL & PS5 DUALSENSE CONTROLLER INTEGRATION (JS)
   ==========================================================================
   Project:  Vision-Based Autonomous Robotic Arm
   File:     teleop_panel.js
   Location: dashboard/frontend/js/

   PURPOSE:
     Reads PS5 DualSense inputs live via browser Gamepad API. Visualizes stick
     positions, triggers, and button presses, and previews the Cartesian Inverse
     Kinematics (IK) mapping pipeline for Phase C demonstration collection.

   RELATED DECISIONS:
     - Decision #4: Cartesian IK Teleoperation (PS5 joystick controls X,Y,Z)
     - Decision #5: Safety System
   ========================================================================== */

const TeleopPanel = {
  gamepadIndex: null,
  animFrameId: null,

  buttonNames: [
    'Cross (×)', 'Circle (○)', 'Square (□)', 'Triangle (△)',
    'L1', 'R1', 'L2', 'R2',
    'Share', 'Options', 'L3', 'R3',
    'D-Pad ↑', 'D-Pad ↓', 'D-Pad ←', 'D-Pad →',
    'PS', 'Touchpad'
  ],

  buttonNamesByProfile: {
    ps5: [
      'Cross (×)', 'Circle (○)', 'Square (□)', 'Triangle (△)',
      'L1', 'R1', 'L2', 'R2',
      'Share', 'Options', 'L3', 'R3',
      'D-Pad ↑', 'D-Pad ↓', 'D-Pad ←', 'D-Pad →',
      'PS', 'Touchpad'
    ],
    xbox: [
      'A', 'B', 'X', 'Y',
      'LB', 'RB', 'LT', 'RT',
      'Back', 'Start', 'L3', 'R3',
      'D-Pad ↑', 'D-Pad ↓', 'D-Pad ←', 'D-Pad →',
      'Guide', 'Share'
    ],
    f710: [
      'A', 'B', 'X', 'Y',
      'LB', 'RB', 'Back', 'Start',
      'Logitech', 'L3', 'R3'
    ]
  },

  detectProfile(id) {
    if (/dualsense|wireless controller|playstation|ps5|dual shock/i.test(id)) return 'ps5';
    if (/xbox|series|xinput|microsoft/i.test(id)) return 'xbox';
    return 'f710';
  },

  init() {
    this.cacheDOM();
    this.buildButtonIndicators();
    this.bindEvents();
    this.startLoop();
    this.loadKinematicsConfig();
  },

  cacheDOM() {
    this.statusPill = document.getElementById('ps5StatusPill');
    this.statusText = document.getElementById('ps5StatusText');
    this.leftDot = document.getElementById('ps5LeftDot');
    this.rightDot = document.getElementById('ps5RightDot');
    this.leftVal = document.getElementById('ps5LeftVal');
    this.rightVal = document.getElementById('ps5RightVal');
    this.l2Fill = document.getElementById('ps5L2Fill');
    this.r2Fill = document.getElementById('ps5R2Fill');
    this.l1Fill = document.getElementById('ps5L1Fill');
    this.r1Fill = document.getElementById('ps5R1Fill');
    this.l2Val = document.getElementById('ps5L2Val');
    this.r2Val = document.getElementById('ps5R2Val');
    this.l1Val = document.getElementById('ps5L1Val');
    this.r1Val = document.getElementById('ps5R1Val');
    this.buttonsGrid = document.getElementById('ps5ButtonsGrid');
    this.axesList = document.getElementById('ps5AxesList');
    this.btnSaveKinematics = document.getElementById('btnSaveKinematics');

    if (this.axesList) {
      this.axesList.innerHTML = '<div style="font-family: var(--font-mono); font-size: 0.8rem; color: var(--text-muted); padding: 12px 0;">No controller connected.<br>Press any button on your PS5 / Xbox / F710 controller to display live axes.</div>';
    }
  },

  buildButtonIndicators() {
    if (!this.buttonsGrid) return;
    this.buttonsGrid.innerHTML = '';
    this.buttonNames.forEach((name, i) => {
      const div = document.createElement('div');
      div.className = 'btn-indicator';
      div.id = `ps5Btn${i}`;
      div.textContent = name;
      this.buttonsGrid.appendChild(div);
    });
  },

  bindEvents() {
    if (this.btnSaveKinematics) {
      this.btnSaveKinematics.addEventListener('click', () => this.saveKinematicsConfig());
    }

    window.addEventListener('gamepadconnected', (e) => {
      this.gamepadIndex = e.gamepad.index;
      this.buttonNames = this.buttonNamesByProfile[this.detectProfile(e.gamepad.id)]
        || this.buttonNamesByProfile.f710;
      this.buildButtonIndicators();
      if (this.statusPill && this.statusText) {
        this.statusPill.className = 'status-pill connected';
        this.statusText.textContent = `Connected: ${e.gamepad.id} (${this.detectProfile(e.gamepad.id)} layout)`;
      }
      App.log(`Gamepad Connected: ${e.gamepad.id} (${this.detectProfile(e.gamepad.id)})`);
    });

    window.addEventListener('gamepaddisconnected', (e) => {
      if (e.gamepad.index === this.gamepadIndex) {
        this.gamepadIndex = null;
        if (this.statusPill && this.statusText) {
          this.statusPill.className = 'status-pill';
          this.statusText.textContent = 'Gamepad Disconnected';
        }
        App.log('Gamepad Disconnected.');
      }
    });
  },

  async loadKinematicsConfig() {
    try {
      const res = await fetch('/api/kinematics');
      if (!res.ok) return;
      const cfg = await res.json();
      
      const l1 = document.getElementById('inputL1');
      const l2 = document.getElementById('inputL2');
      const l3 = document.getElementById('inputL3');
      const l4 = document.getElementById('inputL4');
      const gc = document.getElementById('angleGripperClosed');
      const go = document.getElementById('angleGripperOpen');

      if (l1 && cfg.L1) l1.value = cfg.L1;
      if (l2 && cfg.L2) l2.value = cfg.L2;
      if (l3 && cfg.L3) l3.value = cfg.L3;
      if (l4 && cfg.L4) l4.value = cfg.L4;
      if (gc && cfg.gripper_closed) gc.value = cfg.gripper_closed;
      if (go && cfg.gripper_open) go.value = cfg.gripper_open;

      if (cfg.offsets) {
        for (let i = 0; i < 5; i++) {
          const el = document.getElementById(`offsetServo${i}`);
          if (el) el.value = cfg.offsets[i] || 0;
        }
      }
    } catch (e) {
      console.warn('Could not load kinematics config:', e);
    }
  },

  async saveKinematicsConfig() {
    const l1 = parseFloat(document.getElementById('inputL1')?.value || 10.0);
    const l2 = parseFloat(document.getElementById('inputL2')?.value || 14.0);
    const l3 = parseFloat(document.getElementById('inputL3')?.value || 12.0);
    const l4 = parseFloat(document.getElementById('inputL4')?.value || 8.0);
    const gc = parseInt(document.getElementById('angleGripperClosed')?.value || 10, 10);
    const go = parseInt(document.getElementById('angleGripperOpen')?.value || 90, 10);

    const offsets = [];
    for (let i = 0; i < 5; i++) {
      offsets.push(parseInt(document.getElementById(`offsetServo${i}`)?.value || 0, 10));
    }

    try {
      const res = await fetch('/api/kinematics', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ L1: l1, L2: l2, L3: l3, L4: l4, offsets, gripper_closed: gc, gripper_open: go })
      });
      if (res.ok) {
        App.log(`Saved Kinematic Calibration: L1=${l1}cm, L2=${l2}cm, L3=${l3}cm, L4=${l4}cm`);
      }
    } catch (e) {
      App.log(`Failed to save kinematic calibration: ${e.message}`);
    }
  },

  applyDeadzone(val, deadzone = 0.08) {
    return Math.abs(val) < deadzone ? 0 : val;
  },

  startLoop() {
    const update = () => {
      if (this.gamepadIndex !== null) {
        const gp = navigator.getGamepads()[this.gamepadIndex];
        if (gp) {
          this.updateSticks(gp);
          this.updateTriggers(gp);
          this.updateButtons(gp);
          this.updateAxes(gp);
        }
      }
      this.animFrameId = requestAnimationFrame(update);
    };
    update();
  },

  updateSticks(gp) {
    const lx = this.applyDeadzone(gp.axes[0] || 0);
    const ly = this.applyDeadzone(gp.axes[1] || 0);
    const rx = this.applyDeadzone(gp.axes[2] || 0);
    const ry = this.applyDeadzone(gp.axes[3] || 0);

    const maxOffset = 50;
    if (this.leftDot) {
      this.leftDot.style.left = `calc(50% + ${lx * maxOffset}px)`;
      this.leftDot.style.top = `calc(50% + ${ly * maxOffset}px)`;
    }
    if (this.rightDot) {
      this.rightDot.style.left = `calc(50% + ${rx * maxOffset}px)`;
      this.rightDot.style.top = `calc(50% + ${ry * maxOffset}px)`;
    }

    if (this.leftVal) this.leftVal.textContent = `X: ${lx.toFixed(2)} Y: ${ly.toFixed(2)}`;
    if (this.rightVal) this.rightVal.textContent = `X: ${rx.toFixed(2)} Y: ${ry.toFixed(2)}`;
  },

  updateTriggers(gp) {
    const l2 = gp.buttons[6] ? gp.buttons[6].value : 0;
    const r2 = gp.buttons[7] ? gp.buttons[7].value : 0;
    const l1 = gp.buttons[4] ? gp.buttons[4].value : 0;
    const r1 = gp.buttons[5] ? gp.buttons[5].value : 0;

    if (this.l2Fill) this.l2Fill.style.width = `${l2 * 100}%`;
    if (this.r2Fill) this.r2Fill.style.width = `${r2 * 100}%`;
    if (this.l1Fill) this.l1Fill.style.width = `${l1 * 100}%`;
    if (this.r1Fill) this.r1Fill.style.width = `${r1 * 100}%`;

    if (this.l2Val) this.l2Val.textContent = `${Math.round(l2 * 100)}%`;
    if (this.r2Val) this.r2Val.textContent = `${Math.round(r2 * 100)}%`;
    if (this.l1Val) this.l1Val.textContent = `${Math.round(l1 * 100)}%`;
    if (this.r1Val) this.r1Val.textContent = `${Math.round(r1 * 100)}%`;
  },

  updateButtons(gp) {
    for (let i = 0; i < Math.min(gp.buttons.length, this.buttonNames.length); i++) {
      const el = document.getElementById(`ps5Btn${i}`);
      if (el) {
        if (gp.buttons[i].pressed) {
          el.style.backgroundColor = 'var(--accent-primary)';
          el.style.borderColor = 'var(--accent-primary)';
          el.style.color = '#FAF7F2';
        } else {
          el.style.backgroundColor = 'var(--bg-page)';
          el.style.borderColor = 'var(--border-subtle)';
          el.style.color = 'var(--text-muted)';
        }
      }
    }
  },

  updateAxes(gp) {
    if (!this.axesList) return;
    if (this.axesList.children.length !== gp.axes.length) {
      this.axesList.innerHTML = '';
      for (let i = 0; i < gp.axes.length; i++) {
        const row = document.createElement('div');
        row.className = 'axis-row';
        row.style.cssText = 'display: flex; align-items: center; gap: 10px; margin-bottom: 6px;';
        row.innerHTML = `
          <span style="font-family: var(--font-mono); font-size: 0.75rem; color: var(--text-muted); width: 50px;">Axis ${i}</span>
          <div style="flex: 1; height: 8px; background: var(--bg-page); border: 1px solid var(--border-subtle); border-radius: 4px; overflow: hidden; position: relative;">
            <div id="ps5AxisBar${i}" style="position: absolute; height: 100%; background: var(--accent-primary); transition: all 0.05s linear;"></div>
          </div>
          <span id="ps5AxisVal${i}" style="font-family: var(--font-mono); font-size: 0.75rem; color: var(--text-main); width: 45px; text-align: right;">0.00</span>
        `;
        this.axesList.appendChild(row);
      }
    }

    for (let i = 0; i < gp.axes.length; i++) {
      const val = gp.axes[i];
      const pct = ((val + 1) / 2) * 100;
      const bar = document.getElementById(`ps5AxisBar${i}`);
      const valEl = document.getElementById(`ps5AxisVal${i}`);
      if (bar) {
        bar.style.left = val >= 0 ? '50%' : `${pct}%`;
        bar.style.width = `${Math.abs(val) * 50}%`;
      }
      if (valEl) valEl.textContent = val.toFixed(2);
    }
  }
};

document.addEventListener('DOMContentLoaded', () => {
  TeleopPanel.init();
});
