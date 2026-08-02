/* ==========================================================================
   DIGITAL TWIN PANEL — LIVE GAZEBO SIM + 3D URDF TWIN (JS MODULE)
   ==========================================================================
   Project:  Vision-Based Autonomous Robotic Arm
   File:     digital_twin.js
   Location: dashboard/frontend/js/

   PURPOSE:
     Powers the Digital Twin panel:
       - Embeds the raw Gazebo GUI stream (noVNC on :6080) — interactable.
       - Loads /robot_description (latched) into a Three.js urdf-loader scene
         and drives it from live /joint_states (rosbridge :9090).
       - Joint sliders + gripper + pick-and-place publish goals back into the
         simulator (joint_trajectory_controller / finger_controller /
         /arm_pick/trigger).
       - Streams the table & end-effector Gazebo cameras (web_video_server :8080).
       - Optional browser Gamepad -> /joy bridge (PS5 / Xbox / F710) so
         controllers work without WSL usbipd passthrough.
   ========================================================================== */

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import * as UrdfLoaderModule from "urdf-loader";
import * as ROSLIB from "roslib";

const UrdfLoader = UrdfLoaderModule.default ?? UrdfLoaderModule.UrdfLoader;

const HOST = window.location.hostname || "localhost";
const ROS_URL = `ws://${HOST}:9090`;
const VNC_URL = `http://localhost:6080/vnc.html?autoconnect=1&resize=scale&reconnect=1`;
const VIDEO_URL = `http://${HOST}:8080/stream?topic=`;

// URDF joint limits (radians) — mirrors arm_6dof.urdf
const JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"];
const JOINT_LIMITS = {
  joint_1: [-6.2832, 6.2832],
  joint_2: [-2.3562, 2.3562],
  joint_3: [-2.3562, 2.3562],
  joint_4: [-3.1416, 3.1416],
  joint_5: [-2.3562, 2.3562],
  joint_6: [-3.1416, 3.1416],
};
const FINGER_JOINT = "left_finger_joint";
const FINGER_OPEN = 0.0;
const FINGER_CLOSE = 0.8;

const DigitalTwin = {
  ros: null,
  robot: null,
  scene: null,
  camera: null,
  controls: null,
  renderer: null,
  grid: null,
  sliders: {},
  sliderVals: {},
  dragging: {},
  lastTrajSend: 0,
  lastFingerSend: 0,
  jointPos: {},
  fingerPos: 0,
  gamepadBridgeOn: false,
  gamepadAnimId: null,

  init() {
    this.cacheDOM();
    this.buildJointSliders();
    this.setupScene();
    this.setupStream();
    this.connectRos();
    this.bindUi();
    this.bindGamepadEvents();
    this.startLoop();
  },

  cacheDOM() {
    this.rosPill = document.getElementById("dtRosPill");
    this.streamPill = document.getElementById("dtStreamPill");
    this.gazeboCard = document.getElementById("dtGazeboCard");
    this.twinCard = document.getElementById("dtTwinCard");
    this.tabGazebo = document.getElementById("dtTabGazebo");
    this.tabTwin = document.getElementById("dtTabTwin");
    this.vncFrame = document.getElementById("gazeboVncFrame");
    this.openStream = document.getElementById("dtOpenStream");
    this.resetCam = document.getElementById("dtResetCamera");
    this.toggleGrid = document.getElementById("dtToggleGrid");
    this.gripperSlider = document.getElementById("dtGripperSlider");
    this.gripperVal = document.getElementById("dtGripperVal");
    this.gripperOpen = document.getElementById("dtGripperOpen");
    this.gripperClose = document.getElementById("dtGripperClose");
    this.pickBtn = document.getElementById("dtPickBtn");
    this.pickStatus = document.getElementById("dtPickStatus");
    this.camTable = document.getElementById("dtCamTable");
    this.camEe = document.getElementById("dtCamEe");
    this.gpToggle = document.getElementById("dtGamepadToggle");
    this.gpName = document.getElementById("dtGamepadName");
    this.gpProfile = document.getElementById("dtGamepadProfile");
    this.gpAxes = document.getElementById("dtGamepadAxes");
    this.gpButtons = document.getElementById("dtGamepadButtons");
    this.slidersContainer = document.getElementById("dtJointSliders");
  },

  /* ---- Joint sliders (degrees, per URDF limits) -------------------------- */
  buildJointSliders() {
    if (!this.slidersContainer) return;
    this.slidersContainer.innerHTML = "";
    JOINT_NAMES.forEach((name, i) => {
      const [lo, hi] = JOINT_LIMITS[name];
      const wrap = document.createElement("div");
      wrap.innerHTML = `
        <div style="display: flex; justify-content: space-between; margin-bottom: 2px;">
          <span style="font-weight: 600; font-size: 0.85rem; color: var(--text-main);">θ${i + 1} — ${name}</span>
          <span class="servo-value-display" id="dtSliderVal${i}" style="font-size: 0.9rem;">0°</span>
        </div>
        <input type="range" id="dtSlider${i}" min="${(lo * 180 / Math.PI).toFixed(1)}" max="${(hi * 180 / Math.PI).toFixed(1)}" value="0" step="1">`;
      this.slidersContainer.appendChild(wrap);
      const slider = wrap.querySelector("input");
      const val = wrap.querySelector(".servo-value-display");
      this.sliders[name] = slider;
      this.sliderVals[name] = val;

      slider.addEventListener("input", () => {
        this.dragging[name] = true;
        val.textContent = `${slider.value}°`;
        this.sendTrajectory(name);
      });
      slider.addEventListener("change", () => {
        this.dragging[name] = false;
      });
    });
  },

  sendTrajectory(joint) {
    const now = Date.now();
    if (now - this.lastTrajSend < 150) return;
    this.lastTrajSend = now;
    if (!this.ros) return;
    const rad = parseFloat(this.sliders[joint].value) * Math.PI / 180;
    const topic = new ROSLIB.Topic({
      ros: this.ros,
      name: "/joint_trajectory_controller/joint_trajectory",
      messageType: "trajectory_msgs/JointTrajectory",
    });
    topic.publish({
      joint_names: [joint],
      points: [{ positions: [rad], time_from_start: { sec: 1, nanosec: 0 } }],
    });
  },

  /* ---- 3D scene ---------------------------------------------------------- */
  setupScene() {
    const viewer = document.getElementById("digitalTwinViewport");
    if (!viewer) return;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x2a2624);

    this.camera = new THREE.PerspectiveCamera(
      50, viewer.clientWidth / viewer.clientHeight, 0.01, 50);
    this.camera.position.set(1.1, -0.9, 0.9);

    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setSize(viewer.clientWidth, viewer.clientHeight);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    viewer.appendChild(this.renderer.domElement);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.target.set(0.15, 0.0, 0.5);
    this.controls.update();

    this.scene.add(new THREE.AmbientLight(0xffffff, 0.7));
    const sun = new THREE.DirectionalLight(0xffffff, 1.1);
    sun.position.set(1.5, 2, 3);
    this.scene.add(sun);

    this.grid = new THREE.GridHelper(2.4, 12, 0xd0c8b8, 0x8c8275);
    this.grid.position.y = -0.001;
    this.scene.add(this.grid);
  },

  /* ---- Stream status + iframe ------------------------------------------- */
  setupStream() {
    this.probeStream();
    if (this.vncFrame) this.vncFrame.src = VNC_URL;
    if (this.openStream) {
      this.openStream.addEventListener("click", () => {
        window.open("http://localhost:6080/vnc.html?autoconnect=1&resize=scale", "_blank");
      });
    }
    setInterval(() => this.probeStream(), 15000);
  },

  async probeStream() {
    try {
      const res = await fetch("http://localhost:6080/", { mode: "no-cors" });
      // no-cors gives an opaque response; a fetch that resolves = server up
      if (this.streamPill) {
        this.streamPill.className = "status-pill connected";
        this.streamPill.innerHTML = '<span class="status-dot"></span><span>Gazebo Stream: Live (:6080)</span>';
      }
    } catch (e) {
      if (this.streamPill) {
        this.streamPill.className = "status-pill";
        this.streamPill.innerHTML = '<span class="status-dot"></span><span>Gazebo Stream: Offline (start noVNC stream)</span>';
      }
    }
  },

  /* ---- ROS --------------------------------------------------------------- */
  connectRos() {
    this.ros = new ROSLIB.Ros({ url: ROS_URL });

    this.ros.on("connection", () => {
      this.setRosPill(true, "ROS Bridge: Connected (:9090)");
      this.loadRobot();
      this.setupPickService();
    });
    this.ros.on("error", () => this.setRosPill(false, "ROS Bridge: Connection Error"));
    this.ros.on("close", () => this.setRosPill(false, "ROS Bridge: Disconnected"));

    this.subscribeJointStates();
  },

  setRosPill(ok, text) {
    if (!this.rosPill) return;
    this.rosPill.className = ok ? "status-pill connected" : "status-pill";
    this.rosPill.innerHTML = `<span class="status-dot"></span><span>${text}</span>`;
  },

  subscribeJointStates() {
    const onReady = () => {
      if (!this.ros) return;
      const jsTopic = new ROSLIB.Topic({
        ros: this.ros, name: "/joint_states", messageType: "sensor_msgs/JointState",
      });
      jsTopic.subscribe((m) => this.onJointStates(m));
    };
    // ros may not be connected at module init; hook both paths
    if (this.ros) {
      if (this.ros.isConnected) onReady();
      else this.ros.on("connection", onReady);
    }
  },

  onJointStates(m) {
    m.name.forEach((n, i) => {
      const pos = m.position[i];
      this.jointPos[n] = pos;
      if (n === FINGER_JOINT) this.fingerPos = pos;

      if (this.robot && this.robot.setJointValue) {
        try { this.robot.setJointValue(n, pos); } catch (e) { /* mimic joints etc. */ }
      }
      const idx = JOINT_NAMES.indexOf(n);
      if (idx >= 0) {
        const deg = (pos * 180 / Math.PI).toFixed(1);
        const overlay = document.getElementById(`dtVal${idx}`);
        if (overlay) overlay.textContent = `${deg}°`;
        const slider = this.sliders[n];
        const val = this.sliderVals[n];
        if (slider && !this.dragging[n]) {
          slider.value = deg;
          if (val) val.textContent = `${deg}°`;
        }
      }
      if (n === FINGER_JOINT && this.gripperSlider && !this.gripperSlider.dataset.dragging) {
        const pct = Math.round(((pos - FINGER_OPEN) / (FINGER_CLOSE - FINGER_OPEN)) * 100);
        this.gripperSlider.value = Math.max(0, Math.min(100, pct));
        if (this.gripperVal) this.gripperVal.textContent = `${this.gripperSlider.value}%`;
      }
    });
  },

  async loadRobot() {
    try {
      const descTopic = new ROSLIB.Topic({
        ros: this.ros, name: "/robot_description", messageType: "std_msgs/String",
        queue_length: 1, latch: true,
      });
      descTopic.subscribe(async (msg) => {
        try {
          const loader = new UrdfLoader();
          this.robot = await loader.load({ string: msg.data });
          this.robot.traverse((o) => { if (o.isMesh) o.castShadow = true; });
          this.scene.add(this.robot);
        } catch (err) {
          console.warn("Digital twin URDF load failed:", err);
        }
      });
    } catch (e) {
      console.warn("Could not subscribe /robot_description:", e);
    }
  },

  setupPickService() {
    if (!this.pickBtn) return;
    this.pickBtn.addEventListener("click", () => {
      if (!this.ros) return;
      const svc = new ROSLIB.Service({
        ros: this.ros, name: "/arm_pick/trigger", serviceType: "std_srvs/Trigger",
      });
      svc.callService(new ROSLIB.ServiceRequest(), (resp) => {
        if (this.pickStatus) this.pickStatus.textContent = resp.message || "Pick triggered";
      });
    });
    const statusTopic = new ROSLIB.Topic({
      ros: this.ros, name: "/arm_pick/status", messageType: "std_msgs/String",
    });
    statusTopic.subscribe((m) => {
      if (this.pickStatus) this.pickStatus.textContent = m.data;
    });
  },

  /* ---- UI wiring --------------------------------------------------------- */
  bindUi() {
    if (this.tabGazebo) {
      this.tabGazebo.addEventListener("click", () => {
        this.gazeboCard.style.display = "";
        this.twinCard.style.display = "none";
        this.tabGazebo.className = "btn btn-primary";
        this.tabTwin.className = "btn btn-secondary";
      });
    }
    if (this.tabTwin) {
      this.tabTwin.addEventListener("click", () => {
        this.gazeboCard.style.display = "none";
        this.twinCard.style.display = "";
        this.tabTwin.className = "btn btn-primary";
        this.tabGazebo.className = "btn btn-secondary";
        if (this.renderer) this.renderer.setSize(this.twinCard.clientWidth - 48, 500);
      });
    }

    if (this.resetCam) {
      this.resetCam.addEventListener("click", () => {
        this.camera.position.set(1.1, -0.9, 0.9);
        this.controls.target.set(0.15, 0.0, 0.5);
        this.controls.update();
      });
    }
    if (this.toggleGrid) {
      this.toggleGrid.addEventListener("click", () => {
        this.grid.visible = !this.grid.visible;
      });
    }

    if (this.gripperSlider) {
      this.gripperSlider.addEventListener("input", () => {
        this.gripperSlider.dataset.dragging = "1";
        const pos = FINGER_OPEN + (FINGER_CLOSE - FINGER_OPEN) * this.gripperSlider.value / 100;
        if (this.gripperVal) this.gripperVal.textContent = `${this.gripperSlider.value}%`;
        this.sendFinger(pos);
      });
      this.gripperSlider.addEventListener("change", () => {
        this.gripperSlider.dataset.dragging = "";
      });
    }
    if (this.gripperOpen) {
      this.gripperOpen.addEventListener("click", () => this.sendFinger(FINGER_OPEN));
    }
    if (this.gripperClose) {
      this.gripperClose.addEventListener("click", () => this.sendFinger(FINGER_CLOSE));
    }

    if (this.camTable) this.camTable.src = VIDEO_URL + "/camera_table/image_raw";
    if (this.camEe) this.camEe.src = VIDEO_URL + "/camera_ee/image_raw";
  },

  sendFinger(position) {
    const now = Date.now();
    if (now - this.lastFingerSend < 100) return;
    this.lastFingerSend = now;
    if (!this.ros) return;
    const topic = new ROSLIB.Topic({
      ros: this.ros, name: "/finger_controller/commands",
      messageType: "std_msgs/Float64MultiArray",
    });
    topic.publish({ data: [position] });
  },

  /* ---- Gamepad -> /joy bridge ------------------------------------------- */
  bindGamepadEvents() {
    if (this.gpToggle) {
      this.gpToggle.addEventListener("click", () => {
        this.gamepadBridgeOn = !this.gamepadBridgeOn;
        this.gpToggle.textContent = `Bridge: ${this.gamepadBridgeOn ? "ON" : "OFF"}`;
        this.gpToggle.className = this.gamepadBridgeOn ? "btn btn-primary" : "btn btn-secondary";
        if (this.gamepadBridgeOn) this.startGamepadBridge();
      });
    }
    window.addEventListener("gamepadconnected", (e) => this.showGamepad(e.gamepad));
    window.addEventListener("gamepaddisconnected", () => {
      if (this.gpName) this.gpName.textContent = "—";
      if (this.gpProfile) this.gpProfile.textContent = "—";
      if (this.gpAxes) this.gpAxes.textContent = "—";
      if (this.gpButtons) this.gpButtons.textContent = "—";
    });
  },

  detectGamepadProfile(id) {
    if (/dualsense|wireless controller|playstation|ps5|dual shock/i.test(id)) return "ps5";
    if (/xbox|series|xinput|gamepad.*microsoft/i.test(id)) return "xbox";
    return "f710";
  },

  showGamepad(gp) {
    if (this.gpName) this.gpName.textContent = gp.id;
    if (this.gpAxes) this.gpAxes.textContent = gp.axes.length;
    if (this.gpButtons) this.gpButtons.textContent = gp.buttons.length;
    if (this.gpProfile) this.gpProfile.textContent = this.detectGamepadProfile(gp.id);
  },

  startGamepadBridge() {
    if (this.gamepadAnimId) cancelAnimationFrame(this.gamepadAnimId);
    let last = 0;
    const loop = () => {
      if (!this.gamepadBridgeOn) return;
      const gps = navigator.getGamepads ? navigator.getGamepads() : [];
      const gp = Array.from(gps).find((g) => g && g.connected);
      if (gp) {
        this.showGamepad(gp);
        const now = performance.now();
        if (now - last > 33 && this.ros) {
          last = now;
          const topic = new ROSLIB.Topic({
            ros: this.ros, name: "/joy", messageType: "sensor_msgs/Joy",
          });
          topic.publish({
            axes: Array.from(gp.axes, (v) => (Math.abs(v) < 0.05 ? 0 : v)),
            buttons: Array.from(gp.buttons, (b) => (b.pressed ? 1 : 0)),
          });
        }
      }
      this.gamepadAnimId = requestAnimationFrame(loop);
    };
    loop();
  },

  /* ---- render loop ------------------------------------------------------- */
  startLoop() {
    const tick = () => {
      if (this.renderer) {
        this.controls.update();
        this.renderer.render(this.scene, this.camera);
      }
      requestAnimationFrame(tick);
    };
    tick();
  },
};

document.addEventListener("DOMContentLoaded", () => {
  DigitalTwin.init();
});
