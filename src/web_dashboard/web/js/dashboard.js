import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import * as UrdfLoaderModule from "urdf-loader";
import * as ROSLIB from "roslib";

const UrdfLoader = UrdfLoaderModule.default ?? UrdfLoaderModule.UrdfLoader;
const JointStateHandler = UrdfLoaderModule.JointStateHandler;

const HOST = window.location.hostname || "localhost";
const ROS_URL = `ws://${HOST}:9090`;
const VIDEO_URL = `http://${HOST}:8080/stream?topic=`;

const connEl = document.getElementById("conn");
const statusEl = document.getElementById("status");
const pickBtn = document.getElementById("pick-btn");
const jointsEl = document.getElementById("joints");
const camTable = document.getElementById("cam-table");
const camEe = document.getElementById("cam-ee");

// --- 3D scene -----------------------------------------------------------
const viewer = document.getElementById("viewer");
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0f1216);

const camera = new THREE.PerspectiveCamera(
  50, viewer.clientWidth / viewer.clientHeight, 0.01, 50);
camera.position.set(1.1, -0.9, 0.9);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(viewer.clientWidth, viewer.clientHeight);
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
viewer.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(0.15, 0.0, 0.5);
controls.update();

scene.add(new THREE.AmbientLight(0xffffff, 0.7));
const sun = new THREE.DirectionalLight(0xffffff, 1.1);
sun.position.set(1.5, 2, 3);
scene.add(sun);

const grid = new THREE.GridHelper(2.4, 12, 0x2a3340, 0x1c2330);
grid.position.y = -0.001;
scene.add(grid);

// --- ROS -----------------------------------------------------------------
const ros = new ROSLIB.Ros({ url: ROS_URL });
ros.on("connection", () => {
  connEl.textContent = "connected";
  connEl.className = "conn ok";
});
ros.on("error", () => {
  connEl.textContent = "connection error";
  connEl.className = "conn error";
});
ros.on("close", () => {
  connEl.textContent = "disconnected";
  connEl.className = "conn error";
  statusEl.textContent = "lost connection to ROS";
  statusEl.className = "status error";
});

function setStatus(text) {
  statusEl.textContent = text;
  statusEl.className = "status";
  if (/^ERROR/i.test(text)) statusEl.className = "status error";
  else if (/done|reached/i.test(text)) statusEl.className = "status done";
  else statusEl.className = "status pick";
}

const statusTopic = new ROSLIB.Topic({
  ros, name: "/arm_pick/status", messageType: "std_msgs/String",
});
statusTopic.subscribe((msg) => setStatus(msg.data));

ros.on("connection", () => {
  const descTopic = new ROSLIB.Topic({
    ros, name: "/robot_description", messageType: "std_msgs/String",
    queue_length: 1, latch: true,
  });
  descTopic.subscribe(async (msg) => {
    try {
      const loader = new UrdfLoader();
      const robot = await loader.load({ string: msg.data });
      robot.traverse((o) => { if (o.isMesh) o.castShadow = true; });
      scene.add(robot);
      setStatus("digital twin loaded");

      const jsHandler = new JointStateHandler(ros, robot);
      const jointNames = jsHandler.jointNames;
      jointsEl.innerHTML = "";
      const rows = {};
      jointNames.forEach((name) => {
        const li = document.createElement("li");
        const left = document.createElement("span");
        const right = document.createElement("span");
        left.textContent = name;
        right.textContent = "—";
        li.append(left, right);
        jointsEl.appendChild(li);
        rows[name] = right;
      });

      const jsTopic = new ROSLIB.Topic({
        ros, name: "/joint_states", messageType: "sensor_msgs/JointState",
      });
      jsTopic.subscribe((m) => {
        m.name.forEach((n, i) => {
          if (rows[n]) {
            rows[n].textContent = m.position[i].toFixed(3) + " rad";
            robot.setJointValue(n, m.position[i]);
          }
        });
      });

      pickBtn.disabled = false;
    } catch (err) {
      setStatus("ERROR loading robot: " + err.message);
    }
  });
});

pickBtn.addEventListener("click", () => {
  const svc = new ROSLIB.Service({
    ros, name: "/arm_pick/trigger", serviceType: "std_srvs/Trigger",
  });
  svc.callService(new ROSLIB.ServiceRequest(), (resp) => {
    setStatus(resp.message || "pick triggered");
  });
});

camTable.src = VIDEO_URL + "/camera_table/image_raw";
camEe.src = VIDEO_URL + "/camera_ee/image_raw";

// --- loop ------------------------------------------------------------------
renderer.setAnimationLoop(() => {
  controls.update();
  renderer.render(scene, camera);
});

window.addEventListener("resize", () => {
  camera.aspect = viewer.clientWidth / viewer.clientHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(viewer.clientWidth, viewer.clientHeight);
});
