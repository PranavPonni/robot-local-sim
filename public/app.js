const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101826);

const camera = new THREE.PerspectiveCamera(55, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(4.5, 2.8, 6.5);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight * 0.75);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const dom = document.getElementById('renderer');
dom.innerHTML = '';
dom.appendChild(renderer.domElement);

const grid = new THREE.GridHelper(14, 28, 0x445a88, 0x2b3e61);
scene.add(grid);

const axisHelper = new THREE.AxesHelper(1.8);
scene.add(axisHelper);

const ambientLight = new THREE.AmbientLight(0xc6d4ee, 0.72);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xffffff, 1.2);
dirLight.position.set(6, 10, 6);
dirLight.castShadow = true;
scene.add(dirLight);

const robot = new THREE.Object3D();
robot.position.y = 0.05;
scene.add(robot);

// Visual base
const baseMaterial = new THREE.MeshStandardMaterial({ color: 0x374e78, roughness: 0.3, metalness: 0.4 });
const base = new THREE.Mesh(new THREE.CylinderGeometry(0.4, 0.4, 0.12, 64), baseMaterial);
base.position.y = 0.06;
base.receiveShadow = true;
scene.add(base);

// Debug cube to confirm rendering pipeline
const debugCube = new THREE.Mesh(
  new THREE.BoxGeometry(0.4, 0.4, 0.4),
  new THREE.MeshStandardMaterial({ color: 0xff6600, emissive: 0x400000, metalness: 0.45, roughness: 0.35 })
);
debugCube.position.set(0, 1.4, 0);
scene.add(debugCube);


function linkMesh(length, radius, color) {
  const geo = new THREE.CylinderGeometry(radius, radius, length, 32);
  const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.45, roughness: 0.25, emissive: 0x001127 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.rotation.x = Math.PI / 2;
  return mesh;
}

const ur5 = [
  { name: 'shoulder_pan_joint', axis: 'y', len: 0.18, color: 0x3d7ec4 },
  { name: 'shoulder_lift_joint', axis: 'x', len: 0.42, color: 0x5ca06b },
  { name: 'elbow_joint', axis: 'x', len: 0.39, color: 0xd6a337 },
  { name: 'wrist_1_joint', axis: 'y', len: 0.11, color: 0xcc4f5c },
  { name: 'wrist_2_joint', axis: 'x', len: 0.095, color: 0x8d5ecf },
  { name: 'wrist_3_joint', axis: 'y', len: 0.082, color: 0x3ca8c9 }
];

const joints = [];
let parentNode = robot;

ur5.forEach((j, idx) => {
  const pivot = new THREE.Object3D();
  parentNode.add(pivot);

  const link = linkMesh(j.len * 5.8, 0.12, j.color);
  link.position.set(0, j.len * 2.9, 0);
  pivot.add(link);

  const axisSphere = new THREE.Mesh(new THREE.SphereGeometry(0.1, 12, 12), new THREE.MeshStandardMaterial({ color: 0xffffff, emissive: 0x161a2f }));
  axisSphere.position.set(0, j.len * 5, 0);
  pivot.add(axisSphere);

  joints.push({ pivot, axis: j.axis, name: j.name, angle: 0 });

  const next = new THREE.Object3D();
  next.position.set(0, j.len * 5.8, 0);
  pivot.add(next);
  parentNode = next;
});

const controlsEl = document.getElementById('controls');
controlsEl.innerHTML = '';

const state = new Array(6).fill(0);

ur5.forEach((joint, idx) => {
  const block = document.createElement('div');
  block.className = 'control-block';

  const label = document.createElement('label');
  label.textContent = `${joint.name} (${joint.axis.toUpperCase()})`;
  block.appendChild(label);

  const row = document.createElement('div');
  row.style.display = 'flex';
  row.style.alignItems = 'center';
  row.style.gap = '0.3rem';

  const bMinus = document.createElement('button');
  bMinus.textContent = '-';
  bMinus.style.width = '32px';
  bMinus.style.height = '32px';

  const bPlus = document.createElement('button');
  bPlus.textContent = '+';
  bPlus.style.width = '32px';
  bPlus.style.height = '32px';

  const slider = document.createElement('input');
  slider.type = 'range';
  slider.min = -180;
  slider.max = 180;
  slider.step = 1;
  slider.value = 0;
  slider.style.flex = '1';

  const value = document.createElement('div');
  value.className = 'value';
  value.textContent = '0°';

  const setAngle = (angle) => {
    state[idx] = angle;
    slider.value = angle;
    value.textContent = `${angle.toFixed(0)}°`;
    updateRobot();
  };

  bMinus.onclick = () => setAngle(Math.max(-180, state[idx] - 5));
  bPlus.onclick = () => setAngle(Math.min(180, state[idx] + 5));
  slider.oninput = (e) => setAngle(Number(e.target.value));

  row.appendChild(bMinus);
  row.appendChild(slider);
  row.appendChild(bPlus);

  block.appendChild(row);
  block.appendChild(value);
  controlsEl.appendChild(block);
});

function updateRobot() {
  joints.forEach((joint, idx) => {
    const rad = THREE.MathUtils.degToRad(state[idx]);
    joint.pivot.rotation.set(0, 0, 0);
    if (joint.axis === 'x') joint.pivot.rotation.x = rad;
    if (joint.axis === 'y') joint.pivot.rotation.y = rad;
    if (joint.axis === 'z') joint.pivot.rotation.z = rad;
  });
}

// Try to automatically rotate camera for visibility.
let autoRotateAngle = 0;
function animate() {
  autoRotateAngle += 0.002;
  const radius = 7.2;
  camera.position.x = Math.cos(autoRotateAngle) * radius;
  camera.position.z = Math.sin(autoRotateAngle) * radius;
  camera.lookAt(0, 1.2, 0);

  // debug cube rotation for sanity check
  if (debugCube) {
    debugCube.rotation.x += 0.008;
    debugCube.rotation.y += 0.011;
  }

  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / (window.innerHeight * 0.75);
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight * 0.75);
});

updateRobot();
animate();
