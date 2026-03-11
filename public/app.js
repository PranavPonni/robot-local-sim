console.log('app.js loaded');

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101826);

const camera = new THREE.PerspectiveCamera(45, 1.0, 0.1, 1000);
camera.position.set(2.5, 2.0, 4.5);
camera.lookAt(0, 1.0, 0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setClearColor(0x101826);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const dom = document.getElementById('renderer');
if (!dom) {
  throw new Error('Renderer container element #renderer not found');
}

renderer.setSize(dom.clientWidth || window.innerWidth, dom.clientHeight || window.innerHeight * 0.7);
renderer.domElement.style.display = 'block';
renderer.domElement.style.margin = '0 auto';
dom.appendChild(renderer.domElement);

const grid = new THREE.GridHelper(12, 24, 0x555a80, 0x2d3f62);
scene.add(grid);

const axes = new THREE.AxesHelper(1.5);
scene.add(axes);

const ambientLight = new THREE.AmbientLight(0xb0c9ee, 0.6);
scene.add(ambientLight);

const light = new THREE.DirectionalLight(0xffffff, 1.0);
light.position.set(5, 8, 7);
light.castShadow = true;
scene.add(light);

const robot = new THREE.Object3D();
robot.position.y = 0.1;
scene.add(robot);

// Base
const base = new THREE.Mesh(
  new THREE.CylinderGeometry(0.35, 0.35, 0.1, 64),
  new THREE.MeshStandardMaterial({ color: 0x2f3b61, roughness: 0.35, metalness: 0.45 })
);
base.position.y = 0.05;
base.receiveShadow = true;
scene.add(base);

function createLink(length, radius, color) {
  const geometry = new THREE.CylinderGeometry(radius, radius, length, 32);
  const material = new THREE.MeshStandardMaterial({ color, roughness: 0.28, metalness: 0.53 });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.rotation.x = Math.PI / 2;
  return mesh;
}

const ur5 = [
  { name: 'shoulder_pan_joint', axis: 'y', len: 0.12, color: 0x48a3ea },
  { name: 'shoulder_lift_joint', axis: 'x', len: 0.42, color: 0x65c285 },
  { name: 'elbow_joint', axis: 'x', len: 0.39, color: 0xf1a257 },
  { name: 'wrist_1_joint', axis: 'y', len: 0.11, color: 0xee5a6f },
  { name: 'wrist_2_joint', axis: 'x', len: 0.095, color: 0x8c62cf },
  { name: 'wrist_3_joint', axis: 'y', len: 0.082, color: 0x44b7df }
];

const joints = [];
let parent = robot;

ur5.forEach((jointDef) => {
  const pivot = new THREE.Object3D();
  parent.add(pivot);

  const link = createLink(jointDef.len * 4.2, 0.1, jointDef.color);
  link.position.set(0, jointDef.len * 2.1, 0);
  pivot.add(link);

  const socket = new THREE.Mesh(
    new THREE.SphereGeometry(0.08, 16, 16),
    new THREE.MeshStandardMaterial({ color: 0xeeeeee, emissive: 0x111111, roughness: 0.3, metalness: 0.45 })
  );
  socket.position.set(0, jointDef.len * 4.2, 0);
  pivot.add(socket);

  joints.push({ pivot, axis: jointDef.axis, name: jointDef.name, angle: 0 });

  const next = new THREE.Object3D();
  next.position.set(0, jointDef.len * 4.2, 0);
  pivot.add(next);
  parent = next;
});

// debug cube
const debugCube = new THREE.Mesh(
  new THREE.BoxGeometry(0.2, 0.2, 0.2),
  new THREE.MeshStandardMaterial({ color: 0xff9400, emissive: 0x441700, metalness: 0.45, roughness: 0.2 })
);
debugCube.position.set(0.7, 0.8, 0);
scene.add(debugCube);

const controlsEl = document.getElementById('controls');
controlsEl.innerHTML = '';

const state = new Array(6).fill(0);

ur5.forEach((jointDef, index) => {
  const block = document.createElement('div');
  block.className = 'control-block';

  const label = document.createElement('label');
  label.innerText = `${jointDef.name} (${jointDef.axis.toUpperCase()})`;
  block.appendChild(label);

  const row = document.createElement('div');
  row.style.display = 'flex';
  row.style.alignItems = 'center';
  row.style.gap = '0.3rem';

  const minus = document.createElement('button');
  minus.innerText = '-';
  minus.style.width = '32px';
  minus.style.height = '32px';

  const plus = document.createElement('button');
  plus.innerText = '+';
  plus.style.width = '32px';
  plus.style.height = '32px';

  const slider = document.createElement('input');
  slider.type = 'range';
  slider.min = -180;
  slider.max = 180;
  slider.step = 1;
  slider.value = 0;
  slider.style.flex = '1';

  const value = document.createElement('div');
  value.className = 'value';
  value.innerText = '0°';

  const setJointAngle = (angle) => {
    state[index] = angle;
    value.innerText = `${angle.toFixed(0)}°`;
    slider.value = angle;
    joints.forEach((p, j) => {
      const rad = THREE.MathUtils.degToRad(state[j]);
      p.pivot.rotation.set(0, 0, 0);
      if (p.axis === 'x') p.pivot.rotation.x = rad;
      if (p.axis === 'y') p.pivot.rotation.y = rad;
      if (p.axis === 'z') p.pivot.rotation.z = rad;
    });
  };

  minus.onclick = () => setJointAngle(Math.max(-180, state[index] - 5));
  plus.onclick = () => setJointAngle(Math.min(180, state[index] + 5));
  slider.oninput = (e) => setJointAngle(Number(e.target.value));

  row.appendChild(minus);
  row.appendChild(slider);
  row.appendChild(plus);

  block.appendChild(row);
  block.appendChild(value);
  controlsEl.appendChild(block);
});

function onResize() {
  const width = dom.clientWidth || window.innerWidth;
  const height = dom.clientHeight || window.innerHeight * 0.7;
  renderer.setSize(width, height);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', onResize);

let spin = 0;
function animate() {
  spin += 0.003;
  camera.position.x = Math.cos(spin) * 4.4;
  camera.position.z = Math.sin(spin) * 4.4;
  camera.lookAt(0, 1.1, 0);

  debugCube.rotation.x += 0.008;
  debugCube.rotation.y += 0.01;

  requestAnimationFrame(animate);
  renderer.render(scene, camera);
}

console.log('Starting animation loop');
animate();
