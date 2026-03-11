const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101826);

const camera = new THREE.PerspectiveCamera(50, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(5.4, 3.5, 7.5);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight * 0.75);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const dom = document.getElementById('renderer');
dom.innerHTML = ''; // clear old DOM if any
dom.appendChild(renderer.domElement);

const grid = new THREE.GridHelper(14, 28, 0x445a88, 0x2b3e61);
scene.add(grid);

const axisHelper = new THREE.AxesHelper(1.8);
scene.add(axisHelper);

const ambientLight = new THREE.AmbientLight(0xc6d4ee, 0.64);
scene.add(ambientLight);

const dLight = new THREE.DirectionalLight(0xffffff, 1.15);
dLight.position.set(6, 10, 5);
dLight.castShadow = true;
dLight.shadow.mapSize.set(2048, 2048);
scene.add(dLight);

const robot = new THREE.Object3D();
robot.position.y = 0.05;
scene.add(robot);

function linkMesh(length, radius, color) {
  const geo = new THREE.CylinderGeometry(radius, radius, length, 48);
  const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.45, roughness: 0.25, emissive: 0x041125 });
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
  const link = linkMesh(j.len * 4.5, 0.11, j.color);
  link.position.set(0, j.len * 2.25, 0);
  pivot.add(link);

  joints.push({ pivot, axis: j.axis, name: j.name, angle: 0 });

  const next = new THREE.Object3D();
  next.position.set(0, j.len * 4.5, 0);
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

  const controlsRow = document.createElement('div');
  controlsRow.style.display = 'flex';
  controlsRow.style.alignItems = 'center';
  controlsRow.style.gap = '0.4rem';

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

  const val = document.createElement('div');
  val.className = 'value';
  val.textContent = '0°';

  const setAngle = (angle) => {
    state[idx] = angle;
    slider.value = angle;
    val.textContent = `${angle.toFixed(0)}°`;
    updateRobot();
  };

  bMinus.onclick = () => setAngle(Math.max(-180, state[idx] - 5));
  bPlus.onclick = () => setAngle(Math.min(180, state[idx] + 5));
  slider.oninput = (ev) => setAngle(parseFloat(ev.target.value));

  controlsRow.appendChild(bMinus);
  controlsRow.appendChild(slider);
  controlsRow.appendChild(bPlus);

  block.appendChild(controlsRow);
  block.appendChild(val);

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

function animate() {
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
