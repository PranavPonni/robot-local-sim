const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101826);

const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(3.5, 2.8, 5.2);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight * 0.75);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const dom = document.getElementById('renderer');
dom.appendChild(renderer.domElement);

const grid = new THREE.GridHelper(12, 24, 0x445a88, 0x2b3e61);
scene.add(grid);

const ambientLight = new THREE.AmbientLight(0x86a7d8, 0.72);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xdbe6ff, 1.1);
dirLight.position.set(6, 10, 6);
dirLight.castShadow = true;
dirLight.shadow.mapSize.set(2048, 2048);
scene.add(dirLight);

const robot = new THREE.Object3D();
scene.add(robot);

const UR5Joints = [
  { name: 'shoulder_pan_joint', axis: 'y', length: 0.162, color: 0x6da5f6 },
  { name: 'shoulder_lift_joint', axis: 'x', length: 0.425, color: 0x8bd49f },
  { name: 'elbow_joint', axis: 'x', length: 0.392, color: 0xf9c560 },
  { name: 'wrist_1_joint', axis: 'y', length: 0.109, color: 0xe37f7f },
  { name: 'wrist_2_joint', axis: 'x', length: 0.094, color: 0xa97ae6 },
  { name: 'wrist_3_joint', axis: 'y', length: 0.082, color: 0x39c7f5 }
];

function makeLink(length = 1, color = 0x5a78d4) {
  const geo = new THREE.BoxGeometry(0.16, length, 0.16);
  const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.5, roughness: 0.3, emissive: 0x001227 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.position.y = length / 2;
  return mesh;
}

const joints = [];
let parent = robot;

UR5Joints.forEach((jointDef, i) => {
  const pivot = new THREE.Object3D();
  parent.add(pivot);

  const link = makeLink(jointDef.length * 2.0, jointDef.color);
  pivot.add(link);

  joints.push({ pivot, ...jointDef, angle: 0 });

  const translate = new THREE.Object3D();
  translate.position.y = jointDef.length * 2.0;
  pivot.add(translate);

  parent = translate;
});

robot.position.y = 0.05;

const controlMap = {
  shoulder_pan_joint: { min: -180, max: 180, step: 1 },
  shoulder_lift_joint: { min: -180, max: 180, step: 1 },
  elbow_joint: { min: -180, max: 180, step: 1 },
  wrist_1_joint: { min: -180, max: 180, step: 1 },
  wrist_2_joint: { min: -180, max: 180, step: 1 },
  wrist_3_joint: { min: -180, max: 180, step: 1 }
};

const controlsElement = document.getElementById('controls');
const jointState = UR5Joints.map(() => 0);

function createControlBlock(jointDef, index) {
  const block = document.createElement('div');
  block.className = 'control-block';

  const label = document.createElement('label');
  label.innerText = `${jointDef.name} (${jointDef.axis.toUpperCase()})`;
  block.appendChild(label);

  const layout = document.createElement('div');
  layout.style.display = 'flex';
  layout.style.alignItems = 'center';
  layout.style.gap = '0.4rem';

  const minus = document.createElement('button');
  minus.innerText = '-';
  minus.style.width = '32px';
  minus.style.height = '32px';

  const plus = document.createElement('button');
  plus.innerText = '+';
  plus.style.width = '32px';
  plus.style.height = '32px';

  const valueLabel = document.createElement('div');
  valueLabel.className = 'value';
  valueLabel.innerText = '0°';

  const slider = document.createElement('input');
  slider.type = 'range';
  slider.min = controlMap[jointDef.name].min;
  slider.max = controlMap[jointDef.name].max;
  slider.step = controlMap[jointDef.name].step;
  slider.value = 0;
  slider.style.flex = '1';

  const updateValue = (value) => {
    jointState[index] = value;
    valueLabel.innerText = `${value.toFixed(0)}°`;
    slider.value = value;
    applyPose();
  };

  minus.addEventListener('click', () => {
    const next = Math.max(Number(slider.min), jointState[index] - 5);
    updateValue(next);
  });

  plus.addEventListener('click', () => {
    const next = Math.min(Number(slider.max), jointState[index] + 5);
    updateValue(next);
  });

  slider.addEventListener('input', (event) => {
    updateValue(Number(event.target.value));
  });

  layout.appendChild(minus);
  layout.appendChild(slider);
  layout.appendChild(plus);
  block.appendChild(layout);
  block.appendChild(valueLabel);

  controlsElement.appendChild(block);
}

UR5Joints.forEach(createControlBlock);

function applyPose() {
  joints.forEach((joint, i) => {
    const radians = THREE.MathUtils.degToRad(jointState[i]);

    joint.pivot.rotation.set(0, 0, 0);
    if (joint.axis === 'x') joint.pivot.rotation.x = radians;
    if (joint.axis === 'y') joint.pivot.rotation.y = radians;
    if (joint.axis === 'z') joint.pivot.rotation.z = radians;
  });
}

function animate() {
  requestAnimationFrame(animate);
  camera.lookAt(robot.position);
  renderer.render(scene, camera);
}

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / (window.innerHeight * 0.75);
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight * 0.75);
});

applyPose();
animate();
