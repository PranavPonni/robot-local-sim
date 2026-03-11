const scene = new THREE.Scene();
scene.background = new THREE.Color(0x101826);

const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.1, 1000);
camera.position.set(3.3, 3.0, 6.5);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight * 0.7);
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;

const dom = document.getElementById('renderer');
dom.appendChild(renderer.domElement);

const grid = new THREE.GridHelper(12, 24, 0x445a88, 0x2b3e61);
scene.add(grid);

const ambientLight = new THREE.AmbientLight(0x86a7d8, 0.72);
scene.add(ambientLight);

const dirLight = new THREE.DirectionalLight(0xdbe6ff, 1.1);
dirLight.position.set(5, 10, 5);
dirLight.castShadow = true;
dirLight.shadow.mapSize.set(2048, 2048);
scene.add(dirLight);

const robot = new THREE.Object3D();
scene.add(robot);

function makeLink(length = 1, color = 0x5a78d4) {
  const geo = new THREE.BoxGeometry(0.22, length, 0.22);
  const mat = new THREE.MeshStandardMaterial({ color, metalness: 0.35, roughness: 0.25, emissive: 0x001328 });
  const mesh = new THREE.Mesh(geo, mat);
  mesh.castShadow = true;
  mesh.receiveShadow = true;
  mesh.position.y = length / 2;
  return mesh;
}

const joints = [];
let parent = robot;

for (let i = 0; i < 6; i++) {
  const pivot = new THREE.Object3D();
  parent.add(pivot);

  const length = i === 5 ? 0.4 : 1.0;
  const link = makeLink(length, 0x5fa7f9 - i * 0x001122);
  pivot.add(link);

  joints.push({ pivot, link, angle: 0 });

  const translate = new THREE.Object3D();
  translate.position.y = length;
  pivot.add(translate);

  parent = translate;
}

robot.position.y = 0.15;

const labels = ['Base', 'Shoulder', 'Elbow', 'Wrist 1', 'Wrist 2', 'Gripper'];
const controlsElement = document.getElementById('controls');

const jointState = Array(6).fill(0);

labels.forEach((jointName, i) => {
  const block = document.createElement('div');
  block.className = 'control-block';

  const label = document.createElement('label');
  label.innerText = `${jointName} (deg)`;
  block.appendChild(label);

  const slider = document.createElement('input');
  slider.type = 'range';
  slider.min = -180;
  slider.max = 180;
  slider.value = 0;
  slider.step = (i === 0 ? 1 : 1);
  block.appendChild(slider);

  const currentValue = document.createElement('div');
  currentValue.className = 'value';
  currentValue.innerText = '0°';
  block.appendChild(currentValue);

  slider.addEventListener('input', (event) => {
    const value = Number(event.target.value);
    jointState[i] = value;
    currentValue.innerText = `${value.toFixed(0)}°`;
    applyPose();
  });

  controlsElement.appendChild(block);
});

function applyPose() {
  joints.forEach((joint, i) => {
    const radians = THREE.MathUtils.degToRad(jointState[i]);

    if (i === 0) {
      joint.pivot.rotation.y = radians;
    } else if (i === 1 || i === 3 || i === 5) {
      joint.pivot.rotation.z = radians;
    } else {
      joint.pivot.rotation.x = radians;
    }
  });
}

const target = new THREE.Vector3();
const controls = {
  x: 0,
  y: 0,
  z: 0,
};

function animate() {
  requestAnimationFrame(animate);
  camera.lookAt(robot.position);
  renderer.render(scene, camera);
}

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / (window.innerHeight * 0.7);
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight * 0.7);
});

applyPose();
animate();
