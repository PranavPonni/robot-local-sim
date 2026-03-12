import * as THREE from '/modules/three/build/three.module.js';
import { OrbitControls } from '/modules/three/examples/jsm/controls/OrbitControls.js';

const container = document.getElementById('renderer');
if (!container) {
  throw new Error('renderer container missing');
}

if (!window.WebGLRenderingContext) {
  container.innerHTML = '<div style="color:#fff;padding:20px">WebGL is not available in this browser.</div>';
  throw new Error('WebGL unsupported');
}

function initScene() {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x101826);

  const camera = new THREE.PerspectiveCamera(45, window.innerWidth / (window.innerHeight * 0.75), 0.1, 1000);
  camera.position.set(4.5, 3.2, 7.2);

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight * 0.75);
  renderer.setPixelRatio(window.devicePixelRatio);
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  container.innerHTML = '';
  container.appendChild(renderer.domElement);

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.05;
  controls.minDistance = 2;
  controls.maxDistance = 20;
  controls.target.set(0, 1.2, 0);

  scene.add(new THREE.AxesHelper(2.5));
  scene.add(new THREE.GridHelper(14, 28, 0x2b3f64, 0x344d70));

  scene.add(new THREE.AmbientLight(0xffffff, 0.55));
  const dir = new THREE.DirectionalLight(0xffffff, 1.15);
  dir.position.set(5, 7, 5);
  dir.castShadow = true;
  scene.add(dir);

  const base = new THREE.Mesh(
    new THREE.CylinderGeometry(0.4, 0.4, 0.12, 64),
    new THREE.MeshStandardMaterial({ color: 0x2f3b61, roughness: 0.35, metalness: 0.45 })
  );
  base.position.y = 0.06;
  base.receiveShadow = true;
  scene.add(base);

  const jointSpec = [
    { name: 'shoulder_pan_joint', axis: 'y', len: 0.12, color: 0x4f8efa },
    { name: 'shoulder_lift_joint', axis: 'x', len: 0.42, color: 0x5fcf8a },
    { name: 'elbow_joint', axis: 'x', len: 0.39, color: 0xf6ae4d },
    { name: 'wrist_1_joint', axis: 'y', len: 0.11, color: 0xef6eb2 },
    { name: 'wrist_2_joint', axis: 'x', len: 0.095, color: 0x9064d1 },
    { name: 'wrist_3_joint', axis: 'y', len: 0.082, color: 0x57c4ef }
  ];

  const joints = [];
  let parent = base;

  jointSpec.forEach((joint, i) => {
    const pivot = new THREE.Object3D();
    parent.add(pivot);
    pivot.position.y = i === 0 ? 0.06 : jointSpec[i - 1].len * 4.2;

    const link = new THREE.Mesh(
      new THREE.CylinderGeometry(0.08, 0.08, joint.len * 4.2, 24),
      new THREE.MeshStandardMaterial({ color: joint.color, roughness: 0.3, metalness: 0.5 })
    );
    link.position.y = joint.len * 2.1;
    link.rotation.x = Math.PI / 2;
    link.castShadow = true;
    link.receiveShadow = true;
    pivot.add(link);

    const marker = new THREE.Mesh(
      new THREE.SphereGeometry(0.09, 16, 16),
      new THREE.MeshStandardMaterial({ color: 0xdddddd, roughness: 0.25, metalness: 0.6 })
    );
    marker.position.y = joint.len * 4.2;
    pivot.add(marker);

    joints.push({ pivot, axis: joint.axis, value: 0, name: joint.name });

    const next = new THREE.Object3D();
    next.position.y = joint.len * 4.2;
    pivot.add(next);
    parent = next;
  });

  const debugCube = new THREE.Mesh(
    new THREE.BoxGeometry(0.2, 0.2, 0.2),
    new THREE.MeshStandardMaterial({ color: 0xff8000, emissive: 0x221100, roughness: 0.35, metalness: 0.45 })
  );
  debugCube.position.set(1.2, 1.15, 0.3);
  scene.add(debugCube);

  const controlsDiv = document.getElementById('controls');
  controlsDiv.innerHTML = '';

  joints.forEach((joint, idx) => {
    const card = document.createElement('div');
    card.className = 'control-block';
    const title = document.createElement('label');
    title.innerText = joint.name;

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

    const setJoint = (valueAngle) => {
      joint.value = valueAngle;
      value.innerText = `${valueAngle.toFixed(0)}°`;
      slider.value = valueAngle;
      joints.forEach((j) => {
        const rad = THREE.MathUtils.degToRad(j.value);
        j.pivot.rotation.set(0, 0, 0);
        if (j.axis === 'x') j.pivot.rotation.x = rad;
        if (j.axis === 'y') j.pivot.rotation.y = rad;
        if (j.axis === 'z') j.pivot.rotation.z = rad;
      });
    };

    minus.onclick = () => setJoint(Math.max(-180, joint.value - 5));
    plus.onclick = () => setJoint(Math.min(180, joint.value + 5));
    slider.oninput = (e) => setJoint(Number(e.target.value));

    row.appendChild(minus);
    row.appendChild(slider);
    row.appendChild(plus);

    card.appendChild(title);
    card.appendChild(row);
    card.appendChild(value);
    controlsDiv.appendChild(card);
  });

  function resize() {
    const w = window.innerWidth;
    const h = Math.max(400, window.innerHeight * 0.75);
    renderer.setSize(w, h);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
  }

  window.addEventListener('resize', resize);
  resize();

  let t = 0;
  function animate() {
    t += 0.004;
    camera.position.x = Math.cos(t) * 7.0;
    camera.position.z = Math.sin(t) * 7.0;
    camera.lookAt(0, 1.2, 0);

    debugCube.rotation.x += 0.012;
    debugCube.rotation.y += 0.01;

    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(animate);
  }

  console.log('Three.js version', THREE.REVISION);
  animate();
}

initScene();
