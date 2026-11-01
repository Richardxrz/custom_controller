/* 6 轴 URDF 查看器：three r128 + URDFLoader + OrbitControls. */

const view = document.getElementById('view');
const loading = document.getElementById('loading');
const modelLabel = document.getElementById('model');
const jointBox = document.getElementById('joints');
const followBox = document.getElementById('follow');
const params = new URLSearchParams(location.search);

window.addEventListener('error', (e) => {
  loading.textContent = `JS 错误：${e.message}`;
});
window.addEventListener('unhandledrejection', (e) => {
  loading.textContent = `Promise 错误：${e.reason}`;
});

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0e1116);

const camera = new THREE.PerspectiveCamera(45, innerWidth / innerHeight, 0.01, 100);
camera.position.set(1.8, 1.4, 2.0);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 1.5));
renderer.setSize(innerWidth, innerHeight);
renderer.outputEncoding = THREE.sRGBEncoding;
view.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;
controls.screenSpacePanning = true;
controls.target.set(0, 0.5, 0);
controls.update();

// 均匀环境光：不计算方向光/阴影，接近建模软件的平光视图。
scene.add(new THREE.AmbientLight(0xffffff, 1.4));

const grid = new THREE.GridHelper(6, 60, 0x2d404b, 0x1a252d);
scene.add(grid);
scene.add(new THREE.AxesHelper(0.6));

const robotFrame = new THREE.Group();
robotFrame.rotation.x = -Math.PI / 2; // URDF Z-up -> three Y-up
scene.add(robotFrame);

let robot = null;
let joints = [];
let jointSliders = [];
let follow = false;
let baseCam = null;

followBox.addEventListener('change', () => { follow = followBox.checked; });
document.getElementById('reset').addEventListener('click', () => {
  if (baseCam) {
    camera.position.copy(baseCam.pos);
    controls.target.copy(baseCam.target);
  }
});

function fitCamera(object) {
  const box = new THREE.Box3().setFromObject(object);
  const center = box.getCenter(new THREE.Vector3());
  const size = box.getSize(new THREE.Vector3());
  const radius = Math.max(size.x, size.y, size.z) * 0.75;
  const dist = Math.max(radius * 2.6, 0.9);
  camera.position.set(center.x + dist, center.y + dist * 0.65, center.z + dist);
  controls.target.copy(center);
  controls.update();
  baseCam = {
    pos: camera.position.clone(),
    target: controls.target.clone(),
  };
}

function setJoint(joint, rad) {
  const lo = joint.limit && isFinite(joint.limit.lower) ? joint.limit.lower : -Math.PI;
  const hi = joint.limit && isFinite(joint.limit.upper) ? joint.limit.upper : Math.PI;
  joint.setJointValue(Math.min(hi, Math.max(lo, rad)));
}

function buildJointUI() {
  jointBox.innerHTML = '';
  jointSliders = [];
  joints.forEach((joint, index) => {
    const lo = joint.limit && isFinite(joint.limit.lower) ? joint.limit.lower : -Math.PI;
    const hi = joint.limit && isFinite(joint.limit.upper) ? joint.limit.upper : Math.PI;
    const row = document.createElement('div');
    row.className = 'row';
    const lab = document.createElement('div');
    lab.className = 'lab';
    lab.innerHTML = `<span>${joint.name}</span><span class="val">0°</span>`;
    const slider = document.createElement('input');
    slider.type = 'range';
    slider.min = String(Math.round(lo * 180 / Math.PI));
    slider.max = String(Math.round(hi * 180 / Math.PI));
    slider.step = '0.5';
    slider.value = '0';
    slider.addEventListener('input', () => {
      const rad = Number(slider.value) * Math.PI / 180;
      setJoint(joint, rad);
      lab.querySelector('.val').textContent = `${Number(slider.value).toFixed(1)}°`;
    });
    row.appendChild(lab);
    row.appendChild(slider);
    jointBox.appendChild(row);
    jointSliders.push({ slider, val: lab.querySelector('.val'), joint });
  });
}

const palette = {
  base_link: 0x39434b, link1: 0xb8c3cb, link2: 0xd9a72e,
  link3: 0xb8c3cb, link4: 0xd9a72e, link5: 0xb8c3cb,
  link6: 0x39434b, end_link: 0xcfd6db,
};

function colorOf(obj) {
  let n = obj;
  while (n) {
    if (n.urdfName && palette[n.urdfName] !== undefined) return palette[n.urdfName];
    n = n.parent;
  }
  return 0xb8c3cb;
}

function styleRobot() {
  if (!robot) return;
  robot.traverse((child) => {
    if (!child.isMesh) return;
    child.material = new THREE.MeshLambertMaterial({
      color: colorOf(child), side: THREE.DoubleSide,
    });
  });
}

function loadModel() {
  const manager = new THREE.LoadingManager();
  manager.onProgress = (url, loaded, total) => {
    loading.textContent = `加载网格 ${Math.round(loaded / Math.max(total, 1) * 100)}% (${loaded}/${total})`;
  };
  manager.onLoad = () => {
    if (robot) {
      styleRobot();
      fitCamera(robotFrame);
    }
    loading.classList.add('hide');
  };
  manager.onError = (url) => {
    loading.classList.remove('hide');
    loading.textContent = `资源失败：${url}`;
  };
  const loader = new URDFLoader(manager);
  const origMesh = loader.loadMeshCb.bind(loader);
  loader.loadMeshCb = (path, mgr, done) => {
    try { origMesh(path, mgr, done); }
    catch (e) { loading.classList.remove('hide'); loading.textContent = `mesh 错误 ${path}: ${e}`; }
  };
  loader.load('/model/urdf', (loaded) => {
    robot = loaded;
    robotFrame.add(robot);
    joints = Object.values(robot.joints || {})
      .filter((j) => j.jointType !== 'fixed')
      .sort((a, b) => a.name.localeCompare(b.name, undefined, { numeric: true }));
    buildJointUI();
    fitCamera(robotFrame);
    modelLabel.textContent = `${joints.length} 个可动关节`;
    const q = params.get('q');
    if (q) {
      q.split(',').map(Number).slice(0, joints.length).forEach((rad, i) => {
        if (!isFinite(rad)) return;
        setJoint(joints[i], rad);
        const s = jointSliders[i];
        if (s) {
          const deg = rad * 180 / Math.PI;
          s.slider.value = String(deg);
          s.val.textContent = `${deg.toFixed(1)}°`;
        }
      });
    }
    if (params.get('follow') === '1') {
      followBox.checked = true;
      follow = true;
    }
    loading.textContent = '解析完成，加载网格…';
  }, undefined, (err) => {
    loading.textContent = `加载失败：${err && err.message ? err.message : err}`;
  });
}

async function pollAngles() {
  if (!follow || !joints.length) return;
  try {
    const res = await fetch('/angles', { cache: 'no-store' });
    const vals = await res.json();
    if (!Array.isArray(vals)) return;
    vals.slice(0, joints.length).forEach((rad, i) => {
      const j = joints[i];
      setJoint(j, Number(rad));
      const s = jointSliders[i];
      if (s) {
        const deg = Number(rad) * 180 / Math.PI;
        s.slider.value = String(deg);
        s.val.textContent = `${deg.toFixed(1)}°`;
      }
    });
  } catch (e) { /* ignore */ }
}

addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}

loadModel();
setInterval(pollAngles, 120);
animate();
