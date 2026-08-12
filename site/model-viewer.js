import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const stage = document.querySelector("#model-stage");
const canvas = document.querySelector("#model-canvas");
const loading = document.querySelector("#model-loading");
const progress = document.querySelector("#model-progress");
const hint = document.querySelector("#model-hint");
const rotateButton = document.querySelector("#model-rotate");
const resetButton = document.querySelector("#model-reset");

if (stage && canvas) {
  const scene = new THREE.Scene();
  const camera = new THREE.PerspectiveCamera(34, 1, 0.001, 20);
  camera.up.set(0, 0, 1);

  let renderer;
  try {
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true, powerPreference: "high-performance" });
  } catch (error) {
    showModelError();
  }

  if (renderer) {
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;

    const controls = new OrbitControls(camera, canvas);
    controls.enableDamping = true;
    controls.dampingFactor = 0.055;
    controls.autoRotate = true;
    controls.autoRotateSpeed = 0.75;
    controls.screenSpacePanning = false;
    controls.minDistance = 0.055;
    controls.maxDistance = 0.32;
    controls.zoomToCursor = true;

    scene.add(new THREE.HemisphereLight(0xbceeff, 0x08111e, 2.2));

    const keyLight = new THREE.DirectionalLight(0xffffff, 4.2);
    keyLight.position.set(0.14, -0.1, 0.18);
    scene.add(keyLight);

    const rimLight = new THREE.DirectionalLight(0x42d9ff, 3.4);
    rimLight.position.set(-0.16, 0.12, 0.08);
    scene.add(rimLight);

    const fillLight = new THREE.DirectionalLight(0x557cff, 2.2);
    fillLight.position.set(0.04, 0.17, -0.05);
    scene.add(fillLight);

    const grid = new THREE.GridHelper(0.19, 18, 0x2e7891, 0x16364a);
    grid.rotation.x = Math.PI / 2;
    grid.material.transparent = true;
    grid.material.opacity = 0.22;
    grid.position.z = -0.004;
    scene.add(grid);

    let homePosition = new THREE.Vector3(0.11, -0.12, 0.085);
    let homeTarget = new THREE.Vector3();

    const loader = new GLTFLoader();
    loader.load(
      "./assets/automech-watch.glb",
      (gltf) => {
        const model = gltf.scene;
        scene.add(model);

        model.traverse((object) => {
          if (!object.isMesh) return;
          object.geometry.computeVertexNormals();
          const materials = Array.isArray(object.material) ? object.material : [object.material];
          materials.forEach((material) => {
            if (!material) return;
            material.metalness = 0.62;
            material.roughness = 0.27;
            material.envMapIntensity = 1.1;
          });
        });

        const bounds = new THREE.Box3().setFromObject(model);
        const center = bounds.getCenter(new THREE.Vector3());
        const sphere = bounds.getBoundingSphere(new THREE.Sphere());
        homeTarget = center.clone();
        homePosition = center.clone().add(new THREE.Vector3(sphere.radius * 2.25, -sphere.radius * 2.45, sphere.radius * 1.65));
        camera.position.copy(homePosition);
        controls.target.copy(homeTarget);
        controls.minDistance = sphere.radius * 1.15;
        controls.maxDistance = sphere.radius * 7;
        controls.update();

        loading.classList.add("loaded");
        setTimeout(() => { loading.hidden = true; }, 500);
        hint.classList.add("visible");
        setTimeout(() => hint.classList.remove("visible"), 4800);
      },
      (event) => {
        if (!event.total) return;
        progress.textContent = `${Math.min(100, Math.round((event.loaded / event.total) * 100))}%`;
      },
      () => showModelError()
    );

    function resetView() {
      camera.position.copy(homePosition);
      controls.target.copy(homeTarget);
      controls.update();
    }

    rotateButton.addEventListener("click", () => {
      controls.autoRotate = !controls.autoRotate;
      rotateButton.classList.toggle("active", controls.autoRotate);
      rotateButton.setAttribute("aria-pressed", String(controls.autoRotate));
    });

    resetButton.addEventListener("click", resetView);
    controls.addEventListener("start", () => hint.classList.remove("visible"));

    const resizeObserver = new ResizeObserver(() => {
      const width = stage.clientWidth;
      const height = stage.clientHeight;
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
      renderer.setSize(width, height, false);
    });
    resizeObserver.observe(stage);

    function animate() {
      controls.update();
      renderer.render(scene, camera);
      requestAnimationFrame(animate);
    }
    animate();
  }
}

function showModelError() {
  const isChinese = document.documentElement.lang.startsWith("zh");
  loading.hidden = false;
  loading.classList.add("error");
  loading.querySelector("strong").textContent = isChinese ? "三维模型加载失败" : "Unable to load the 3D model";
  progress.textContent = isChinese ? "请刷新页面重试" : "Refresh the page to retry";
}
