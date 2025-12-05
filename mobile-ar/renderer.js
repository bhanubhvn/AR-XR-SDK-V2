// renderer.js - Three.js 3D Renderer for AR

const Renderer = {
  scene: null,
  camera: null,
  renderer: null,
  model: null,
  modelBaseSize: 1,
  isReady: false,

  // Model offset values - adjust using alignment-tool
  modelOffset: {
    position: { x: -0.010, y: 0.000, z: 0.030 },
    rotation: { x: -1.571, y: 0.000, z: 0.000 },
    scale: 0.050,
  },

  async init(canvas) {
    // Scene
    this.scene = new THREE.Scene()

    // Camera at origin for 6DoF (pose matrix moves the model)
    this.camera = new THREE.PerspectiveCamera(
      45,
      canvas.width / canvas.height,
      0.01,
      100
    )
    this.camera.position.set(0, 0, 0)

    // Renderer (WebGL)
    this.renderer = new THREE.WebGLRenderer({
      canvas,
      alpha: true,
      antialias: true,
    })
    this.renderer.setSize(canvas.width, canvas.height)
    this.renderer.setClearColor(0x000000, 0)

    // Lights
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.6))

    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8)
    dirLight.position.set(5, 10, 7)
    this.scene.add(dirLight)

    const fillLight = new THREE.DirectionalLight(0xffffff, 0.4)
    fillLight.position.set(-5, -5, 5)
    this.scene.add(fillLight)

    this.isReady = true
    console.log("[Renderer] Initialized with WebGL")
  },

  async loadModel(url) {
    return new Promise((resolve, reject) => {
      const loader = new THREE.GLTFLoader()

      loader.load(
        url,
        (gltf) => {
          this.model = gltf.scene

          // Center model at origin
          const box = new THREE.Box3().setFromObject(this.model)
          const center = box.getCenter(new THREE.Vector3())
          this.model.position.sub(center)

          // Store base scale for later use
          const size = box.getSize(new THREE.Vector3())
          this.modelBaseSize = Math.max(size.x, size.y, size.z)

          this.model.visible = false
          this.scene.add(this.model)

          resolve(this.model)
        },
        null,
        reject
      )
    })
  },

  setProjectionMatrix(matrix) {
    if (!this.camera || !matrix) return
    this.camera.projectionMatrix.fromArray(matrix)
    this.camera.projectionMatrixInverse
      .copy(this.camera.projectionMatrix)
      .invert()
  },

  updateWithPose(poseMatrix) {
    if (!this.model || !poseMatrix) {
      if (this.model) this.model.visible = false
      return
    }

    this.model.visible = true

    // Build offset matrix from modelOffset values
    const offsetMatrix = new THREE.Matrix4()
    const offsetPos = new THREE.Vector3(
      this.modelOffset.position.x,
      this.modelOffset.position.y,
      this.modelOffset.position.z
    )
    const offsetQuat = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(
        this.modelOffset.rotation.x,
        this.modelOffset.rotation.y,
        this.modelOffset.rotation.z
      )
    )
    const offsetScale = new THREE.Vector3(
      this.modelOffset.scale,
      this.modelOffset.scale,
      this.modelOffset.scale
    )
    offsetMatrix.compose(offsetPos, offsetQuat, offsetScale)

    // Final matrix = poseMatrix * offsetMatrix
    const finalMatrix = new THREE.Matrix4()
    finalMatrix.copy(poseMatrix).multiply(offsetMatrix)

    // Apply to model
    this.model.matrixAutoUpdate = false
    this.model.matrix.copy(finalMatrix)
    this.model.matrixWorldNeedsUpdate = true
    this.model.updateMatrixWorld(true)
  },

  render() {
    if (!this.isReady) return
    this.renderer.render(this.scene, this.camera)
  },

  clear() {
    if (this.renderer) {
      this.renderer.autoClear = false
      this.renderer.clearColor()
      this.renderer.clearDepth()
    }
  },

  setVisible(visible) {
    if (this.model) this.model.visible = visible
  },

  dispose() {
    if (this.model) this.scene.remove(this.model)
    if (this.renderer) this.renderer.dispose()
    this.isReady = false
  },
}
