// alignment.js - 3D Model Alignment Tool

const AlignmentTool = {
  scene: null,
  camera: null,
  renderer: null,
  orbitControls: null,
  transformControls: null,
  model: null,
  modelGroup: null,
  marker: null,
  cameraHelper: null,
  gridBox: null,
  isDragging: false, // Track dragging state

  // Initial offset values (matches renderer.js)
  // Rotation stored in DEGREES, converted to radians when saving
  offset: {
    position: { x: -0.01, y: 0, z: 0.03 },
    rotation: { x: -90, y: 0, z: 0 },
    scale: 0.035,
  },

  init() {
    this.setupScene()
    this.setupControls()
    this.createGridBox()
    this.createMarker()
    this.createCameraReference()
    this.loadModel()
    this.setupUI()
    this.animate()
  },

  setupScene() {
    const canvas = document.getElementById("canvas3d")
    const container = document.querySelector(".viewport-area")

    this.scene = new THREE.Scene()
    this.scene.background = new THREE.Color(0xffffff)

    const aspect = container.clientWidth / container.clientHeight
    this.camera = new THREE.PerspectiveCamera(45, aspect, 0.01, 100)

    // Isometric view
    this.camera.position.set(1.5, 1.5, 1.5)
    this.camera.lookAt(0, 0, 0)

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true })
    this.renderer.setSize(container.clientWidth, container.clientHeight)
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2))

    // Lights
    this.scene.add(new THREE.AmbientLight(0xffffff, 0.7))
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8)
    dirLight.position.set(5, 10, 7)
    this.scene.add(dirLight)

    window.addEventListener("resize", () => this.onResize())
  },

  createGridBox() {
    const size = 2
    const divisions = 20
    const color1 = 0xcccccc
    const color2 = 0xe0e0e0

    // Floor grid (XZ plane)
    const floorGrid = new THREE.GridHelper(size, divisions, color1, color2)
    floorGrid.position.y = -size / 2
    this.scene.add(floorGrid)

    // Back wall grid (XY plane)
    const backGrid = new THREE.GridHelper(size, divisions, color1, color2)
    backGrid.rotation.x = Math.PI / 2
    backGrid.position.z = -size / 2
    this.scene.add(backGrid)

    // Left wall grid (YZ plane)
    const leftGrid = new THREE.GridHelper(size, divisions, color1, color2)
    leftGrid.rotation.z = Math.PI / 2
    leftGrid.position.x = -size / 2
    this.scene.add(leftGrid)

    // Corner lines
    const lineMat = new THREE.LineBasicMaterial({ color: 0xaaaaaa })
    const corners = [
      [[-size / 2, -size / 2, -size / 2], [-size / 2, size / 2, -size / 2]],
      [[-size / 2, -size / 2, -size / 2], [size / 2, -size / 2, -size / 2]],
      [[-size / 2, -size / 2, -size / 2], [-size / 2, -size / 2, size / 2]],
    ]
    corners.forEach(([start, end]) => {
      const geo = new THREE.BufferGeometry().setFromPoints([
        new THREE.Vector3(...start),
        new THREE.Vector3(...end)
      ])
      this.scene.add(new THREE.Line(geo, lineMat))
    })

    // Axes at origin
    this.scene.add(new THREE.AxesHelper(0.3))
  },

  setupControls() {
    this.orbitControls = new THREE.OrbitControls(this.camera, this.renderer.domElement)
    this.orbitControls.enableDamping = true
    this.orbitControls.dampingFactor = 0.05
    this.orbitControls.minDistance = 0.5
    this.orbitControls.maxDistance = 10

    this.transformControls = new THREE.TransformControls(this.camera, this.renderer.domElement)
    this.transformControls.setSize(0.6)

    // Handle Dragging State
    this.transformControls.addEventListener("dragging-changed", (e) => {
      this.orbitControls.enabled = !e.value
      this.isDragging = e.value

      // Final update on release
      if (!e.value) {
        this.syncOffsetFromModel()
        this.syncUIFromModel()
      }
    })

    // Real-time update during interaction
    this.transformControls.addEventListener("change", () => {
      if (this.isDragging) {
        this.syncOffsetFromModel()
        this.syncUIFromModel()
      }
    })

    this.scene.add(this.transformControls)
  },

  createMarker() {
    // Target image on XY plane - size matches pose-solver.js markerSize (0.15m)
    const geometry = new THREE.PlaneGeometry(0.15, 0.15)
    const material = new THREE.MeshBasicMaterial({
      color: 0xffffff,
      side: THREE.DoubleSide,
      transparent: true,
      opacity: 0.9
    })
    this.marker = new THREE.Mesh(geometry, material)

    // Load target image
    new THREE.TextureLoader().load("../assets/ranger-base-image.jpg", (texture) => {
      this.marker.material.map = texture
      this.marker.material.needsUpdate = true
    })

    // Border
    const edges = new THREE.EdgesGeometry(geometry)
    this.marker.add(new THREE.LineSegments(edges, new THREE.LineBasicMaterial({ color: 0x00ff88, linewidth: 2 })))

    this.scene.add(this.marker)
  },

  createCameraReference() {
    // Create a small camera icon to show phone orientation
    const camGroup = new THREE.Group()

    // Camera body
    const bodyGeo = new THREE.BoxGeometry(0.08, 0.05, 0.02)
    const bodyMat = new THREE.MeshBasicMaterial({ color: 0x3b82f6, wireframe: true })
    const body = new THREE.Mesh(bodyGeo, bodyMat)
    camGroup.add(body)

    // Lens
    const lensGeo = new THREE.CylinderGeometry(0.015, 0.015, 0.02, 8)
    const lensMat = new THREE.MeshBasicMaterial({ color: 0x60a5fa })
    const lens = new THREE.Mesh(lensGeo, lensMat)
    lens.rotation.x = Math.PI / 2
    lens.position.z = 0.02
    camGroup.add(lens)

    // View direction arrow
    const arrowGeo = new THREE.ConeGeometry(0.02, 0.05, 4)
    const arrowMat = new THREE.MeshBasicMaterial({ color: 0x22c55e })
    const arrow = new THREE.Mesh(arrowGeo, arrowMat)
    arrow.rotation.x = -Math.PI / 2
    arrow.position.z = 0.06
    camGroup.add(arrow)

    // Position camera reference behind the marker (where phone would be)
    camGroup.position.set(0, 0, 0.8)

    this.cameraHelper = camGroup
    this.scene.add(camGroup)
  },

  loadModel() {
    new THREE.GLTFLoader().load("../assets/ranger-3d-model.glb", (gltf) => {
      this.model = gltf.scene
      this.prepareModel()
    })
  },

  prepareModel() {
    if (this.modelGroup) {
      this.scene.remove(this.modelGroup)
      this.transformControls.detach()
    }
    if (!this.model) return

    const box = new THREE.Box3().setFromObject(this.model)
    const center = box.getCenter(new THREE.Vector3())
    this.model.position.set(-center.x, -box.min.y, -center.z)

    this.modelGroup = new THREE.Group()
    this.modelGroup.add(this.model)
    this.scene.add(this.modelGroup)
    this.transformControls.attach(this.modelGroup)
    this.updateModel()
  },

  setupUI() {
    const inputs = ["posX-num", "posY-num", "posZ-num", "rotX-num", "rotY-num", "rotZ-num", "scale-num"]

    inputs.forEach((id) => {
      const input = document.getElementById(id)
      if (!input) return

      input.addEventListener("input", () => {
        const val = parseFloat(input.value) || 0
        if (id.startsWith("pos")) {
          this.offset.position[id.charAt(3).toLowerCase()] = val
        } else if (id.startsWith("rot")) {
          this.offset.rotation[id.charAt(3).toLowerCase()] = val
        } else if (id.startsWith("scale")) {
          this.offset.scale = val
        }
        this.updateModel()
      })
    })

    // Initialize UI with current values
    this.syncUIFromModel()

    // Keyboard shortcuts
    window.addEventListener("keydown", (e) => {
      if (document.activeElement.tagName === "INPUT") return
      if (e.key === "g") this.setTransformMode("translate")
      if (e.key === "r") this.setTransformMode("rotate")
      if (e.key === "s") this.setTransformMode("scale")
    })
  },

  setTransformMode(mode) {
    this.transformControls.setMode(mode)
  },

  resetView() {
    this.camera.position.set(1.5, 1.5, 1.5)
    this.camera.lookAt(0, 0, 0)
    this.orbitControls.reset()
  },

  handleImageUpload(input) {
    if (input.files && input.files[0]) {
      const reader = new FileReader()
      reader.onload = (e) => {
        new THREE.TextureLoader().load(e.target.result, (texture) => {
          this.marker.material.map = texture
          this.marker.material.needsUpdate = true
        })
      }
      reader.readAsDataURL(input.files[0])
    }
  },

  handleModelUpload(input) {
    if (input.files && input.files[0]) {
      const url = URL.createObjectURL(input.files[0])
      new THREE.GLTFLoader().load(url, (gltf) => {
        this.model = gltf.scene
        this.prepareModel()
      })
    }
  },

  syncOffsetFromModel() {
    if (!this.modelGroup) return

    const p = this.modelGroup.position
    const r = this.modelGroup.rotation
    const s = this.modelGroup.scale

    this.offset.position.x = p.x
    this.offset.position.y = p.y
    this.offset.position.z = p.z

    // Invert the X rotation logic used in updateModel
    this.offset.rotation.x = - (r.x * 180 / Math.PI)
    this.offset.rotation.y = (r.y * 180 / Math.PI)
    this.offset.rotation.z = (r.z * 180 / Math.PI)

    this.offset.scale = s.x
  },

  syncUIFromModel() {
    const setVal = (id, val) => {
      const el = document.getElementById(id)
      if (el) el.value = parseFloat(val.toFixed(3))
    }

    setVal("posX-num", this.offset.position.x)
    setVal("posY-num", this.offset.position.y)
    setVal("posZ-num", this.offset.position.z)
    setVal("rotX-num", this.offset.rotation.x)
    setVal("rotY-num", this.offset.rotation.y)
    setVal("rotZ-num", this.offset.rotation.z)
    setVal("scale-num", this.offset.scale)
  },

  updateModel() {
    if (!this.modelGroup) return
    const { position, rotation, scale } = this.offset

    this.modelGroup.position.set(position.x, position.y, position.z)
    // Negate X rotation to match AR coordinate system
    this.modelGroup.rotation.set(
      (-rotation.x * Math.PI) / 180,  // Negated to match AR
      (rotation.y * Math.PI) / 180,
      (rotation.z * Math.PI) / 180
    )
    this.modelGroup.scale.setScalar(scale)
  },

  onResize() {
    const container = document.querySelector(".viewport-area")
    if (!container) return
    this.camera.aspect = container.clientWidth / container.clientHeight
    this.camera.updateProjectionMatrix()
    this.renderer.setSize(container.clientWidth, container.clientHeight)
  },

  animate() {
    requestAnimationFrame(() => this.animate())
    this.orbitControls.update()
    this.renderer.render(this.scene, this.camera)
  },
}

function resetValues() {
  AlignmentTool.offset = {
    position: { x: 0, y: 0, z: 0 },
    rotation: { x: 0, y: 0, z: 0 },
    scale: 1
  }
  AlignmentTool.updateModel()
  AlignmentTool.syncUIFromModel()
}

async function submitAndUpdateFiles() {
  const status = document.getElementById("status")
  status.textContent = "Updating..."

  try {
    const response = await fetch("/update-renderer", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(AlignmentTool.offset),
    })
    const data = await response.json()

    if (data.success) {
      status.textContent = "Updated!"
      status.style.color = "#22c55e"
    } else {
      status.textContent = "Error: " + data.message
      status.style.color = "#ef4444"
    }
  } catch (err) {
    status.textContent = "Server not running"
    status.style.color = "#ef4444"
  }

  setTimeout(() => (status.textContent = ""), 3000)
}

window.addEventListener("DOMContentLoaded", () => AlignmentTool.init())
