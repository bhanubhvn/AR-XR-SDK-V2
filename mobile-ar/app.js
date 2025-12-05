// app.js - Main AR Application

const App = {
  // Assets
  TARGET_IMAGE: "assets/ranger-base-image.jpg",
  MODEL_3D: "assets/ranger-3d-model.glb",

  // State
  engine: null,
  video: null,
  canvas: null,
  ctx: null,
  canvas3d: null,
  isTracking: false,
  animationId: null,

  // Processing
  processCanvas: null,
  processCtx: null,
  processWidth: 480, // Higher resolution for distant detection

  // Smoothing
  smoothedCorners: null,
  lastFoundTime: 0,

  // IMU fusion
  imu: {
    enabled: true,
    blendWeightStrong: 0.4, // when visual is stable
    blendWeightWeak: 0.35, // when visual is less stable
    imuSmoothing: 0.15, // slerp factor on IMU quaternion itself
    yawBiasStrength: 0.05, // how fast to realign IMU yaw toward visual
  },
  imuQuat: null, // smoothed IMU quaternion
  imuQuatRaw: null, // latest raw IMU quaternion
  imuCorrectionQuat: new THREE.Quaternion(), // yaw bias correction
  imuActive: false,
  screenOrientation: 0,

  // DOM elements
  els: {},

  async init() {
    this._cacheElements()
    this._updateLoading("Loading AR Engine...")

    try {
      // Initialize AR engine
      this.engine = new AREngine()
      this.engine.init(cv)

      // Setup processing canvas
      this.processCanvas = document.createElement("canvas")
      this.processCtx = this.processCanvas.getContext("2d", {
        willReadFrequently: true,
      })

      // Load target image
      this._updateLoading("Loading target image...")
      await this._loadTarget()

      // Start camera
      this._updateLoading("Starting camera...")
      await this._startCamera()

      // Initialize pose solver with processing dimensions
      const pHeight = Math.round(
        this.processWidth * (this.displayHeight / this.displayWidth)
      )
      PoseSolver.init(cv, this.processWidth, pHeight)

      // Initialize 3D renderer
      this._updateLoading("Loading 3D model...")
      await this._initRenderer()
      await Renderer.loadModel(this.MODEL_3D)

      // Set projection matrix to match camera intrinsics
      const projMatrix = PoseSolver.getProjectionMatrix(0.01, 100)
      Renderer.setProjectionMatrix(projMatrix)

      // IMU (device orientation) fusion
      await this._initIMU()

      // Hide loading, start tracking
      this.els.loading.classList.add("hidden")
      this.els.status.classList.remove("hidden")
      this._startTracking()
    } catch (err) {
      console.error("[App] Init failed:", err)
      this._showError(err.message)
    }
  },

  _cacheElements() {
    this.els = {
      loading: document.getElementById("loading"),
      loadingText: document.getElementById("loading-text"),
      status: document.getElementById("status"),
      statusIcon: document.getElementById("status-icon"),
      statusText: document.getElementById("status-text"),
      foundIndicator: document.getElementById("found-indicator"),
    }
    this.video = document.getElementById("video")
    this.canvas = document.getElementById("canvas")
    this.ctx = this.canvas.getContext("2d", { willReadFrequently: true })
    this.canvas3d = document.getElementById("canvas3d")
  },

  _updateLoading(text) {
    if (this.els.loadingText) {
      this.els.loadingText.textContent = text
    }
  },

  async _loadTarget() {
    const img = new Image()
    img.crossOrigin = "anonymous"

    await new Promise((resolve, reject) => {
      img.onload = resolve
      img.onerror = () => reject(new Error("Failed to load target image"))
      img.src = this.TARGET_IMAGE
    })

    // Get image data
    const tempCanvas = document.createElement("canvas")
    tempCanvas.width = img.width
    tempCanvas.height = img.height
    const tempCtx = tempCanvas.getContext("2d")
    tempCtx.drawImage(img, 0, 0)
    const imageData = tempCtx.getImageData(0, 0, img.width, img.height)

    const result = this.engine.processTarget(imageData)
    console.log("[App] Target loaded:", result)
  },

  async _startCamera() {
    const stream = await navigator.mediaDevices.getUserMedia({
      video: {
        facingMode: "environment",
        width: { ideal: 1280 },
        height: { ideal: 720 },
      },
    })

    this.video.srcObject = stream
    await this.video.play()

    // Get container size for canvas dimensions
    const container = document.getElementById("ar-container")
    const rect = container.getBoundingClientRect()

    // Use container size for both canvases
    this.displayWidth = Math.floor(rect.width)
    this.displayHeight = Math.floor(rect.height)

    this.canvas.width = this.displayWidth
    this.canvas.height = this.displayHeight
  },

  async _initRenderer() {
    this.canvas3d.width = this.displayWidth
    this.canvas3d.height = this.displayHeight
    await Renderer.init(this.canvas3d)
  },

  _startTracking() {
    this.isTracking = true
    this._loop()
  },

  // Draw video in "cover" mode (fill canvas, crop excess)
  _drawVideoCover() {
    const vw = this.video.videoWidth
    const vh = this.video.videoHeight
    const cw = this.canvas.width
    const ch = this.canvas.height

    const videoAspect = vw / vh
    const canvasAspect = cw / ch

    let sx, sy, sw, sh

    if (canvasAspect > videoAspect) {
      // Canvas is wider - crop top/bottom of video
      sw = vw
      sh = vw / canvasAspect
      sx = 0
      sy = (vh - sh) / 2
    } else {
      // Canvas is taller - crop left/right of video
      sh = vh
      sw = vh * canvasAspect
      sx = (vw - sw) / 2
      sy = 0
    }

    // Store crop info for coordinate mapping
    this.videoCrop = { sx, sy, sw, sh }

    this.ctx.drawImage(this.video, sx, sy, sw, sh, 0, 0, cw, ch)
  },

  _loop() {
    if (!this.isTracking) return

    // Draw video to canvas (cover mode - fill and crop)
    this._drawVideoCover()

    // Prepare external GPU texture for future WebGPU compute
    if (Renderer.updateExternalTexture) {
      Renderer.updateExternalTexture(this.video)
    }

    // Clear 3D
    Renderer.clear()

    // Process frame
    const corners = this._processFrame()

    if (corners) {
      // Smooth corners
      const smoothed = this._smoothCorners(corners)
      this.lastFoundTime = Date.now()

      // Solve 6DoF pose from corners (at processing resolution)
      const pose = PoseSolver.solve(this._toProcessingCoords(smoothed))

      if (pose && pose.matrix) {
        const fusedMatrix = this._blendPoseWithIMU(
          pose.matrix,
          this._currentIMUBlend()
        )
        Renderer.updateWithPose(fusedMatrix)
      }
      this._setFound(true)
    } else {
      // Hide after timeout
      if (Date.now() - this.lastFoundTime > 200) {
        Renderer.setVisible(false)
        this._setFound(false)
        this.smoothedCorners = null
      }
    }

    Renderer.render()
    this.animationId = requestAnimationFrame(() => this._loop())
  },

  _processFrame() {
    if (!this.videoCrop) return null

    const { sx, sy, sw, sh } = this.videoCrop
    const cw = this.canvas.width
    const ch = this.canvas.height

    // Processing size (maintain canvas aspect ratio)
    const pWidth = this.processWidth
    const pHeight = Math.round(pWidth * (ch / cw))

    if (
      this.processCanvas.width !== pWidth ||
      this.processCanvas.height !== pHeight
    ) {
      this.processCanvas.width = pWidth
      this.processCanvas.height = pHeight
    }

    // Draw cropped video region to processing canvas
    this.processCtx.drawImage(this.video, sx, sy, sw, sh, 0, 0, pWidth, pHeight)
    const imageData = this.processCtx.getImageData(0, 0, pWidth, pHeight)

    const result = this.engine.processFrame(imageData)

    if (result && result.corners) {
      // Scale corners from processing size to display canvas size
      const scaleX = cw / pWidth
      const scaleY = ch / pHeight

      return result.corners.map((p) => ({
        x: p.x * scaleX,
        y: p.y * scaleY,
      }))
    }

    return null
  },

  // Convert display coords back to processing coords for PoseSolver
  _toProcessingCoords(corners) {
    const scaleX = this.processWidth / this.canvas.width
    const scaleY = this.processCanvas.height / this.canvas.height
    return corners.map((c) => ({ x: c.x * scaleX, y: c.y * scaleY }))
  },

  _smoothCorners(newCorners) {
    if (!this.smoothedCorners) {
      this.smoothedCorners = newCorners
      return newCorners
    }

    const smoothed = []
    for (let i = 0; i < 4; i++) {
      const dx = newCorners[i].x - this.smoothedCorners[i].x
      const dy = newCorners[i].y - this.smoothedCorners[i].y
      const distance = Math.hypot(dx, dy)

      // Dynamic smoothing
      let factor = 0.1
      if (distance > 20) factor = 0.8
      else if (distance > 5) factor = 0.4
      else if (distance > 1) factor = 0.2

      smoothed.push({
        x: this.smoothedCorners[i].x + dx * factor,
        y: this.smoothedCorners[i].y + dy * factor,
      })
    }

    this.smoothedCorners = smoothed
    return smoothed
  },

  _blendPoseWithIMU(poseMatrix, weight = this.imu.blendWeightStrong) {
    if (!this.imu.enabled || !this.imuActive || !this.imuQuat) {
      return poseMatrix
    }

    const newPos = new THREE.Vector3()
    const newQuat = new THREE.Quaternion()
    const newScale = new THREE.Vector3()
    poseMatrix.decompose(newPos, newQuat, newScale)

    // Apply yaw bias correction to IMU quaternion
    const imuCorrected = new THREE.Quaternion()
      .copy(this.imuCorrectionQuat)
      .multiply(this.imuQuat)

    const blended = new THREE.Quaternion().copy(newQuat)
    blended.slerp(imuCorrected, weight)

    const finalMatrix = new THREE.Matrix4()
    finalMatrix.compose(newPos, blended, newScale)
    return finalMatrix
  },

  _currentIMUBlend() {
    // If we have stable corners, use strong weight; otherwise weak
    return this.smoothedCorners
      ? this.imu.blendWeightStrong
      : this.imu.blendWeightWeak
  },

  _alignIMUYawToVisual(visualQuat) {
    if (!this.imuActive || !this.imuQuat) return

    const imuCorrected = new THREE.Quaternion()
      .copy(this.imuCorrectionQuat)
      .multiply(this.imuQuat)

    const eVisual = new THREE.Euler().setFromQuaternion(visualQuat, "YXZ")
    const eIMU = new THREE.Euler().setFromQuaternion(imuCorrected, "YXZ")

    const deltaYaw = eVisual.y - eIMU.y
    const wrapped = Math.atan2(Math.sin(deltaYaw), Math.cos(deltaYaw)) // wrap to [-pi,pi]

    const yawAdjust = wrapped * this.imu.yawBiasStrength
    if (Math.abs(yawAdjust) < 1e-6) return

    const qAdjust = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 1, 0),
      yawAdjust
    )
    this.imuCorrectionQuat.multiply(qAdjust)
  },

  _setFound(found) {
    const { status, statusIcon, statusText, foundIndicator } = this.els
    const container = document.getElementById("ar-container")

    if (found) {
      status.classList.remove("searching")
      status.classList.add("found")
      statusText.textContent = "Tracking"
      foundIndicator.classList.remove("hidden")
      container.classList.add("target-found")
    } else {
      status.classList.remove("found")
      status.classList.add("searching")
      statusText.textContent = "Searching..."
      foundIndicator.classList.add("hidden")
      container.classList.remove("target-found")
    }
  },

  async _initIMU() {
    if (!this.imu.enabled) return

    // Screen orientation
    this._updateScreenOrientation()
    window.addEventListener(
      "orientationchange",
      this._updateScreenOrientation.bind(this)
    )
    if (screen.orientation && screen.orientation.addEventListener) {
      screen.orientation.addEventListener(
        "change",
        this._updateScreenOrientation.bind(this)
      )
    }

    // Permission (iOS)
    if (
      typeof DeviceOrientationEvent !== "undefined" &&
      typeof DeviceOrientationEvent.requestPermission === "function"
    ) {
      try {
        const res = await DeviceOrientationEvent.requestPermission()
        if (res !== "granted") {
          console.warn("[IMU] Permission denied")
          return
        }
      } catch (e) {
        console.warn("[IMU] Permission request failed", e)
        return
      }
    }

    window.addEventListener(
      "deviceorientation",
      this._onDeviceOrientation.bind(this)
    )
    this.imuQuatRaw = new THREE.Quaternion()
    this.imuQuat = new THREE.Quaternion()
    this.imuCorrectionQuat = new THREE.Quaternion()
    console.log("[IMU] Device orientation enabled")
  },

  _updateScreenOrientation() {
    const angle =
      (screen.orientation && typeof screen.orientation.angle === "number"
        ? screen.orientation.angle
        : window.orientation || 0) || 0
    this.screenOrientation = THREE.MathUtils.degToRad(angle)
  },

  _onDeviceOrientation(event) {
    const { alpha, beta, gamma } = event
    if (alpha === null || beta === null || gamma === null) return

    const zee = new THREE.Vector3(0, 0, 1)
    const euler = new THREE.Euler()
    const q0 = new THREE.Quaternion()
    const q1 = new THREE.Quaternion(-Math.sqrt(0.5), 0, 0, Math.sqrt(0.5)) // -PI/2 around X

    const deg2rad = Math.PI / 180
    euler.set(beta * deg2rad, alpha * deg2rad, -gamma * deg2rad, "YXZ")

    const quat = new THREE.Quaternion()
    quat.setFromEuler(euler)
    quat.multiply(q1)
    quat.multiply(q0.setFromAxisAngle(zee, -this.screenOrientation))

    if (!this.imuQuat) this.imuQuat = new THREE.Quaternion()
    if (!this.imuQuatRaw) this.imuQuatRaw = new THREE.Quaternion()

    this.imuQuatRaw.copy(quat)
    // Smooth IMU quaternion
    this.imuQuat.slerp(this.imuQuatRaw, this.imu.imuSmoothing)
    this.imuActive = true
  },

  _showError(message) {
    const container = document.getElementById("ar-container")
    const errorDiv = document.createElement("div")
    errorDiv.className = "error-overlay"
    errorDiv.innerHTML = `
      <h2>Error</h2>
      <p>${message}</p>
    `
    container.appendChild(errorDiv)
  },

  stop() {
    this.isTracking = false
    if (this.animationId) {
      cancelAnimationFrame(this.animationId)
    }
    if (this.video.srcObject) {
      this.video.srcObject.getTracks().forEach((t) => t.stop())
    }
    if (this.engine) {
      this.engine.dispose()
    }
    window.removeEventListener("deviceorientation", this._onDeviceOrientation)
    window.removeEventListener(
      "orientationchange",
      this._updateScreenOrientation
    )
    if (screen.orientation && screen.orientation.removeEventListener) {
      screen.orientation.removeEventListener(
        "change",
        this._updateScreenOrientation
      )
    }
    Renderer.dispose()
  },
}
