// app.js - Main AR Application

const App = {
  // Assets
  TARGET_IMAGE: "assets/ranger-base-image.jpg",
  MODEL_3D: "assets/ranger-3d-model.glb",

  // State
  worker: null,
  video: null,
  canvas: null,
  ctx: null,
  canvas3d: null,
  isTracking: false,
  animationId: null,

  // Processing
  processCanvas: null,
  processCtx: null,
  processWidth: 480,
  sharedBuffer: null,
  sharedView: null,
  workerBusy: false,

  // Pose & Prediction
  lastPose: null,
  lastPoseTime: 0,
  velocity: {
    position: new THREE.Vector3(),
    rotation: new THREE.Quaternion(),
    scale: new THREE.Vector3()
  },

  // Smoothing (Visual only for now, Phase 2 will improve this)
  smoothedCorners: null,
  lastFoundTime: 0,

  // IMU fusion (Phase 4 placeholder/prep)
  imu: { enabled: true },

  // DOM elements
  els: {},

  async init() {
    this._cacheElements()
    this._updateLoading("Initializing Worker...")

    try {
      // 1. Initialize Worker (OpenCV)
      this.worker = new Worker('cv-worker.js')
      this.worker.onmessage = (e) => this._onWorkerMessage(e)

      await new Promise(resolve => {
        this._workerReadyResolve = resolve
      })
      this.worker.postMessage({ type: 'INIT' })

      // 2. Start camera
      this._updateLoading("Starting camera...")
      await this._startCamera()

      // 3. Initialize 3D renderer & Model
      this._updateLoading("Loading 3D model...")
      await this._initRenderer()
      await Renderer.loadModel(this.MODEL_3D)

      // 4. Load target image
      this._updateLoading("Loading target image...")
      await this._loadTarget()

      // 5. Setup Helper Canvas & SharedArrayBuffer
      this.processCanvas = document.createElement("canvas")
      this.processCtx = this.processCanvas.getContext("2d", { willReadFrequently: true })
      this._setupSharedBuffer()

      // Hide loading, start tracking
      this.els.loading.classList.add("hidden")
      this.els.status.classList.remove("hidden")
      this._startTracking()

    } catch (err) {
      console.error("[App] Init failed:", err)
      this._showError(err.message)
    }
  },

  _onWorkerMessage(e) {
    const { type, data, result, corners, matrix, timestamp } = e.data

    switch (type) {
      case 'CV_READY':
        if (this._workerReadyResolve) this._workerReadyResolve()
        break

      case 'TARGET_LOADED':
        console.log("[App] Target loaded:", result)
        break

      case 'PROJECTION_MATRIX':
        Renderer.setProjectionMatrix(matrix)
        break

      case 'POSE':
        this.workerBusy = false
        this._handleWorkerPose(result, corners, timestamp)
        break
    }
  },

  _handleWorkerPose(pose, corners, timestamp) {
    const now = Date.now()

    if (pose && pose.matrix) {
      this.lastFoundTime = now

      // Convert array to Matrix4
      const mat = new THREE.Matrix4().fromArray(pose.matrix)

      // Calculate velocity for Dead Reckoning
      if (this.lastPose) {
        const dt = (now - this.lastPoseTime) / 1000
        if (dt > 0) {
          // Position velocity
          const currPos = new THREE.Vector3()
          const prevPos = new THREE.Vector3()
          mat.decompose(currPos, new THREE.Quaternion(), new THREE.Vector3())
          this.lastPose.decompose(prevPos, new THREE.Quaternion(), new THREE.Vector3())

          this.velocity.position.subVectors(currPos, prevPos).divideScalar(dt)
        }
      }

      this.lastPose = mat
      this.lastPoseTime = now
      this._setFound(true)

      // Debug corners
      this.smoothedCorners = corners // Map to screen coords if needed, but we usually map in render
    } else {
      // Lost tracking
      if (now - this.lastFoundTime > 200) {
        this._setFound(false)
        this.velocity.position.set(0, 0, 0)
        this.lastPose = null
        Renderer.setVisible(false)
      }
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

    // Send to worker
    // Note: We can't send ImageData directly usually, but we can send the data buffer
    // Or just clean object.
    this.worker.postMessage({
      type: 'LOAD_TARGET',
      imageData: {
        width: imageData.width,
        height: imageData.height,
        data: imageData.data
      }
    })
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

    // Get container size
    const container = document.getElementById("ar-container")
    const rect = container.getBoundingClientRect()
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

  _setupSharedBuffer() {
    // Processing size
    const cw = this.canvas.width
    const ch = this.canvas.height
    const pWidth = this.processWidth
    const pHeight = Math.round(pWidth * (ch / cw))

    this.processCanvas.width = pWidth
    this.processCanvas.height = pHeight

    // Init SAB
    try {
      if (window.SharedArrayBuffer) {
        this.sharedBuffer = new SharedArrayBuffer(pWidth * pHeight * 4)
        this.sharedView = new Uint8ClampedArray(this.sharedBuffer)
        console.log("[App] SharedArrayBuffer enabled")
      } else {
        console.warn("[App] SharedArrayBuffer NOT supported, falling back to copy")
      }

      // Inform worker
      this.worker.postMessage({
        type: 'SETUP_SHARED',
        buffer: this.sharedBuffer, // Will be null if not supported, worker should handle
        width: pWidth,
        height: pHeight
      })

      // Also trigger resize in worker to init PoseSolver
      this.worker.postMessage({
        type: 'RESIZE',
        width: pWidth,
        height: pHeight
      })

    } catch (e) {
      console.warn("[App] Error setting up SharedArrayBuffer:", e)
    }
  },

  _startTracking() {
    this.isTracking = true
    this._loop()
  },

  _drawVideoCover() {
    const vw = this.video.videoWidth
    const vh = this.video.videoHeight
    const cw = this.canvas.width
    const ch = this.canvas.height
    const videoAspect = vw / vh
    const canvasAspect = cw / ch
    let sx, sy, sw, sh

    if (canvasAspect > videoAspect) {
      sw = vw
      sh = vw / canvasAspect
      sx = 0
      sy = (vh - sh) / 2
    } else {
      sh = vh
      sw = vh * canvasAspect
      sx = (vw - sw) / 2
      sy = 0
    }
    this.videoCrop = { sx, sy, sw, sh }
    this.ctx.drawImage(this.video, sx, sy, sw, sh, 0, 0, cw, ch)
  },

  _loop() {
    if (!this.isTracking) return

    // 1. Render Video
    this._drawVideoCover()

    if (Renderer.updateExternalTexture) {
      Renderer.updateExternalTexture(this.video)
    }

    Renderer.clear()

    // 2. Dead Reckoning (Prediction) on Main Thread
    if (this.lastPose) {
      const dt = (Date.now() - this.lastPoseTime) / 1000
      // Simple linear prediction for position
      const predictedPos = new THREE.Vector3()
      const posePos = new THREE.Vector3()
      const poseRot = new THREE.Quaternion()
      const poseScale = new THREE.Vector3()

      this.lastPose.decompose(posePos, poseRot, poseScale)
      predictedPos.copy(posePos).addScaledVector(this.velocity.position, dt)

      // Reconstruct matrix
      const predictedMatrix = new THREE.Matrix4()
      predictedMatrix.compose(predictedPos, poseRot, poseScale)

      Renderer.updateWithPose(predictedMatrix)
    }

    Renderer.render()

    // 3. Offload Processing
    if (!this.workerBusy) {
      this._sendFrameToWorker()
    }

    this.animationId = requestAnimationFrame(() => this._loop())
  },

  _sendFrameToWorker() {
    if (!this.videoCrop) return

    this.workerBusy = true
    const { sx, sy, sw, sh } = this.videoCrop
    const w = this.processCanvas.width
    const h = this.processCanvas.height

    // Draw to small canvas
    this.processCtx.drawImage(this.video, sx, sy, sw, sh, 0, 0, w, h)

    // Get data
    const imageData = this.processCtx.getImageData(0, 0, w, h)

    if (this.sharedView) {
      // Copy to SharedBuffer
      this.sharedView.set(imageData.data)
      this.worker.postMessage({
        type: 'PROCESS',
        useShared: true,
        timestamp: Date.now()
      })
    } else {
      // Fallback: Post full data
      this.worker.postMessage({
        type: 'PROCESS',
        imageData: {
          data: imageData.data, // This will be cloned
          width: w,
          height: h
        },
        timestamp: Date.now()
      })
    }
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

  _showError(message) {
    const container = document.getElementById("ar-container")
    const errorDiv = document.createElement("div")
    errorDiv.className = "error-overlay"
    errorDiv.innerHTML = `<h2>Error</h2><p>${message}</p>`
    container.appendChild(errorDiv)
  },

  stop() {
    this.isTracking = false
    if (this.animationId) cancelAnimationFrame(this.animationId)
    if (this.video.srcObject) this.video.srcObject.getTracks().forEach((t) => t.stop())
    if (this.worker) this.worker.terminate()
    Renderer.dispose()
  }
}
