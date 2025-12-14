// slam-demo.js - Main SLAM Demonstration Application
//
// This is the main controller for the SLAM prototype demo.
// It orchestrates all SLAM components and provides a complete AR pipeline.
//
// OPTIMIZATION NOTES:
// [OPT-1] Consider Web Workers for CV processing
// [OPT-2] Implement frame skipping for low-end devices
// [OPT-3] Use OffscreenCanvas for parallel processing
// [OPT-4] Implement adaptive quality settings

class SLAMDemo {
    constructor() {
        // ============================================================
        // ASSETS
        // ============================================================
        this.TARGET_IMAGE = '../mobile-ar/assets/ranger-base-image.jpg'
        this.MODEL_3D = '../mobile-ar/assets/ranger-3d-model.glb'

        // ============================================================
        // CONFIGURATION
        // ============================================================
        this.config = {
            processWidth: 640,              // Processing resolution
            markerSize: 0.15,               // Physical marker size (meters)
            enableDebugOverlay: true,       // Show debug visualization
            enableIMU: true,                // Enable IMU fusion
            enableBundleAdjustment: true,   // Enable local BA
            baInterval: 30,                 // Frames between BA runs
        }

        // ============================================================
        // STATE
        // ============================================================
        this.cv = null                    // OpenCV instance
        this.isInitialized = false
        this.isTracking = false
        this.frameNumber = 0

        // Video/Canvas
        this.video = null
        this.canvas = null
        this.ctx = null
        this.canvas3d = null
        this.processCanvas = null
        this.processCtx = null
        this.displayWidth = 0
        this.displayHeight = 0

        // SLAM Components
        this.featureTracker = null
        this.mapManager = null
        this.keyframeHandler = null
        this.poseEstimator = null
        this.bundleAdjuster = null

        // Visualization
        this.debugVisualizer = null
        this.debugCanvas = null

        // Target marker
        this.targetKeypoints = null
        this.targetDescriptors = null
        this.targetSize = null

        // Tracking state
        this.currentPose = null
        this.lastMarkerCorners = null
        this.lostFrameCount = 0
        this.maxLostFrames = 30

        // IMU state
        this.imuQuat = null
        this.imuActive = false

        // Statistics
        this.stats = {
            fps: 0,
            frameTime: 0,
            trackedFeatures: 0,
            mapPoints: 0,
            keyframes: 0,
            poseInliers: 0,
        }
        this.fpsHistory = []
        this.lastFrameTime = 0

        console.log('[SLAMDemo] Created')
    }

    // ============================================================
    // INITIALIZATION
    // ============================================================
    async init() {
        this._updateStatus('Initializing SLAM components...')

        try {
            // Get OpenCV instance
            this.cv = cv

            // Cache DOM elements
            this._cacheElements()

            // Initialize processing canvas
            this.processCanvas = document.createElement('canvas')
            this.processCtx = this.processCanvas.getContext('2d', {
                willReadFrequently: true,
            })

            // Initialize SLAM components
            this._updateStatus('Creating SLAM pipeline...')
            this.featureTracker = new FeatureTracker(this.cv)
            this.mapManager = new MapManager(this.cv)
            this.keyframeHandler = new KeyframeHandler(this.cv)
            this.poseEstimator = new PoseEstimator(this.cv)
            this.bundleAdjuster = new BundleAdjuster(this.cv)

            // Load target image
            this._updateStatus('Loading target image...')
            await this._loadTargetImage()

            // Start camera
            this._updateStatus('Starting camera...')
            await this._startCamera()

            // Initialize pose estimator with camera parameters
            const pHeight = Math.round(
                this.config.processWidth * (this.displayHeight / this.displayWidth)
            )
            this.poseEstimator.init(this.config.processWidth, pHeight)

            // Initialize debug visualizer
            if (this.config.enableDebugOverlay) {
                this.debugVisualizer = new DebugVisualizer(this.debugCanvas)
            }

            // Initialize 3D renderer
            this._updateStatus('Loading 3D model...')
            await this._initRenderer()

            // Initialize IMU
            if (this.config.enableIMU) {
                await this._initIMU()
            }

            // Hide loading, show controls
            document.getElementById('loading').classList.add('hidden')
            document.getElementById('controls').classList.remove('hidden')

            this.isInitialized = true
            this._updateStatus('Ready - Point camera at target', 'success')

            // Start tracking
            this._startTracking()

        } catch (error) {
            console.error('[SLAMDemo] Init failed:', error)
            this._updateStatus(`Error: ${error.message}`, 'error')
        }
    }

    // ============================================================
    // CACHE DOM ELEMENTS
    // ============================================================
    _cacheElements() {
        this.video = document.getElementById('video')
        this.canvas = document.getElementById('canvas')
        this.ctx = this.canvas.getContext('2d', { willReadFrequently: true })
        this.canvas3d = document.getElementById('canvas3d')
        this.debugCanvas = document.getElementById('debug-canvas')
    }

    // ============================================================
    // LOAD TARGET IMAGE
    // ============================================================
    async _loadTargetImage() {
        const img = new Image()
        img.crossOrigin = 'anonymous'

        await new Promise((resolve, reject) => {
            img.onload = resolve
            img.onerror = () => reject(new Error('Failed to load target image'))
            img.src = this.TARGET_IMAGE
        })

        // Extract features from target image
        const tempCanvas = document.createElement('canvas')
        tempCanvas.width = img.width
        tempCanvas.height = img.height
        const tempCtx = tempCanvas.getContext('2d')
        tempCtx.drawImage(img, 0, 0)

        const imageData = tempCtx.getImageData(0, 0, img.width, img.height)

        // Convert to OpenCV Mat
        const cv = this.cv
        const src = cv.matFromImageData(imageData)
        const gray = new cv.Mat()
        cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY)

        // Enhance contrast
        const enhanced = new cv.Mat()
        cv.equalizeHist(gray, enhanced)

        // Detect ORB features
        const orb = new cv.ORB(2000)
        this.targetKeypoints = new cv.KeyPointVector()
        this.targetDescriptors = new cv.Mat()
        orb.detectAndCompute(enhanced, new cv.Mat(), this.targetKeypoints, this.targetDescriptors)

        this.targetSize = { width: img.width, height: img.height }

        // Cleanup
        src.delete()
        gray.delete()
        enhanced.delete()
        orb.delete()

        console.log(`[SLAMDemo] Target loaded: ${this.targetKeypoints.size()} features`)
    }

    // ============================================================
    // START CAMERA
    // ============================================================
    async _startCamera() {
        const stream = await navigator.mediaDevices.getUserMedia({
            video: {
                facingMode: 'environment',
                width: { ideal: 1280 },
                height: { ideal: 720 },
            },
        })

        this.video.srcObject = stream
        await this.video.play()

        // Get container dimensions
        const container = document.getElementById('container')
        const rect = container.getBoundingClientRect()

        this.displayWidth = Math.floor(rect.width)
        this.displayHeight = Math.floor(rect.height)

        // Set canvas sizes
        this.canvas.width = this.displayWidth
        this.canvas.height = this.displayHeight
        this.canvas3d.width = this.displayWidth
        this.canvas3d.height = this.displayHeight
        this.debugCanvas.width = this.displayWidth
        this.debugCanvas.height = this.displayHeight

        console.log(`[SLAMDemo] Camera started: ${this.displayWidth}x${this.displayHeight}`)
    }

    // ============================================================
    // INITIALIZE 3D RENDERER
    // ============================================================
    async _initRenderer() {
        // Reuse renderer from mobile-ar
        Renderer.modelOffset = {
            position: { x: -0.010, y: 0.000, z: 0.030 },
            rotation: { x: -1.571, y: 0.000, z: 0.000 },
            scale: 0.050,
        }

        await Renderer.init(this.canvas3d)
        await Renderer.loadModel(this.MODEL_3D)

        // Set projection matrix
        const projMatrix = this.poseEstimator.getProjectionMatrix(0.01, 100)
        Renderer.setProjectionMatrix(projMatrix)

        console.log('[SLAMDemo] Renderer initialized')
    }

    // ============================================================
    // INITIALIZE IMU
    // ============================================================
    async _initIMU() {
        // Request permission on iOS
        if (typeof DeviceOrientationEvent !== 'undefined' &&
            typeof DeviceOrientationEvent.requestPermission === 'function') {
            try {
                const res = await DeviceOrientationEvent.requestPermission()
                if (res !== 'granted') {
                    console.warn('[SLAMDemo] IMU permission denied')
                    return
                }
            } catch (e) {
                console.warn('[SLAMDemo] IMU permission request failed', e)
                return
            }
        }

        window.addEventListener('deviceorientation', (event) => {
            this._onDeviceOrientation(event)
        })

        this.imuQuat = new THREE.Quaternion()
        console.log('[SLAMDemo] IMU initialized')
    }

    _onDeviceOrientation(event) {
        const { alpha, beta, gamma } = event
        if (alpha === null || beta === null || gamma === null) return

        const deg2rad = Math.PI / 180
        const euler = new THREE.Euler(
            beta * deg2rad,
            alpha * deg2rad,
            -gamma * deg2rad,
            'YXZ'
        )

        const quat = new THREE.Quaternion().setFromEuler(euler)
        const q1 = new THREE.Quaternion(-Math.sqrt(0.5), 0, 0, Math.sqrt(0.5))
        quat.multiply(q1)

        this.imuQuat.slerp(quat, 0.15)
        this.imuActive = true
    }

    // ============================================================
    // START TRACKING
    // ============================================================
    _startTracking() {
        this.isTracking = true
        this.lastFrameTime = performance.now()
        this._loop()
    }

    // ============================================================
    // MAIN PROCESSING LOOP
    // ============================================================
    _loop() {
        if (!this.isTracking) return

        const now = performance.now()
        const dt = now - this.lastFrameTime
        this.lastFrameTime = now

        // Update FPS
        this._updateFPS(dt)

        // Draw video to canvas (cover mode)
        this._drawVideoCover()

        // Clear 3D renderer
        Renderer.clear()

        // Clear debug overlay
        if (this.debugVisualizer) {
            this.debugVisualizer.clear()
        }

        // ============================================================
        // SLAM PROCESSING
        // ============================================================

        // Get grayscale processing frame
        const grayFrame = this._getProcessingFrame()

        // Step 1: Feature tracking
        const trackingResult = this.featureTracker.track(grayFrame)
        this.stats.trackedFeatures = trackingResult.points.length

        // Step 2: Check if we need to detect the marker
        let markerCorners = null
        if (this.lostFrameCount > 10 || this.frameNumber % 30 === 0) {
            markerCorners = this._detectMarker(grayFrame)
        }

        // Step 3: Pose estimation
        let poseResult = null

        if (markerCorners) {
            // Use marker for pose
            this.lastMarkerCorners = markerCorners
            this.lostFrameCount = 0

            poseResult = this.poseEstimator.estimatePoseFromMarker(
                markerCorners,
                this.config.markerSize,
                now
            )

            if (poseResult) {
                this.currentPose = poseResult.pose
                this.stats.poseInliers = poseResult.inlierCount

                // Create keyframe if needed
                this._maybeCreateKeyframe(grayFrame, trackingResult, poseResult)
            }
        } else if (this.currentPose && trackingResult.points.length > 20) {
            // Try to estimate pose from tracked features + map points
            const correspondences = this.mapManager.getCorrespondences(
                trackingResult.points,
                this.keyframeHandler.lastKeyframe?.id
            )

            if (correspondences.points3D.length >= 6) {
                poseResult = this.poseEstimator.estimatePose(
                    correspondences.points2D,
                    correspondences.points3D,
                    now
                )

                if (poseResult) {
                    this.currentPose = poseResult.pose
                    this.stats.poseInliers = poseResult.inlierCount
                }
            }

            this.lostFrameCount++
        } else {
            this.lostFrameCount++
        }

        // Step 4: Update map points
        if (poseResult && this.keyframeHandler.lastKeyframe) {
            this._updateMapPoints(trackingResult, poseResult)
        }

        // Step 5: Cull map points periodically
        if (this.frameNumber % 50 === 0) {
            this.mapManager.cullPoints(this.frameNumber)
        }

        // Step 6: Bundle adjustment periodically
        if (this.config.enableBundleAdjustment &&
            this.frameNumber % this.config.baInterval === 0 &&
            this.keyframeHandler.keyframes.length >= 3) {
            this._runBundleAdjustment()
        }

        // Step 7: Render 3D model
        if (this.currentPose && this.lostFrameCount < this.maxLostFrames) {
            let finalPose = this._toThreeMatrix(this.currentPose)

            // Blend with IMU
            if (this.config.enableIMU && this.imuActive) {
                finalPose = this._blendWithIMU(finalPose)
            }

            Renderer.updateWithPose(finalPose)
        } else {
            Renderer.setVisible(false)
        }

        Renderer.render()

        // Step 8: Debug visualization
        if (this.debugVisualizer && this.config.enableDebugOverlay) {
            this._drawDebugOverlay(trackingResult, markerCorners)
        }

        // Update statistics
        this._updateStats()

        // Cleanup
        grayFrame.delete()

        this.frameNumber++
        requestAnimationFrame(() => this._loop())
    }

    // ============================================================
    // DRAW VIDEO (COVER MODE)
    // ============================================================
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
    }

    // ============================================================
    // GET PROCESSING FRAME
    // ============================================================
    _getProcessingFrame() {
        const cv = this.cv
        const pWidth = this.config.processWidth
        const pHeight = Math.round(pWidth * (this.displayHeight / this.displayWidth))

        if (this.processCanvas.width !== pWidth || this.processCanvas.height !== pHeight) {
            this.processCanvas.width = pWidth
            this.processCanvas.height = pHeight
        }

        // Draw video to processing canvas
        const { sx, sy, sw, sh } = this.videoCrop
        this.processCtx.drawImage(this.video, sx, sy, sw, sh, 0, 0, pWidth, pHeight)

        // Get ImageData
        const imageData = this.processCtx.getImageData(0, 0, pWidth, pHeight)

        // Convert to grayscale
        const src = cv.matFromImageData(imageData)
        const gray = new cv.Mat()
        cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY)
        src.delete()

        return gray
    }

    // ============================================================
    // DETECT MARKER
    // ============================================================
    _detectMarker(grayFrame) {
        const cv = this.cv

        // Enhance contrast
        const enhanced = new cv.Mat()
        cv.equalizeHist(grayFrame, enhanced)

        // Detect ORB features
        const orb = new cv.ORB(1000)
        const keypoints = new cv.KeyPointVector()
        const descriptors = new cv.Mat()
        orb.detectAndCompute(enhanced, new cv.Mat(), keypoints, descriptors)

        enhanced.delete()

        if (descriptors.rows < 10) {
            keypoints.delete()
            descriptors.delete()
            orb.delete()
            return null
        }

        // Match with target
        const matcher = new cv.BFMatcher(cv.NORM_HAMMING, false)
        const matches = new cv.DMatchVectorVector()
        matcher.knnMatch(this.targetDescriptors, descriptors, matches, 2)

        // Ratio test
        const goodMatches = []
        for (let i = 0; i < matches.size(); i++) {
            const pair = matches.get(i)
            if (pair.size() >= 2) {
                const m = pair.get(0)
                const n = pair.get(1)
                if (m.distance < 0.8 * n.distance) {
                    goodMatches.push(m)
                }
            }
        }

        matches.delete()

        let corners = null

        if (goodMatches.length >= 10) {
            // Compute homography
            const srcPoints = []
            const dstPoints = []

            for (const match of goodMatches) {
                const tkp = this.targetKeypoints.get(match.queryIdx)
                const fkp = keypoints.get(match.trainIdx)
                srcPoints.push(tkp.pt.x, tkp.pt.y)
                dstPoints.push(fkp.pt.x, fkp.pt.y)
            }

            const srcMat = cv.matFromArray(goodMatches.length, 1, cv.CV_32FC2, srcPoints)
            const dstMat = cv.matFromArray(goodMatches.length, 1, cv.CV_32FC2, dstPoints)
            const inliers = new cv.Mat()

            const H = cv.findHomography(srcMat, dstMat, cv.RANSAC, 4.0, inliers)

            // Count inliers
            let inlierCount = 0
            for (let i = 0; i < inliers.rows; i++) {
                if (inliers.data[i]) inlierCount++
            }

            srcMat.delete()
            dstMat.delete()
            inliers.delete()

            if (inlierCount >= 6 && !H.empty()) {
                // Get corners
                const { width, height } = this.targetSize
                const targetCorners = cv.matFromArray(4, 1, cv.CV_32FC2, [
                    0, 0, width, 0, width, height, 0, height,
                ])
                const transformedCorners = new cv.Mat()
                cv.perspectiveTransform(targetCorners, transformedCorners, H)

                // Scale to display coordinates
                const scaleX = this.displayWidth / grayFrame.cols
                const scaleY = this.displayHeight / grayFrame.rows

                corners = []
                for (let i = 0; i < 4; i++) {
                    corners.push({
                        x: transformedCorners.data32F[i * 2] * scaleX,
                        y: transformedCorners.data32F[i * 2 + 1] * scaleY,
                    })
                }

                targetCorners.delete()
                transformedCorners.delete()
            }

            H.delete()
        }

        keypoints.delete()
        descriptors.delete()
        matcher.delete()
        orb.delete()

        return corners
    }

    // ============================================================
    // MAYBE CREATE KEYFRAME
    // ============================================================
    _maybeCreateKeyframe(grayFrame, trackingResult, poseResult) {
        const should = this.keyframeHandler.shouldCreateKeyframe(
            poseResult.pose,
            trackingResult,
            this.frameNumber,
            { covisiblePoints: this.mapManager.stats.activePoints }
        )

        if (should) {
            const keyframe = this.keyframeHandler.createKeyframe({
                frameNumber: this.frameNumber,
                pose: poseResult.pose,
                features: trackingResult.points,
                grayFrame: grayFrame,
            })

            if (keyframe) {
                // Add to map manager
                this.mapManager.addKeyframe(keyframe)
                this.stats.keyframes = this.keyframeHandler.keyframes.length
            }
        }
    }

    // ============================================================
    // UPDATE MAP POINTS
    // ============================================================
    _updateMapPoints(trackingResult, poseResult) {
        // Try to triangulate new points between current frame and last keyframe
        const lastKF = this.keyframeHandler.lastKeyframe
        if (!lastKF || !poseResult) return

        // For each tracked point not in map, try to triangulate
        // [OPT-5] Implement batch triangulation
        for (const point of trackingResult.points) {
            if (point.age < 5) continue  // Wait for stable tracking

            // Check if point is already in map
            const key = `${lastKF.id}_${point.id}`
            if (this.mapManager.featureToPoint.has(key)) continue

            // Find corresponding point in last keyframe
            const kfFeature = lastKF.features.find(f => f.id === point.id)
            if (!kfFeature) continue

            // Triangulate (simplified - using marker plane assumption)
            // [OPT-6] Implement proper triangulation with multiple views
        }

        this.stats.mapPoints = this.mapManager.stats.activePoints
    }

    // ============================================================
    // RUN BUNDLE ADJUSTMENT
    // ============================================================
    _runBundleAdjustment() {
        const camera = {
            fx: this.config.processWidth,
            fy: this.config.processWidth,
            cx: this.config.processWidth / 2,
            cy: (this.config.processWidth * this.displayHeight / this.displayWidth) / 2,
        }

        this.bundleAdjuster.optimizeLocal(
            this.keyframeHandler.keyframes,
            this.mapManager.mapPoints,
            camera
        )
    }

    // ============================================================
    // BLEND WITH IMU
    // ============================================================
    _blendWithIMU(poseMatrix) {
        if (!this.imuQuat) return poseMatrix

        const pos = new THREE.Vector3()
        const quat = new THREE.Quaternion()
        const scale = new THREE.Vector3()
        poseMatrix.decompose(pos, quat, scale)

        // Blend rotation
        quat.slerp(this.imuQuat, 0.3)

        const result = new THREE.Matrix4()
        result.compose(pos, quat, scale)
        return result
    }

    // ============================================================
    // CONVERT TO THREE.Matrix4
    // ============================================================
    _toThreeMatrix(pose) {
        const matrix = new THREE.Matrix4()
        matrix.fromArray(pose)
        return matrix
    }

    // ============================================================
    // DRAW DEBUG OVERLAY
    // ============================================================
    _drawDebugOverlay(trackingResult, markerCorners) {
        const vis = this.debugVisualizer

        // Draw tracked features (scaled to display coords)
        const scaleX = this.displayWidth / this.config.processWidth
        const scaleY = this.displayHeight / (this.config.processWidth * this.displayHeight / this.displayWidth)

        const scaledPoints = trackingResult.points.map(p => ({
            ...p,
            x: p.x * scaleX,
            y: p.y * scaleY,
        }))

        vis.drawTrackedFeatures({ points: scaledPoints, stats: trackingResult.stats })

        // Draw marker corners
        if (markerCorners) {
            vis.drawMarkerCorners(markerCorners, '#00FF00')
        } else if (this.lastMarkerCorners && this.lostFrameCount < 10) {
            vis.drawMarkerCorners(this.lastMarkerCorners, '#FFFF00')
        }

        // Draw SLAM stats
        vis.drawStats({
            'FPS': Math.round(this.stats.fps),
            'Frame': this.frameNumber,
            'Features': this.stats.trackedFeatures,
            'Map Points': this.stats.mapPoints,
            'Keyframes': this.stats.keyframes,
            'Inliers': this.stats.poseInliers,
            'Lost': this.lostFrameCount,
        }, 10, this.displayHeight - 200)
    }

    // ============================================================
    // UPDATE FPS
    // ============================================================
    _updateFPS(dt) {
        this.fpsHistory.push(1000 / dt)
        if (this.fpsHistory.length > 30) {
            this.fpsHistory.shift()
        }
        this.stats.fps = this.fpsHistory.reduce((a, b) => a + b, 0) / this.fpsHistory.length
        this.stats.frameTime = dt
    }

    // ============================================================
    // UPDATE STATS DISPLAY
    // ============================================================
    _updateStats() {
        // Update DOM stats if available
        const fpsEl = document.getElementById('fps')
        if (fpsEl) {
            fpsEl.textContent = Math.round(this.stats.fps)
        }
    }

    // ============================================================
    // UPDATE STATUS MESSAGE
    // ============================================================
    _updateStatus(message, type = 'info') {
        const statusEl = document.getElementById('status-text')
        if (statusEl) {
            statusEl.textContent = message
            statusEl.className = `status-${type}`
        }
        console.log(`[SLAMDemo] ${message}`)
    }

    // ============================================================
    // TOGGLE DEBUG OVERLAY
    // ============================================================
    toggleDebug() {
        this.config.enableDebugOverlay = !this.config.enableDebugOverlay
        this.debugCanvas.style.display = this.config.enableDebugOverlay ? 'block' : 'none'
    }

    // ============================================================
    // RESET SLAM
    // ============================================================
    reset() {
        this.featureTracker.reset()
        this.mapManager.reset()
        this.keyframeHandler.reset()
        this.poseEstimator.reset()
        this.bundleAdjuster.reset()

        this.currentPose = null
        this.lastMarkerCorners = null
        this.lostFrameCount = 0
        this.frameNumber = 0

        this.stats = {
            fps: 0,
            frameTime: 0,
            trackedFeatures: 0,
            mapPoints: 0,
            keyframes: 0,
            poseInliers: 0,
        }

        if (this.debugVisualizer) {
            this.debugVisualizer.clearTrails()
        }

        this._updateStatus('SLAM reset', 'info')
    }

    // ============================================================
    // STOP
    // ============================================================
    stop() {
        this.isTracking = false

        if (this.video.srcObject) {
            this.video.srcObject.getTracks().forEach(t => t.stop())
        }

        this.featureTracker.dispose()
        this.mapManager.dispose()
        this.keyframeHandler.dispose()
        this.poseEstimator.dispose()
        this.bundleAdjuster.dispose()

        if (this.debugVisualizer) {
            this.debugVisualizer.dispose()
        }

        Renderer.dispose()

        console.log('[SLAMDemo] Stopped')
    }
}

// ============================================================
// GLOBAL INSTANCE
// ============================================================
const slamDemo = new SLAMDemo()
