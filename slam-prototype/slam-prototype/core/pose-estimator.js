// pose-estimator.js - High-Accuracy 6DoF Pose Estimation with Motion Model
//
// OPTIMIZATION NOTES:
// [OPT-1] Consider GPU-accelerated PnP using WebGL compute
// [OPT-2] Implement pose prediction using Kalman filter
// [OPT-3] Use efficient rotation representations (quaternions) internally
// [OPT-4] Consider using EPnP for initial estimate, then refine with LM
// [OPT-5] Implement multi-hypothesis tracking for ambiguous poses

class PoseEstimator {
    constructor(cv) {
        this.cv = cv

        // ============================================================
        // CONFIGURATION - Tuned for highest accuracy
        // ============================================================
        this.config = {
            // PnP solver settings
            pnpMethod: 'SOLVEPNP_ITERATIVE',   // Most accurate for planar targets
            iterativeIterations: 100,           // LM iterations
            iterativeReprojError: 1e-8,         // Convergence threshold

            // RANSAC settings (for PnP RANSAC)
            ransacIterations: 200,              // Higher = more robust
            ransacReprojThreshold: 2.0,         // Pixels
            ransacConfidence: 0.999,            // Target confidence

            // Minimum correspondences
            minCorrespondences: 6,              // Minimum 2D-3D matches
            minInlierRatio: 0.5,                // Minimum inlier ratio for valid pose

            // Motion model
            useMotionModel: true,
            motionDecay: 0.9,                   // Velocity decay per frame
            motionPredictionWeight: 0.3,        // Weight of motion prediction

            // Pose filtering
            usePoseFilter: true,
            positionFilterAlpha: 0.7,           // Position smoothing (higher = more responsive)
            rotationFilterAlpha: 0.7,           // Rotation smoothing

            // Outlier rejection
            maxPositionJump: 0.3,               // Maximum position change (meters) per frame
            maxRotationJump: 30,                // Maximum rotation change (degrees) per frame
        }

        // ============================================================
        // STATE
        // ============================================================

        // Camera intrinsics
        this.cameraMatrix = null            // 3x3 camera matrix
        this.distCoeffs = null              // Distortion coefficients
        this.imageWidth = 0
        this.imageHeight = 0

        // Pose state
        this.currentPose = null             // Current pose (4x4 matrix as Float64Array)
        this.lastPose = null                // Previous pose
        this.velocity = {                   // Estimated velocity
            linear: [0, 0, 0],                // m/s
            angular: [0, 0, 0],               // rad/s
        }
        this.lastTimestamp = 0

        // Pose history for filtering
        this.poseHistory = []
        this.maxPoseHistory = 10

        // Statistics
        this.stats = {
            correspondenceCount: 0,
            inlierCount: 0,
            inlierRatio: 0,
            reprojectionError: 0,
            motionPredictionUsed: false,
            processingTimeMs: 0,
        }

        console.log('[PoseEstimator] Initialized')
    }

    // ============================================================
    // INITIALIZE CAMERA
    // ============================================================
    init(imageWidth, imageHeight, focalLength = null) {
        const cv = this.cv

        this.imageWidth = imageWidth
        this.imageHeight = imageHeight

        // Camera intrinsics (approximation for mobile cameras)
        // Focal length ≈ image width for typical mobile FOV (~60°)
        const fx = focalLength || imageWidth
        const fy = focalLength || imageWidth
        const cx = imageWidth / 2
        const cy = imageHeight / 2

        this.cameraMatrix = cv.matFromArray(3, 3, cv.CV_64F, [
            fx, 0, cx,
            0, fy, cy,
            0, 0, 1,
        ])

        // No lens distortion (mobile cameras usually pre-correct)
        this.distCoeffs = cv.matFromArray(4, 1, cv.CV_64F, [0, 0, 0, 0])

        console.log(`[PoseEstimator] Camera initialized: ${imageWidth}x${imageHeight}, f=${fx}`)
    }

    // ============================================================
    // ESTIMATE POSE FROM 2D-3D CORRESPONDENCES
    // ============================================================
    // 
    // Core pose estimation using PnP with RANSAC.
    // Incorporates motion prediction and pose filtering.
    // 
    // @param {Array} points2D - 2D image points [[x, y], ...]
    // @param {Array} points3D - 3D world points [[x, y, z], ...]
    // @param {number} timestamp - Current timestamp (ms)
    // @returns {Object|null} - Pose result or null if failed
    // ============================================================
    estimatePose(points2D, points3D, timestamp = Date.now()) {
        const startTime = performance.now()
        const cv = this.cv

        // ============================================================
        // STEP 1: Validate input
        // ============================================================

        if (!points2D || !points3D ||
            points2D.length !== points3D.length ||
            points2D.length < this.config.minCorrespondences) {
            this.stats.correspondenceCount = points2D?.length || 0
            this.stats.processingTimeMs = performance.now() - startTime
            return null
        }

        this.stats.correspondenceCount = points2D.length

        // ============================================================
        // STEP 2: Motion prediction
        // ============================================================
        // [OPT-6] Implement extended Kalman filter for better prediction

        let predictedPose = null
        if (this.config.useMotionModel && this.lastPose && this.lastTimestamp > 0) {
            const dt = (timestamp - this.lastTimestamp) / 1000  // seconds
            predictedPose = this._predictPose(this.lastPose, this.velocity, dt)
            this.stats.motionPredictionUsed = true
        } else {
            this.stats.motionPredictionUsed = false
        }

        // ============================================================
        // STEP 3: SolvePnP with RANSAC
        // ============================================================

        // Convert to OpenCV format
        const objectPoints = cv.matFromArray(
            points3D.length, 1, cv.CV_64FC3,
            points3D.flat()
        )
        const imagePoints = cv.matFromArray(
            points2D.length, 1, cv.CV_64FC2,
            points2D.flat()
        )

        const rvec = new cv.Mat()
        const tvec = new cv.Mat()
        const inliers = new cv.Mat()

        // Use initial guess from prediction if available
        let useExtrinsicGuess = false
        if (predictedPose) {
            const { rvecInit, tvecInit } = this._poseToRvecTvec(predictedPose)
            rvec.delete()
            tvec.delete()
            // Copy initial values
            // [OPT-7] Implement proper initial guess usage
            useExtrinsicGuess = false  // Disabled for now - needs proper implementation
        }

        // SolvePnP RANSAC for robustness
        // [OPT-8] Consider using solvePnPGeneric for multiple solutions
        const success = cv.solvePnPRansac(
            objectPoints,
            imagePoints,
            this.cameraMatrix,
            this.distCoeffs,
            rvec,
            tvec,
            useExtrinsicGuess,
            this.config.ransacIterations,
            this.config.ransacReprojThreshold,
            this.config.ransacConfidence,
            inliers
        )

        // Check inlier count
        const inlierCount = inliers.rows
        const inlierRatio = inlierCount / points2D.length

        this.stats.inlierCount = inlierCount
        this.stats.inlierRatio = inlierRatio

        // Clean up input mats
        objectPoints.delete()
        imagePoints.delete()

        if (!success || inlierRatio < this.config.minInlierRatio) {
            rvec.delete()
            tvec.delete()
            inliers.delete()
            this.stats.processingTimeMs = performance.now() - startTime
            return null
        }

        // ============================================================
        // STEP 4: Refine with all inliers using iterative method
        // ============================================================
        // [OPT-9] Consider bundle adjustment for multi-view refinement

        const inlierIndices = []
        for (let i = 0; i < inliers.rows; i++) {
            inlierIndices.push(inliers.data32S[i])
        }
        inliers.delete()

        // Extract inlier points
        const inlierPoints3D = inlierIndices.map(i => points3D[i])
        const inlierPoints2D = inlierIndices.map(i => points2D[i])

        const objectPointsInliers = cv.matFromArray(
            inlierPoints3D.length, 1, cv.CV_64FC3,
            inlierPoints3D.flat()
        )
        const imagePointsInliers = cv.matFromArray(
            inlierPoints2D.length, 1, cv.CV_64FC2,
            inlierPoints2D.flat()
        )

        // Refine using iterative solver
        cv.solvePnP(
            objectPointsInliers,
            imagePointsInliers,
            this.cameraMatrix,
            this.distCoeffs,
            rvec,
            tvec,
            true,  // useExtrinsicGuess - refine from RANSAC result
            cv.SOLVEPNP_ITERATIVE
        )

        // Calculate reprojection error
        const projectedPoints = new cv.Mat()
        cv.projectPoints(
            objectPointsInliers,
            rvec,
            tvec,
            this.cameraMatrix,
            this.distCoeffs,
            projectedPoints
        )

        let totalError = 0
        for (let i = 0; i < projectedPoints.rows; i++) {
            const px = projectedPoints.data64F[i * 2]
            const py = projectedPoints.data64F[i * 2 + 1]
            const dx = px - inlierPoints2D[i][0]
            const dy = py - inlierPoints2D[i][1]
            totalError += Math.sqrt(dx * dx + dy * dy)
        }
        this.stats.reprojectionError = totalError / projectedPoints.rows

        projectedPoints.delete()
        objectPointsInliers.delete()
        imagePointsInliers.delete()

        // ============================================================
        // STEP 5: Convert to pose matrix
        // ============================================================

        // Convert rotation vector to rotation matrix
        const rotMat = new cv.Mat()
        cv.Rodrigues(rvec, rotMat)

        // Extract values
        const R = []
        for (let i = 0; i < 9; i++) R.push(rotMat.data64F[i])
        const t = [tvec.data64F[0], tvec.data64F[1], tvec.data64F[2]]

        rvec.delete()
        tvec.delete()
        rotMat.delete()

        // Build 4x4 pose matrix with coordinate conversion
        const rawPose = this._buildPoseMatrix(R, t)

        // ============================================================
        // STEP 6: Outlier rejection
        // ============================================================

        if (this.lastPose && !this._isValidPoseTransition(this.lastPose, rawPose)) {
            console.warn('[PoseEstimator] Pose jump rejected')
            this.stats.processingTimeMs = performance.now() - startTime
            return null
        }

        // ============================================================
        // STEP 7: Pose filtering
        // ============================================================

        let filteredPose
        if (this.config.usePoseFilter && this.lastPose) {
            filteredPose = this._filterPose(this.lastPose, rawPose)
        } else {
            filteredPose = rawPose
        }

        // ============================================================
        // STEP 8: Update velocity estimate
        // ============================================================

        if (this.lastPose && this.lastTimestamp > 0) {
            const dt = (timestamp - this.lastTimestamp) / 1000
            if (dt > 0) {
                this._updateVelocity(this.lastPose, filteredPose, dt)
            }
        }

        // ============================================================
        // STEP 9: Store state
        // ============================================================

        this.lastPose = this.currentPose
        this.currentPose = filteredPose
        this.lastTimestamp = timestamp

        // Add to history
        this.poseHistory.push({
            pose: new Float64Array(filteredPose),
            timestamp,
        })
        if (this.poseHistory.length > this.maxPoseHistory) {
            this.poseHistory.shift()
        }

        this.stats.processingTimeMs = performance.now() - startTime

        // ============================================================
        // STEP 10: Build result
        // ============================================================

        return {
            pose: filteredPose,
            matrix: this._toThreeMatrix4(filteredPose),
            position: [filteredPose[12], filteredPose[13], filteredPose[14]],
            rotation: this._extractEuler(filteredPose),
            inlierCount,
            inlierRatio,
            reprojectionError: this.stats.reprojectionError,
        }
    }

    // ============================================================
    // ESTIMATE POSE FROM MARKER CORNERS (Like original system)
    // ============================================================
    // 
    // Convenience method for planar marker tracking.
    // Uses known marker size and corner positions.
    // 
    // @param {Array} corners - 4 corner points in image coords [{x, y}, ...]
    // @param {number} markerSize - Physical marker size in meters
    // @returns {Object|null} - Pose result
    // ============================================================
    estimatePoseFromMarker(corners, markerSize = 0.15, timestamp = Date.now()) {
        if (!corners || corners.length !== 4) return null

        const half = markerSize / 2

        // 3D object points: marker centered at origin, lying on XY plane (Z=0)
        // Order matches corner order: TL, TR, BR, BL
        const points3D = [
            [-half, half, 0],   // TL
            [half, half, 0],    // TR
            [half, -half, 0],   // BR
            [-half, -half, 0],  // BL
        ]

        // 2D image points from detection
        const points2D = corners.map(c => [c.x, c.y])

        return this.estimatePose(points2D, points3D, timestamp)
    }

    // ============================================================
    // PREDICT POSE FROM MOTION MODEL
    // ============================================================
    _predictPose(lastPose, velocity, dt) {
        // Simple constant velocity prediction
        // [OPT-10] Implement proper SE3 integration

        const predicted = new Float64Array(16)
        predicted.set(lastPose)

        // Apply linear velocity
        predicted[12] += velocity.linear[0] * dt
        predicted[13] += velocity.linear[1] * dt
        predicted[14] += velocity.linear[2] * dt

        // Apply angular velocity (simplified - just using current rotation)
        // [OPT-11] Implement proper rotation integration using exponential map

        return predicted
    }

    // ============================================================
    // UPDATE VELOCITY ESTIMATE
    // ============================================================
    _updateVelocity(prevPose, currPose, dt) {
        if (dt <= 0) return

        // Linear velocity
        this.velocity.linear[0] = (currPose[12] - prevPose[12]) / dt
        this.velocity.linear[1] = (currPose[13] - prevPose[13]) / dt
        this.velocity.linear[2] = (currPose[14] - prevPose[14]) / dt

        // Apply decay
        this.velocity.linear[0] *= this.config.motionDecay
        this.velocity.linear[1] *= this.config.motionDecay
        this.velocity.linear[2] *= this.config.motionDecay

        // Angular velocity (simplified)
        // [OPT-12] Implement proper SO3 velocity computation
        this.velocity.angular = [0, 0, 0]
    }

    // ============================================================
    // FILTER POSE (EXPONENTIAL SMOOTHING)
    // ============================================================
    _filterPose(prevPose, newPose) {
        const filtered = new Float64Array(16)

        const posAlpha = this.config.positionFilterAlpha
        const rotAlpha = this.config.rotationFilterAlpha

        // Filter rotation (3x3 part) - simplified linear interpolation
        // [OPT-13] Use proper quaternion slerp for rotation filtering
        for (let i = 0; i < 12; i++) {
            if (i === 3 || i === 7 || i === 11) {
                filtered[i] = newPose[i]  // Skip translation column
            } else {
                filtered[i] = prevPose[i] * (1 - rotAlpha) + newPose[i] * rotAlpha
            }
        }

        // Filter translation
        filtered[12] = prevPose[12] * (1 - posAlpha) + newPose[12] * posAlpha
        filtered[13] = prevPose[13] * (1 - posAlpha) + newPose[13] * posAlpha
        filtered[14] = prevPose[14] * (1 - posAlpha) + newPose[14] * posAlpha
        filtered[15] = 1

        // Re-orthogonalize rotation matrix
        // [OPT-14] Use SVD for proper orthogonalization
        this._orthogonalizeRotation(filtered)

        return filtered
    }

    // ============================================================
    // VALIDATE POSE TRANSITION
    // ============================================================
    _isValidPoseTransition(prevPose, newPose) {
        // Check position jump
        const dx = newPose[12] - prevPose[12]
        const dy = newPose[13] - prevPose[13]
        const dz = newPose[14] - prevPose[14]
        const distance = Math.sqrt(dx * dx + dy * dy + dz * dz)

        if (distance > this.config.maxPositionJump) {
            console.warn(`[PoseEstimator] Position jump: ${distance.toFixed(3)}m`)
            return false
        }

        // Check rotation jump
        const dotProduct =
            prevPose[0] * newPose[0] + prevPose[1] * newPose[1] + prevPose[2] * newPose[2] +
            prevPose[4] * newPose[4] + prevPose[5] * newPose[5] + prevPose[6] * newPose[6] +
            prevPose[8] * newPose[8] + prevPose[9] * newPose[9] + prevPose[10] * newPose[10]

        const cosAngle = Math.max(-1, Math.min(1, (dotProduct - 1) / 2))
        const angleDeg = Math.acos(cosAngle) * 180 / Math.PI

        if (angleDeg > this.config.maxRotationJump) {
            console.warn(`[PoseEstimator] Rotation jump: ${angleDeg.toFixed(1)}°`)
            return false
        }

        return true
    }

    // ============================================================
    // BUILD POSE MATRIX FROM R, t
    // ============================================================
    _buildPoseMatrix(R, t) {
        // OpenCV to Three.js coordinate conversion
        // OpenCV: X-right, Y-down, Z-forward
        // Three.js: X-right, Y-up, Z-backward
        // Flip Y and Z: multiply by diag(1, -1, -1)

        const pose = new Float64Array(16)

        // Column-major for Three.js
        pose[0] = R[0]
        pose[1] = -R[3]
        pose[2] = -R[6]
        pose[3] = 0

        pose[4] = -R[1]
        pose[5] = R[4]
        pose[6] = R[7]
        pose[7] = 0

        pose[8] = -R[2]
        pose[9] = R[5]
        pose[10] = R[8]
        pose[11] = 0

        pose[12] = t[0]
        pose[13] = -t[1]
        pose[14] = -t[2]
        pose[15] = 1

        return pose
    }

    // ============================================================
    // CONVERT POSE TO THREE.Matrix4
    // ============================================================
    _toThreeMatrix4(pose) {
        // Assuming THREE is available globally
        if (typeof THREE === 'undefined') {
            return pose  // Return raw array
        }

        const matrix = new THREE.Matrix4()
        matrix.fromArray(pose)
        return matrix
    }

    // ============================================================
    // EXTRACT EULER ANGLES
    // ============================================================
    _extractEuler(pose) {
        // Extract rotation matrix
        const m11 = pose[0], m12 = pose[4], m13 = pose[8]
        const m21 = pose[1], m22 = pose[5], m23 = pose[9]
        const m31 = pose[2], m32 = pose[6], m33 = pose[10]

        // YXZ order (common for cameras)
        const y = Math.asin(Math.max(-1, Math.min(1, -m31)))
        let x, z

        if (Math.abs(m31) < 0.9999) {
            x = Math.atan2(m32, m33)
            z = Math.atan2(m21, m11)
        } else {
            x = Math.atan2(-m23, m22)
            z = 0
        }

        return {
            x: x * 180 / Math.PI,
            y: y * 180 / Math.PI,
            z: z * 180 / Math.PI,
        }
    }

    // ============================================================
    // ORTHOGONALIZE ROTATION MATRIX
    // ============================================================
    _orthogonalizeRotation(pose) {
        // Simple Gram-Schmidt orthogonalization
        // [OPT-15] Use SVD for more robust orthogonalization

        // Extract columns
        const x = [pose[0], pose[1], pose[2]]
        const y = [pose[4], pose[5], pose[6]]

        // Normalize x
        const xLen = Math.sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2])
        x[0] /= xLen; x[1] /= xLen; x[2] /= xLen

        // Orthogonalize y to x
        const dot = x[0] * y[0] + x[1] * y[1] + x[2] * y[2]
        y[0] -= dot * x[0]; y[1] -= dot * x[1]; y[2] -= dot * x[2]

        // Normalize y
        const yLen = Math.sqrt(y[0] * y[0] + y[1] * y[1] + y[2] * y[2])
        y[0] /= yLen; y[1] /= yLen; y[2] /= yLen

        // Compute z = x × y
        const z = [
            x[1] * y[2] - x[2] * y[1],
            x[2] * y[0] - x[0] * y[2],
            x[0] * y[1] - x[1] * y[0],
        ]

        // Update pose
        pose[0] = x[0]; pose[1] = x[1]; pose[2] = x[2]
        pose[4] = y[0]; pose[5] = y[1]; pose[6] = y[2]
        pose[8] = z[0]; pose[9] = z[1]; pose[10] = z[2]
    }

    // ============================================================
    // CONVERT POSE TO RVEC/TVEC
    // ============================================================
    _poseToRvecTvec(pose) {
        const cv = this.cv

        // Extract rotation matrix (reverse coordinate conversion)
        const R = cv.matFromArray(3, 3, cv.CV_64F, [
            pose[0], -pose[4], -pose[8],
            -pose[1], pose[5], pose[9],
            -pose[2], pose[6], pose[10],
        ])

        const rvec = new cv.Mat()
        cv.Rodrigues(R, rvec)
        R.delete()

        // Extract translation (reverse coordinate conversion)
        const tvec = cv.matFromArray(3, 1, cv.CV_64F, [
            pose[12], -pose[13], -pose[14],
        ])

        return { rvecInit: rvec, tvecInit: tvec }
    }

    // ============================================================
    // GET PROJECTION MATRIX
    // ============================================================
    getProjectionMatrix(near = 0.01, far = 100) {
        if (!this.cameraMatrix) return null

        const fx = this.cameraMatrix.data64F[0]
        const fy = this.cameraMatrix.data64F[4]
        const cx = this.cameraMatrix.data64F[2]
        const cy = this.cameraMatrix.data64F[5]
        const w = this.imageWidth
        const h = this.imageHeight

        // OpenGL projection matrix from camera intrinsics
        const A = (2 * fx) / w
        const B = (2 * fy) / h
        const C = (w - 2 * cx) / w
        const D = (2 * cy - h) / h
        const E = -(far + near) / (far - near)
        const F = (-2 * far * near) / (far - near)

        // Column-major for Three.js
        return [A, 0, 0, 0, 0, B, 0, 0, C, D, E, -1, 0, 0, F, 0]
    }

    // ============================================================
    // GET CURRENT POSE
    // ============================================================
    getCurrentPose() {
        return this.currentPose ? new Float64Array(this.currentPose) : null
    }

    // ============================================================
    // GET STATISTICS
    // ============================================================
    getStats() {
        return { ...this.stats }
    }

    // ============================================================
    // CLEANUP
    // ============================================================
    reset() {
        this.currentPose = null
        this.lastPose = null
        this.velocity = { linear: [0, 0, 0], angular: [0, 0, 0] }
        this.lastTimestamp = 0
        this.poseHistory = []
        this.stats = {
            correspondenceCount: 0,
            inlierCount: 0,
            inlierRatio: 0,
            reprojectionError: 0,
            motionPredictionUsed: false,
            processingTimeMs: 0,
        }
        console.log('[PoseEstimator] Reset')
    }

    dispose() {
        if (this.cameraMatrix) this.cameraMatrix.delete()
        if (this.distCoeffs) this.distCoeffs.delete()
        this.reset()
        console.log('[PoseEstimator] Disposed')
    }
}
