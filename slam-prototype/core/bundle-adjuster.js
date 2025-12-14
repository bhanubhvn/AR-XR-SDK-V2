// bundle-adjuster.js - Local Bundle Adjustment for SLAM
//
// OPTIMIZATION NOTES:
// [OPT-1] Consider WebAssembly port of Ceres or g2o for production
// [OPT-2] Implement sparse Schur complement for efficient optimization
// [OPT-3] Use GPU-accelerated Jacobian computation
// [OPT-4] Implement robust cost functions (Huber, Cauchy)
// [OPT-5] Consider incremental BA (iSAM-style)
// [OPT-6] Use covisibility graph to select optimization window

class BundleAdjuster {
    constructor(cv) {
        this.cv = cv

        // ============================================================
        // CONFIGURATION - Tuned for highest accuracy
        // ============================================================
        this.config = {
            // Optimization settings
            maxIterations: 20,              // LM iterations per BA call
            convergenceThreshold: 1e-6,     // Gradient norm threshold
            lambdaInitial: 1e-3,            // Initial LM damping
            lambdaMax: 1e10,                // Maximum damping
            lambdaMin: 1e-10,               // Minimum damping
            lambdaFactor: 10,               // Damping adjustment factor

            // Local BA window
            windowSize: 5,                  // Number of recent keyframes to optimize
            fixFirstPose: true,             // Fix first pose in window (gauge freedom)

            // Robust estimation
            useRobustCost: true,
            huberDelta: 1.5,                // Huber loss threshold (pixels)

            // Point optimization
            optimizePoints: true,           // Also optimize 3D points
            minPointObservations: 2,        // Minimum observations to optimize point

            // Performance
            decimationFactor: 1,            // Skip every N points (1 = no skip)
        }

        // ============================================================
        // STATE
        // ============================================================

        this.isOptimizing = false

        // Statistics
        this.stats = {
            iterations: 0,
            initialError: 0,
            finalError: 0,
            improvement: 0,
            optimizedPoses: 0,
            optimizedPoints: 0,
            processingTimeMs: 0,
        }

        console.log('[BundleAdjuster] Initialized')
    }

    // ============================================================
    // LOCAL BUNDLE ADJUSTMENT
    // ============================================================
    // 
    // Optimizes recent keyframe poses and associated map points.
    // Uses Levenberg-Marquardt algorithm with robust cost function.
    // 
    // @param {Array} keyframes - Array of keyframe objects
    // @param {Map} mapPoints - Map of pointId -> MapPoint
    // @param {Object} cameraMatrix - Camera intrinsics {fx, fy, cx, cy}
    // @returns {Object} - Optimization result
    // ============================================================
    optimizeLocal(keyframes, mapPoints, cameraMatrix) {
        const startTime = performance.now()

        if (this.isOptimizing) {
            console.warn('[BundleAdjuster] Already optimizing')
            return null
        }

        this.isOptimizing = true

        try {
            // ============================================================
            // STEP 1: Select optimization window
            // ============================================================

            const windowKFs = this._selectWindow(keyframes)
            if (windowKFs.length < 2) {
                console.log('[BundleAdjuster] Not enough keyframes for BA')
                return null
            }

            // ============================================================
            // STEP 2: Collect observations
            // ============================================================
            // Format: { keyframeIdx, pointId, observation: [u, v] }

            const { observations, pointsInWindow, pointIdToIdx, kfIdToIdx } =
                this._collectObservations(windowKFs, mapPoints)

            if (observations.length < 10) {
                console.log('[BundleAdjuster] Not enough observations for BA')
                return null
            }

            // ============================================================
            // STEP 3: Initialize parameter vector
            // ============================================================
            // Parameters: [pose0, pose1, ..., point0, point1, ...]
            // Pose: 6 parameters (rotation vector + translation)
            // Point: 3 parameters (x, y, z)

            const numPoses = windowKFs.length
            const numPoints = pointsInWindow.length
            const numPoseParams = numPoses * 6
            const numPointParams = numPoints * 3
            const totalParams = numPoseParams + numPointParams

            let params = new Float64Array(totalParams)

            // Initialize poses
            for (let i = 0; i < numPoses; i++) {
                const pose = windowKFs[i].pose
                const rvec = this._poseToRvec(pose)
                const tvec = [pose[12], pose[13], pose[14]]

                params[i * 6 + 0] = rvec[0]
                params[i * 6 + 1] = rvec[1]
                params[i * 6 + 2] = rvec[2]
                params[i * 6 + 3] = tvec[0]
                params[i * 6 + 4] = tvec[1]
                params[i * 6 + 5] = tvec[2]
            }

            // Initialize points
            for (let i = 0; i < numPoints; i++) {
                const point = pointsInWindow[i]
                params[numPoseParams + i * 3 + 0] = point.position[0]
                params[numPoseParams + i * 3 + 1] = point.position[1]
                params[numPoseParams + i * 3 + 2] = point.position[2]
            }

            // ============================================================
            // STEP 4: Levenberg-Marquardt optimization
            // ============================================================

            let lambda = this.config.lambdaInitial
            let prevError = this._computeTotalError(
                params, observations, cameraMatrix,
                numPoses, pointIdToIdx, kfIdToIdx
            )
            this.stats.initialError = prevError

            let iteration = 0
            let converged = false

            while (iteration < this.config.maxIterations && !converged) {
                // Compute Jacobian and residuals
                const { J, r } = this._computeJacobianAndResiduals(
                    params, observations, cameraMatrix,
                    numPoses, numPoints, pointIdToIdx, kfIdToIdx
                )

                // Compute Hessian approximation: H = J^T * J
                // And gradient: g = J^T * r
                const { H, g } = this._computeHessianAndGradient(J, r)

                // Add damping to diagonal (LM)
                for (let i = 0; i < H.length; i++) {
                    H[i * (totalParams + 1)] += lambda
                }

                // Solve for update: (H + λI) * Δ = -g
                // [OPT-7] Use sparse solver for large systems
                const delta = this._solveLinearSystem(H, g, totalParams)

                if (!delta) {
                    lambda *= this.config.lambdaFactor
                    iteration++
                    continue
                }

                // Apply update to get new parameters
                const newParams = new Float64Array(totalParams)
                for (let i = 0; i < totalParams; i++) {
                    // Skip first pose if fixed
                    if (this.config.fixFirstPose && i < 6) {
                        newParams[i] = params[i]
                    } else {
                        newParams[i] = params[i] - delta[i]
                    }
                }

                // Compute new error
                const newError = this._computeTotalError(
                    newParams, observations, cameraMatrix,
                    numPoses, pointIdToIdx, kfIdToIdx
                )

                // Accept or reject update
                if (newError < prevError) {
                    params = newParams
                    const improvement = (prevError - newError) / prevError

                    if (improvement < this.config.convergenceThreshold) {
                        converged = true
                    }

                    prevError = newError
                    lambda = Math.max(this.config.lambdaMin, lambda / this.config.lambdaFactor)
                } else {
                    lambda = Math.min(this.config.lambdaMax, lambda * this.config.lambdaFactor)
                }

                iteration++
            }

            this.stats.iterations = iteration
            this.stats.finalError = prevError
            this.stats.improvement = (this.stats.initialError - this.stats.finalError) /
                this.stats.initialError

            // ============================================================
            // STEP 5: Update keyframes and map points
            // ============================================================

            // Update poses
            for (let i = 0; i < numPoses; i++) {
                if (this.config.fixFirstPose && i === 0) continue

                const rvec = [
                    params[i * 6 + 0],
                    params[i * 6 + 1],
                    params[i * 6 + 2],
                ]
                const tvec = [
                    params[i * 6 + 3],
                    params[i * 6 + 4],
                    params[i * 6 + 5],
                ]

                windowKFs[i].pose = this._rvecTvecToPose(rvec, tvec)
            }
            this.stats.optimizedPoses = numPoses - (this.config.fixFirstPose ? 1 : 0)

            // Update points
            if (this.config.optimizePoints) {
                for (let i = 0; i < numPoints; i++) {
                    pointsInWindow[i].position[0] = params[numPoseParams + i * 3 + 0]
                    pointsInWindow[i].position[1] = params[numPoseParams + i * 3 + 1]
                    pointsInWindow[i].position[2] = params[numPoseParams + i * 3 + 2]
                }
                this.stats.optimizedPoints = numPoints
            }

            this.stats.processingTimeMs = performance.now() - startTime

            console.log(
                `[BundleAdjuster] Completed: ${iteration} iters, ` +
                `error ${this.stats.initialError.toFixed(4)} -> ${this.stats.finalError.toFixed(4)} ` +
                `(${(this.stats.improvement * 100).toFixed(1)}% improvement)`
            )

            return {
                success: true,
                stats: { ...this.stats },
            }

        } finally {
            this.isOptimizing = false
        }
    }

    // ============================================================
    // SELECT OPTIMIZATION WINDOW
    // ============================================================
    _selectWindow(keyframes) {
        // Select recent keyframes for local BA
        // [OPT-8] Consider covisibility-based window selection
        const start = Math.max(0, keyframes.length - this.config.windowSize)
        return keyframes.slice(start)
    }

    // ============================================================
    // COLLECT OBSERVATIONS
    // ============================================================
    _collectObservations(keyframes, mapPoints) {
        const observations = []
        const pointIdToIdx = new Map()
        const kfIdToIdx = new Map()
        const pointsInWindow = []

        // Build keyframe index map
        for (let i = 0; i < keyframes.length; i++) {
            kfIdToIdx.set(keyframes[i].id, i)
        }

        // Collect observations from each keyframe
        for (let kfIdx = 0; kfIdx < keyframes.length; kfIdx++) {
            const kf = keyframes[kfIdx]

            for (const pointId of kf.mapPointIds) {
                const point = mapPoints.get(pointId)
                if (!point) continue

                // Check if point is observed in this keyframe
                const featureIdx = point.observations.get(kf.id)
                if (featureIdx === undefined) continue

                // Get 2D observation
                const feature = kf.features[featureIdx]
                if (!feature) continue

                // Add point to optimization set if not already added
                if (!pointIdToIdx.has(pointId)) {
                    if (point.observations.size < this.config.minPointObservations) {
                        continue  // Skip points with too few observations
                    }
                    pointIdToIdx.set(pointId, pointsInWindow.length)
                    pointsInWindow.push(point)
                }

                observations.push({
                    kfIdx,
                    pointIdx: pointIdToIdx.get(pointId),
                    u: feature.x,
                    v: feature.y,
                })
            }
        }

        return { observations, pointsInWindow, pointIdToIdx, kfIdToIdx }
    }

    // ============================================================
    // COMPUTE TOTAL ERROR
    // ============================================================
    _computeTotalError(params, observations, camera, numPoses, pointIdToIdx, kfIdToIdx) {
        const numPoseParams = numPoses * 6
        let totalError = 0

        for (const obs of observations) {
            // Get pose parameters
            const poseOffset = obs.kfIdx * 6
            const rvec = [params[poseOffset], params[poseOffset + 1], params[poseOffset + 2]]
            const tvec = [params[poseOffset + 3], params[poseOffset + 4], params[poseOffset + 5]]

            // Get point parameters
            const pointOffset = numPoseParams + obs.pointIdx * 3
            const point = [
                params[pointOffset],
                params[pointOffset + 1],
                params[pointOffset + 2],
            ]

            // Project point to image
            const projected = this._projectPoint(point, rvec, tvec, camera)

            // Compute reprojection error
            const ex = projected.u - obs.u
            const ey = projected.v - obs.v
            let error = ex * ex + ey * ey

            // Apply robust cost if enabled
            if (this.config.useRobustCost) {
                error = this._huberCost(Math.sqrt(error), this.config.huberDelta)
            }

            totalError += error
        }

        return totalError
    }

    // ============================================================
    // COMPUTE JACOBIAN AND RESIDUALS
    // ============================================================
    // 
    // Computes the Jacobian matrix and residual vector for LM.
    // [OPT-9] Consider analytic Jacobian for better performance
    // ============================================================
    _computeJacobianAndResiduals(params, observations, camera, numPoses, numPoints, pointIdToIdx, kfIdToIdx) {
        const numPoseParams = numPoses * 6
        const numPointParams = numPoints * 3
        const totalParams = numPoseParams + numPointParams
        const numResiduals = observations.length * 2

        // Allocate (sparse representation would be better for large problems)
        // [OPT-10] Use sparse matrix representation
        const J = new Float64Array(numResiduals * totalParams)
        const r = new Float64Array(numResiduals)

        const eps = 1e-6  // Numerical differentiation step

        for (let i = 0; i < observations.length; i++) {
            const obs = observations[i]
            const residualIdx = i * 2

            // Get current pose and point
            const poseOffset = obs.kfIdx * 6
            const pointOffset = numPoseParams + obs.pointIdx * 3

            const rvec = [params[poseOffset], params[poseOffset + 1], params[poseOffset + 2]]
            const tvec = [params[poseOffset + 3], params[poseOffset + 4], params[poseOffset + 5]]
            const point = [params[pointOffset], params[pointOffset + 1], params[pointOffset + 2]]

            // Compute current projection
            const proj = this._projectPoint(point, rvec, tvec, camera)
            r[residualIdx] = proj.u - obs.u
            r[residualIdx + 1] = proj.v - obs.v

            // Numerical Jacobian for pose parameters
            // [OPT-11] Replace with analytic Jacobian
            for (let j = 0; j < 6; j++) {
                const paramIdx = poseOffset + j
                const original = params[paramIdx]

                params[paramIdx] = original + eps
                const projPlus = this._projectPoint(
                    point,
                    [params[poseOffset], params[poseOffset + 1], params[poseOffset + 2]],
                    [params[poseOffset + 3], params[poseOffset + 4], params[poseOffset + 5]],
                    camera
                )

                params[paramIdx] = original - eps
                const projMinus = this._projectPoint(
                    point,
                    [params[poseOffset], params[poseOffset + 1], params[poseOffset + 2]],
                    [params[poseOffset + 3], params[poseOffset + 4], params[poseOffset + 5]],
                    camera
                )

                params[paramIdx] = original

                J[residualIdx * totalParams + paramIdx] = (projPlus.u - projMinus.u) / (2 * eps)
                J[(residualIdx + 1) * totalParams + paramIdx] = (projPlus.v - projMinus.v) / (2 * eps)
            }

            // Numerical Jacobian for point parameters
            if (this.config.optimizePoints) {
                for (let j = 0; j < 3; j++) {
                    const paramIdx = pointOffset + j
                    const original = params[paramIdx]

                    params[paramIdx] = original + eps
                    const projPlus = this._projectPoint(
                        [params[pointOffset], params[pointOffset + 1], params[pointOffset + 2]],
                        rvec, tvec, camera
                    )

                    params[paramIdx] = original - eps
                    const projMinus = this._projectPoint(
                        [params[pointOffset], params[pointOffset + 1], params[pointOffset + 2]],
                        rvec, tvec, camera
                    )

                    params[paramIdx] = original

                    J[residualIdx * totalParams + paramIdx] = (projPlus.u - projMinus.u) / (2 * eps)
                    J[(residualIdx + 1) * totalParams + paramIdx] = (projPlus.v - projMinus.v) / (2 * eps)
                }
            }
        }

        return { J, r }
    }

    // ============================================================
    // COMPUTE HESSIAN AND GRADIENT
    // ============================================================
    _computeHessianAndGradient(J, r) {
        const numResiduals = r.length
        const numParams = J.length / numResiduals

        // H = J^T * J
        const H = new Float64Array(numParams * numParams)
        // g = J^T * r
        const g = new Float64Array(numParams)

        // [OPT-12] Use BLAS-like optimized matrix operations
        for (let i = 0; i < numParams; i++) {
            for (let j = 0; j < numParams; j++) {
                let sum = 0
                for (let k = 0; k < numResiduals; k++) {
                    sum += J[k * numParams + i] * J[k * numParams + j]
                }
                H[i * numParams + j] = sum
            }

            let gSum = 0
            for (let k = 0; k < numResiduals; k++) {
                gSum += J[k * numParams + i] * r[k]
            }
            g[i] = gSum
        }

        return { H, g }
    }

    // ============================================================
    // SOLVE LINEAR SYSTEM
    // ============================================================
    // 
    // Solves H * x = g using Cholesky decomposition
    // [OPT-13] Use sparse solver for large systems
    // ============================================================
    _solveLinearSystem(H, g, n) {
        // Simple Gaussian elimination (for small systems)
        // [OPT-14] Replace with proper sparse Cholesky

        // Copy H and g
        const A = new Float64Array(H)
        const b = new Float64Array(g)

        // Forward elimination
        for (let i = 0; i < n; i++) {
            // Find pivot
            let maxRow = i
            let maxVal = Math.abs(A[i * n + i])
            for (let k = i + 1; k < n; k++) {
                if (Math.abs(A[k * n + i]) > maxVal) {
                    maxRow = k
                    maxVal = Math.abs(A[k * n + i])
                }
            }

            if (maxVal < 1e-10) {
                return null  // Singular matrix
            }

            // Swap rows
            if (maxRow !== i) {
                for (let j = 0; j < n; j++) {
                    const tmp = A[i * n + j]
                    A[i * n + j] = A[maxRow * n + j]
                    A[maxRow * n + j] = tmp
                }
                const tmp = b[i]
                b[i] = b[maxRow]
                b[maxRow] = tmp
            }

            // Eliminate
            for (let k = i + 1; k < n; k++) {
                const factor = A[k * n + i] / A[i * n + i]
                for (let j = i; j < n; j++) {
                    A[k * n + j] -= factor * A[i * n + j]
                }
                b[k] -= factor * b[i]
            }
        }

        // Back substitution
        const x = new Float64Array(n)
        for (let i = n - 1; i >= 0; i--) {
            let sum = b[i]
            for (let j = i + 1; j < n; j++) {
                sum -= A[i * n + j] * x[j]
            }
            x[i] = sum / A[i * n + i]
        }

        return x
    }

    // ============================================================
    // PROJECT POINT
    // ============================================================
    _projectPoint(point, rvec, tvec, camera) {
        // Rodrigues rotation
        const theta = Math.sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2])

        let R
        if (theta < 1e-10) {
            R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        } else {
            const k = [rvec[0] / theta, rvec[1] / theta, rvec[2] / theta]
            const c = Math.cos(theta)
            const s = Math.sin(theta)
            const v = 1 - c

            R = [
                c + k[0] * k[0] * v,
                k[0] * k[1] * v - k[2] * s,
                k[0] * k[2] * v + k[1] * s,
                k[0] * k[1] * v + k[2] * s,
                c + k[1] * k[1] * v,
                k[1] * k[2] * v - k[0] * s,
                k[0] * k[2] * v - k[1] * s,
                k[1] * k[2] * v + k[0] * s,
                c + k[2] * k[2] * v,
            ]
        }

        // Apply rotation and translation
        const Xc = R[0] * point[0] + R[1] * point[1] + R[2] * point[2] + tvec[0]
        const Yc = R[3] * point[0] + R[4] * point[1] + R[5] * point[2] + tvec[1]
        const Zc = R[6] * point[0] + R[7] * point[1] + R[8] * point[2] + tvec[2]

        // Project
        const u = camera.fx * Xc / Zc + camera.cx
        const v = camera.fy * Yc / Zc + camera.cy

        return { u, v }
    }

    // ============================================================
    // HUBER COST
    // ============================================================
    _huberCost(x, delta) {
        const absX = Math.abs(x)
        if (absX <= delta) {
            return 0.5 * x * x
        } else {
            return delta * (absX - 0.5 * delta)
        }
    }

    // ============================================================
    // POSE TO RVEC
    // ============================================================
    _poseToRvec(pose) {
        // Extract rotation matrix (assuming pose is 16-element array in column-major)
        // [OPT-15] Use quaternion representation
        const R = [
            pose[0], pose[4], pose[8],
            pose[1], pose[5], pose[9],
            pose[2], pose[6], pose[10],
        ]

        // Rodrigues: rotation matrix to rotation vector
        const trace = R[0] + R[4] + R[8]
        const angle = Math.acos(Math.max(-1, Math.min(1, (trace - 1) / 2)))

        if (angle < 1e-6) {
            return [0, 0, 0]
        }

        const s = 2 * Math.sin(angle)
        return [
            (R[7] - R[5]) / s * angle,
            (R[2] - R[6]) / s * angle,
            (R[3] - R[1]) / s * angle,
        ]
    }

    // ============================================================
    // RVEC/TVEC TO POSE
    // ============================================================
    _rvecTvecToPose(rvec, tvec) {
        // Rodrigues rotation
        const theta = Math.sqrt(rvec[0] * rvec[0] + rvec[1] * rvec[1] + rvec[2] * rvec[2])

        let R
        if (theta < 1e-10) {
            R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        } else {
            const k = [rvec[0] / theta, rvec[1] / theta, rvec[2] / theta]
            const c = Math.cos(theta)
            const s = Math.sin(theta)
            const v = 1 - c

            R = [
                c + k[0] * k[0] * v,
                k[0] * k[1] * v - k[2] * s,
                k[0] * k[2] * v + k[1] * s,
                k[0] * k[1] * v + k[2] * s,
                c + k[1] * k[1] * v,
                k[1] * k[2] * v - k[0] * s,
                k[0] * k[2] * v - k[1] * s,
                k[1] * k[2] * v + k[0] * s,
                c + k[2] * k[2] * v,
            ]
        }

        // Build pose matrix (column-major for Three.js)
        return new Float64Array([
            R[0], R[3], R[6], 0,
            R[1], R[4], R[7], 0,
            R[2], R[5], R[8], 0,
            tvec[0], tvec[1], tvec[2], 1,
        ])
    }

    // ============================================================
    // STATISTICS
    // ============================================================
    getStats() {
        return { ...this.stats }
    }

    // ============================================================
    // CLEANUP
    // ============================================================
    reset() {
        this.isOptimizing = false
        this.stats = {
            iterations: 0,
            initialError: 0,
            finalError: 0,
            improvement: 0,
            optimizedPoses: 0,
            optimizedPoints: 0,
            processingTimeMs: 0,
        }
        console.log('[BundleAdjuster] Reset')
    }

    dispose() {
        this.reset()
        console.log('[BundleAdjuster] Disposed')
    }
}
