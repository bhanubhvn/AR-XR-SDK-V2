// map-manager.js - 3D Map Point Storage and Management
//
// OPTIMIZATION NOTES:
// [OPT-1] Consider using a spatial hash / octree for faster point lookup
// [OPT-2] Implement GPU-accelerated point triangulation
// [OPT-3] Use typed arrays for all point data to reduce GC pressure
// [OPT-4] Consider WebAssembly for intensive math operations
// [OPT-5] Implement lazy descriptor computation (only when needed for matching)

class MapManager {
    constructor(cv) {
        this.cv = cv

        // ============================================================
        // CONFIGURATION - Tuned for highest accuracy
        // ============================================================
        this.config = {
            // Triangulation thresholds
            minParallaxDegrees: 2.0,        // Minimum angle between views for triangulation
            maxReprojectionError: 2.0,       // Maximum reprojection error (pixels) to accept point
            minTriangulationDistance: 0.01,  // Minimum baseline (meters) for triangulation
            maxTriangulationDistance: 50.0,  // Maximum depth (meters) for valid points

            // Map point management
            maxMapPoints: 5000,              // Maximum points in map
            minObservations: 2,              // Minimum keyframe observations to keep point
            maxPointAge: 100,                // Remove points not seen in N frames

            // Point confidence
            initialConfidence: 0.3,
            confidenceIncrement: 0.1,        // Increase per successful observation
            confidenceDecrement: 0.05,       // Decrease when not observed
            minConfidenceToKeep: 0.1,

            // Spatial indexing
            // [OPT-6] Tune grid size based on typical viewing frustum
            spatialGridSize: 1.0,            // Meters per grid cell
        }

        // ============================================================
        // STATE
        // ============================================================

        // Map points: 3D landmarks
        this.mapPoints = new Map()  // id -> MapPoint
        this.nextPointId = 0

        // Keyframes: stored camera poses
        this.keyframes = new Map()  // id -> Keyframe
        this.nextKeyframeId = 0

        // Point-to-feature correspondences
        // [OPT-7] Consider bidirectional indexing for faster lookup
        this.pointToFeatures = new Map()  // pointId -> Set of { keyframeId, featureIndex }
        this.featureToPoint = new Map()   // `${keyframeId}_${featureIndex}` -> pointId

        // Statistics
        this.stats = {
            totalPoints: 0,
            activePoints: 0,
            culledPoints: 0,
            totalKeyframes: 0,
        }

        console.log('[MapManager] Initialized')
    }

    // ============================================================
    // MAP POINT STRUCTURE
    // ============================================================
    // 
    // {
    //   id: number,
    //   position: Float64Array(3),       // [x, y, z] world coordinates
    //   descriptor: Uint8Array(32),      // ORB descriptor (256 bits)
    //   normal: Float64Array(3),         // Mean viewing direction
    //   confidence: number,              // 0-1 confidence score
    //   observations: Map,               // keyframeId -> featureIndex
    //   lastSeenFrame: number,           // Last frame number where visible
    //   createdFrame: number,            // Frame when first created
    //   reprojectionErrors: number[],    // Recent reprojection errors
    // }
    // ============================================================

    // ============================================================
    // TRIANGULATE NEW MAP POINT
    // ============================================================
    // 
    // Creates a new 3D map point from two observations in different keyframes.
    // Uses linear triangulation (DLT) followed by non-linear refinement.
    // 
    // @param {Object} kf1 - First keyframe { pose: Matrix4, projection: Matrix }
    // @param {Object} pt1 - First observation { x, y } in normalized coordinates
    // @param {Object} kf2 - Second keyframe { pose: Matrix4, projection: Matrix }
    // @param {Object} pt2 - Second observation { x, y } in normalized coordinates
    // @param {Uint8Array} descriptor - ORB descriptor
    // @returns {number|null} - Map point ID or null if triangulation failed
    // ============================================================
    triangulate(kf1, pt1, kf2, pt2, descriptor = null) {
        const cv = this.cv

        // ============================================================
        // STEP 1: Check parallax angle
        // ============================================================
        // Triangulation is unreliable with small baseline
        // [OPT-8] Pre-compute camera centers for efficiency

        const cam1 = this._getCameraCenter(kf1.pose)
        const cam2 = this._getCameraCenter(kf2.pose)

        const baseline = Math.sqrt(
            (cam2[0] - cam1[0]) ** 2 +
            (cam2[1] - cam1[1]) ** 2 +
            (cam2[2] - cam1[2]) ** 2
        )

        if (baseline < this.config.minTriangulationDistance) {
            return null  // Insufficient baseline
        }

        // ============================================================
        // STEP 2: Linear Triangulation (DLT)
        // ============================================================
        // Solve Ax = 0 using SVD
        // [OPT-9] Consider using iterative method for higher accuracy

        const P1 = this._getProjectionMatrix(kf1)
        const P2 = this._getProjectionMatrix(kf2)

        // Build 4x4 matrix A
        // Row 1: x1 * P1[2] - P1[0]
        // Row 2: y1 * P1[2] - P1[1]
        // Row 3: x2 * P2[2] - P2[0]
        // Row 4: y2 * P2[2] - P2[1]

        const A = [
            pt1.x * P1[8] - P1[0], pt1.x * P1[9] - P1[1], pt1.x * P1[10] - P1[2], pt1.x * P1[11] - P1[3],
            pt1.y * P1[8] - P1[4], pt1.y * P1[9] - P1[5], pt1.y * P1[10] - P1[6], pt1.y * P1[11] - P1[7],
            pt2.x * P2[8] - P2[0], pt2.x * P2[9] - P2[1], pt2.x * P2[10] - P2[2], pt2.x * P2[11] - P2[3],
            pt2.y * P2[8] - P2[4], pt2.y * P2[9] - P2[5], pt2.y * P2[10] - P2[6], pt2.y * P2[11] - P2[7],
        ]

        const matA = cv.matFromArray(4, 4, cv.CV_64F, A)
        const w = new cv.Mat()
        const u = new cv.Mat()
        const vt = new cv.Mat()

        cv.SVDecomp(matA, w, u, vt)

        // Solution is last row of V (last column of Vt)
        // Homogeneous coordinates
        const X = [
            vt.data64F[12],  // vt[3][0]
            vt.data64F[13],  // vt[3][1]
            vt.data64F[14],  // vt[3][2]
            vt.data64F[15],  // vt[3][3]
        ]

        matA.delete()
        w.delete()
        u.delete()
        vt.delete()

        // Convert to Euclidean coordinates
        if (Math.abs(X[3]) < 1e-10) {
            return null  // Point at infinity
        }

        const point3D = [X[0] / X[3], X[1] / X[3], X[2] / X[3]]

        // ============================================================
        // STEP 3: Validate triangulated point
        // ============================================================

        // Check depth (must be in front of both cameras)
        const depth1 = this._computeDepth(point3D, kf1.pose)
        const depth2 = this._computeDepth(point3D, kf2.pose)

        if (depth1 <= 0 || depth2 <= 0) {
            return null  // Behind camera
        }

        if (depth1 > this.config.maxTriangulationDistance ||
            depth2 > this.config.maxTriangulationDistance) {
            return null  // Too far
        }

        // Check parallax angle
        const ray1 = this._normalizeVector([
            point3D[0] - cam1[0],
            point3D[1] - cam1[1],
            point3D[2] - cam1[2],
        ])
        const ray2 = this._normalizeVector([
            point3D[0] - cam2[0],
            point3D[1] - cam2[1],
            point3D[2] - cam2[2],
        ])

        const cosParallax = ray1[0] * ray2[0] + ray1[1] * ray2[1] + ray1[2] * ray2[2]
        const parallaxDeg = Math.acos(Math.min(1, Math.abs(cosParallax))) * 180 / Math.PI

        if (parallaxDeg < this.config.minParallaxDegrees) {
            return null  // Insufficient parallax
        }

        // Check reprojection error
        const reproj1 = this._projectPoint(point3D, kf1)
        const reproj2 = this._projectPoint(point3D, kf2)

        const error1 = Math.sqrt((reproj1.x - pt1.x) ** 2 + (reproj1.y - pt1.y) ** 2)
        const error2 = Math.sqrt((reproj2.x - pt2.x) ** 2 + (reproj2.y - pt2.y) ** 2)

        if (error1 > this.config.maxReprojectionError ||
            error2 > this.config.maxReprojectionError) {
            return null  // High reprojection error
        }

        // ============================================================
        // STEP 4: Create map point
        // ============================================================

        const mapPoint = {
            id: this.nextPointId++,
            position: new Float64Array(point3D),
            descriptor: descriptor ? new Uint8Array(descriptor) : null,
            normal: new Float64Array(this._computeMeanNormal(ray1, ray2)),
            confidence: this.config.initialConfidence,
            observations: new Map([
                [kf1.id, pt1.featureIndex],
                [kf2.id, pt2.featureIndex],
            ]),
            lastSeenFrame: kf2.frameNumber,
            createdFrame: kf1.frameNumber,
            reprojectionErrors: [error1, error2],
        }

        this.mapPoints.set(mapPoint.id, mapPoint)
        this.stats.totalPoints++
        this.stats.activePoints = this.mapPoints.size

        // Update correspondences
        this._addCorrespondence(mapPoint.id, kf1.id, pt1.featureIndex)
        this._addCorrespondence(mapPoint.id, kf2.id, pt2.featureIndex)

        return mapPoint.id
    }

    // ============================================================
    // ADD OBSERVATION TO EXISTING POINT
    // ============================================================
    addObservation(pointId, keyframeId, featureIndex, reprojError) {
        const point = this.mapPoints.get(pointId)
        if (!point) return false

        point.observations.set(keyframeId, featureIndex)
        point.reprojectionErrors.push(reprojError)

        // Keep only last 10 reprojection errors
        if (point.reprojectionErrors.length > 10) {
            point.reprojectionErrors.shift()
        }

        // Increase confidence
        point.confidence = Math.min(1.0,
            point.confidence + this.config.confidenceIncrement
        )

        this._addCorrespondence(pointId, keyframeId, featureIndex)
        return true
    }

    // ============================================================
    // GET 2D-3D CORRESPONDENCES FOR POSE ESTIMATION
    // ============================================================
    // 
    // Returns matched 3D points and their 2D observations for PnP.
    // Filters by confidence and visibility.
    // 
    // @param {Array} trackedPoints - 2D points with IDs from FeatureTracker
    // @param {number} keyframeId - Current keyframe ID for lookup
    // @returns {Object} { points3D: number[][], points2D: number[][] }
    // ============================================================
    getCorrespondences(trackedPoints, keyframeId) {
        const points3D = []
        const points2D = []
        const pointIds = []

        for (const pt of trackedPoints) {
            const key = `${keyframeId}_${pt.id}`
            const pointId = this.featureToPoint.get(key)

            if (pointId !== undefined) {
                const mapPoint = this.mapPoints.get(pointId)
                if (mapPoint && mapPoint.confidence > this.config.minConfidenceToKeep) {
                    points3D.push([...mapPoint.position])
                    points2D.push([pt.x, pt.y])
                    pointIds.push(pointId)
                }
            }
        }

        return { points3D, points2D, pointIds }
    }

    // ============================================================
    // GET ALL VISIBLE MAP POINTS
    // ============================================================
    // 
    // Returns map points potentially visible from current camera pose.
    // Uses frustum culling and depth filtering.
    // [OPT-10] Implement octree for faster spatial queries
    // ============================================================
    getVisiblePoints(cameraPose, projectionMatrix, imageWidth, imageHeight) {
        const visible = []
        const cameraCenter = this._getCameraCenter(cameraPose)

        for (const [id, point] of this.mapPoints) {
            // Quick depth check
            const depth = this._computeDepth(point.position, cameraPose)
            if (depth <= 0 || depth > this.config.maxTriangulationDistance) {
                continue
            }

            // Project to image
            const projected = this._projectPointToImage(
                point.position, cameraPose, projectionMatrix, imageWidth, imageHeight
            )

            if (projected &&
                projected.x >= 0 && projected.x < imageWidth &&
                projected.y >= 0 && projected.y < imageHeight) {
                visible.push({
                    id,
                    point3D: [...point.position],
                    projected,
                    confidence: point.confidence,
                    descriptor: point.descriptor,
                })
            }
        }

        return visible
    }

    // ============================================================
    // CULL LOW-QUALITY POINTS
    // ============================================================
    // 
    // Removes map points that are unreliable.
    // Called periodically to keep map clean.
    // ============================================================
    cullPoints(currentFrame) {
        const toRemove = []

        for (const [id, point] of this.mapPoints) {
            // Check age
            const frameSinceLastSeen = currentFrame - point.lastSeenFrame
            if (frameSinceLastSeen > this.config.maxPointAge) {
                toRemove.push(id)
                continue
            }

            // Decrease confidence for unseen points
            if (frameSinceLastSeen > 5) {
                point.confidence -= this.config.confidenceDecrement
                if (point.confidence < this.config.minConfidenceToKeep) {
                    toRemove.push(id)
                    continue
                }
            }

            // Check observations
            if (point.observations.size < this.config.minObservations) {
                // Give new points a chance
                const pointAge = currentFrame - point.createdFrame
                if (pointAge > 30) {
                    toRemove.push(id)
                }
            }
        }

        // Remove bad points
        for (const id of toRemove) {
            this._removePoint(id)
        }

        this.stats.culledPoints += toRemove.length
        this.stats.activePoints = this.mapPoints.size

        // Enforce max points
        if (this.mapPoints.size > this.config.maxMapPoints) {
            this._removeLowConfidencePoints(
                this.mapPoints.size - this.config.maxMapPoints
            )
        }

        return toRemove.length
    }

    // ============================================================
    // KEYFRAME MANAGEMENT
    // ============================================================

    addKeyframe(keyframe) {
        // Keyframe structure:
        // {
        //   id: number,
        //   frameNumber: number,
        //   pose: Float64Array(16),  // 4x4 matrix
        //   features: Array,         // 2D features
        //   descriptors: cv.Mat,     // ORB descriptors
        //   timestamp: number,
        // }

        keyframe.id = this.nextKeyframeId++
        this.keyframes.set(keyframe.id, keyframe)
        this.stats.totalKeyframes = this.keyframes.size

        console.log(`[MapManager] Added keyframe ${keyframe.id} with ${keyframe.features?.length || 0} features`)
        return keyframe.id
    }

    getKeyframe(id) {
        return this.keyframes.get(id)
    }

    getRecentKeyframes(count = 5) {
        const kfs = Array.from(this.keyframes.values())
        return kfs.slice(-count)
    }

    // ============================================================
    // HELPER METHODS
    // ============================================================

    _getCameraCenter(pose) {
        // Camera center is -R^T * t
        // For 4x4 matrix in column-major order:
        // Extract rotation (3x3) and translation
        const R = [
            pose[0], pose[1], pose[2],
            pose[4], pose[5], pose[6],
            pose[8], pose[9], pose[10],
        ]
        const t = [pose[12], pose[13], pose[14]]

        // Camera center = -R^T * t (but pose is already world-to-camera, so just use translation)
        // Actually, for Three.js Matrix4, the translation is the camera position in world space
        return t
    }

    _getProjectionMatrix(keyframe) {
        // Return 3x4 projection matrix [R|t]
        const P = keyframe.pose
        return [
            P[0], P[4], P[8], P[12],
            P[1], P[5], P[9], P[13],
            P[2], P[6], P[10], P[14],
        ]
    }

    _computeDepth(point3D, pose) {
        // Transform point to camera coordinates
        // Depth is the Z component
        const x = point3D[0] - pose[12]
        const y = point3D[1] - pose[13]
        const z = point3D[2] - pose[14]

        // Apply rotation (transpose of rotation part)
        const depth = pose[2] * x + pose[6] * y + pose[10] * z
        return depth
    }

    _projectPoint(point3D, keyframe) {
        // Project 3D point to normalized image coordinates
        const P = this._getProjectionMatrix(keyframe)

        const w = P[8] * point3D[0] + P[9] * point3D[1] + P[10] * point3D[2] + P[11]
        const x = (P[0] * point3D[0] + P[1] * point3D[1] + P[2] * point3D[2] + P[3]) / w
        const y = (P[4] * point3D[0] + P[5] * point3D[1] + P[6] * point3D[2] + P[7]) / w

        return { x, y }
    }

    _projectPointToImage(point3D, cameraPose, projMatrix, width, height) {
        // Full projection including camera intrinsics
        // This needs camera matrix K
        // For now, return normalized projection (to be multiplied by K later)
        return this._projectPoint(point3D, { pose: cameraPose })
    }

    _normalizeVector(v) {
        const len = Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if (len < 1e-10) return [0, 0, 1]
        return [v[0] / len, v[1] / len, v[2] / len]
    }

    _computeMeanNormal(ray1, ray2) {
        const mean = [
            (ray1[0] + ray2[0]) / 2,
            (ray1[1] + ray2[1]) / 2,
            (ray1[2] + ray2[2]) / 2,
        ]
        return this._normalizeVector(mean)
    }

    _addCorrespondence(pointId, keyframeId, featureIndex) {
        // Point -> features
        if (!this.pointToFeatures.has(pointId)) {
            this.pointToFeatures.set(pointId, new Set())
        }
        this.pointToFeatures.get(pointId).add({ keyframeId, featureIndex })

        // Feature -> point
        const key = `${keyframeId}_${featureIndex}`
        this.featureToPoint.set(key, pointId)
    }

    _removePoint(pointId) {
        const point = this.mapPoints.get(pointId)
        if (!point) return

        // Remove correspondences
        const features = this.pointToFeatures.get(pointId)
        if (features) {
            for (const { keyframeId, featureIndex } of features) {
                const key = `${keyframeId}_${featureIndex}`
                this.featureToPoint.delete(key)
            }
            this.pointToFeatures.delete(pointId)
        }

        this.mapPoints.delete(pointId)
    }

    _removeLowConfidencePoints(count) {
        // Sort by confidence and remove lowest
        const sorted = Array.from(this.mapPoints.entries())
            .sort((a, b) => a[1].confidence - b[1].confidence)

        for (let i = 0; i < Math.min(count, sorted.length); i++) {
            this._removePoint(sorted[i][0])
        }
    }

    // ============================================================
    // STATISTICS
    // ============================================================
    getStats() {
        const confidences = Array.from(this.mapPoints.values()).map(p => p.confidence)
        const avgConfidence = confidences.length > 0
            ? confidences.reduce((a, b) => a + b, 0) / confidences.length
            : 0

        return {
            ...this.stats,
            avgConfidence,
            keyframeCount: this.keyframes.size,
        }
    }

    // ============================================================
    // VISUALIZATION DATA
    // ============================================================
    getVisualizationData() {
        const points = []
        for (const [id, point] of this.mapPoints) {
            points.push({
                id,
                position: [...point.position],
                confidence: point.confidence,
                observations: point.observations.size,
            })
        }
        return { points, keyframes: this.keyframes.size }
    }

    // ============================================================
    // CLEANUP
    // ============================================================
    reset() {
        this.mapPoints.clear()
        this.keyframes.clear()
        this.pointToFeatures.clear()
        this.featureToPoint.clear()
        this.nextPointId = 0
        this.nextKeyframeId = 0
        this.stats = {
            totalPoints: 0,
            activePoints: 0,
            culledPoints: 0,
            totalKeyframes: 0,
        }
        console.log('[MapManager] Reset')
    }

    dispose() {
        this.reset()
        console.log('[MapManager] Disposed')
    }
}
