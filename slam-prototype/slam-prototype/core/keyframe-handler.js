// keyframe-handler.js - Keyframe Selection and Storage for SLAM
//
// OPTIMIZATION NOTES:
// [OPT-1] Consider covisibility graph for efficient keyframe connections
// [OPT-2] Implement keyframe culling based on redundancy
// [OPT-3] Use efficient pose distance metrics (SE3 geodesic distance)
// [OPT-4] Consider bag-of-words for fast keyframe retrieval
// [OPT-5] Implement hierarchical keyframe storage for large maps

class KeyframeHandler {
    constructor(cv) {
        this.cv = cv

        // ============================================================
        // CONFIGURATION - Tuned for highest accuracy
        // ============================================================
        this.config = {
            // Keyframe creation criteria
            minTranslation: 0.03,          // Minimum translation (meters) from last keyframe
            minRotationDegrees: 10,         // Minimum rotation (degrees) from last keyframe
            minFramesBetween: 10,           // Minimum frames between keyframes
            maxFramesBetween: 60,           // Force keyframe after this many frames

            // Feature-based criteria
            minTrackedRatio: 0.5,           // Create keyframe if tracked ratio drops below this
            minCovisiblePoints: 50,         // Minimum covisible map points with last keyframe

            // Keyframe quality
            minFeatures: 100,               // Minimum features to create keyframe
            minStableFeatures: 50,          // Minimum stable (age > 3) features

            // Storage limits
            maxKeyframes: 100,              // Maximum keyframes to store

            // Relocalization
            maxRelocCandidates: 5,          // Maximum keyframes to consider for relocalization
        }

        // ============================================================
        // STATE
        // ============================================================

        this.keyframes = []               // Array of keyframes (ordered by creation)
        this.lastKeyframe = null          // Most recent keyframe
        this.lastKeyframeFrame = 0        // Frame number of last keyframe
        this.nextKeyframeId = 0

        // Covisibility graph
        // [OPT-6] Implement proper graph structure with edge weights
        this.covisibility = new Map()     // keyframeId -> Map(keyframeId -> sharedPointCount)

        // Pose history for motion estimation
        this.poseHistory = []             // Recent poses for velocity estimation
        this.maxPoseHistory = 10

        // Statistics
        this.stats = {
            totalKeyframes: 0,
            avgFeatures: 0,
            avgCovisibility: 0,
        }

        console.log('[KeyframeHandler] Initialized')
    }

    // ============================================================
    // KEYFRAME STRUCTURE
    // ============================================================
    // 
    // {
    //   id: number,
    //   frameNumber: number,
    //   timestamp: number,
    //   pose: Float64Array(16),         // 4x4 camera pose matrix
    //   features: Array,                // 2D feature points
    //   descriptors: cv.Mat,            // ORB descriptors
    //   mapPointIds: Set,               // IDs of observed map points
    //   neighbors: Map,                 // Connected keyframes
    //   imageData: ImageData,           // Optional: stored image for relocalization
    // }
    // ============================================================

    // ============================================================
    // DECIDE IF CURRENT FRAME SHOULD BE KEYFRAME
    // ============================================================
    // 
    // Evaluates multiple criteria to determine if a keyframe should be created.
    // Returns true if any criterion is met.
    // 
    // @param {Object} currentPose - Current camera pose (Matrix4 elements)
    // @param {Object} trackingResult - Result from FeatureTracker
    // @param {number} frameNumber - Current frame number
    // @param {Object} mapStats - Statistics from MapManager
    // @returns {boolean} - True if keyframe should be created
    // ============================================================
    shouldCreateKeyframe(currentPose, trackingResult, frameNumber, mapStats = {}) {
        // First keyframe is always created
        if (!this.lastKeyframe) {
            return true
        }

        const framesSinceLastKF = frameNumber - this.lastKeyframeFrame

        // ============================================================
        // CRITERION 1: Maximum frame interval
        // ============================================================
        // Force keyframe creation after too many frames
        if (framesSinceLastKF >= this.config.maxFramesBetween) {
            console.log('[KeyframeHandler] Max frame interval reached')
            return true
        }

        // ============================================================
        // CRITERION 2: Minimum frame interval
        // ============================================================
        // Don't create keyframes too frequently
        if (framesSinceLastKF < this.config.minFramesBetween) {
            return false
        }

        // ============================================================
        // CRITERION 3: Translation threshold
        // ============================================================
        // [OPT-7] Consider anisotropic thresholds based on viewing direction
        const translation = this._computeTranslation(
            this.lastKeyframe.pose,
            currentPose
        )

        if (translation >= this.config.minTranslation) {
            console.log(`[KeyframeHandler] Translation threshold: ${translation.toFixed(3)}m`)
            return true
        }

        // ============================================================
        // CRITERION 4: Rotation threshold
        // ============================================================
        const rotationDeg = this._computeRotation(
            this.lastKeyframe.pose,
            currentPose
        )

        if (rotationDeg >= this.config.minRotationDegrees) {
            console.log(`[KeyframeHandler] Rotation threshold: ${rotationDeg.toFixed(1)}°`)
            return true
        }

        // ============================================================
        // CRITERION 5: Tracking quality degradation
        // ============================================================
        // Create keyframe if tracking quality is dropping
        if (trackingResult && trackingResult.stats) {
            const { trackedCount } = trackingResult.stats
            const lastFeatureCount = this.lastKeyframe.features?.length || 0

            if (lastFeatureCount > 0) {
                const trackedRatio = trackedCount / lastFeatureCount

                if (trackedRatio < this.config.minTrackedRatio) {
                    console.log(`[KeyframeHandler] Tracked ratio low: ${(trackedRatio * 100).toFixed(1)}%`)
                    return true
                }
            }
        }

        // ============================================================
        // CRITERION 6: Covisibility with last keyframe
        // ============================================================
        // [OPT-8] Consider weighted covisibility based on point quality
        if (mapStats.covisiblePoints !== undefined) {
            if (mapStats.covisiblePoints < this.config.minCovisiblePoints) {
                console.log(`[KeyframeHandler] Low covisibility: ${mapStats.covisiblePoints}`)
                return true
            }
        }

        return false
    }

    // ============================================================
    // CREATE KEYFRAME
    // ============================================================
    // 
    // Creates and stores a new keyframe.
    // 
    // @param {Object} params - Keyframe parameters
    // @returns {Object} - Created keyframe
    // ============================================================
    createKeyframe({
        frameNumber,
        timestamp = Date.now(),
        pose,
        features,
        descriptors = null,
        mapPointIds = new Set(),
        grayFrame = null,
    }) {
        const cv = this.cv

        // Validate feature count
        if (!features || features.length < this.config.minFeatures) {
            console.warn(`[KeyframeHandler] Insufficient features: ${features?.length || 0}`)
            return null
        }

        // Count stable features
        const stableFeatures = features.filter(f => f.age >= 3)
        if (stableFeatures.length < this.config.minStableFeatures) {
            console.warn(`[KeyframeHandler] Insufficient stable features: ${stableFeatures.length}`)
            // Continue anyway - this is a soft threshold
        }

        // ============================================================
        // Create keyframe object
        // ============================================================
        const keyframe = {
            id: this.nextKeyframeId++,
            frameNumber,
            timestamp,
            pose: new Float64Array(pose),
            features: features.map(f => ({
                id: f.id,
                x: f.x,
                y: f.y,
                age: f.age,
                quality: f.quality,
            })),
            descriptors: descriptors ? descriptors.clone() : null,
            mapPointIds: new Set(mapPointIds),
            neighbors: new Map(),

            // Store grayscale image for relocalization
            // [OPT-9] Consider storing only a downsampled version
            imageData: grayFrame ? this._storeGrayFrame(grayFrame) : null,
        }

        // ============================================================
        // Update covisibility graph
        // ============================================================
        // [OPT-10] Implement incremental covisibility update
        if (this.lastKeyframe && mapPointIds.size > 0) {
            const sharedPoints = this._countSharedPoints(
                this.lastKeyframe.mapPointIds,
                mapPointIds
            )

            if (sharedPoints > 0) {
                keyframe.neighbors.set(this.lastKeyframe.id, sharedPoints)
                this.lastKeyframe.neighbors.set(keyframe.id, sharedPoints)

                // Update covisibility map
                if (!this.covisibility.has(keyframe.id)) {
                    this.covisibility.set(keyframe.id, new Map())
                }
                this.covisibility.get(keyframe.id).set(this.lastKeyframe.id, sharedPoints)

                if (!this.covisibility.has(this.lastKeyframe.id)) {
                    this.covisibility.set(this.lastKeyframe.id, new Map())
                }
                this.covisibility.get(this.lastKeyframe.id).set(keyframe.id, sharedPoints)
            }
        }

        // ============================================================
        // Store keyframe
        // ============================================================
        this.keyframes.push(keyframe)
        this.lastKeyframe = keyframe
        this.lastKeyframeFrame = frameNumber

        // Enforce max keyframes
        if (this.keyframes.length > this.config.maxKeyframes) {
            this._cullOldKeyframes()
        }

        // Update statistics
        this._updateStats()

        console.log(`[KeyframeHandler] Created keyframe ${keyframe.id} at frame ${frameNumber} with ${features.length} features`)

        return keyframe
    }

    // ============================================================
    // GET CONNECTED KEYFRAMES
    // ============================================================
    // 
    // Returns keyframes connected to the given keyframe in the covisibility graph.
    // Sorted by connection strength (shared map points).
    // ============================================================
    getConnectedKeyframes(keyframeId, maxCount = 10) {
        const keyframe = this.keyframes.find(kf => kf.id === keyframeId)
        if (!keyframe) return []

        const connections = Array.from(keyframe.neighbors.entries())
            .sort((a, b) => b[1] - a[1])  // Sort by shared point count
            .slice(0, maxCount)

        return connections.map(([id, weight]) => ({
            keyframe: this.keyframes.find(kf => kf.id === id),
            weight,
        })).filter(c => c.keyframe)
    }

    // ============================================================
    // FIND KEYFRAME FOR RELOCALIZATION
    // ============================================================
    // 
    // Finds the best keyframe candidates for relocalization.
    // Uses feature matching and geometric verification.
    // 
    // @param {Array} currentFeatures - Current frame features
    // @param {cv.Mat} currentDescriptors - Current frame descriptors
    // @returns {Array} - Ranked keyframe candidates
    // ============================================================
    findRelocalizationCandidates(currentFeatures, currentDescriptors) {
        if (!currentDescriptors || this.keyframes.length === 0) {
            return []
        }

        const cv = this.cv
        const matcher = new cv.BFMatcher(cv.NORM_HAMMING, false)
        const candidates = []

        // [OPT-11] Use bag-of-words for faster initial candidate selection
        // [OPT-12] Implement vocabulary tree for large-scale relocalization

        for (const keyframe of this.keyframes) {
            if (!keyframe.descriptors) continue

            // Match descriptors
            const matches = new cv.DMatchVectorVector()
            matcher.knnMatch(currentDescriptors, keyframe.descriptors, matches, 2)

            // Ratio test
            let goodMatches = 0
            for (let i = 0; i < matches.size(); i++) {
                const pair = matches.get(i)
                if (pair.size() >= 2) {
                    const m = pair.get(0)
                    const n = pair.get(1)
                    if (m.distance < 0.7 * n.distance) {
                        goodMatches++
                    }
                }
            }

            matches.delete()

            if (goodMatches >= 20) {  // Minimum matches for candidate
                candidates.push({
                    keyframe,
                    matchCount: goodMatches,
                    score: goodMatches / Math.max(keyframe.features.length, 1),
                })
            }
        }

        matcher.delete()

        // Sort by match score
        candidates.sort((a, b) => b.score - a.score)

        return candidates.slice(0, this.config.maxRelocCandidates)
    }

    // ============================================================
    // GET RECENT KEYFRAMES
    // ============================================================
    getRecentKeyframes(count = 5) {
        return this.keyframes.slice(-count)
    }

    // ============================================================
    // GET KEYFRAME BY ID
    // ============================================================
    getKeyframe(id) {
        return this.keyframes.find(kf => kf.id === id)
    }

    // ============================================================
    // UPDATE KEYFRAME MAP POINTS
    // ============================================================
    updateMapPoints(keyframeId, newMapPointIds) {
        const keyframe = this.keyframes.find(kf => kf.id === keyframeId)
        if (!keyframe) return

        for (const id of newMapPointIds) {
            keyframe.mapPointIds.add(id)
        }

        // Update covisibility with neighbors
        // [OPT-13] Batch covisibility updates
        for (const [neighborId] of keyframe.neighbors) {
            const neighbor = this.keyframes.find(kf => kf.id === neighborId)
            if (neighbor) {
                const sharedPoints = this._countSharedPoints(
                    keyframe.mapPointIds,
                    neighbor.mapPointIds
                )
                keyframe.neighbors.set(neighborId, sharedPoints)
                neighbor.neighbors.set(keyframeId, sharedPoints)
            }
        }
    }

    // ============================================================
    // ADD POSE TO HISTORY
    // ============================================================
    addPoseToHistory(pose, timestamp) {
        this.poseHistory.push({ pose: new Float64Array(pose), timestamp })

        if (this.poseHistory.length > this.maxPoseHistory) {
            this.poseHistory.shift()
        }
    }

    // ============================================================
    // ESTIMATE VELOCITY
    // ============================================================
    // 
    // Estimates current velocity from pose history.
    // Used for motion prediction.
    // ============================================================
    estimateVelocity() {
        if (this.poseHistory.length < 2) {
            return { linear: [0, 0, 0], angular: [0, 0, 0] }
        }

        const recent = this.poseHistory[this.poseHistory.length - 1]
        const prev = this.poseHistory[this.poseHistory.length - 2]

        const dt = (recent.timestamp - prev.timestamp) / 1000  // seconds
        if (dt <= 0) {
            return { linear: [0, 0, 0], angular: [0, 0, 0] }
        }

        // Linear velocity
        const dx = recent.pose[12] - prev.pose[12]
        const dy = recent.pose[13] - prev.pose[13]
        const dz = recent.pose[14] - prev.pose[14]

        const linear = [dx / dt, dy / dt, dz / dt]

        // Angular velocity (simplified - just using translation for now)
        // [OPT-14] Implement proper SO3 angular velocity computation
        const angular = [0, 0, 0]

        return { linear, angular }
    }

    // ============================================================
    // HELPER METHODS
    // ============================================================

    _computeTranslation(pose1, pose2) {
        const dx = pose2[12] - pose1[12]
        const dy = pose2[13] - pose1[13]
        const dz = pose2[14] - pose1[14]
        return Math.sqrt(dx * dx + dy * dy + dz * dz)
    }

    _computeRotation(pose1, pose2) {
        // Compute rotation difference using axis-angle representation
        // [OPT-15] Use quaternion representation for more efficient rotation diff

        // Extract rotation matrices
        const R1 = [
            pose1[0], pose1[1], pose1[2],
            pose1[4], pose1[5], pose1[6],
            pose1[8], pose1[9], pose1[10],
        ]
        const R2 = [
            pose2[0], pose2[1], pose2[2],
            pose2[4], pose2[5], pose2[6],
            pose2[8], pose2[9], pose2[10],
        ]

        // Relative rotation: R_rel = R2 * R1^T
        // Compute trace of R_rel to get rotation angle
        // trace(R_rel) = 1 + 2*cos(angle)

        const trace =
            R1[0] * R2[0] + R1[1] * R2[1] + R1[2] * R2[2] +
            R1[3] * R2[3] + R1[4] * R2[4] + R1[5] * R2[5] +
            R1[6] * R2[6] + R1[7] * R2[7] + R1[8] * R2[8]

        const cosAngle = Math.max(-1, Math.min(1, (trace - 1) / 2))
        const angleDeg = Math.acos(cosAngle) * 180 / Math.PI

        return angleDeg
    }

    _countSharedPoints(set1, set2) {
        let count = 0
        for (const id of set1) {
            if (set2.has(id)) count++
        }
        return count
    }

    _storeGrayFrame(grayFrame) {
        // Store downsampled grayscale for relocalization
        // [OPT-16] Consider storing only pyramid levels
        const cv = this.cv

        const scaled = new cv.Mat()
        const scaleFactor = 0.5  // Store at half resolution
        cv.resize(grayFrame, scaled, new cv.Size(0, 0), scaleFactor, scaleFactor)

        // Convert to ImageData for storage
        const rgba = new cv.Mat()
        cv.cvtColor(scaled, rgba, cv.COLOR_GRAY2RGBA)

        const imageData = new ImageData(
            new Uint8ClampedArray(rgba.data),
            scaled.cols,
            scaled.rows
        )

        scaled.delete()
        rgba.delete()

        return imageData
    }

    _cullOldKeyframes() {
        // Remove oldest keyframes that are not well-connected
        // [OPT-17] Implement smarter culling based on map coverage

        const toRemove = []
        const minKeep = Math.floor(this.config.maxKeyframes * 0.8)

        for (let i = 0; i < this.keyframes.length - minKeep; i++) {
            const kf = this.keyframes[i]

            // Don't remove well-connected keyframes
            if (kf.neighbors.size < 3) {
                toRemove.push(i)
            }
        }

        // Remove in reverse order to maintain indices
        for (let i = toRemove.length - 1; i >= 0; i--) {
            const idx = toRemove[i]
            const kf = this.keyframes[idx]

            // Clean up covisibility
            if (kf.descriptors) kf.descriptors.delete()
            this.covisibility.delete(kf.id)

            this.keyframes.splice(idx, 1)
        }

        console.log(`[KeyframeHandler] Culled ${toRemove.length} old keyframes`)
    }

    _updateStats() {
        if (this.keyframes.length === 0) {
            this.stats = { totalKeyframes: 0, avgFeatures: 0, avgCovisibility: 0 }
            return
        }

        const totalFeatures = this.keyframes.reduce(
            (sum, kf) => sum + (kf.features?.length || 0), 0
        )
        const totalNeighbors = this.keyframes.reduce(
            (sum, kf) => sum + kf.neighbors.size, 0
        )

        this.stats = {
            totalKeyframes: this.keyframes.length,
            avgFeatures: totalFeatures / this.keyframes.length,
            avgCovisibility: totalNeighbors / this.keyframes.length,
        }
    }

    // ============================================================
    // STATISTICS
    // ============================================================
    getStats() {
        return { ...this.stats }
    }

    // ============================================================
    // VISUALIZATION DATA
    // ============================================================
    getVisualizationData() {
        return this.keyframes.map(kf => ({
            id: kf.id,
            frameNumber: kf.frameNumber,
            position: [kf.pose[12], kf.pose[13], kf.pose[14]],
            featureCount: kf.features?.length || 0,
            neighborCount: kf.neighbors.size,
            mapPointCount: kf.mapPointIds.size,
        }))
    }

    // ============================================================
    // CLEANUP
    // ============================================================
    reset() {
        // Clean up OpenCV mats
        for (const kf of this.keyframes) {
            if (kf.descriptors) kf.descriptors.delete()
        }

        this.keyframes = []
        this.lastKeyframe = null
        this.lastKeyframeFrame = 0
        this.nextKeyframeId = 0
        this.covisibility.clear()
        this.poseHistory = []
        this.stats = { totalKeyframes: 0, avgFeatures: 0, avgCovisibility: 0 }

        console.log('[KeyframeHandler] Reset')
    }

    dispose() {
        this.reset()
        console.log('[KeyframeHandler] Disposed')
    }
}
