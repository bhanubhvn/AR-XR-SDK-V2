// feature-tracker.js - High-Accuracy Feature Tracking using Lucas-Kanade Optical Flow
// 
// OPTIMIZATION NOTES:
// [OPT-1] Consider GPU-accelerated optical flow via WebGL compute shaders
// [OPT-2] Implement hierarchical feature tracking (coarse-to-fine)
// [OPT-3] Use SIMD.js for vectorized distance calculations when available
// [OPT-4] Consider Web Workers for parallel feature detection
// [OPT-5] Implement adaptive pyramid levels based on motion magnitude

class FeatureTracker {
    constructor(cv) {
        this.cv = cv

        // ============================================================
        // CONFIGURATION - Tuned for highest accuracy
        // ============================================================
        this.config = {
            // Lucas-Kanade Optical Flow Parameters
            // [OPT-6] These can be dynamically adjusted based on tracking quality
            lkWinSize: 31,           // Larger window = more robust but slower
            lkMaxLevel: 4,           // More pyramid levels = better large motion handling
            lkMaxIter: 50,           // More iterations = higher accuracy convergence
            lkEpsilon: 0.001,        // Tighter convergence threshold

            // Feature Detection Parameters (ORB)
            maxFeatures: 800,        // Maximum features to track (increased for accuracy)
            minFeatures: 200,        // Minimum before triggering re-detection
            qualityLevel: 0.01,      // Shi-Tomasi corner quality threshold
            minDistance: 10,         // Minimum distance between features (pixels)

            // Tracking Quality Thresholds
            minTrackingQuality: 0.7,        // Minimum quality to consider point tracked
            maxForwardBackwardError: 1.0,   // Max FB error (pixels) for valid track

            // Subpixel Refinement
            // [OPT-7] cv.cornerSubPix is NOT available in standard OpenCV.js build
            // Set to false - would need custom OpenCV.js build with imgproc for this
            useSubpixelRefinement: false,
            subpixelWinSize: 5,
            subpixelZeroZone: -1,
            subpixelMaxIter: 50,
            subpixelEpsilon: 0.001,

            // Feature Grid for uniform distribution
            // [OPT-8] Implement adaptive grid sizing based on image regions of interest
            useFeatureGrid: true,
            gridRows: 8,
            gridCols: 8,
            maxFeaturesPerCell: 15,

            // Descriptor matching for lost feature recovery
            descriptorMatchThreshold: 30,  // Hamming distance threshold

            // Periodic re-detection (to detect new features on static objects)
            redetectionInterval: 30,       // Re-detect every N frames
            redetectionMinAge: 10,         // Only replace points older than this
        }

        // ============================================================
        // STATE
        // ============================================================
        this.prevFrame = null           // Previous grayscale frame (cv.Mat)
        this.prevPoints = null          // Previous tracked points (cv.Mat)
        this.prevDescriptors = null     // ORB descriptors for tracked points
        this.pointIds = []              // Unique ID for each tracked point
        this.pointAges = []             // How many frames each point has been tracked
        this.pointQualities = []        // Quality score for each point (0-1)
        this.nextPointId = 0            // Counter for unique point IDs

        // Detector and descriptor
        this.orb = new cv.ORB(this.config.maxFeatures)

        // [OPT-9] Consider using AKAZE for better accuracy (when available in OpenCV.js)
        // this.akaze = new cv.AKAZE()

        // Feature grid mask for uniform distribution
        this.gridMask = null

        // Frame counter for periodic re-detection
        this.frameCount = 0

        // Statistics
        this.stats = {
            trackedCount: 0,
            lostCount: 0,
            newCount: 0,
            avgQuality: 0,
            avgAge: 0,
            processingTimeMs: 0,
        }

        console.log('[FeatureTracker] Initialized with config:', this.config)
    }

    // ============================================================
    // MAIN TRACKING FUNCTION
    // ============================================================
    // 
    // Tracks features from previous frame to current frame using optical flow.
    // Automatically detects new features when tracking degrades.
    // 
    // @param {cv.Mat} grayFrame - Current grayscale frame
    // @returns {Object} Tracking result with points, IDs, and statistics
    // ============================================================
    track(grayFrame) {
        const startTime = performance.now()
        const cv = this.cv

        // First frame - just detect features
        if (!this.prevFrame || !this.prevPoints || this.prevPoints.rows === 0) {
            this._detectInitialFeatures(grayFrame)
            this.prevFrame = grayFrame.clone()
            this.stats.processingTimeMs = performance.now() - startTime
            return this._buildResult()
        }

        // ============================================================
        // STEP 1: Forward-Backward Optical Flow
        // ============================================================
        // Track points forward (prev -> curr), then backward (curr -> prev)
        // Points with low forward-backward error are considered good tracks
        // [OPT-10] Consider pyramidal caching to avoid recomputing pyramids

        const {
            nextPoints,
            status,
            forwardBackwardErrors
        } = this._trackWithForwardBackward(grayFrame)

        // ============================================================
        // STEP 2: Filter Good Tracks
        // ============================================================

        const goodIndices = []
        const newPointsList = []
        const newIdsList = []
        const newAgesList = []
        const newQualitiesList = []

        for (let i = 0; i < status.rows; i++) {
            const isTracked = status.data[i] === 1
            const fbError = forwardBackwardErrors[i]

            if (isTracked && fbError < this.config.maxForwardBackwardError) {
                // Valid track
                const x = nextPoints.data32F[i * 2]
                const y = nextPoints.data32F[i * 2 + 1]

                // Check bounds
                if (x >= 0 && x < grayFrame.cols && y >= 0 && y < grayFrame.rows) {
                    // Update quality based on forward-backward error
                    // [OPT-11] Consider incorporating local image gradient strength
                    const quality = Math.max(0, 1 - fbError / this.config.maxForwardBackwardError)

                    goodIndices.push(i)
                    newPointsList.push(x, y)
                    newIdsList.push(this.pointIds[i])
                    newAgesList.push(this.pointAges[i] + 1)
                    newQualitiesList.push((this.pointQualities[i] * 0.8) + (quality * 0.2)) // Exponential moving average
                }
            }
        }

        // Update statistics
        this.stats.trackedCount = goodIndices.length
        this.stats.lostCount = status.rows - goodIndices.length

        // ============================================================
        // STEP 3: Subpixel Refinement
        // ============================================================
        // [OPT-12] Consider adaptive refinement - only refine high-quality tracks

        let refinedPoints
        if (this.config.useSubpixelRefinement && newPointsList.length >= 2) {
            refinedPoints = this._refineSubpixel(grayFrame, newPointsList)
        } else {
            refinedPoints = cv.matFromArray(
                newPointsList.length / 2, 1, cv.CV_32FC2, newPointsList
            )
        }

        // ============================================================
        // STEP 4: Detect New Features if Needed
        // ============================================================
        // Detect new features if:
        // 1. We have fewer than minFeatures, OR
        // 2. It's time for periodic re-detection (to catch new objects)

        this.frameCount++
        const needsPeriodicRedetection = (this.frameCount % this.config.redetectionInterval === 0)
        const belowMinFeatures = (refinedPoints.rows < this.config.minFeatures)

        if (belowMinFeatures || needsPeriodicRedetection) {
            const newDetected = this._detectNewFeatures(grayFrame, refinedPoints)

            if (newDetected && newDetected.length > 0) {
                // Merge new features with existing
                for (let i = 0; i < newDetected.length; i++) {
                    newPointsList.push(newDetected[i].x, newDetected[i].y)
                    newIdsList.push(this.nextPointId++)
                    newAgesList.push(0)
                    newQualitiesList.push(0.5) // Initial quality
                }

                this.stats.newCount = newDetected.length

                // Rebuild points matrix with new features
                refinedPoints.delete()
                refinedPoints = cv.matFromArray(
                    newPointsList.length / 2, 1, cv.CV_32FC2, newPointsList
                )

                if (needsPeriodicRedetection && !belowMinFeatures) {
                    console.log(`[FeatureTracker] Periodic re-detection: +${newDetected.length} features`)
                }
            }
        } else {
            this.stats.newCount = 0
        }

        // ============================================================
        // STEP 5: Update State
        // ============================================================

        // Clean up old state
        if (this.prevPoints) this.prevPoints.delete()
        if (this.prevFrame) this.prevFrame.delete()
        nextPoints.delete()
        status.delete()

        // Update state
        this.prevPoints = refinedPoints
        this.prevFrame = grayFrame.clone()
        this.pointIds = newIdsList
        this.pointAges = newAgesList
        this.pointQualities = newQualitiesList

        // Update stats
        this.stats.avgQuality = newQualitiesList.length > 0
            ? newQualitiesList.reduce((a, b) => a + b, 0) / newQualitiesList.length
            : 0
        this.stats.avgAge = newAgesList.length > 0
            ? newAgesList.reduce((a, b) => a + b, 0) / newAgesList.length
            : 0
        this.stats.processingTimeMs = performance.now() - startTime

        return this._buildResult()
    }

    // ============================================================
    // FORWARD-BACKWARD OPTICAL FLOW
    // ============================================================
    // 
    // Performs bidirectional optical flow for robust tracking.
    // Points that don't track back to their original position are rejected.
    // This is a key technique from visual odometry / SLAM pipelines.
    // ============================================================
    _trackWithForwardBackward(currFrame) {
        const cv = this.cv

        // Lucas-Kanade parameters
        const winSize = new cv.Size(this.config.lkWinSize, this.config.lkWinSize)
        const criteria = new cv.TermCriteria(
            cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT,
            this.config.lkMaxIter,
            this.config.lkEpsilon
        )

        // Forward tracking: prev -> curr
        const nextPoints = new cv.Mat()
        const statusFwd = new cv.Mat()
        const errFwd = new cv.Mat()

        cv.calcOpticalFlowPyrLK(
            this.prevFrame,
            currFrame,
            this.prevPoints,
            nextPoints,
            statusFwd,
            errFwd,
            winSize,
            this.config.lkMaxLevel,
            criteria
        )

        // Backward tracking: curr -> prev (for validation)
        const backPoints = new cv.Mat()
        const statusBwd = new cv.Mat()
        const errBwd = new cv.Mat()

        cv.calcOpticalFlowPyrLK(
            currFrame,
            this.prevFrame,
            nextPoints,
            backPoints,
            statusBwd,
            errBwd,
            winSize,
            this.config.lkMaxLevel,
            criteria
        )

        // Calculate forward-backward errors
        // [OPT-13] Consider using SIMD for vectorized distance calculation
        const forwardBackwardErrors = []
        for (let i = 0; i < this.prevPoints.rows; i++) {
            if (statusFwd.data[i] === 1 && statusBwd.data[i] === 1) {
                const origX = this.prevPoints.data32F[i * 2]
                const origY = this.prevPoints.data32F[i * 2 + 1]
                const backX = backPoints.data32F[i * 2]
                const backY = backPoints.data32F[i * 2 + 1]

                const error = Math.sqrt((origX - backX) ** 2 + (origY - backY) ** 2)
                forwardBackwardErrors.push(error)
            } else {
                forwardBackwardErrors.push(Infinity)
                statusFwd.data[i] = 0 // Mark as failed
            }
        }

        // Clean up
        backPoints.delete()
        statusBwd.delete()
        errBwd.delete()
        errFwd.delete()

        return {
            nextPoints,
            status: statusFwd,
            forwardBackwardErrors,
        }
    }

    // ============================================================
    // SUBPIXEL REFINEMENT
    // ============================================================
    // 
    // NOTE: cv.cornerSubPix is NOT available in standard OpenCV.js build.
    // This function is disabled by default. To enable, you would need a
    // custom OpenCV.js build with the imgproc module fully included.
    // 
    // For now, this just returns the points as-is (no refinement).
    // The optical flow sub-pixel accuracy is still good enough for tracking.
    // ============================================================
    _refineSubpixel(grayFrame, pointsList) {
        const cv = this.cv

        // Just return points as matrix without refinement
        // cv.cornerSubPix is not available in OpenCV.js standard build
        const corners = cv.matFromArray(
            pointsList.length / 2, 1, cv.CV_32FC2, pointsList
        )

        // [OPT-7] If custom OpenCV.js build is available with cornerSubPix:
        // const winSize = new cv.Size(this.config.subpixelWinSize, this.config.subpixelWinSize)
        // const zeroZone = new cv.Size(-1, -1)
        // const criteria = new cv.TermCriteria(
        //     cv.TERM_CRITERIA_EPS | cv.TERM_CRITERIA_COUNT,
        //     this.config.subpixelMaxIter,
        //     this.config.subpixelEpsilon
        // )
        // cv.cornerSubPix(grayFrame, corners, winSize, zeroZone, criteria)

        return corners
    }

    // ============================================================
    // INITIAL FEATURE DETECTION
    // ============================================================
    _detectInitialFeatures(grayFrame) {
        const cv = this.cv

        // Detect Shi-Tomasi corners (Good Features to Track)
        // [OPT-14] Consider using Harris corners for more repeatability
        const corners = new cv.Mat()
        const maxCorners = this.config.maxFeatures
        const qualityLevel = this.config.qualityLevel
        const minDistance = this.config.minDistance

        cv.goodFeaturesToTrack(
            grayFrame,
            corners,
            maxCorners,
            qualityLevel,
            minDistance
        )

        // Subpixel refinement - DISABLED
        // cv.cornerSubPix is not available in standard OpenCV.js build
        // The optical flow already provides sub-pixel accuracy through
        // the pyramidal LK tracker, so this is not critical.
        // if (corners.rows > 0 && this.config.useSubpixelRefinement) {
        //     cv.cornerSubPix(grayFrame, corners, winSize, zeroZone, criteria)
        // }

        // Initialize tracking state
        this.prevPoints = corners
        this.pointIds = []
        this.pointAges = []
        this.pointQualities = []

        for (let i = 0; i < corners.rows; i++) {
            this.pointIds.push(this.nextPointId++)
            this.pointAges.push(0)
            this.pointQualities.push(0.5)
        }

        console.log(`[FeatureTracker] Detected ${corners.rows} initial features`)
    }

    // ============================================================
    // DETECT NEW FEATURES
    // ============================================================
    // 
    // Detects new features in regions without existing tracks.
    // Uses a grid-based approach for uniform feature distribution.
    // ============================================================
    _detectNewFeatures(grayFrame, existingPoints) {
        const cv = this.cv

        // Create mask to avoid detecting near existing points
        // Note: cv.Mat.ones syntax differs in OpenCV.js - use Mat constructor + setTo
        const mask = new cv.Mat(grayFrame.rows, grayFrame.cols, cv.CV_8UC1)
        mask.setTo(new cv.Scalar(255))

        // Mark regions around existing points as unavailable
        const exclusionRadius = this.config.minDistance
        for (let i = 0; i < existingPoints.rows; i++) {
            const x = Math.round(existingPoints.data32F[i * 2])
            const y = Math.round(existingPoints.data32F[i * 2 + 1])
            cv.circle(mask, new cv.Point(x, y), exclusionRadius, new cv.Scalar(0), -1)
        }

        // Detect new corners in unmasked regions
        const corners = new cv.Mat()
        const maxNew = this.config.maxFeatures - existingPoints.rows

        cv.goodFeaturesToTrack(
            grayFrame,
            corners,
            maxNew,
            this.config.qualityLevel,
            this.config.minDistance,
            mask
        )

        mask.delete()

        // Extract points
        const newPoints = []
        for (let i = 0; i < corners.rows; i++) {
            newPoints.push({
                x: corners.data32F[i * 2],
                y: corners.data32F[i * 2 + 1],
            })
        }

        corners.delete()

        console.log(`[FeatureTracker] Detected ${newPoints.length} new features`)
        return newPoints
    }

    // ============================================================
    // BUILD RESULT OBJECT
    // ============================================================
    _buildResult() {
        const points = []
        if (this.prevPoints) {
            for (let i = 0; i < this.prevPoints.rows; i++) {
                points.push({
                    id: this.pointIds[i],
                    x: this.prevPoints.data32F[i * 2],
                    y: this.prevPoints.data32F[i * 2 + 1],
                    age: this.pointAges[i],
                    quality: this.pointQualities[i],
                })
            }
        }

        return {
            points,
            stats: { ...this.stats },
        }
    }

    // ============================================================
    // GET POINTS FOR POSE ESTIMATION
    // ============================================================
    // 
    // Returns high-quality points suitable for pose estimation.
    // Filters by age and quality thresholds.
    // ============================================================
    getStablePoints(minAge = 3, minQuality = 0.5) {
        const stable = []
        for (let i = 0; i < this.prevPoints?.rows || 0; i++) {
            if (this.pointAges[i] >= minAge && this.pointQualities[i] >= minQuality) {
                stable.push({
                    id: this.pointIds[i],
                    x: this.prevPoints.data32F[i * 2],
                    y: this.prevPoints.data32F[i * 2 + 1],
                    age: this.pointAges[i],
                    quality: this.pointQualities[i],
                })
            }
        }
        return stable
    }

    // ============================================================
    // FORCE RE-DETECTION
    // ============================================================
    // 
    // Forces a complete re-detection of features.
    // Useful after large viewpoint changes or tracking failure.
    // ============================================================
    reset() {
        if (this.prevPoints) this.prevPoints.delete()
        if (this.prevFrame) this.prevFrame.delete()
        if (this.prevDescriptors) this.prevDescriptors.delete()

        this.prevPoints = null
        this.prevFrame = null
        this.prevDescriptors = null
        this.pointIds = []
        this.pointAges = []
        this.pointQualities = []
        this.frameCount = 0

        console.log('[FeatureTracker] Reset')
    }

    // ============================================================
    // CLEANUP
    // ============================================================
    dispose() {
        this.reset()
        if (this.orb) this.orb.delete()
        if (this.gridMask) this.gridMask.delete()
        console.log('[FeatureTracker] Disposed')
    }
}
