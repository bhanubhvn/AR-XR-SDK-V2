// ar-engine.js - OpenCV-based AR Target Detection

class AREngine {
  constructor() {
    this.cv = null
    this.orb = null
    this.matcher = null
    this.targetKeypoints = null
    this.targetDescriptors = null
    this.targetSize = null
    this.isReady = false

    this.config = {
      maxFeatures: 2000,        // More features for distant detection
      ratioThreshold: 0.85,     // Relaxed for distant/blurry matches
      minMatches: 6,            // Lower threshold for distant targets
      minInliers: 5,            // Accept fewer inliers at distance
      ransacThreshold: 4.0,     // More tolerance for perspective distortion
      singleMatchThreshold: 45, // Higher distance threshold
    }
  }

  init(cv) {
    this.cv = cv
    this.orb = new cv.ORB(this.config.maxFeatures)
    this.matcher = new cv.BFMatcher(cv.NORM_HAMMING, false)
    this.isReady = true
  }

  processTarget(imageData) {
    if (!this.isReady) throw new Error("AREngine not initialized")
    const cv = this.cv

    // Cleanup previous
    if (this.targetDescriptors) this.targetDescriptors.delete()
    if (this.targetKeypoints) this.targetKeypoints.delete()

    // Convert to grayscale
    const src = cv.matFromImageData(imageData)
    const gray = new cv.Mat()
    cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY)

    // Enhance contrast
    const enhanced = new cv.Mat()
    cv.equalizeHist(gray, enhanced)

    // Detect features
    const keypoints = new cv.KeyPointVector()
    const descriptors = new cv.Mat()
    this.orb.detectAndCompute(enhanced, new cv.Mat(), keypoints, descriptors)

    this.targetKeypoints = keypoints
    this.targetDescriptors = descriptors
    this.targetSize = { width: gray.cols, height: gray.rows }

    // Cleanup
    src.delete()
    gray.delete()
    enhanced.delete()

    return {
      keypoints: keypoints.size(),
      width: this.targetSize.width,
      height: this.targetSize.height,
    }
  }

  processFrame(imageData) {
    if (!this.isReady || !this.targetDescriptors) return null
    const cv = this.cv

    // Convert frame
    const src = cv.matFromImageData(imageData)
    const gray = new cv.Mat()
    cv.cvtColor(src, gray, cv.COLOR_RGBA2GRAY)

    const enhanced = new cv.Mat()
    cv.equalizeHist(gray, enhanced)

    // Detect features
    const frameKeypoints = new cv.KeyPointVector()
    const frameDescriptors = new cv.Mat()
    this.orb.detectAndCompute(
      enhanced,
      new cv.Mat(),
      frameKeypoints,
      frameDescriptors
    )

    let result = { matches: 0, inliers: 0, corners: null }

    if (frameDescriptors.rows > 5 && this.targetDescriptors.rows > 5) {
      // KNN match
      const knnMatches = new cv.DMatchVectorVector()
      this.matcher.knnMatch(
        this.targetDescriptors,
        frameDescriptors,
        knnMatches,
        2
      )

      // Ratio test
      const goodMatches = []
      for (let i = 0; i < knnMatches.size(); i++) {
        const pair = knnMatches.get(i)
        if (pair.size() >= 2) {
          const m = pair.get(0)
          const n = pair.get(1)
          if (m.distance < this.config.ratioThreshold * n.distance) {
            goodMatches.push(m)
          }
        }
      }

      result.matches = goodMatches.length

      // Compute homography if enough matches
      if (goodMatches.length >= this.config.minMatches) {
        const homographyResult = this._computeHomography(
          goodMatches,
          frameKeypoints
        )
        if (homographyResult) {
          result = homographyResult
        }
      }

      knnMatches.delete()
    }

    // Cleanup
    src.delete()
    gray.delete()
    enhanced.delete()
    frameKeypoints.delete()
    frameDescriptors.delete()

    return result
  }

  _computeHomography(matches, frameKeypoints) {
    const cv = this.cv

    const srcPoints = []
    const dstPoints = []

    for (const match of matches) {
      const tkp = this.targetKeypoints.get(match.queryIdx)
      const fkp = frameKeypoints.get(match.trainIdx)
      srcPoints.push(tkp.pt.x, tkp.pt.y)
      dstPoints.push(fkp.pt.x, fkp.pt.y)
    }

    const srcMat = cv.matFromArray(matches.length, 1, cv.CV_32FC2, srcPoints)
    const dstMat = cv.matFromArray(matches.length, 1, cv.CV_32FC2, dstPoints)
    const inlierMask = new cv.Mat()

    const H = cv.findHomography(
      srcMat,
      dstMat,
      cv.RANSAC,
      this.config.ransacThreshold,
      inlierMask
    )

    // Count inliers
    let inliers = 0
    for (let i = 0; i < inlierMask.rows; i++) {
      if (inlierMask.data[i]) inliers++
    }

    srcMat.delete()
    dstMat.delete()
    inlierMask.delete()

    if (
      H.empty() ||
      inliers < this.config.minInliers ||
      !this._isValidHomography(H)
    ) {
      H.delete()
      return { matches: matches.length, inliers, corners: null }
    }

    // Get corners
    const corners = this._getCorners(H)
    H.delete()

    return { matches: matches.length, inliers, corners }
  }

  _isValidHomography(H) {
    const h = []
    for (let i = 0; i < 9; i++) h.push(H.data64F[i])

    const det = h[0] * h[4] - h[1] * h[3]
    if (det < 0.05 || det > 20) return false  // Wider range for distant detection

    const scale = Math.sqrt(det)
    return scale >= 0.05 && scale <= 8  // Allow smaller apparent size at distance
  }

  _getCorners(H) {
    const cv = this.cv
    const { width, height } = this.targetSize

    const corners = cv.matFromArray(4, 1, cv.CV_32FC2, [
      0,
      0,
      width,
      0,
      width,
      height,
      0,
      height,
    ])
    const transformed = new cv.Mat()
    cv.perspectiveTransform(corners, transformed, H)

    const points = []
    for (let i = 0; i < 4; i++) {
      points.push({
        x: transformed.data32F[i * 2],
        y: transformed.data32F[i * 2 + 1],
      })
    }

    corners.delete()
    transformed.delete()
    return points
  }

  dispose() {
    if (this.orb) this.orb.delete()
    if (this.matcher) this.matcher.delete()
    if (this.targetDescriptors) this.targetDescriptors.delete()
    if (this.targetKeypoints) this.targetKeypoints.delete()
    this.isReady = false
  }
}
