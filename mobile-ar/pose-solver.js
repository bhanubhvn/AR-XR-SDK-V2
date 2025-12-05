// pose-solver.js - 6DoF Pose Estimation using OpenCV solvePnP

const PoseSolver = {
  cv: null,
  cameraMatrix: null,
  distCoeffs: null,
  imageWidth: 0,
  imageHeight: 0,

  // Physical marker size in meters (MUST match your printed marker)
  markerSize: 0.15, // 15cm default

  init(cv, imageWidth, imageHeight) {
    this.cv = cv
    this.imageWidth = imageWidth
    this.imageHeight = imageHeight

    // Camera intrinsics (approximation for mobile cameras)
    // Focal length ≈ image width for typical mobile FOV (~60°)
    const focalLength = imageWidth
    const cx = imageWidth / 2
    const cy = imageHeight / 2

    this.cameraMatrix = cv.matFromArray(3, 3, cv.CV_64F, [
      focalLength,
      0,
      cx,
      0,
      focalLength,
      cy,
      0,
      0,
      1,
    ])

    // No lens distortion (mobile cameras are usually pre-corrected)
    this.distCoeffs = cv.matFromArray(4, 1, cv.CV_64F, [0, 0, 0, 0])

    console.log(
      `[PoseSolver] Initialized: ${imageWidth}x${imageHeight}, marker=${this.markerSize}m`
    )
  },

  setMarkerSize(meters) {
    this.markerSize = meters
  },

  /**
   * Solve 6DoF pose from 4 corner points
   * @param {Array} corners - 4 corners in image coords [TL, TR, BR, BL]
   * @returns {Object|null} - { rotation: [9], translation: [3], matrix: THREE.Matrix4 }
   */
  solve(corners) {
    if (!this.cv || !corners || corners.length !== 4) return null

    const cv = this.cv
    const half = this.markerSize / 2

    // 3D object points: marker centered at origin, lying on XY plane (Z=0)
    // Order matches corner order: TL, TR, BR, BL
    const objectPoints = cv.matFromArray(4, 1, cv.CV_64FC3, [
      -half,
      half,
      0, // TL
      half,
      half,
      0, // TR
      half,
      -half,
      0, // BR
      -half,
      -half,
      0, // BL
    ])

    // 2D image points from detection
    const imagePoints = cv.matFromArray(4, 1, cv.CV_64FC2, [
      corners[0].x,
      corners[0].y,
      corners[1].x,
      corners[1].y,
      corners[2].x,
      corners[2].y,
      corners[3].x,
      corners[3].y,
    ])

    const rvec = new cv.Mat()
    const tvec = new cv.Mat()

    // Solve PnP using ITERATIVE method (more stable than IPPE for planar targets)
    const success = cv.solvePnP(
      objectPoints,
      imagePoints,
      this.cameraMatrix,
      this.distCoeffs,
      rvec,
      tvec,
      false,
      cv.SOLVEPNP_ITERATIVE
    )

    if (!success) {
      objectPoints.delete()
      imagePoints.delete()
      rvec.delete()
      tvec.delete()
      return null
    }

    // Convert rotation vector to rotation matrix
    const rotMat = new cv.Mat()
    cv.Rodrigues(rvec, rotMat)

    // Extract values
    const R = []
    for (let i = 0; i < 9; i++) R.push(rotMat.data64F[i])

    const t = [tvec.data64F[0], tvec.data64F[1], tvec.data64F[2]]

    // Build Three.js matrix with coordinate conversion
    const matrix = this._buildMatrix(R, t)

    // Cleanup
    objectPoints.delete()
    imagePoints.delete()
    rvec.delete()
    tvec.delete()
    rotMat.delete()

    return { rotation: R, translation: t, matrix }
  },

  /**
   * Build Three.js Matrix4 from OpenCV rotation and translation
   * Handles coordinate system conversion:
   * - OpenCV: X-right, Y-down, Z-forward
   * - Three.js: X-right, Y-up, Z-backward
   */
  _buildMatrix(R, t) {
    // OpenCV to Three.js coordinate flip
    // Flip Y and Z: multiply R and t by diag(1, -1, -1)
    const matrix = new THREE.Matrix4()

    // Apply coordinate conversion directly in matrix construction
    // Row 0: R[0], R[1], R[2] → R[0], -R[1], -R[2]
    // Row 1: R[3], R[4], R[5] → -R[3], R[4], R[5]
    // Row 2: R[6], R[7], R[8] → -R[6], R[7], R[8]

    matrix.set(
      R[0],
      -R[1],
      -R[2],
      t[0],
      -R[3],
      R[4],
      R[5],
      -t[1],
      -R[6],
      R[7],
      R[8],
      -t[2],
      0,
      0,
      0,
      1
    )

    return matrix
  },

  /**
   * Get projection matrix for Three.js camera
   * Matches OpenCV camera intrinsics
   */
  getProjectionMatrix(near, far) {
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
  },

  dispose() {
    if (this.cameraMatrix) this.cameraMatrix.delete()
    if (this.distCoeffs) this.distCoeffs.delete()
    this.cv = null
  },
}
