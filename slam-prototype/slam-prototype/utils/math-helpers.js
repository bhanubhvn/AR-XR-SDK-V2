// math-helpers.js - Mathematical Utilities for SLAM
//
// OPTIMIZATION NOTES:
// [OPT-1] Consider SIMD.js for vectorized operations
// [OPT-2] Use typed arrays for all computations
// [OPT-3] Implement object pooling to reduce GC pressure
// [OPT-4] Consider WebAssembly for intensive math operations

const MathHelpers = {

    // ============================================================
    // MATRIX OPERATIONS
    // ============================================================

    /**
     * Multiply two 4x4 matrices (column-major)
     */
    mat4Multiply(a, b) {
        const result = new Float64Array(16)

        for (let i = 0; i < 4; i++) {
            for (let j = 0; j < 4; j++) {
                let sum = 0
                for (let k = 0; k < 4; k++) {
                    sum += a[k * 4 + j] * b[i * 4 + k]
                }
                result[i * 4 + j] = sum
            }
        }

        return result
    },

    /**
     * Invert a 4x4 matrix
     */
    mat4Inverse(m) {
        const result = new Float64Array(16)

        const n11 = m[0], n21 = m[1], n31 = m[2], n41 = m[3]
        const n12 = m[4], n22 = m[5], n32 = m[6], n42 = m[7]
        const n13 = m[8], n23 = m[9], n33 = m[10], n43 = m[11]
        const n14 = m[12], n24 = m[13], n34 = m[14], n44 = m[15]

        const t11 = n23 * n34 * n42 - n24 * n33 * n42 + n24 * n32 * n43 - n22 * n34 * n43 - n23 * n32 * n44 + n22 * n33 * n44
        const t12 = n14 * n33 * n42 - n13 * n34 * n42 - n14 * n32 * n43 + n12 * n34 * n43 + n13 * n32 * n44 - n12 * n33 * n44
        const t13 = n13 * n24 * n42 - n14 * n23 * n42 + n14 * n22 * n43 - n12 * n24 * n43 - n13 * n22 * n44 + n12 * n23 * n44
        const t14 = n14 * n23 * n32 - n13 * n24 * n32 - n14 * n22 * n33 + n12 * n24 * n33 + n13 * n22 * n34 - n12 * n23 * n34

        const det = n11 * t11 + n21 * t12 + n31 * t13 + n41 * t14

        if (Math.abs(det) < 1e-10) {
            console.warn('[MathHelpers] Singular matrix, cannot invert')
            return null
        }

        const invDet = 1 / det

        result[0] = t11 * invDet
        result[1] = (n24 * n33 * n41 - n23 * n34 * n41 - n24 * n31 * n43 + n21 * n34 * n43 + n23 * n31 * n44 - n21 * n33 * n44) * invDet
        result[2] = (n22 * n34 * n41 - n24 * n32 * n41 + n24 * n31 * n42 - n21 * n34 * n42 - n22 * n31 * n44 + n21 * n32 * n44) * invDet
        result[3] = (n23 * n32 * n41 - n22 * n33 * n41 - n23 * n31 * n42 + n21 * n33 * n42 + n22 * n31 * n43 - n21 * n32 * n43) * invDet
        result[4] = t12 * invDet
        result[5] = (n13 * n34 * n41 - n14 * n33 * n41 + n14 * n31 * n43 - n11 * n34 * n43 - n13 * n31 * n44 + n11 * n33 * n44) * invDet
        result[6] = (n14 * n32 * n41 - n12 * n34 * n41 - n14 * n31 * n42 + n11 * n34 * n42 + n12 * n31 * n44 - n11 * n32 * n44) * invDet
        result[7] = (n12 * n33 * n41 - n13 * n32 * n41 + n13 * n31 * n42 - n11 * n33 * n42 - n12 * n31 * n43 + n11 * n32 * n43) * invDet
        result[8] = t13 * invDet
        result[9] = (n14 * n23 * n41 - n13 * n24 * n41 - n14 * n21 * n43 + n11 * n24 * n43 + n13 * n21 * n44 - n11 * n23 * n44) * invDet
        result[10] = (n12 * n24 * n41 - n14 * n22 * n41 + n14 * n21 * n42 - n11 * n24 * n42 - n12 * n21 * n44 + n11 * n22 * n44) * invDet
        result[11] = (n13 * n22 * n41 - n12 * n23 * n41 - n13 * n21 * n42 + n11 * n23 * n42 + n12 * n21 * n43 - n11 * n22 * n43) * invDet
        result[12] = t14 * invDet
        result[13] = (n13 * n24 * n31 - n14 * n23 * n31 + n14 * n21 * n33 - n11 * n24 * n33 - n13 * n21 * n34 + n11 * n23 * n34) * invDet
        result[14] = (n14 * n22 * n31 - n12 * n24 * n31 - n14 * n21 * n32 + n11 * n24 * n32 + n12 * n21 * n34 - n11 * n22 * n34) * invDet
        result[15] = (n12 * n23 * n31 - n13 * n22 * n31 + n13 * n21 * n32 - n11 * n23 * n32 - n12 * n21 * n33 + n11 * n22 * n33) * invDet

        return result
    },

    /**
     * Create identity matrix
     */
    mat4Identity() {
        return new Float64Array([
            1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1,
        ])
    },

    /**
     * Transform a 3D point by a 4x4 matrix
     */
    transformPoint(matrix, point) {
        const x = point[0]
        const y = point[1]
        const z = point[2]

        const w = matrix[3] * x + matrix[7] * y + matrix[11] * z + matrix[15]

        return [
            (matrix[0] * x + matrix[4] * y + matrix[8] * z + matrix[12]) / w,
            (matrix[1] * x + matrix[5] * y + matrix[9] * z + matrix[13]) / w,
            (matrix[2] * x + matrix[6] * y + matrix[10] * z + matrix[14]) / w,
        ]
    },

    // ============================================================
    // QUATERNION OPERATIONS
    // ============================================================

    /**
     * Multiply two quaternions (w, x, y, z format)
     */
    quatMultiply(a, b) {
        return [
            a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
        ]
    },

    /**
     * Quaternion conjugate
     */
    quatConjugate(q) {
        return [q[0], -q[1], -q[2], -q[3]]
    },

    /**
     * Normalize quaternion
     */
    quatNormalize(q) {
        const len = Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
        if (len < 1e-10) return [1, 0, 0, 0]
        return [q[0] / len, q[1] / len, q[2] / len, q[3] / len]
    },

    /**
     * Spherical linear interpolation between quaternions
     */
    quatSlerp(a, b, t) {
        // Compute dot product
        let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]

        // If dot is negative, negate one quaternion to take shorter path
        if (dot < 0) {
            b = [-b[0], -b[1], -b[2], -b[3]]
            dot = -dot
        }

        // Clamp dot
        dot = Math.min(dot, 1)

        // Calculate interpolation parameters
        const theta = Math.acos(dot)
        const sinTheta = Math.sin(theta)

        let s0, s1
        if (sinTheta > 1e-6) {
            s0 = Math.sin((1 - t) * theta) / sinTheta
            s1 = Math.sin(t * theta) / sinTheta
        } else {
            // Linear interpolation for small angles
            s0 = 1 - t
            s1 = t
        }

        return this.quatNormalize([
            s0 * a[0] + s1 * b[0],
            s0 * a[1] + s1 * b[1],
            s0 * a[2] + s1 * b[2],
            s0 * a[3] + s1 * b[3],
        ])
    },

    /**
     * Quaternion from rotation matrix
     */
    quatFromMatrix(m) {
        const trace = m[0] + m[5] + m[10]
        let w, x, y, z

        if (trace > 0) {
            const s = 0.5 / Math.sqrt(trace + 1)
            w = 0.25 / s
            x = (m[6] - m[9]) * s
            y = (m[8] - m[2]) * s
            z = (m[1] - m[4]) * s
        } else if (m[0] > m[5] && m[0] > m[10]) {
            const s = 2 * Math.sqrt(1 + m[0] - m[5] - m[10])
            w = (m[6] - m[9]) / s
            x = 0.25 * s
            y = (m[4] + m[1]) / s
            z = (m[8] + m[2]) / s
        } else if (m[5] > m[10]) {
            const s = 2 * Math.sqrt(1 + m[5] - m[0] - m[10])
            w = (m[8] - m[2]) / s
            x = (m[4] + m[1]) / s
            y = 0.25 * s
            z = (m[9] + m[6]) / s
        } else {
            const s = 2 * Math.sqrt(1 + m[10] - m[0] - m[5])
            w = (m[1] - m[4]) / s
            x = (m[8] + m[2]) / s
            y = (m[9] + m[6]) / s
            z = 0.25 * s
        }

        return this.quatNormalize([w, x, y, z])
    },

    /**
     * Rotation matrix from quaternion
     */
    matrixFromQuat(q) {
        const x = q[1], y = q[2], z = q[3], w = q[0]
        const x2 = x + x, y2 = y + y, z2 = z + z
        const xx = x * x2, xy = x * y2, xz = x * z2
        const yy = y * y2, yz = y * z2, zz = z * z2
        const wx = w * x2, wy = w * y2, wz = w * z2

        return new Float64Array([
            1 - (yy + zz), xy + wz, xz - wy, 0,
            xy - wz, 1 - (xx + zz), yz + wx, 0,
            xz + wy, yz - wx, 1 - (xx + yy), 0,
            0, 0, 0, 1,
        ])
    },

    // ============================================================
    // VECTOR OPERATIONS
    // ============================================================

    /**
     * Vector dot product
     */
    dot(a, b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
    },

    /**
     * Vector cross product
     */
    cross(a, b) {
        return [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    },

    /**
     * Vector length
     */
    length(v) {
        return Math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    },

    /**
     * Normalize vector
     */
    normalize(v) {
        const len = this.length(v)
        if (len < 1e-10) return [0, 0, 0]
        return [v[0] / len, v[1] / len, v[2] / len]
    },

    /**
     * Vector subtraction
     */
    subtract(a, b) {
        return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
    },

    /**
     * Vector addition
     */
    add(a, b) {
        return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
    },

    /**
     * Scalar multiply
     */
    scale(v, s) {
        return [v[0] * s, v[1] * s, v[2] * s]
    },

    /**
     * Distance between two points
     */
    distance(a, b) {
        const dx = b[0] - a[0]
        const dy = b[1] - a[1]
        const dz = b[2] - a[2]
        return Math.sqrt(dx * dx + dy * dy + dz * dz)
    },

    // ============================================================
    // ROTATION CONVERSIONS
    // ============================================================

    /**
     * Euler angles (radians) to rotation matrix
     * Order: ZYX (yaw, pitch, roll)
     */
    eulerToMatrix(euler) {
        const [rx, ry, rz] = euler
        const cx = Math.cos(rx), sx = Math.sin(rx)
        const cy = Math.cos(ry), sy = Math.sin(ry)
        const cz = Math.cos(rz), sz = Math.sin(rz)

        return new Float64Array([
            cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz, 0,
            cy * sz, cx * cz + sx * sy * sz, cx * sy * sz - cz * sx, 0,
            -sy, cy * sx, cx * cy, 0,
            0, 0, 0, 1,
        ])
    },

    /**
     * Rotation matrix to Euler angles (radians)
     * Order: ZYX
     */
    matrixToEuler(m) {
        const sy = -m[2]

        if (Math.abs(sy) < 0.9999) {
            const ry = Math.asin(sy)
            const rx = Math.atan2(m[6] / Math.cos(ry), m[10] / Math.cos(ry))
            const rz = Math.atan2(m[1] / Math.cos(ry), m[0] / Math.cos(ry))
            return [rx, ry, rz]
        } else {
            // Gimbal lock
            const rz = 0
            if (sy > 0) {
                const ry = Math.PI / 2
                const rx = Math.atan2(m[4], m[5])
                return [rx, ry, rz]
            } else {
                const ry = -Math.PI / 2
                const rx = Math.atan2(-m[4], -m[5])
                return [rx, ry, rz]
            }
        }
    },

    /**
     * Axis-angle to rotation matrix
     */
    axisAngleToMatrix(axis, angle) {
        const c = Math.cos(angle)
        const s = Math.sin(angle)
        const t = 1 - c
        const x = axis[0], y = axis[1], z = axis[2]

        return new Float64Array([
            t * x * x + c, t * x * y + s * z, t * x * z - s * y, 0,
            t * x * y - s * z, t * y * y + c, t * y * z + s * x, 0,
            t * x * z + s * y, t * y * z - s * x, t * z * z + c, 0,
            0, 0, 0, 1,
        ])
    },

    // ============================================================
    // STATISTICS
    // ============================================================

    /**
     * Calculate mean of array
     */
    mean(arr) {
        if (arr.length === 0) return 0
        return arr.reduce((a, b) => a + b, 0) / arr.length
    },

    /**
     * Calculate standard deviation
     */
    stdDev(arr) {
        if (arr.length < 2) return 0
        const m = this.mean(arr)
        const variance = arr.reduce((sum, x) => sum + (x - m) ** 2, 0) / arr.length
        return Math.sqrt(variance)
    },

    /**
     * Calculate median
     */
    median(arr) {
        if (arr.length === 0) return 0
        const sorted = [...arr].sort((a, b) => a - b)
        const mid = Math.floor(sorted.length / 2)
        return sorted.length % 2 !== 0
            ? sorted[mid]
            : (sorted[mid - 1] + sorted[mid]) / 2
    },

    // ============================================================
    // GEOMETRY
    // ============================================================

    /**
     * Check if point is inside frustum
     */
    isInFrustum(point, cameraPose, fov, aspect, near, far) {
        // Transform to camera space
        const invPose = this.mat4Inverse(cameraPose)
        if (!invPose) return false

        const local = this.transformPoint(invPose, point)

        // Check near/far
        if (local[2] > -near || local[2] < -far) return false

        // Check frustum sides
        const halfFovRad = fov * Math.PI / 360
        const tanHalfFov = Math.tan(halfFovRad)
        const yBound = -local[2] * tanHalfFov
        const xBound = yBound * aspect

        return Math.abs(local[0]) <= xBound && Math.abs(local[1]) <= yBound
    },

    /**
     * Compute angle between two vectors (radians)
     */
    angleBetween(a, b) {
        const dot = this.dot(this.normalize(a), this.normalize(b))
        return Math.acos(Math.max(-1, Math.min(1, dot)))
    },

    // ============================================================
    // DEGREES/RADIANS
    // ============================================================

    degToRad(deg) {
        return deg * Math.PI / 180
    },

    radToDeg(rad) {
        return rad * 180 / Math.PI
    },

    // ============================================================
    // CLAMPING
    // ============================================================

    clamp(value, min, max) {
        return Math.max(min, Math.min(max, value))
    },

    lerp(a, b, t) {
        return a + (b - a) * t
    },
}
