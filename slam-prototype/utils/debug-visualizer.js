// debug-visualizer.js - Visual Debugging Overlays for SLAM
//
// OPTIMIZATION NOTES:
// [OPT-1] Consider WebGL for hardware-accelerated overlay rendering
// [OPT-2] Implement object pooling for debug primitives
// [OPT-3] Use requestAnimationFrame batching for multiple overlays
// [OPT-4] Add level-of-detail for large point counts

class DebugVisualizer {
    constructor(canvas) {
        this.canvas = canvas
        this.ctx = canvas.getContext('2d')

        // ============================================================
        // CONFIGURATION
        // ============================================================
        this.config = {
            // Feature points
            trackedPointRadius: 4,
            trackedPointColor: '#00FF00',       // Green for tracked
            newPointColor: '#00BFFF',           // Blue for new
            lostPointColor: '#FF4444',          // Red for lost
            stablePointColor: '#FFD700',        // Gold for stable (high age)

            // Map points
            mapPointRadius: 6,
            mapPointColor: '#FF00FF',           // Magenta for map points

            // Trails
            showTrails: true,
            trailLength: 10,
            trailColor: 'rgba(0, 255, 0, 0.3)',

            // IDs
            showPointIds: false,
            idFontSize: 10,
            idColor: '#FFFFFF',

            // Quality visualization
            showQualityBars: false,
            qualityBarWidth: 3,
            qualityBarHeight: 15,

            // Pose visualization
            axesLength: 50,
            axesLineWidth: 3,

            // Grid
            showGrid: false,
            gridSpacing: 50,
            gridColor: 'rgba(255, 255, 255, 0.1)',

            // Statistics overlay
            showStats: true,
            statsBackgroundColor: 'rgba(0, 0, 0, 0.7)',
            statsFontColor: '#00FF00',
            statsFontSize: 14,
        }

        // ============================================================
        // STATE
        // ============================================================
        this.trails = new Map()  // pointId -> array of past positions
        this.lastStats = {}

        console.log('[DebugVisualizer] Initialized')
    }

    // ============================================================
    // CLEAR CANVAS
    // ============================================================
    clear() {
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height)
    }

    // ============================================================
    // DRAW TRACKED FEATURES
    // ============================================================
    // 
    // Visualizes feature tracking state with color-coded points.
    // 
    // @param {Object} trackingResult - Result from FeatureTracker
    // ============================================================
    drawTrackedFeatures(trackingResult) {
        if (!trackingResult || !trackingResult.points) return

        const ctx = this.ctx
        const cfg = this.config

        for (const point of trackingResult.points) {
            // Determine color based on age and quality
            let color = cfg.trackedPointColor
            let radius = cfg.trackedPointRadius

            if (point.age === 0) {
                color = cfg.newPointColor
            } else if (point.age > 10 && point.quality > 0.6) {
                color = cfg.stablePointColor
                radius = cfg.trackedPointRadius + 2
            }

            // Draw point
            ctx.beginPath()
            ctx.arc(point.x, point.y, radius, 0, Math.PI * 2)
            ctx.fillStyle = color
            ctx.fill()

            // Draw border
            ctx.strokeStyle = 'rgba(255, 255, 255, 0.5)'
            ctx.lineWidth = 1
            ctx.stroke()

            // Draw trail
            if (cfg.showTrails) {
                this._updateTrail(point.id, point.x, point.y)
                this._drawTrail(point.id)
            }

            // Draw ID
            if (cfg.showPointIds) {
                ctx.font = `${cfg.idFontSize}px monospace`
                ctx.fillStyle = cfg.idColor
                ctx.fillText(point.id, point.x + radius + 2, point.y + 3)
            }

            // Draw quality bar
            if (cfg.showQualityBars) {
                const barHeight = point.quality * cfg.qualityBarHeight
                ctx.fillStyle = this._qualityToColor(point.quality)
                ctx.fillRect(
                    point.x - cfg.qualityBarWidth / 2,
                    point.y - radius - 2 - barHeight,
                    cfg.qualityBarWidth,
                    barHeight
                )
            }
        }

        // Draw statistics
        if (cfg.showStats) {
            this._drawTrackingStats(trackingResult.stats)
        }
    }

    // ============================================================
    // DRAW MAP POINTS (PROJECTED)
    // ============================================================
    // 
    // Visualizes 3D map points projected to 2D.
    // 
    // @param {Array} projectedPoints - Array of {x, y, confidence, id}
    // ============================================================
    drawMapPoints(projectedPoints) {
        if (!projectedPoints) return

        const ctx = this.ctx
        const cfg = this.config

        for (const point of projectedPoints) {
            const alpha = 0.3 + point.confidence * 0.7

            // Draw outer ring
            ctx.beginPath()
            ctx.arc(point.x, point.y, cfg.mapPointRadius, 0, Math.PI * 2)
            ctx.strokeStyle = cfg.mapPointColor
            ctx.lineWidth = 2
            ctx.globalAlpha = alpha
            ctx.stroke()

            // Draw inner dot
            ctx.beginPath()
            ctx.arc(point.x, point.y, 2, 0, Math.PI * 2)
            ctx.fillStyle = cfg.mapPointColor
            ctx.fill()

            ctx.globalAlpha = 1
        }
    }

    // ============================================================
    // DRAW MARKER CORNERS
    // ============================================================
    // 
    // Visualizes detected marker corners and quadrilateral.
    // ============================================================
    drawMarkerCorners(corners, color = '#00FF00') {
        if (!corners || corners.length !== 4) return

        const ctx = this.ctx

        // Draw quadrilateral
        ctx.beginPath()
        ctx.moveTo(corners[0].x, corners[0].y)
        for (let i = 1; i < 4; i++) {
            ctx.lineTo(corners[i].x, corners[i].y)
        }
        ctx.closePath()
        ctx.strokeStyle = color
        ctx.lineWidth = 3
        ctx.stroke()

        // Fill with transparency
        ctx.fillStyle = `${color}33`
        ctx.fill()

        // Draw corner points with labels
        const labels = ['TL', 'TR', 'BR', 'BL']
        for (let i = 0; i < 4; i++) {
            ctx.beginPath()
            ctx.arc(corners[i].x, corners[i].y, 8, 0, Math.PI * 2)
            ctx.fillStyle = color
            ctx.fill()
            ctx.strokeStyle = '#FFFFFF'
            ctx.lineWidth = 2
            ctx.stroke()

            ctx.font = '12px monospace'
            ctx.fillStyle = '#FFFFFF'
            ctx.fillText(labels[i], corners[i].x + 10, corners[i].y + 4)
        }
    }

    // ============================================================
    // DRAW POSE AXES
    // ============================================================
    // 
    // Visualizes 3D pose as XYZ axes projected to 2D.
    // 
    // @param {Object} origin - 2D origin point {x, y}
    // @param {Object} axes - 2D axis endpoints {x: {x, y}, y: {x, y}, z: {x, y}}
    // ============================================================
    drawPoseAxes(origin, axes) {
        if (!origin || !axes) return

        const ctx = this.ctx

        // X axis (red)
        this._drawArrow(origin, axes.x, '#FF0000', 'X')

        // Y axis (green)
        this._drawArrow(origin, axes.y, '#00FF00', 'Y')

        // Z axis (blue)
        this._drawArrow(origin, axes.z, '#0000FF', 'Z')

        // Origin point
        ctx.beginPath()
        ctx.arc(origin.x, origin.y, 5, 0, Math.PI * 2)
        ctx.fillStyle = '#FFFFFF'
        ctx.fill()
    }

    // ============================================================
    // DRAW REPROJECTION ERRORS
    // ============================================================
    // 
    // Visualizes reprojection errors as lines from observed to projected.
    // ============================================================
    drawReprojectionErrors(observed, projected) {
        if (!observed || !projected || observed.length !== projected.length) return

        const ctx = this.ctx

        for (let i = 0; i < observed.length; i++) {
            const o = observed[i]
            const p = projected[i]

            // Error line
            ctx.beginPath()
            ctx.moveTo(o.x, o.y)
            ctx.lineTo(p.x, p.y)

            // Color based on error magnitude
            const error = Math.sqrt((o.x - p.x) ** 2 + (o.y - p.y) ** 2)
            if (error < 1) {
                ctx.strokeStyle = '#00FF00'
            } else if (error < 3) {
                ctx.strokeStyle = '#FFFF00'
            } else {
                ctx.strokeStyle = '#FF0000'
            }

            ctx.lineWidth = 1
            ctx.stroke()

            // Projected point
            ctx.beginPath()
            ctx.arc(p.x, p.y, 3, 0, Math.PI * 2)
            ctx.fillStyle = '#FF00FF'
            ctx.fill()
        }
    }

    // ============================================================
    // DRAW STATISTICS OVERLAY
    // ============================================================
    drawStats(stats, x = 10, y = 10) {
        const ctx = this.ctx
        const cfg = this.config

        // Merge with last stats for persistence
        this.lastStats = { ...this.lastStats, ...stats }

        const lines = []
        for (const [key, value] of Object.entries(this.lastStats)) {
            let displayValue = value
            if (typeof value === 'number') {
                displayValue = Number.isInteger(value) ? value : value.toFixed(3)
            }
            lines.push(`${key}: ${displayValue}`)
        }

        const lineHeight = cfg.statsFontSize + 4
        const padding = 8
        const width = 220
        const height = lines.length * lineHeight + padding * 2

        // Background
        ctx.fillStyle = cfg.statsBackgroundColor
        ctx.fillRect(x, y, width, height)

        // Border
        ctx.strokeStyle = cfg.statsFontColor
        ctx.lineWidth = 1
        ctx.strokeRect(x, y, width, height)

        // Text
        ctx.font = `${cfg.statsFontSize}px monospace`
        ctx.fillStyle = cfg.statsFontColor

        for (let i = 0; i < lines.length; i++) {
            ctx.fillText(lines[i], x + padding, y + padding + (i + 1) * lineHeight - 4)
        }
    }

    // ============================================================
    // DRAW GRID
    // ============================================================
    drawGrid() {
        const ctx = this.ctx
        const cfg = this.config

        ctx.strokeStyle = cfg.gridColor
        ctx.lineWidth = 1

        // Vertical lines
        for (let x = 0; x < this.canvas.width; x += cfg.gridSpacing) {
            ctx.beginPath()
            ctx.moveTo(x, 0)
            ctx.lineTo(x, this.canvas.height)
            ctx.stroke()
        }

        // Horizontal lines
        for (let y = 0; y < this.canvas.height; y += cfg.gridSpacing) {
            ctx.beginPath()
            ctx.moveTo(0, y)
            ctx.lineTo(this.canvas.width, y)
            ctx.stroke()
        }
    }

    // ============================================================
    // DRAW CROSSHAIR
    // ============================================================
    drawCrosshair(x, y, size = 20, color = '#00FF00') {
        const ctx = this.ctx

        ctx.strokeStyle = color
        ctx.lineWidth = 2

        // Horizontal
        ctx.beginPath()
        ctx.moveTo(x - size, y)
        ctx.lineTo(x + size, y)
        ctx.stroke()

        // Vertical
        ctx.beginPath()
        ctx.moveTo(x, y - size)
        ctx.lineTo(x, y + size)
        ctx.stroke()
    }

    // ============================================================
    // DRAW STATUS MESSAGE
    // ============================================================
    drawStatus(message, type = 'info') {
        const ctx = this.ctx
        const colors = {
            info: '#00BFFF',
            success: '#00FF00',
            warning: '#FFD700',
            error: '#FF4444',
        }

        const y = this.canvas.height - 40
        const padding = 10

        ctx.font = '16px monospace'
        const width = ctx.measureText(message).width + padding * 2

        // Background
        ctx.fillStyle = 'rgba(0, 0, 0, 0.8)'
        ctx.fillRect(this.canvas.width / 2 - width / 2, y, width, 30)

        // Text
        ctx.fillStyle = colors[type] || colors.info
        ctx.textAlign = 'center'
        ctx.fillText(message, this.canvas.width / 2, y + 20)
        ctx.textAlign = 'left'
    }

    // ============================================================
    // DRAW HISTOGRAM
    // ============================================================
    drawHistogram(data, x, y, width, height, color = '#00FF00') {
        if (!data || data.length === 0) return

        const ctx = this.ctx
        const max = Math.max(...data)
        const barWidth = width / data.length

        // Background
        ctx.fillStyle = 'rgba(0, 0, 0, 0.5)'
        ctx.fillRect(x, y, width, height)

        // Bars
        ctx.fillStyle = color
        for (let i = 0; i < data.length; i++) {
            const barHeight = (data[i] / max) * height
            ctx.fillRect(
                x + i * barWidth,
                y + height - barHeight,
                barWidth - 1,
                barHeight
            )
        }
    }

    // ============================================================
    // HELPER: DRAW ARROW
    // ============================================================
    _drawArrow(from, to, color, label = '') {
        const ctx = this.ctx
        const cfg = this.config

        const dx = to.x - from.x
        const dy = to.y - from.y
        const angle = Math.atan2(dy, dx)
        const length = Math.sqrt(dx * dx + dy * dy)

        // Line
        ctx.beginPath()
        ctx.moveTo(from.x, from.y)
        ctx.lineTo(to.x, to.y)
        ctx.strokeStyle = color
        ctx.lineWidth = cfg.axesLineWidth
        ctx.stroke()

        // Arrowhead
        const headLength = 10
        ctx.beginPath()
        ctx.moveTo(to.x, to.y)
        ctx.lineTo(
            to.x - headLength * Math.cos(angle - Math.PI / 6),
            to.y - headLength * Math.sin(angle - Math.PI / 6)
        )
        ctx.lineTo(
            to.x - headLength * Math.cos(angle + Math.PI / 6),
            to.y - headLength * Math.sin(angle + Math.PI / 6)
        )
        ctx.closePath()
        ctx.fillStyle = color
        ctx.fill()

        // Label
        if (label) {
            ctx.font = '12px monospace'
            ctx.fillStyle = color
            ctx.fillText(label, to.x + 5, to.y - 5)
        }
    }

    // ============================================================
    // HELPER: UPDATE TRAIL
    // ============================================================
    _updateTrail(pointId, x, y) {
        if (!this.trails.has(pointId)) {
            this.trails.set(pointId, [])
        }

        const trail = this.trails.get(pointId)
        trail.push({ x, y })

        if (trail.length > this.config.trailLength) {
            trail.shift()
        }
    }

    // ============================================================
    // HELPER: DRAW TRAIL
    // ============================================================
    _drawTrail(pointId) {
        const trail = this.trails.get(pointId)
        if (!trail || trail.length < 2) return

        const ctx = this.ctx

        ctx.beginPath()
        ctx.moveTo(trail[0].x, trail[0].y)
        for (let i = 1; i < trail.length; i++) {
            ctx.lineTo(trail[i].x, trail[i].y)
        }
        ctx.strokeStyle = this.config.trailColor
        ctx.lineWidth = 2
        ctx.stroke()
    }

    // ============================================================
    // HELPER: QUALITY TO COLOR
    // ============================================================
    _qualityToColor(quality) {
        // Red -> Yellow -> Green gradient
        const r = Math.round(255 * (1 - quality))
        const g = Math.round(255 * quality)
        return `rgb(${r}, ${g}, 0)`
    }

    // ============================================================
    // HELPER: DRAW TRACKING STATS
    // ============================================================
    _drawTrackingStats(stats) {
        if (!stats) return

        this.drawStats({
            'Tracked': stats.trackedCount || 0,
            'New': stats.newCount || 0,
            'Lost': stats.lostCount || 0,
            'Avg Quality': (stats.avgQuality || 0).toFixed(2),
            'Avg Age': (stats.avgAge || 0).toFixed(1),
            'Time (ms)': (stats.processingTimeMs || 0).toFixed(1),
        })
    }

    // ============================================================
    // CLEAR TRAILS
    // ============================================================
    clearTrails() {
        this.trails.clear()
    }

    // ============================================================
    // CLEANUP
    // ============================================================
    dispose() {
        this.clearTrails()
        this.clear()
        console.log('[DebugVisualizer] Disposed')
    }
}
