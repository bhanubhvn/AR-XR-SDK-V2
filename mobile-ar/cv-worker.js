// cv-worker.js
// Handles OpenCV processing in a separate thread

// Import OpenCV and Three.js via local proxy to satisfy COEP headers
importScripts('/proxy?url=' + encodeURIComponent('https://docs.opencv.org/4.10.0/opencv.js'))
importScripts('/proxy?url=' + encodeURIComponent('https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js'))

// Import Engine Logic
importScripts('ar-engine.js')
importScripts('pose-solver.js')


let cvReady = false
let engine = null
let sharedBuffer = null
let sharedView = null
let width = 0
let height = 0

// Wait for OpenCV to initialize
cv.onRuntimeInitialized = () => {
    cvReady = true
    console.log('[Worker] OpenCV initialized')
    postMessage({ type: 'CV_READY' })
}

self.onmessage = function (e) {
    const { type } = e.data
    // Use the full message as data payload since App.js sends flat objects
    const data = e.data

    if (!cvReady && type !== 'INIT') return

    try {
        switch (type) {
            case 'INIT':
                handleInit()
                break

            case 'SETUP_SHARED':
                handleSetupShared(data)
                break

            case 'LOAD_TARGET':
                handleLoadTarget(data)
                break

            case 'RESIZE':
                handleResize(data)
                break

            case 'PROCESS':
                handleProcess(data)
                break
        }
    } catch (err) {
        console.error('[Worker] Error:', err)
    }
}

function handleInit() {
    engine = new AREngine()
    engine.init(cv)
    console.log('[Worker] Engine initialized')
}

function handleSetupShared(data) {
    sharedBuffer = data.buffer
    width = data.width
    height = data.height
    sharedView = new Uint8ClampedArray(sharedBuffer)
    console.log(`[Worker] Shared buffer setup: ${width}x${height}`)
}

function handleLoadTarget(data) {
    const { imageData } = data
    // data.imageData is an object { width, height, data: Uint8ClampedArray } from main thread
    // We reconstruct the argument for engine.processTarget
    const res = engine.processTarget(imageData)
    postMessage({ type: 'TARGET_LOADED', result: res })
}

function handleResize(data) {
    width = data.width
    height = data.height
    PoseSolver.init(cv, width, height)

    // Send back projection matrix
    const projMatrix = PoseSolver.getProjectionMatrix(0.01, 100)
    postMessage({ type: 'PROJECTION_MATRIX', matrix: projMatrix })
}

function handleProcess(data) {
    if (!engine || !sharedView) return

    // 1. Get image data from shared buffer or payload
    // If not using shared buffer yet, handle data.imageData
    let inputData
    if (data.useShared && sharedView) {
        inputData = {
            data: sharedView,
            width: width,
            height: height
        }
    } else if (data.imageData) {
        inputData = data.imageData
    } else {
        return
    }

    // 2. Detect Features
    const result = engine.processFrame(inputData)

    if (!result) {
        postMessage({ type: 'POSE', result: null, timestamp: data.timestamp })
        return
    }

    // 3. Solve Pose if corners found
    let pose = null
    if (result.corners) {
        pose = PoseSolver.solve(result.corners)

        // Flatten matrix to array for transfer
        if (pose && pose.matrix) {
            pose.matrix = pose.matrix.elements
            // Add z-depth for optical flow scaling (Nano-SLAM prep)
            pose.z = pose.translation[2];
        }
    }

    // 4. Send back result
    postMessage({
        type: 'POSE',
        result: pose,
        corners: result.corners, // For debugging/smoothing
        matches: result.matches,
        timestamp: data.timestamp
    })
}
