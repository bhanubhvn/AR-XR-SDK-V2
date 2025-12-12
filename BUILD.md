# AR SDK - Technical Architecture & Build Guide

A balanced high-performance marker-based AR SDK for mobile browsers, leveraging **Dual-Thread Architecture** for 60fps rendering even during heavy computer vision tasks.

---

## Technology Stack

| Component | Technology | Role |
| :--- | :--- | :--- |
| **CV Engine** | OpenCV.js (WebAssembly) | Feature Detection (ORB) & Matching (BFMatcher) |
| **Pose Solver** | OpenCV solvePnP | 6DoF Pose Estimation (IPPE_SQUARE) |
| **3D Engine** | Three.js | WebGL Rendering & Scenegraph |
| **Threading** | Web Workers + SharedArrayBuffer | Parallel Processing & Zero-Copy Transfer |
| **Server** | Node Express + Proxy | Secure Headers (COOP/COEP) |

---

## Complete File Structure

```
AR-XR-SDK-V2/
├── server.js                   # Express Server with Proxy & Security Headers
├── landing.html                # Entry portal
├── package.json                # Dependencies
├── BUILD.md                    # This file
│
└── mobile-ar/                  # AR Application
    ├── index.html              # Viewer entry point
    ├── app.js                  # Main Thread Controller (UI, Camera, Rendering)
    ├── cv-worker.js            # Worker Thread (OpenCV Logic)
    ├── ar-engine.js            # CV Detection Class (Shared Logic)
    ├── pose-solver.js          # Math Solver (Shared Logic)
    ├── renderer.js             # Three.js Wrapper
    ├── style.css               # Styling
    │
    └── assets/                 # Models & Targets
```

---

## Dual-Thread Architecture

The system is decoupled into two parallel threads to ensure the UI and Camera never freeze.

### 1. Main Thread (`app.js`)
- **Responsibility**: "The Presenter"
- Captures Camera Frames (30fps).
- Renders 3D Scene (60fps).
- **Dead Reckoning**: Since CV takes ~50ms, the Main Thread *predicts* the current marker position based on the last known velocity, ensuring smooth motion between worker updates.
- **Shared Memory**: Writes raw video pixels to a `SharedArrayBuffer` so the Worker can read them instantly without copying data.

### 2. Worker Thread (`cv-worker.js`)
- **Responsibility**: "The Calculator"
- Runs entirely separate from the UI.
- Loads OpenCV.js via a local Proxy (to satisfy security headers).
- **Process**:
  1. Reads frame from `SharedArrayBuffer`.
  2. Runs `AREngine` (ORB Detection).
  3. Runs `PoseSolver` (solvePnP).
  4. Posts 6DoF Matrix back to Main.

---

## Application Flow

```
┌─────────────────────────────────────────────────────────────┐
│                  INITIALIZATION SEQUENCE                    │
├─────────────────────────────────────────────────────────────┤
│ 1. App.init()                                               │
│    ├── Spawns cv-worker.js                                  │
│    │   └── Worker loads OpenCV via /proxy                   │
│    ├── Starts Camera (1280x720)                             │
│    ├── Inits Three.js Renderer & Loads Model                │
│    ├── Loads Target Image -> Sends to Worker                │
│    └── Sets up SharedArrayBuffer (480px) -> Sends to Worker │
└──────────────────────────────┬──────────────────────────────┘
                               │
                               ▼
┌─────────────────────────────────────────────────────────────┐
│                      RUNTIME LOOP                           │
├──────────────────────────────┬──────────────────────────────┤
│        MAIN THREAD           │        WORKER THREAD         │
│         (@60fps)             │          (@20-30fps)         │
├──────────────────────────────┼──────────────────────────────┤
│ 1. Draw Video to Canvas      │                              │
│ 2. Predict Pose (Velocity)   │ 1. Receive 'PROCESS' signal  │
│ 3. Buffer Video -> SAB       │ 2. Read SAB (Zero-Copy)      │
│ 4. Signal Worker ('PROCESS') │ 3. ORB Feature Match         │
│ 5. Render 3D Scene           │ 4. Solve PnP (6DoF)          │
│                              │ 5. Post {Matrix, Timestamp}  │
│ <---- Receives Update <------┘                              │
│ 6. Update Velocity & Pose    │                              │
└─────────────────────────────────────────────────────────────┘
```

---

## Server Configuration (`server.js`)

Modern web features utilize strict security isolation. The server uses:

1.  **COOP/COEP Headers**:
    *   `Cross-Origin-Opener-Policy: same-origin`
    *   `Cross-Origin-Embedder-Policy: require-corp`
    *   *Why?* Required to enable `SharedArrayBuffer` security standard.

2.  **Proxy Endpoint** (`/proxy?url=...`):
    *   Fetches external scripts (docs.opencv.org, cdnjs).
    *   Injects `Cross-Origin-Resource-Policy: cross-origin`.
    *   *Why?* External CDNs often lack CORP headers, which causes them to be blocked by the browser when COEP is enabled.

---

## Key Algorithms

### 1. ORB Feature Matching
*   **Detector**: ORB (Oriented FAST and Rotated BRIEF). Fast and rotation invariant.
*   **Matching**: Hamming Distance with Lowe's Ratio Test (0.85).
*   **Verification**: RANSAC Homography check to reject false positives.

### 2. Dead Reckoning (Smoothing)
To hide the 30-50ms latency of the Computer Vision:
$$ P_{render} = P_{last\_vision} + (Velocity \times \Delta t) $$
The Main Thread extrapolates the marker's position every frame.

### 3. Smart Downsampling
*   **Camera**: 720p (for high-res display background).
*   **Vision**: 480p (downsampled for speed).
*   *Coordinate Mapping*: Corners found in 480p are upscaled to 720p before rendering.

---

## Running the Project

```bash
# 1. Install Dependencies
npm install

# 2. Start Server (Auto-Reload recommended)
npx nodemon server

# 3. Expose to Mobile (HTTPS required)
ngrok http 5500
```
