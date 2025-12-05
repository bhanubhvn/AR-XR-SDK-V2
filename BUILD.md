# AR SDK - Technical Architecture & Build Guide

A lightweight marker-based AR SDK for mobile browsers using OpenCV.js + Three.js.

---

## Technology Stack

| Component       | Technology              | Version | Purpose                              |
| --------------- | ----------------------- | ------- | ------------------------------------ |
| **CV Engine**   | OpenCV.js (WebAssembly) | 4.10.0  | ORB feature detection, BF matching   |
| **Pose Solver** | OpenCV solvePnP         | 4.10.0  | 6DoF pose from marker corners        |
| **3D Renderer** | Three.js                | r128    | WebGL rendering, GLTF loading        |
| **Camera**      | WebRTC getUserMedia     | -       | Live video stream from device camera |
| **Language**    | Vanilla JavaScript ES6+ | -       | No framework dependencies            |
| **Styling**     | CSS3                    | -       | Mobile-first responsive layout       |

---

## High-Level System Design

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              MOBILE AR APP                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                              │
│   index.html (Entry)                                                         │
│            │                                                                 │
│            ▼                                                                 │
│        app.js (Controller)                                                   │
│            │ initializes                                                     │
│            ▼                                                                 │
│   ┌───────────────┐   Corners   ┌───────────────┐   Pose (Matrix4) ┌────────┐│
│   │  AREngine     │────────────▶│  PoseSolver   │──────────────────▶│Renderer││
│   │ (OpenCV ORB)  │             │ (solvePnP)    │                   │(Three) ││
│   └───────────────┘             └───────────────┘                   └────────┘│
│                                                                              │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Directory Structure

```
ar-sdk-react/
├── server.js               # Express server (port 5500)
├── landing.html            # Landing page
├── package.json            # Dependencies
└── mobile-ar/
    ├── index.html          # AR viewer entry point
    ├── style.css           # Mobile-first responsive styles
    ├── app.js              # Main controller (init, loop, state)
    ├── ar-engine.js        # OpenCV processing (ORB, matching, homography)
    ├── pose-solver.js      # solvePnP 6DoF + projection matrix
    ├── renderer.js         # Three.js WebGL renderer
    ├── alignment-tool/     # Model offset calibration UI
    │   ├── index.html
    │   ├── style.css
    │   └── alignment.js
    ├── assets/
    │   ├── ranger-base-image.jpg   # Target marker image
    │   └── ranger-3d-model.glb     # 3D model (GLTF/GLB format)
    └── print-marker.html   # Printable marker
```

---

## Configuration Parameters

### AREngine (`ar-engine.js`)

| Parameter         | Value | Description                         |
| ----------------- | ----- | ----------------------------------- |
| `maxFeatures`     | 2000  | ORB keypoints                       |
| `ratioThreshold`  | 0.85  | Lowe's ratio test threshold         |
| `minMatches`      | 6     | Minimum good matches for homography |
| `minInliers`      | 5     | Minimum RANSAC inliers              |
| `ransacThreshold` | 4.0   | RANSAC reprojection error (px)      |

### App Controller (`app.js`)

| Parameter      | Value | Description                                  |
| -------------- | ----- | -------------------------------------------- |
| `processWidth` | 480   | Processing frame width (height keeps aspect) |
| `lostTimeout`  | 200ms | Hide model after target lost                 |

### IMU Fusion (`app.js`)

| Parameter           | Value | Description                                               |
| ------------------- | ----- | --------------------------------------------------------- |
| `blendWeightStrong` | 0.4   | IMU weight when visual pose is stable                     |
| `blendWeightWeak`   | 0.35  | IMU weight when visual pose is less certain               |
| `imuSmoothing`      | 0.15  | Slerp factor on raw IMU quaternion to reduce sensor noise |
| `yawBiasStrength`   | 0.05  | Rate to realign IMU yaw toward visual yaw during tracking |

### Renderer (`renderer.js`)

- Camera: FOV 45°, near 0.01, far 100, positioned at origin
- Lights: Ambient 0.6, Directional 0.8 (5,10,7), Fill 0.4 (-5,-5,5)
- Model offset configured via alignment tool

---

## Running the Project

```bash
# Install dependencies
npm install

# Start server
npm start
```

Server runs at `http://localhost:5500`

**Available pages:**

- Landing Page: `http://localhost:5500`
- AR Viewer: `http://localhost:5500/mobile-ar/index.html`
- Alignment Tool: `http://localhost:5500/mobile-ar/alignment-tool/index.html`

---

## Alignment Tool

The alignment tool allows you to calibrate the 3D model's offset relative to the target marker.

1. Open the alignment tool
2. Adjust position (X, Y, Z), rotation, and scale
3. Click "Apply to Project" to update `renderer.js` automatically

---

## Requirements

- HTTPS (required for camera access)
- Modern mobile browser (Chrome, Safari, Firefox)
- WebGL support
