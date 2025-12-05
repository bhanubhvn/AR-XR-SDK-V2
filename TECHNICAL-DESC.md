# Technical Description

## What this project does

- Shows a 3D model on top of a printed image (marker) using the phone camera.
- Uses computer vision to find the marker, then places the 3D model in the right spot and orientation.
- Uses phone motion sensors (IMU) to keep the model stable when vision gets noisy.

## Main pieces

- **Camera (getUserMedia)**: streams video from the rear camera.
- **Computer Vision (OpenCV.js)**:
  - ORB feature detection (up to 2000 keypoints), BFMatcher with KNN (k=2), ratio test 0.85.
  - RANSAC homography (threshold 4 px) to validate matches and extract 4 marker corners.
- **Pose Solver (solvePnP)**: turns the 4 marker corners into a 6DoF pose (position + rotation) using marker size 0.15 m.
- **Renderer (Three.js WebGL)**:
  - Loads GLTF model, applies model offset (position/rotation/scale) from the alignment tool.
  - Lights: ambient, directional, fill.
- **IMU Fusion (DeviceOrientation)**:
  - Reads alpha/beta/gamma, converts to quaternion, smooths it.
  - Blends visual rotation toward IMU with adaptive weights (strong/weak).
  - Yaw bias correction nudges IMU yaw toward visual yaw to limit drift.
- **Alignment Tool**: adjust the model offset (position/rotation/scale) relative to the marker.

## How a frame is processed

1. Draw camera frame to a 2D canvas (cover mode, cropped to fill).
2. Downscale to 480 px wide for faster CV; convert to grayscale and equalize contrast.
3. Detect ORB features (2000 max) and match against the marker features with BFMatcher + ratio test.
4. If enough good matches, compute homography (RANSAC), extract the 4 marker corners, then solvePnP for 6DoF pose.
5. Blend visual rotation with IMU rotation (slerp) using adaptive weights.
6. Apply model offset from the alignment tool and render the 3D scene.

## IMU fusion

- Read phone orientation (alpha/beta/gamma) and turn it into a quaternion.
- Smooth the IMU quaternion to cut sensor noise.
- Slowly nudge IMU yaw toward the visual yaw to avoid drift.
- Blend visual rotation toward IMU using two weights:
  - `blendWeightStrong` (when vision is stable)
  - `blendWeightWeak` (when vision is weaker)
- Position always comes from vision; only rotation is blended.

## Performance knobs

- `processWidth` (app.js): lower to reduce CV cost (default 480).
- `maxFeatures` (ar-engine.js): lower from 2000 for speed on low-end devices.
- Keep logging minimal in the render loop.

## Key numbers

- Processing width: 480 px.
- ORB features: 2000 max.
- Ratio test: 0.85; min matches: 6; min inliers: 5; RANSAC thresh: 4 px.
- Marker size: 0.15 m.
- IMU: blendWeightStrong 0.4, blendWeightWeak 0.35, imuSmoothing 0.15, yawBiasStrength 0.05.
- Lights: ambient 0.6, directional 0.8 (5,10,7), fill 0.4 (-5,-5,5).

## Files

- `mobile-ar/index.html`: Entry point, loads scripts in order.
- `mobile-ar/app.js`: main controller, camera loop, IMU fusion, state.
- `mobile-ar/ar-engine.js`: ORB detection, matching, homography, corner output.
- `mobile-ar/pose-solver.js`: solvePnP pose + projection matrix.
- `mobile-ar/renderer.js`: Three.js WebGL renderer.
- `mobile-ar/alignment-tool/`: adjust model offset.
- `mobile-ar/assets/`: marker image and 3D model.

## How to verify

- Open `index.html` via the server (`npm start`).
- Allow camera (and motion sensors on iOS).
- Point at the printed marker: model should lock on with smooth rotation.
- Cover the marker briefly: orientation should remain stable; when uncovered it re-locks without big jumps.
