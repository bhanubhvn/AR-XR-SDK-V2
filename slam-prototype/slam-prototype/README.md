# SLAM Prototype

High-accuracy Visual SLAM implementation for the AR SDK.

## Overview

This prototype implements a full Visual SLAM pipeline to replace/augment the existing frame-by-frame marker detection approach. The goal is to achieve:

- **Higher accuracy**: Reduced pose jitter through temporal tracking
- **Better stability**: Map point persistence enables tracking during partial occlusion
- **Improved robustness**: Motion model prediction fills gaps in visual tracking

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        SLAM Pipeline                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│   Camera Frame                                                    │
│        │                                                          │
│        ▼                                                          │
│   ┌─────────────────┐                                             │
│   │ Feature Tracker │ ← Lucas-Kanade Optical Flow                 │
│   │  (feature-tracker.js)                                         │
│   └────────┬────────┘                                             │
│            │ Tracked + New Points                                 │
│            ▼                                                      │
│   ┌─────────────────┐      ┌──────────────────┐                   │
│   │ Pose Estimator  │ ←──→ │   Map Manager    │                   │
│   │ (pose-estimator.js)    │ (map-manager.js) │                   │
│   └────────┬────────┘      └────────┬─────────┘                   │
│            │                        │                             │
│            ▼                        ▼                             │
│   ┌─────────────────┐      ┌──────────────────┐                   │
│   │ Keyframe Handler│      │ Bundle Adjuster  │                   │
│   │(keyframe-handler.js)   │(bundle-adjuster.js)                  │
│   └─────────────────┘      └──────────────────┘                   │
│                                                                   │
│   ───────────────────────────────────────────                     │
│                 ▼                                                 │
│        6DoF Pose → Renderer                                       │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

## Components

### Core SLAM Modules

| Module               | File                       | Purpose                                                                     |
| -------------------- | -------------------------- | --------------------------------------------------------------------------- |
| **Feature Tracker**  | `core/feature-tracker.js`  | Optical flow tracking with forward-backward validation, subpixel refinement |
| **Map Manager**      | `core/map-manager.js`      | 3D point triangulation, storage, and culling                                |
| **Keyframe Handler** | `core/keyframe-handler.js` | Keyframe selection, covisibility graph                                      |
| **Pose Estimator**   | `core/pose-estimator.js`   | 6DoF estimation with motion model, RANSAC, filtering                        |
| **Bundle Adjuster**  | `core/bundle-adjuster.js`  | Levenberg-Marquardt local optimization                                      |

### Utilities

| Module               | File                        | Purpose                         |
| -------------------- | --------------------------- | ------------------------------- |
| **Math Helpers**     | `utils/math-helpers.js`     | Matrix, quaternion, vector math |
| **Debug Visualizer** | `utils/debug-visualizer.js` | Feature and pose visualization  |

## Usage

### Running the Demo

```bash
# From project root
npm start

# Open in browser (use HTTPS for camera access)
# http://localhost:5500/slam-prototype/index.html
```

### Demo Controls

- **Reset SLAM**: Clears all tracking state and map
- **Toggle Debug**: Shows/hides feature tracking overlay

### Requirements

- HTTPS or localhost (required for camera access)
- Modern browser with WebGL support
- Mobile device with rear camera (recommended)

## Configuration

Key parameters in `slam-demo.js`:

```javascript
this.config = {
  processWidth: 640, // Processing resolution
  markerSize: 0.15, // Physical marker size (meters)
  enableDebugOverlay: true, // Show debug visualization
  enableIMU: true, // Enable IMU fusion
  enableBundleAdjustment: true, // Enable local BA
  baInterval: 30, // Frames between BA runs
}
```

## Optimization Notes

All source files contain `[OPT-X]` comments marking potential optimization points:

- `[OPT-1]` through `[OPT-15]` per file
- Each describes a specific optimization opportunity
- Priorities: Higher numbers = more advanced optimizations

Example:

```javascript
// [OPT-6] Consider using AKAZE for better accuracy (when available in OpenCV.js)
// this.akaze = new cv.AKAZE()
```

## Testing

See `VERIFICATION.md` for:

- Test scenarios (A-F)
- Expected metrics and pass criteria
- Debugging procedures
- Performance benchmarks

## Comparison with Original System

| Feature              | Original                      | SLAM                        |
| -------------------- | ----------------------------- | --------------------------- |
| Detection            | Per-frame ORB (2000 features) | Tracked points (~200)       |
| Pose source          | Marker corners only           | Map points + marker         |
| Temporal consistency | None                          | Optical flow + motion model |
| Occlusion handling   | Immediate loss                | Continues with map          |
| Memory               | Low                           | Moderate (map storage)      |

## Integration

After verification passes:

1. Copy SLAM modules to `mobile-ar/core/`
2. Update `app.js` to use SLAM pipeline
3. Keep original detection as fallback/initialization
4. Update BUILD.md and TECHNICAL-DESC.md

## License

MIT
