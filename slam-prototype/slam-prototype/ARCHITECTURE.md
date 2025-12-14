# SLAM Prototype - Architecture Documentation

## What is This Project?

The **SLAM Prototype** is a **Visual SLAM (Simultaneous Localization and Mapping)** implementation for augmented reality applications. It enables:

1. **Real-time Camera Tracking** - Track device position and orientation (6 Degrees of Freedom) in 3D space
2. **World Mapping** - Build and maintain a 3D map of the environment using visual features
3. **Persistent Anchors** - Place virtual objects that remain fixed in the real world as you move around

### Simple Explanation

Imagine you're in a dark room with a flashlight. SLAM works similarly:
- **Feature Detection**: The camera spots distinctive points (like corners or edges) - these are like landmarks you'd remember
- **Feature Tracking**: As you move, the system tracks how these landmarks move in the camera view
- **Triangulation**: By seeing the same landmark from different positions, the system calculates its 3D position (like how our two eyes give us depth perception)
- **Pose Estimation**: Knowing where landmarks are in 3D, the system calculates where YOU are

---

## Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                           SLAM PIPELINE OVERVIEW                               │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                                │
│    ┌─────────────┐         ┌──────────────────────────────────────────────┐   │
│    │   Camera    │────────▶│               SLAM CORE                       │   │
│    │   Frame     │         │                                               │   │
│    └─────────────┘         │   ┌───────────────────┐                      │   │
│                            │   │  Feature Tracker  │◀─── Lucas-Kanade     │   │
│    ┌─────────────┐         │   │  (Optical Flow)   │     Optical Flow     │   │
│    │     IMU     │─────┐   │   └─────────┬─────────┘                      │   │
│    │   Sensor    │     │   │             │                                │   │
│    └─────────────┘     │   │             ▼  Tracked Points                │   │
│                        │   │   ┌───────────────────┐                      │   │
│                        │   │   │  Pose Estimator   │◀─── PnP + RANSAC     │   │
│                        │   │   │  (6DoF Tracking)  │     + Motion Model   │   │
│                        │   │   └─────────┬─────────┘                      │   │
│                        │   │             │                                │   │
│                        │   │             ▼  Camera Pose                   │   │
│                        │   │   ┌───────────────────┐                      │   │
│                        └───┼──▶│  IMU Fusion       │◀─── Quaternion SLERP │   │
│                            │   │  (Rotation)       │                      │   │
│                            │   └─────────┬─────────┘                      │   │
│                            │             │                                │   │
│                            │             ▼                                │   │
│                            └──────────────────────────────────────────────┘   │
│                                          │                                    │
│                                          ▼  Final 6DoF Pose                   │
│                            ┌──────────────────────────────────────────────┐   │
│                            │               3D RENDERER                     │   │
│                            │      (Three.js - Places AR Content)          │   │
│                            └──────────────────────────────────────────────┘   │
│                                                                                │
└──────────────────────────────────────────────────────────────────────────────┘
```

### Data Flow Between Components

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                        WORLD MAPPING ARCHITECTURE                              │
├──────────────────────────────────────────────────────────────────────────────┤
│                                                                                │
│                 FEATURE                    KEYFRAME                           │
│                 TRACKER                    HANDLER                            │
│              ┌───────────┐              ┌───────────┐                         │
│              │ Detects & │              │ Decides   │                         │
│              │  Tracks   │──────────────▶ When to   │                         │
│              │ Features  │              │ Save      │                         │
│              └─────┬─────┘              └─────┬─────┘                         │
│                    │                          │                               │
│                    │                          ▼                               │
│                    │                    ┌───────────┐                         │
│                    │                    │  MAP      │                         │
│                    └───────────────────▶│ MANAGER   │                         │
│                                         │           │                         │
│                                         │ ┌───────┐ │                         │
│    ┌─────────────────────────────────── │ │3D MAP │ │ ◀── PERSISTENT WORLD    │
│    │                                    │ │POINTS │ │     (Anchors/Landmarks) │
│    │                                    │ └───────┘ │                         │
│    │                                    └─────┬─────┘                         │
│    │                                          │                               │
│    │                                          ▼                               │
│    │                                    ┌───────────┐                         │
│    │                                    │  BUNDLE   │                         │
│    │                                    │ ADJUSTER  │◀─── Levenberg-Marquardt │
│    │   Optimized Poses                  │ (Refine)  │     Optimization        │
│    └────────────────────────────────────┴───────────┘                         │
│                                                                                │
└──────────────────────────────────────────────────────────────────────────────┘
```

---

## Core Components

| Component | File | Purpose |
|-----------|------|---------|
| **Feature Tracker** | `core/feature-tracker.js` | Detects and tracks visual features using Lucas-Kanade optical flow with forward-backward validation |
| **Pose Estimator** | `core/pose-estimator.js` | Estimates camera position/rotation using PnP (Perspective-n-Point) with RANSAC and motion model |
| **Map Manager** | `core/map-manager.js` | Stores 3D map points (world landmarks), handles triangulation and point management |
| **Keyframe Handler** | `core/keyframe-handler.js` | Decides when to create keyframes, manages covisibility graph |
| **Bundle Adjuster** | `core/bundle-adjuster.js` | Refines camera poses and 3D points jointly using Levenberg-Marquardt optimization |
| **Debug Visualizer** | `utils/debug-visualizer.js` | Renders debug overlays showing tracked features and pose |
| **Math Helpers** | `utils/math-helpers.js` | Matrix, quaternion, and vector math utilities |

---

## How Tracking Points Are Saved (World Mapping)

The SLAM system builds a **persistent 3D map** of the environment. Here's how tracking points become permanent world landmarks:

### Step 1: Feature Detection and Tracking

```javascript
// In FeatureTracker.track()
// 1. Detect distinctive features using goodFeaturesToTrack (Shi-Tomasi corners)
// 2. Track features between frames using Lucas-Kanade optical flow
// 3. Validate tracking with forward-backward error check
```

### Step 2: Keyframe Creation

The system doesn't save every frame. Instead, it creates **keyframes** at strategic moments:

```javascript
// Keyframe is created when:
shouldCreateKeyframe(currentPose, trackingResult, frameNumber) {
    // 1. Sufficient camera movement (translation > threshold)
    // 2. Sufficient rotation change
    // 3. Low feature overlap with last keyframe
    // 4. Minimum frames since last keyframe
}
```

### Step 3: Triangulation - Creating 3D Map Points

When the same feature is visible from multiple keyframes, it gets **triangulated** into a 3D point:

```
                Keyframe 1                    Keyframe 2
                (Camera at position A)        (Camera at position B)
                     ○                              ○
                      \                            /
                       \  ray1                    /  ray2
                        \                        /
                         \      ★              /
                          \   3D Point         /
                           \   in World      /
                            \      ↓        /
                             ──────●───────
                               Triangulated
                                Map Point
```

```javascript
// In MapManager.triangulate()
// 1. Check parallax angle (minimum 2 degrees)
// 2. Linear triangulation using DLT (Direct Linear Transform)
// 3. Validate: point must be in front of both cameras
// 4. Check reprojection error < 2 pixels
// 5. Store as MapPoint with confidence score
```

### Step 4: Map Point Storage Structure

Each 3D map point is stored with rich metadata:

```javascript
// Map Point Structure (in MapManager)
const mapPoint = {
    id: 42,                                  // Unique identifier
    position: Float64Array([x, y, z]),       // 3D world coordinates
    descriptor: Uint8Array(32),              // ORB feature descriptor
    normal: Float64Array([nx, ny, nz]),      // Mean viewing direction
    confidence: 0.85,                        // Quality score (0-1)
    observations: Map([                      // Which keyframes see this point
        [kf1_id, feature_index_1],
        [kf2_id, feature_index_2],
    ]),
    lastSeenFrame: 150,                      // For cleanup
    createdFrame: 100,                       // Birth frame
    reprojectionErrors: [0.5, 0.8, ...],     // History for quality assessment
}
```

### Step 5: Bundle Adjustment (Refinement)

Periodically, all keyframe poses and map points are jointly optimized:

```javascript
// In BundleAdjuster.optimizeLocal()
// Uses Levenberg-Marquardt to minimize reprojection error
// across all observations simultaneously
```

---

## Purpose of Anchors in SLAM

In SLAM terminology, **anchors** (also called **map points** or **landmarks**) serve several critical purposes:

### 1. **Localization Reference**

Anchors are fixed 3D points in the world. By matching currently visible features to known anchors, the system can determine exactly where the camera is.

```
Without Anchors (Frame-by-Frame):
  - Each frame: Detect marker → Estimate pose
  - If marker occluded: TRACKING LOST

With Anchors (SLAM):
  - Map remembers 100+ 3D points
  - Match any subset to continue tracking
  - Marker partially hidden: STILL WORKS
```

### 2. **AR Content Placement**

When you place a virtual object at a location, that location is defined relative to nearby anchors:

```javascript
// Conceptual anchor usage for AR
const virtualObject = {
    anchorId: 42,                    // Reference to nearest map point
    offsetFromAnchor: [0.1, 0, 0.2], // Relative position
}

// When rendering:
const anchorWorldPos = mapManager.get(42).position
const objectWorldPos = add(anchorWorldPos, offset)
// Object stays fixed even as camera moves!
```

### 3. **Persistence and Relocalization**

Anchors enable:
- **Session continuity**: Save and reload maps
- **Relocalization**: If tracking is lost, re-find position by matching features to known anchors

### 4. **Temporal Smoothing**

Unlike per-frame marker detection, anchors provide temporal consistency:

```
Frame 1: Pose from 50 anchor matches → Pose A
Frame 2: Pose from 48 anchor matches → Pose A' (very similar)
Frame 3: Pose from 52 anchor matches → Pose A'' (smooth motion)

Result: Jitter-free AR experience
```

---

## How to Use This Approach in mobile-ar Project

### Integration Path

The SLAM prototype can enhance the existing `mobile-ar` project by providing:

1. **Fallback Tracking**: When marker is not visible
2. **Reduced Jitter**: Temporal consistency from tracked features
3. **Extended Tracking Area**: AR works beyond just the marker

### Step-by-Step Integration

#### Step 1: Copy SLAM Core Modules

```
mobile-ar/
├── core/                    ← Create this folder
│   ├── feature-tracker.js   ← Copy from slam-prototype
│   ├── map-manager.js
│   ├── keyframe-handler.js
│   ├── pose-estimator.js
│   └── bundle-adjuster.js
└── utils/
    └── math-helpers.js
```

#### Step 2: Modify app.js

```javascript
// In mobile-ar/app.js

// Import SLAM components
import { FeatureTracker } from './core/feature-tracker.js'
import { MapManager } from './core/map-manager.js'
import { PoseEstimator } from './core/pose-estimator.js'

class App {
    async init() {
        // Existing marker detection...
        
        // Add SLAM components
        this.featureTracker = new FeatureTracker(cv)
        this.mapManager = new MapManager(cv)
        this.poseEstimator = new PoseEstimator(cv)
    }
    
    processFrame(grayFrame) {
        // Step 1: Try marker detection (current approach)
        const markerPose = this.detectMarker(grayFrame)
        
        if (markerPose) {
            // Marker found - use its pose and seed SLAM
            this.currentPose = markerPose
            this.maybeCreateKeyframe(grayFrame, markerPose)
        } else {
            // Step 2: Marker not visible - fall back to SLAM
            const trackingResult = this.featureTracker.track(grayFrame)
            const correspondences = this.mapManager.getCorrespondences(trackingResult.points)
            
            if (correspondences.points3D.length >= 6) {
                this.currentPose = this.poseEstimator.estimatePose(
                    correspondences.points2D,
                    correspondences.points3D
                )
            }
        }
        
        // Render AR content using this.currentPose
        this.renderer.updateWithPose(this.currentPose)
    }
}
```

#### Step 3: Initialize on Marker Detection

```javascript
// When marker is first detected, initialize SLAM map around it
onMarkerFirstDetected(markerCorners, pose) {
    // Create initial keyframe at marker location
    this.keyframeHandler.createKeyframe({
        frameNumber: this.frameNumber,
        pose: pose,
        features: this.featureTracker.track(this.currentFrame).points,
    })
    
    // Marker corners become initial anchors/map points
    // Their 3D positions are known from marker size
}
```

### Architecture After Integration

```
┌────────────────────────────────────────────────────────────┐
│                    mobile-ar + SLAM                         │
├────────────────────────────────────────────────────────────┤
│                                                             │
│    Camera Frame                                             │
│         │                                                   │
│         ├──────────────────────┬───────────────────────┐   │
│         ▼                      ▼                       ▼   │
│    ┌─────────────┐      ┌─────────────┐      ┌──────────┐ │
│    │   Marker    │      │   Feature   │      │   IMU    │ │
│    │  Detection  │      │   Tracker   │      │  Fusion  │ │
│    │ (ORB Match) │      │ (Opt.Flow)  │      │          │ │
│    └──────┬──────┘      └──────┬──────┘      └────┬─────┘ │
│           │                    │                   │       │
│           ▼                    ▼                   │       │
│    ┌─────────────┐      ┌─────────────┐           │       │
│    │ Marker Pose │      │  SLAM Pose  │           │       │
│    │  (Primary)  │      │ (Fallback)  │           │       │
│    └──────┬──────┘      └──────┬──────┘           │       │
│           │                    │                   │       │
│           └─────────┬──────────┘                   │       │
│                     ▼                              │       │
│              ┌─────────────┐                       │       │
│              │  Pose Fusion│◀──────────────────────┘       │
│              │  & Filter   │                               │
│              └──────┬──────┘                               │
│                     ▼                                      │
│              ┌─────────────┐                               │
│              │  3D Render  │                               │
│              │  (Three.js) │                               │
│              └─────────────┘                               │
│                                                             │
└────────────────────────────────────────────────────────────┘
```

### Benefits After Integration

| Aspect | Before (Marker Only) | After (Marker + SLAM) |
|--------|---------------------|----------------------|
| **Tracking when marker occluded** | Lost | Continues via map points |
| **Pose jitter** | High (per-frame detection) | Low (temporal tracking) |
| **Computational efficiency** | 2000 ORB features/frame | ~200 tracked points |
| **Extended tracking area** | Marker only | Beyond marker |
| **Relocalization** | Requires marker | Can use any known features |

---

## Performance Considerations

### Current Prototype (Optimized for Accuracy)

- Feature tracking: ~200 points @ 30fps
- Bundle adjustment: Every 30 frames
- Map size: Up to 5000 points

### For Mobile Deployment

Consider these optimizations (marked as `[OPT-X]` in source):

1. **Web Workers**: Move CV processing off main thread
2. **Adaptive Quality**: Reduce features on slower devices
3. **Sparse BA**: Only optimize recent keyframes
4. **Spatial Hash**: Faster map point lookup

---

## Summary

| Concept | What It Does |
|---------|--------------|
| **SLAM** | Builds 3D map while tracking camera position |
| **Feature Tracker** | Follows visual points between frames |
| **Map Points/Anchors** | Permanent 3D landmarks in the world |
| **Keyframes** | Saved camera views for triangulation |
| **Triangulation** | Creates 3D points from 2D observations |
| **Bundle Adjustment** | Refines everything for accuracy |
| **PnP + RANSAC** | Calculates pose from 2D-3D matches |

The SLAM approach provides **superior tracking stability** compared to per-frame marker detection, enabling robust AR experiences even when the original marker is not fully visible.
