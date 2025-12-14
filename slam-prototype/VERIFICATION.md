# SLAM Verification Guide

Comprehensive testing and verification procedures for the SLAM prototype.

---

## Quick Start

```bash
# From project root
npm start

# Open in browser
# http://localhost:5500/slam-prototype/index.html
```

---

## Test Scenarios

### Scenario A: Static Camera Test

**Purpose**: Verify pose stability when camera is stationary.

**Steps**:

1. Point camera at target marker, hold completely still
2. Observe for 5 seconds
3. Check debug overlay statistics

**Expected Results**:
| Metric | Target | How to Verify |
|--------|--------|---------------|
| Position jitter | < 5mm | Check pose stability in stats |
| Rotation jitter | < 1° | Observe 3D model stability |
| Feature tracking | > 50 stable features | Check "Tracked" count in overlay |
| FPS | > 25 | Check FPS counter |

**Pass Criteria**: Model appears stable without visible shaking.

---

### Scenario B: Slow Movement Test

**Purpose**: Verify tracking continuity during gentle camera motion.

**Steps**:

1. Start with camera pointing at marker
2. Slowly orbit around the marker (full 180°)
3. Maintain ~0.5m distance

**Expected Results**:
| Metric | Target | How to Verify |
|--------|--------|---------------|
| Track continuity | No loss | Model should stay visible |
| Feature persistence | Avg age > 5 | Check "Avg Age" in debug stats |
| Pose smoothness | Smooth transitions | Visual observation |

**Pass Criteria**: Model tracks smoothly throughout movement.

---

### Scenario C: Partial Occlusion Test

**Purpose**: Verify tracking with partial marker visibility.

**Steps**:

1. Point camera at marker until tracking starts
2. Cover 30-50% of marker with your hand
3. Observe if tracking continues
4. Uncover marker

**Expected Results**:
| Metric | Target | How to Verify |
|--------|--------|---------------|
| Track maintenance | Tracking continues | Model stays visible |
| Feature fallback | Uses map points | Check "Map Points" count |
| Recovery time | < 0.5s when uncovered | Time the recovery |

**Pass Criteria**: Tracking maintained with partial occlusion.

---

### Scenario D: Complete Occlusion Test

**Purpose**: Verify relocalization after tracking loss.

**Steps**:

1. Establish stable tracking
2. Completely cover marker for 2 seconds
3. Uncover and observe recovery

**Expected Results**:
| Metric | Target | How to Verify |
|--------|--------|---------------|
| Lost frames | < 30 before hiding model | Check "Lost" counter |
| Recovery time | < 1 second | Time from uncover to stable |
| Pose accuracy | Correct position | Model should appear in same place |

**Pass Criteria**: Tracking recovers within 1 second.

---

### Scenario E: Distance Test

**Purpose**: Verify tracking range.

**Steps**:

1. Start at 0.5m from marker
2. Slowly back away while maintaining view
3. Note distance when tracking is lost

**Expected Results**:
| Metric | Target | How to Verify |
|--------|--------|---------------|
| Maximum distance | > 2.5m | Measure or estimate distance |
| Feature detection | Stable at 2m | Check tracked count |
| Pose accuracy | Consistent scale | Model size should stay proportional |

**Pass Criteria**: Tracking maintained to at least 2.5m.

---

### Scenario F: Rapid Movement Test

**Purpose**: Verify tracking robustness during fast camera motion.

**Steps**:

1. Establish stable tracking
2. Quickly (but not violently) move camera left-right
3. Stop and observe recovery

**Expected Results**:
| Metric | Target | How to Verify |
|--------|--------|---------------|
| Track survival | Some frames lost, but recovers | Lost counter spikes then recovers |
| Feature re-detection | New features added | Check "New" count |
| No crashes | App remains functional | No errors in console |

**Pass Criteria**: App handles rapid motion gracefully.

---

## Debug Overlay Reference

### Feature Colors

| Color      | Meaning                                  |
| ---------- | ---------------------------------------- |
| 🟢 Green   | Tracked feature (normal)                 |
| 🔵 Blue    | Newly detected feature                   |
| 🟡 Gold    | Stable feature (age > 10, quality > 0.6) |
| 🟣 Magenta | Projected map point                      |

### Statistics Panel

| Stat            | Description                   | Target Value |
| --------------- | ----------------------------- | ------------ |
| **Tracked**     | Currently tracked 2D features | 50-200       |
| **New**         | Features detected this frame  | 0-20         |
| **Lost**        | Features lost this frame      | < 10         |
| **Avg Quality** | Mean feature quality (0-1)    | > 0.6        |
| **Avg Age**     | Mean feature age in frames    | > 5          |
| **Map Points**  | Total 3D points in map        | 0-500        |
| **Keyframes**   | Total stored keyframes        | 1-20         |
| **Inliers**     | PnP inliers for current pose  | > 10         |
| **Lost**        | Frames since last good pose   | 0            |
| **FPS**         | Frames per second             | > 25         |

---

## Performance Benchmarks

### Target Performance (Mobile)

| Metric          | Low-End | Mid-Range | High-End |
| --------------- | ------- | --------- | -------- |
| FPS             | > 20    | > 30      | > 45     |
| Processing time | < 50ms  | < 30ms    | < 20ms   |
| Memory usage    | < 100MB | < 80MB    | < 60MB   |
| Feature count   | 100     | 200       | 300      |

### How to Measure

1. **FPS**: Built-in counter in top-right corner
2. **Processing time**: Check browser DevTools Performance tab
3. **Memory**: Chrome DevTools → Memory tab → Heap snapshot
4. **Feature count**: Debug overlay statistics

---

## Comparison with Original System

### A/B Testing

To compare SLAM performance with the original system:

1. Open original AR: `http://localhost:5500/mobile-ar/index.html`
2. Open SLAM demo: `http://localhost:5500/slam-prototype/index.html`
3. Test same scenarios on both
4. Record metrics

### Comparison Table

| Metric               | Original         | SLAM Target        |
| -------------------- | ---------------- | ------------------ |
| Pose jitter (static) | ~10mm            | < 5mm              |
| Tracking distance    | 2m               | 3m                 |
| Occlusion recovery   | Immediate loss   | Continues tracking |
| FPS                  | ~25              | ~25                |
| Feature matching     | Per-frame (2000) | Tracked (200)      |

---

## Common Issues & Solutions

### Issue: Low Feature Count

**Symptoms**: Tracked count < 50, frequent tracking loss

**Solutions**:

- Ensure adequate lighting
- Use a textured marker with high contrast
- Clean camera lens
- Reduce `config.minDistance` in feature-tracker.js

### Issue: High Jitter

**Symptoms**: 3D model shakes even when camera is still

**Solutions**:

- Increase `positionFilterAlpha` in pose-estimator.js (try 0.5)
- Reduce `lkEpsilon` in feature-tracker.js
- Enable IMU fusion

### Issue: Tracking Loss on Movement

**Symptoms**: Loses tracking during normal camera movement

**Solutions**:

- Increase `lkMaxLevel` in feature-tracker.js (try 5)
- Increase `maxForwardBackwardError` (try 2.0)
- Check lighting conditions

### Issue: Poor Relocalization

**Symptoms**: Takes too long to recover after marker is re-visible

**Solutions**:

- Decrease marker detection interval in slam-demo.js
- Increase ORB feature count for marker detection
- Store more keyframes

---

## Console Logging

Enable verbose logging by adding to browser console:

```javascript
// Track all SLAM components
window.DEBUG_SLAM = true
```

### Log Prefixes

| Prefix              | Component                   |
| ------------------- | --------------------------- |
| `[FeatureTracker]`  | Feature tracking operations |
| `[MapManager]`      | Map point operations        |
| `[KeyframeHandler]` | Keyframe decisions          |
| `[PoseEstimator]`   | Pose estimation             |
| `[BundleAdjuster]`  | Optimization runs           |
| `[SLAMDemo]`        | Main application            |

---

## Automated Metrics Collection

Add this snippet to collect test metrics:

```javascript
// Run in console during testing
const metrics = {
  startTime: Date.now(),
  fps: [],
  tracked: [],
  mapPoints: [],

  collect() {
    this.fps.push(slamDemo.stats.fps)
    this.tracked.push(slamDemo.stats.trackedFeatures)
    this.mapPoints.push(slamDemo.stats.mapPoints)
  },

  report() {
    const duration = (Date.now() - this.startTime) / 1000
    console.log("=== Test Report ===")
    console.log(`Duration: ${duration.toFixed(1)}s`)
    console.log(
      `Avg FPS: ${(
        this.fps.reduce((a, b) => a + b, 0) / this.fps.length
      ).toFixed(1)}`
    )
    console.log(
      `Avg Tracked: ${(
        this.tracked.reduce((a, b) => a + b, 0) / this.tracked.length
      ).toFixed(1)}`
    )
    console.log(`Max Map Points: ${Math.max(...this.mapPoints)}`)
  },
}

// Start collecting
const collectInterval = setInterval(() => metrics.collect(), 100)

// After test, run:
// clearInterval(collectInterval)
// metrics.report()
```

---

## Next Steps After Verification

1. **If all tests pass**: Proceed to Phase 4 (Integration)
2. **If some tests fail**:
   - Document failing scenarios
   - Identify component needing improvement
   - Tune parameters or fix bugs
3. **Performance tuning**:
   - Profile with Chrome DevTools
   - Identify bottlenecks
   - Apply optimizations noted in code (OPT comments)
