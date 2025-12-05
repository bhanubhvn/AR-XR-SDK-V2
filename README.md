# AR SDK

Lightweight marker-based AR for mobile browsers. Detects image targets and overlays 3D models.

## Tech Stack

- **OpenCV.js** - Feature detection & tracking
- **Three.js** - 3D rendering (WebGL)
- **Vanilla JS** - No frameworks

## Quick Start

```bash
npm install
npm start
```

Open on mobile: `https://localhost:5500`

## How It Works

1. Point camera at target image (`assets/ranger-base-image.jpg`)
2. App detects the image using ORB features
3. 3D model appears on top of the target

## Project Structure

```
├── server.js         # Express server
├── landing.html      # Landing page
└── mobile-ar/
    ├── index.html        # AR viewer
    ├── style.css         # Mobile UI
    ├── app.js            # Main controller
    ├── ar-engine.js      # OpenCV detection
    ├── pose-solver.js    # 6DoF pose estimation
    ├── renderer.js       # Three.js 3D
    ├── alignment-tool/   # Model offset calibration
    └── assets/
        ├── ranger-base-image.jpg
        └── ranger-3d-model.glb
```

## Alignment Tool

The alignment tool (`/mobile-ar/alignment-tool/`) lets you adjust the 3D model's position, rotation, and scale relative to the target image. Click "Apply to Project" to automatically update `renderer.js`.

## Customization

**Change target image:** Replace `assets/ranger-base-image.jpg`

**Change 3D model:** Replace `assets/ranger-3d-model.glb` and update path in `app.js`

**Tune detection:** Edit `ar-engine.js` config:

```javascript
this.config = {
  maxFeatures: 2000, // More = better detection, slower
  ratioThreshold: 0.85, // Lower = stricter matching
  minMatches: 6, // Minimum matches to detect
}
```

## Requirements

- HTTPS (required for camera)
- Modern mobile browser
- WebGL support

## License

MIT
