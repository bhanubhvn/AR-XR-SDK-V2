const express = require('express')
const bodyParser = require('body-parser')
const cors = require('cors')
const fs = require('fs')
const path = require('path')

const app = express()
const PORT = 5500

app.use(cors())
app.use(bodyParser.json())

// Serve static files
app.use(express.static(__dirname))

// Path to the renderer file
const RENDERER_PATH = path.join(__dirname, 'mobile-ar', 'renderer.js')

app.post('/update-renderer', (req, res) => {
    const { position, rotation, scale } = req.body

    if (!position || !rotation || scale === undefined) {
        return res.status(400).json({ success: false, message: 'Missing offset data' })
    }

    // Negate X rotation to convert from alignment tool coords to AR coords
    const rotX = ((rotation.x * Math.PI) / 180).toFixed(3)
    const rotY = ((rotation.y * Math.PI) / 180).toFixed(3)
    const rotZ = ((rotation.z * Math.PI) / 180).toFixed(3)

    const newBlock = `modelOffset: {
    position: { x: ${position.x.toFixed(3)}, y: ${position.y.toFixed(3)}, z: ${position.z.toFixed(3)} },
    rotation: { x: ${rotX}, y: ${rotY}, z: ${rotZ} },
    scale: ${scale.toFixed(3)},
  },`

    const regex = /modelOffset:\s*\{[\s\S]*?scale:\s*[\d\.\-]+,?\s*\},/g

    try {
        if (fs.existsSync(RENDERER_PATH)) {
            let content = fs.readFileSync(RENDERER_PATH, 'utf8')
            if (regex.test(content)) {
                content = content.replace(regex, newBlock)
                fs.writeFileSync(RENDERER_PATH, content, 'utf8')
                console.log('Updated renderer.js')
            }
        }

        res.json({ success: true, message: 'File updated successfully' })

    } catch (error) {
        console.error('Error updating file:', error)
        res.status(500).json({ success: false, message: 'Internal server error' })
    }
})

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'landing.html'))
})

app.listen(PORT, () => {
    console.log(`\n🚀 AR SDK Server running at http://localhost:${PORT}\n`)
    console.log('Available pages:')
    console.log('  • Landing Page:  http://localhost:5500')
    console.log('  • AR Viewer:     http://localhost:5500/mobile-ar/index.html')
    console.log('  • XR Creator:    http://localhost:5500/mobile-ar/alignment-tool/index.html\n')
})
