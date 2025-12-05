/**
 * ui-components.js
 * Handles UI interactions, state management for the sidebar, and file updates.
 */

const UI = {
  state: {
    selectedAsset: "model", // 'model', 'camera', 'sun', 'zap'
    transformMode: "translate",
  },

  init() {
    // Initialize icons
    if (window.lucide) {
      lucide.createIcons()
    }

    // Bind sidebar clicks
    document.querySelectorAll(".asset-item").forEach((item) => {
      item.addEventListener("click", (e) => {
        const type = item.dataset.type
        this.selectAsset(type)
      })
    })

    // Bind tool buttons
    document.querySelectorAll(".tool-btn[data-mode]").forEach((btn) => {
      btn.addEventListener("click", (e) => {
        const mode = btn.dataset.mode
        this.setTransformMode(mode)
      })
    })

    // Initial state
    this.selectAsset("model")
    this.setTransformMode("translate")
  },

  selectAsset(type) {
    this.state.selectedAsset = type

    // Update sidebar active state
    document.querySelectorAll(".asset-item").forEach((el) => {
      if (el.dataset.type === type) el.classList.add("active")
      else el.classList.remove("active")
    })

    // Show/Hide property panels
    // For now we only have model properties implemented
    const modelProps = document.getElementById("prop-model")
    const emptyProps = document.getElementById("prop-empty")

    if (type === "model") {
      modelProps.style.display = "block"
      emptyProps.style.display = "none"
      // Enable transform controls in 3D view
      if (AlignmentTool.transformControls) {
        AlignmentTool.transformControls.enabled = true
        AlignmentTool.transformControls.visible = true
      }
    } else {
      modelProps.style.display = "none"
      emptyProps.style.display = "flex"
      // Disable transform controls for non-model items (placeholder)
      if (AlignmentTool.transformControls) {
        AlignmentTool.transformControls.enabled = false
        AlignmentTool.transformControls.visible = false
      }
    }
  },

  setTransformMode(mode) {
    this.state.transformMode = mode

    // Update 3D tool
    if (window.AlignmentTool) {
      AlignmentTool.setTransformMode(mode)
    }

    // Update UI buttons
    document.querySelectorAll(".tool-btn[data-mode]").forEach((btn) => {
      if (btn.dataset.mode === mode) btn.classList.add("active")
      else btn.classList.remove("active")
    })
  },

  resetValues() {
    if (window.resetValues) window.resetValues()
  },

  copyCode() {
    const code = document.getElementById("output").textContent
    navigator.clipboard.writeText(code).then(() => {
      const status = document.getElementById("status-msg")
      status.textContent = "Copied!"
      setTimeout(() => (status.textContent = ""), 2000)
    })
  },

  /**
   * Uses File System Access API to update renderer files.
   * Note: This requires user interaction/permission.
   */
  async submitAndUpdateFiles() {
    if (!window.showOpenFilePicker) {
      alert(
        "Your browser does not support the File System Access API. Please copy the code manually."
      )
      return
    }

    const status = document.getElementById("status-msg")
    const generatedCode = document.getElementById("output").textContent

    // Parse the generated object string back to values to inject
    // We'll just inject the whole block or use regex.
    // The generated code format is:
    // const modelOffset = {
    //   position: { x: ..., y: ..., z: ... },
    //   rotation: { x: ..., y: ..., z: ... },
    //   scale: ...
    // }

    // We need to extract just the object content to replace the existing object in the file.
    // Or simpler: replace the entire `modelOffset: { ... },` block in the target file.

    try {
      status.textContent = "Select renderer.js..."
      const [fileHandle] = await window.showOpenFilePicker({
        types: [
          {
            description: "JavaScript Files",
            accept: { "text/javascript": [".js"] },
          },
        ],
        multiple: false,
      })

      if (!fileHandle) return

      // Ask for Read/Write permission
      const options = { mode: "readwrite" }
      if ((await fileHandle.queryPermission(options)) !== "granted") {
        if ((await fileHandle.requestPermission(options)) !== "granted") {
          status.textContent = "Permission denied."
          return
        }
      }

      const file = await fileHandle.getFile()
      const text = await file.text()

      // Regex to find modelOffset object
      // matches modelOffset: { ... } allowing for multiline
      // We assume standard formatting from our previous edits
      const regex =
        /modelOffset:\s*{[^}]*?position:[^}]*?rotation:[^}]*?scale:[^}]*?},/gs

      // Construct new object string matching the file indentation (2 spaces)
      // We have to manually parse the text content from #output or use AlignmentTool.offset
      const { position, rotation, scale } = AlignmentTool.offset

      const newBlock = `modelOffset: {
    position: { x: ${position.x.toFixed(2)}, y: ${position.y.toFixed(
        2
      )}, z: ${position.z.toFixed(2)} },
    rotation: { x: ${((rotation.x * Math.PI) / 180).toFixed(3)}, y: ${(
        (rotation.y * Math.PI) /
        180
      ).toFixed(3)}, z: ${((rotation.z * Math.PI) / 180).toFixed(3)} },
    scale: ${scale.toFixed(3)},
  },`

      if (!regex.test(text)) {
        status.textContent = "Could not find modelOffset in file."
        return
      }

      const newText = text.replace(regex, newBlock)

      const writable = await fileHandle.createWritable()
      await writable.write(newText)
      await writable.close()

      status.textContent = "Updated! Select renderer-webgpu.js?"

      // Optional: loop for the second file
      if (
        confirm(
          "renderer.js updated! Do you want to update renderer-webgpu.js as well?"
        )
      ) {
        await this.submitAndUpdateFiles() // Recursive call for simplicity, though user has to pick file again
      } else {
        status.textContent = "Done."
      }
    } catch (err) {
      console.error(err)
      status.textContent = "Error updating file."
    }
  },
}

// Global expose for HTML click handlers
window.UI = UI
