# Unlook 3D Scanner - Documentation Index

This directory contains organized documentation for the Unlook 3D Scanner project, including the gesture recognition system integration.

---

## 📚 Documentation Structure

### 🏗️ Architecture
High-level system design and architecture documents.

- **[System Overview](architecture/README_API.md)** - Complete API documentation
- **[Camera System](architecture/CAMERA_SYNC_SOLUTION.md)** - Camera synchronization architecture
- **[Camera Implementation](architecture/CAMERA_SYSTEM.md)** - Camera system implementation details
- **[API Implementation](architecture/API_IMPLEMENTATION_SUMMARY.md)** - API implementation summary
- **[GUI Architecture](architecture/GUI_CONVERSION_SUMMARY.md)** - GUI system architecture
- **[Point Cloud Export](architecture/PLY_EXPORT_IMPLEMENTATION_SUMMARY.md)** - PLY export implementation
- **[Gesture Recognition Analysis](architecture/GESTURE_RECOGNITION_ANALYSIS.md)** - Complete gesture recognition technical analysis

### 🔧 Hardware
Hardware setup, configuration, and integration guides.

- **[Hardware Setup Guide](hardware/HARDWARE_SETUP.md)** - Complete hardware setup instructions
- **[AS1170 CM5 Update](hardware/AS1170_CM5_UPDATE_SUMMARY.md)** - AS1170 controller CM5 updates
- **[AS1170 Debug System](hardware/AS1170_DEBUG_SYSTEM_SUMMARY.md)** - AS1170 debugging guide
- **[AS1170 Dual VCSEL](hardware/AS1170_DUAL_VCSEL_IMPLEMENTATION_SUMMARY.md)** - Dual VCSEL implementation
- **[Dual AS1170 Flood Configuration](hardware/DUAL_AS1170_FLOOD_CONFIGURATION.md)** - Flood illuminator config
- **[VCSEL Integration](hardware/VCSEL_INTEGRATION_SUMMARY.md)** - VCSEL system integration
- **[VCSEL Parameters](hardware/VCSEL_PARAMS_IMPLEMENTATION.md)** - VCSEL parameter configuration
- **[Dual VCSEL Quick Reference](hardware/DUAL_VCSEL_QUICK_REFERENCE.md)** - Quick reference guide
- **[Dual VCSEL Temporal Matching](hardware/DUAL_VCSEL_TEMPORAL_MATCHING.md)** - Temporal processing with VCSEL

### 💻 Implementation
Detailed implementation guides for specific features and algorithms.

- **[Ambient SGBM Fixes](implementation/AMBIENT_SGBM_FIXES.md)** - SGBM ambient light fixes
- **[Ambient Subtraction Fix](implementation/AMBIENT_SUBTRACTION_FIX.md)** - Ambient subtraction bug fixes
- **[Ambient Subtraction Implementation](implementation/AMBIENT_SUBTRACTION_IMPLEMENTATION.md)** - Complete ambient subtraction
- **[Frame Averaging](implementation/FRAME_AVERAGING_IMPLEMENTATION.md)** - Frame averaging implementation
- **[Stereo Optimization](implementation/STEREO_OPTIMIZATION_REPORT.md)** - Stereo matching optimization
- **[High Quality SGBM Parameters](implementation/HIGH_QUALITY_SGBM_PARAMS.md)** - Optimal SGBM parameters
- **[Temporal Stereo](implementation/TEMPORAL_STEREO_IMPLEMENTATION_COMPLETE.md)** - Temporal stereo processing
- **[Direct Disparity Fix](implementation/DIRECT_DISPARITY_FIX.md)** - Disparity computation fixes

### 🔍 Troubleshooting
Solutions to common problems and debugging guides.

- **[Troubleshooting Guide](troubleshooting/TROUBLESHOOTING.md)** - General troubleshooting
- **[Sync Fix Summary](troubleshooting/SYNC_FIX_SUMMARY.md)** - Camera synchronization fixes
- **[SBGGR10 Fix](troubleshooting/SBGGR10_FIX_SUMMARY.md)** - Bayer format handling fixes

### 👨‍💻 Development
Development setup, build system, and coding guidelines.

- **[Build System Overview](development/BUILD_SYSTEM_OVERVIEW.md)** - CMake build system guide
- **[Install Dependencies](development/INSTALL_DEPENDENCIES.md)** - Dependency installation
- **[Qt Design Studio Setup](development/QT_DESIGN_STUDIO_SETUP.md)** - Qt Designer setup
- **[Auto Exposure Optimization](development/AUTO_EXPOSURE_OPTIMIZATION.md)** - Camera auto-exposure
- **[ML Depth Refinement Research](development/ML_DEPTH_REFINEMENT_RESEARCH.md)** - ML research notes
- **[Codebase Refactoring Analysis](development/CODEBASE_REFACTORING_ANALYSIS.md)** - Complete refactoring analysis

### 📜 History
Historical notes, commit summaries, and session logs.

- **[Commit State Summary](history/COMMIT_STATE_SUMMARY.md)** - Git commit summaries
- **[Next Session Prompt](history/NEXT_SESSION_PROMPT.md)** - Session continuation notes

---

## 🚀 Quick Start

### For New Developers
1. Read the main **[README.md](../README.md)** in the repository root
2. Review **[PROJECT_GUIDELINES.md](../PROJECT_GUIDELINES.md)** for coding standards
3. Check **[CLAUDE.md](../CLAUDE.md)** for AI assistant guidance
4. Follow **[Hardware Setup Guide](hardware/HARDWARE_SETUP.md)**
5. Review **[Build System Overview](development/BUILD_SYSTEM_OVERVIEW.md)**

### For AI Assistants (Claude Code)
1. Start with **[CLAUDE.md](../CLAUDE.md)** for project-specific guidance
2. Reference **[PROJECT_GUIDELINES.md](../PROJECT_GUIDELINES.md)** for standards
3. Check architecture documents for system understanding
4. Use implementation guides for feature development

### For Gesture Recognition Development
1. **[Gesture Recognition Analysis](architecture/GESTURE_RECOGNITION_ANALYSIS.md)** - Complete technical analysis
2. **[Codebase Refactoring Analysis](development/CODEBASE_REFACTORING_ANALYSIS.md)** - Code organization
3. **[Camera System](architecture/CAMERA_SYSTEM.md)** - Existing camera integration
4. **[Hardware Setup](hardware/HARDWARE_SETUP.md)** - Hardware requirements

---

## 📝 Documentation Standards

### File Naming
- Use descriptive names with underscores: `FEATURE_NAME_DESCRIPTION.md`
- All caps for major documentation files
- Lowercase for technical details: `api-guide.md`

### Content Structure
- Start with Executive Summary or Overview
- Include Table of Contents for long documents
- Use clear headings and subheadings
- Add code examples where relevant
- Include diagrams for architecture

### Maintenance
- Update documentation when code changes
- Archive obsolete docs to `history/`
- Keep README.md index current
- Link related documents

---

## 🔗 External Resources

- **Main Repository**: [GitHub](https://github.com/SupernovaIndustries/unlook-standalone)
- **Website**: [Supernova Industries](https://supernovaindustries.it)
- **MediaPipe**: [Google MediaPipe](https://mediapipe.dev/)
- **ONNX Runtime**: [ONNX Runtime](https://onnxruntime.ai/)
- **OpenCV**: [OpenCV Documentation](https://docs.opencv.org/)
- **Qt5**: [Qt Documentation](https://doc.qt.io/)

---

**Last Updated**: 2025-10-13
**Maintainers**: Unlook Development Team
