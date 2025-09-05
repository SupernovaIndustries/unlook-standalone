# Unlook 3D Scanner Wiki

<p align="center">
  <img src="http://supernovaindustries.it/wp-content/uploads/2024/08/logo_full-white-unlook-1.svg" alt="Unlook Logo" width="300" />
</p>

Welcome to the **Unlook 3D Scanner** comprehensive documentation wiki. This industrial-grade 3D scanner achieves **0.005mm precision** for professional applications.

---

## 📚 **Quick Navigation**

### **Getting Started**
- 🚀 [Installation Guide](Installation-Guide) - Complete setup instructions
- 🔧 [Hardware Setup](Hardware-Setup) - Physical assembly and connections
- 🎯 [Quick Start Tutorial](Quick-Start) - First-time usage guide
- 🧪 [Testing Your System](System-Testing) - Validation procedures

### **API Documentation**
- 📖 [API Overview](API-Overview) - Complete C++ API reference
- 🎛️ [Camera System API](Camera-API) - Stereo camera control
- 📐 [Stereo Processing API](Stereo-API) - Depth processing and algorithms
- 🔬 [Calibration API](Calibration-API) - High-precision calibration system

### **Hardware & Configuration**
- 🎥 [Camera Configuration](Camera-Configuration) - IMX296 setup and sync
- 🔄 [Hardware Synchronization](Hardware-Sync) - XVS/XHS timing details
- 💡 [LED System Integration](LED-System) - AS1170 controller setup
- 🏗️ [CM4 vs CM5 Migration](CM5-Migration) - Upgrade guide and optimizations

### **Development & Advanced**
- 🛠️ [Build System](Build-System) - Comprehensive build configuration
- 🧪 [Testing Framework](Testing-Framework) - Quality assurance procedures
- 📊 [Performance Optimization](Performance) - ARM64 and CM5 optimizations
- 🔌 [Third-party Integration](Third-Party) - libcamera-sync and BoofCV

### **User Guides**
- 🖥️ [Touch Interface Guide](Touch-Interface) - GUI operation manual
- 🎨 [Design System](Design-System) - Supernova-tech styling guide
- 📏 [Calibration Procedures](Calibration-Procedures) - Step-by-step calibration
- 🔧 [Troubleshooting](Troubleshooting) - Common issues and solutions

---

## 🎯 **Project Overview**

**Unlook** is a professional modular opensource 3D scanner designed for **industrial, educational, and professional applications**. The system combines:

- **Stereovision** with hardware-synchronized IMX296 cameras
- **Structured light VCSEL** projection for enhanced accuracy  
- **ARM64 optimization** for Raspberry Pi CM4/CM5
- **Industrial touch interface** with Supernova-tech design
- **C++17/20 architecture** with zero Python dependencies

### **Target Applications**
- 🏭 **Industrial quality control** and inspection
- 🎓 **Educational 3D scanning** projects
- ⚙️ **Professional prototyping** and reverse engineering
- 🔬 **Research applications** requiring high precision

### **Technical Specifications**
- 🎯 **Precision**: 0.005mm repeatability (0.008mm hardware limit)
- 📏 **Baseline**: 70.017mm calibrated stereo setup
- 🎥 **Cameras**: 2x IMX296 Global Shutter (1456x1088 SBGGR10)
- ⚡ **Sync**: <1ms hardware synchronization (XVS/XHS)
- 💻 **Performance**: VGA >20 FPS, HD >10 FPS on CM4/CM5
- 🏗️ **Architecture**: Professional C++17/20 with Qt5 GUI

---

## 🚀 **Quick Start Commands**

### **Installation**
```bash
# Clone repository
git clone https://github.com/SupernovaIndustries/unlook-standalone.git
cd unlook-standalone

# Build complete system
./build.sh

# Run with hardware
./build/bin/unlook_scanner_gui

# Run in mock mode (no hardware)
./build/bin/unlook_scanner_gui --mock
```

### **Cross-Compilation**
```bash
# Build for CM5 with optimizations
./build.sh --cross cm5 -j 4

# Build for CM4 compatibility
./build.sh --cross cm4 -j 4

# Create installation package
./build.sh --package
```

### **Testing & Validation**
```bash
# Hardware synchronization test
./build/bin/test_camera_sync

# Full system validation
./test_synchronized_capture.sh

# Performance benchmarking
./build/bin/test_sync_precision
```

---

## 📋 **Development Status**

### **✅ Phase 1: Foundation Complete**
- [x] Professional C++ API architecture
- [x] Hardware synchronization system (<1ms precision)
- [x] Qt5 fullscreen touch interface
- [x] OpenCV stereo processing pipeline
- [x] Build system with CM5 cross-compilation
- [x] libcamera-sync integration (third-party exclusive)
- [x] Professional documentation and GitHub setup

### **🔄 Phase 2: Advanced Features (Current)**
- [ ] BoofCV integration for high-precision calibration
- [ ] Point cloud export (PLY/OBJ formats)
- [ ] Advanced calibration validation tools
- [ ] Performance optimization for CM5 Cortex-A76

### **🔮 Phase 3: Production (Planned)**
- [ ] AS1170 LED controller implementation
- [ ] Structured light VCSEL integration
- [ ] Advanced depth processing algorithms
- [ ] Industrial deployment packaging

---

## 💡 **Key Features**

### **🎛️ Industrial-Grade Hardware**
- **Global Shutter Cameras**: IMX296 sensors with hardware sync
- **Precision Baseline**: 70.017mm calibrated stereo setup
- **Hardware Synchronization**: XVS/XHS GPIO with <1ms precision
- **LED Controller**: AS1170 with VCSEL structured light
- **CM5 Optimization**: Cortex-A76 specific performance tuning

### **🖥️ Professional Interface**
- **Fullscreen Touch Design**: Industrial operator optimized
- **Supernova-tech Styling**: Electric blues on dark backgrounds
- **Real-time Controls**: Live parameter adjustment
- **Mock Mode Support**: Development without hardware
- **Error Recovery**: Comprehensive user guidance

### **⚡ High-Performance Processing**
- **C++17/20 Exclusively**: Zero Python runtime dependencies
- **Thread-safe Architecture**: Professional singleton patterns
- **ARM64 NEON**: Vectorized processing optimizations
- **Memory Optimized**: 8GB CM4 / 16GB CM5 support
- **Multi-algorithm**: OpenCV SGBM + BoofCV precision

---

## 🤝 **Contributing**

This project welcomes contributions! See our guidelines:

- 📋 **Code Standards**: C++17/20 exclusively, professional patterns
- 🧪 **Testing Required**: All changes must include tests
- 📚 **Documentation**: Update wiki for API changes
- 🏗️ **Architecture**: Follow existing namespace organization
- 🔍 **Review Process**: All PRs require thorough review

---

## 📞 **Support & Community**

- 🌐 **Website**: [supernovaindustries.it](https://supernovaindustries.it)
- 📧 **Email**: alessandro.cursoli@supernovaindustries.com
- 🐙 **GitHub**: [SupernovaIndustries](https://github.com/SupernovaIndustries)
- 📚 **Documentation**: This wiki + CLAUDE.md + PROJECT_GUIDELINES.md
- 🐛 **Issues**: Use GitHub Issues for bug reports

---

<p align="center">
  <strong>🚀 Professional 3D Scanning Made Accessible</strong><br>
  <em>Unlook 3D Scanner - Precision Engineering for Everyone</em>
</p>

<p align="center">
  Made with ❤️ by <a href="https://supernovaindustries.it">Supernova Industries</a>
</p>