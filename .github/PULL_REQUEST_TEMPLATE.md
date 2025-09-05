# Unlook 3D Scanner - Pull Request

## 📋 **Description**
Brief description of changes and why they were made.

## 🎯 **Type of Change**
- [ ] 🐛 Bug fix (non-breaking change which fixes an issue)
- [ ] ✨ New feature (non-breaking change which adds functionality)
- [ ] 💥 Breaking change (fix or feature that would cause existing functionality to not work as expected)
- [ ] 📚 Documentation update
- [ ] 🏗️ Build system changes
- [ ] 🧪 Tests addition/modification
- [ ] ♻️ Code refactoring (no functional changes)

## 🔧 **Component Areas**
Which system components are affected:
- [ ] Camera System (`src/camera/`)
- [ ] Stereo Processing (`src/stereo/`)
- [ ] Calibration System (`src/calibration/`)
- [ ] Touch Interface (`src/gui/`)
- [ ] Core API (`src/api/`)
- [ ] Hardware Integration (`src/hardware/`)
- [ ] Build System (`CMakeLists.txt`, `build.sh`)
- [ ] Third-party Dependencies (`third-party/`)

## 🧪 **Testing**
- [ ] **Unit tests** pass (`./build.sh --tests`)
- [ ] **Build validation** successful (`./build.sh --validate`)
- [ ] **Hardware testing** completed (if applicable)
- [ ] **Mock mode testing** completed
- [ ] **Cross-compilation** tested (if applicable)
- [ ] **Performance impact** assessed
- [ ] **Memory usage** verified (no leaks)

## 🖥️ **Interface Changes**
If GUI changes are included:
- [ ] Touch interface tested on actual touchscreen
- [ ] Supernova-tech design system followed
- [ ] Minimum 48px touch targets maintained
- [ ] ESC key functionality preserved
- [ ] Fullscreen behavior validated

## 🏗️ **Architecture Compliance**
- [ ] **C++17/20 standards** followed exclusively
- [ ] **No Python dependencies** introduced
- [ ] **Thread-safety** maintained where required
- [ ] **RAII principles** applied
- [ ] **Professional C++ patterns** used
- [ ] **API consistency** maintained
- [ ] **Namespace organization** followed

## 📊 **Performance Impact**
- **Build Time**: No change / Faster / Slower (explain)
- **Runtime Performance**: No change / Improved / Degraded (explain)
- **Memory Usage**: No change / Reduced / Increased (explain)
- **Hardware Requirements**: No change / Additional requirements (explain)

## 🔍 **Code Quality**
- [ ] Code follows project style guidelines
- [ ] Functions are well-documented
- [ ] Complex algorithms include references/explanations
- [ ] Error handling is comprehensive
- [ ] Resource management follows RAII
- [ ] Thread synchronization is correct (if applicable)

## 📋 **Hardware Compatibility**
- [ ] **Raspberry Pi CM4** compatibility maintained
- [ ] **Raspberry Pi CM5** optimization included (if applicable)
- [ ] **IMX296 cameras** functionality preserved
- [ ] **Hardware synchronization** not broken
- [ ] **Mock mode** still functional

## 📚 **Documentation**
- [ ] **Code comments** added/updated where necessary
- [ ] **API documentation** updated (if public API changed)
- [ ] **README.md** updated (if user-facing changes)
- [ ] **CLAUDE.md** updated (if development process changed)
- [ ] **Build instructions** updated (if build process changed)

## 🔄 **Migration Notes**
If this affects CM4→CM5 migration:
- [ ] Changes are backward compatible with CM4
- [ ] CM5-specific optimizations are optional
- [ ] Migration path is clearly documented
- [ ] No breaking changes for existing deployments

## 📝 **Additional Notes**
Any additional information that reviewers should know:

## 📑 **Checklist**
- [ ] I have performed a self-review of my own code
- [ ] I have commented my code, particularly in hard-to-understand areas
- [ ] I have made corresponding changes to the documentation
- [ ] My changes generate no new warnings or errors
- [ ] I have added tests that prove my fix is effective or that my feature works
- [ ] New and existing unit tests pass locally with my changes
- [ ] Any dependent changes have been merged and published

## 🔗 **Related Issues**
Closes #(issue number)
Related to #(issue number)