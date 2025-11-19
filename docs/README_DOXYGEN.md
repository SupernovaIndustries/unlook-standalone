# Unlook API Documentation

Professional API documentation generated with Doxygen.

## Generating Documentation

### Prerequisites

```bash
sudo apt-get install doxygen graphviz
```

### Generate HTML Documentation

From the project root directory:

```bash
doxygen Doxyfile
```

This will generate documentation in `docs/html/` directory.

### View Documentation

Open the main page in your browser:

```bash
# On Raspberry Pi with desktop
firefox docs/html/index.html

# Or use any browser
xdg-open docs/html/index.html
```

## Documentation Features

The generated documentation includes:

### Class Documentation
- **Complete class hierarchy** with inheritance graphs
- **Detailed method documentation** with parameters and return values
- **Member variable documentation**
- **Usage examples** where available

### Module Organization
Documentation is organized by namespace:
- `unlook::api` - Public API (UnlookScanner, CameraSystem, DepthProcessor)
- `unlook::core` - Core functionality (Logger, Configuration, Exceptions)
- `unlook::camera` - Camera management and hardware sync
- `unlook::stereo` - Stereo matching algorithms (SGM-Census)
- `unlook::calibration` - Calibration loading and validation
- `unlook::hardware` - I2C, GPIO, LED control (AS1170)
- `unlook::pointcloud` - Point cloud processing (Open3D)
- `unlook::mesh` - Mesh generation and optimization
- `unlook::gui` - Qt5 graphical interface

### Diagrams
- **Class diagrams** showing relationships
- **Inheritance diagrams** for class hierarchies
- **Collaboration diagrams** showing class interactions
- **Include dependency graphs**
- **Call graphs** (when enabled)

### Search Functionality
- Full-text search across all documentation
- Class/function name search
- File search

## Customizing Documentation

Edit `Doxyfile` in the project root to customize:

- `PROJECT_NUMBER` - Version number
- `OUTPUT_DIRECTORY` - Where to generate docs (default: `docs/`)
- `EXTRACT_PRIVATE` - Include private members (default: NO)
- `CALL_GRAPH` - Generate call graphs (default: NO, slow)
- `CALLER_GRAPH` - Generate caller graphs (default: NO, slow)

## Documentation Standards

When writing code, follow Doxygen comment style:

### Header File Example

```cpp
/**
 * @file StereoMatcher.hpp
 * @brief Abstract base class for stereo matching algorithms
 * @author Unlook Team
 * @date 2025-11-19
 */

namespace unlook {
namespace stereo {

/**
 * @class StereoMatcher
 * @brief Abstract interface for stereo matching algorithms
 *
 * Provides a common interface for different stereo matching
 * implementations (SGBM, SGM-Census, etc.)
 *
 * @note All derived classes must implement compute()
 */
class StereoMatcher {
public:
    /**
     * @brief Compute disparity map from stereo pair
     *
     * @param leftImage Left rectified image (CV_8UC1)
     * @param rightImage Right rectified image (CV_8UC1)
     * @return Disparity map (CV_16SC1, scaled by 16)
     *
     * @throws std::invalid_argument if images are not grayscale
     * @throws std::runtime_error if computation fails
     *
     * @note Images must be rectified before calling
     * @see cv::stereoRectify() for rectification
     */
    virtual cv::Mat compute(
        const cv::Mat& leftImage,
        const cv::Mat& rightImage) = 0;

    /**
     * @brief Set disparity range
     * @param minDisparity Minimum disparity (usually 0)
     * @param numDisparities Number of disparities (must be divisible by 16)
     */
    void setDisparityRange(int minDisparity, int numDisparities);

private:
    int minDisparity_;     ///< Minimum disparity value
    int numDisparities_;   ///< Number of disparity levels
};

} // namespace stereo
} // namespace unlook
```

### Source File Example

```cpp
/**
 * @file StereoMatcher.cpp
 * @brief Implementation of StereoMatcher base class
 */

#include <unlook/stereo/StereoMatcher.hpp>

namespace unlook {
namespace stereo {

void StereoMatcher::setDisparityRange(int minDisp, int numDisp) {
    if (numDisp % 16 != 0) {
        throw std::invalid_argument(
            "numDisparities must be divisible by 16");
    }
    minDisparity_ = minDisp;
    numDisparities_ = numDisp;
}

} // namespace stereo
} // namespace unlook
```

## Special Documentation Tags

Doxygen supports many special commands:

- `@brief` - Short description
- `@param` - Parameter description
- `@return` - Return value description
- `@throws` - Exception that may be thrown
- `@note` - Important note
- `@warning` - Warning message
- `@see` - Cross-reference
- `@example` - Code example
- `@code` ... `@endcode` - Code block
- `@deprecated` - Mark as deprecated
- `@todo` - TODO item

## Publishing Documentation

### For GitHub Pages

1. Generate documentation:
   ```bash
   doxygen Doxyfile
   ```

2. Copy to gh-pages branch:
   ```bash
   git checkout gh-pages
   cp -r docs/html/* .
   git add .
   git commit -m "Update API documentation"
   git push origin gh-pages
   ```

3. Enable GitHub Pages in repository settings

### For Local Viewing

Simply open `docs/html/index.html` in any modern browser.

## Maintenance

### Updating Documentation

Documentation should be regenerated:
- After adding new classes/functions
- After modifying public APIs
- Before major releases
- When preparing presentations/demos

### Clean Build

To regenerate from scratch:

```bash
rm -rf docs/html docs/latex
doxygen Doxyfile
```

## Troubleshooting

### Missing Graphs

Install Graphviz:
```bash
sudo apt-get install graphviz
```

### Slow Generation

Disable call graphs in `Doxyfile`:
```
CALL_GRAPH = NO
CALLER_GRAPH = NO
```

### Missing Classes

Check that:
1. Header files are in `include/unlook/`
2. Files match `FILE_PATTERNS` in `Doxyfile`
3. Files are not in `EXCLUDE_PATTERNS`

---

**For more information**: https://www.doxygen.nl/manual/index.html
