/**
 * @file BoofCVStereoConfig.java
 * @brief Configuration class for BoofCV stereo matching with industrial precision
 * 
 * @author Unlook 3D Scanner Team
 * @date 2025-01-15
 */

package unlook.stereo;

/**
 * Configuration parameters for BoofCV stereo matching
 * Optimized for 70mm baseline and 0.005mm precision target
 */
public class BoofCVStereoConfig implements Cloneable {
    
    // Algorithm selection
    public int algorithm = 3; // SUBPIXEL_SGM for highest precision
    public int errorType = 0; // CENSUS for robust matching
    
    // Core disparity parameters
    public int minDisparity = 0;
    public int maxDisparity = 128;
    public int regionRadius = 3; // Matching region radius (3-7 optimal)
    
    // Quality parameters
    public boolean subpixelEnabled = true; // Enable sub-pixel precision
    public boolean leftRightValidation = true; // Left-right consistency check
    public double maxError = 0.15; // Maximum per-pixel error threshold
    public double textureThreshold = 0.1; // Texture threshold for valid matches
    
    // Performance parameters
    public int numThreads = 4; // Number of processing threads
    public boolean useGpuAcceleration = false; // GPU acceleration (if available)
    
    // Industrial precision settings (70mm baseline configuration)
    public double baselineMm = 70.017; // Stereo baseline in mm (from calibration)
    public double focalLengthPx = 1200.0; // Focal length in pixels
    public boolean enforcePrecisionTarget = true; // Enforce 0.005mm target
    
    /**
     * Default constructor with optimal settings for 0.005mm precision
     */
    public BoofCVStereoConfig() {
        // Configuration optimized for 70mm baseline industrial scanning
        this.algorithm = 3; // SUBPIXEL_SGM
        this.errorType = 0; // CENSUS
        this.minDisparity = 0;
        this.maxDisparity = 160; // Covers depth range for 70mm baseline
        this.regionRadius = 3; // Optimal for precision vs speed
        this.subpixelEnabled = true;
        this.leftRightValidation = true;
        this.maxError = 0.15;
        this.textureThreshold = 0.1;
        this.numThreads = 4;
        this.useGpuAcceleration = false;
        this.baselineMm = 70.017; // From calib_boofcv_test3.yaml
        this.focalLengthPx = 1200.0; // Approximate for 6mm lens
        this.enforcePrecisionTarget = true;
    }
    
    /**
     * Constructor with custom algorithm and precision settings
     */
    public BoofCVStereoConfig(int algorithm, boolean subpixelEnabled, double baselineMm) {
        this(); // Call default constructor
        this.algorithm = algorithm;
        this.subpixelEnabled = subpixelEnabled;
        this.baselineMm = baselineMm;
    }
    
    /**
     * Create a deep copy of the configuration
     */
    @Override
    public BoofCVStereoConfig clone() {
        try {
            BoofCVStereoConfig cloned = (BoofCVStereoConfig) super.clone();
            return cloned;
        } catch (CloneNotSupportedException e) {
            // This should never happen since we implement Cloneable
            throw new RuntimeException("Failed to clone BoofCVStereoConfig", e);
        }
    }
    
    /**
     * Get algorithm name as string
     */
    public String getAlgorithmName() {
        switch (algorithm) {
            case 0: return "Dense Block Matching";
            case 1: return "Dense Semi-Global Matching";
            case 2: return "Sub-pixel Block Matching";
            case 3: return "Sub-pixel Semi-Global Matching";
            default: return "Unknown Algorithm";
        }
    }
    
    /**
     * Get error type name as string
     */
    public String getErrorTypeName() {
        switch (errorType) {
            case 0: return "CENSUS Transform";
            case 1: return "Normalized Cross Correlation";
            case 2: return "Sum of Absolute Differences";
            case 3: return "Sum of Squared Differences";
            default: return "Unknown Error Type";
        }
    }
    
    /**
     * Validate configuration parameters
     */
    public boolean isValid() {
        if (minDisparity < 0 || maxDisparity <= minDisparity) {
            return false;
        }
        if (regionRadius < 1 || regionRadius > 10) {
            return false;
        }
        if (baselineMm <= 0 || focalLengthPx <= 0) {
            return false;
        }
        if (numThreads < 1 || numThreads > 16) {
            return false;
        }
        return true;
    }
    
    /**
     * Get configuration summary as string
     */
    @Override
    public String toString() {
        return String.format("BoofCVStereoConfig{algorithm=%s, subpixel=%s, baseline=%.3fmm, " +
                           "disparity=[%d,%d], region=%d, precision_target=%s}",
                           getAlgorithmName(), subpixelEnabled, baselineMm,
                           minDisparity, maxDisparity, regionRadius, enforcePrecisionTarget);
    }
}