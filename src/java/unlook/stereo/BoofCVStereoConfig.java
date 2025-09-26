package unlook.stereo;

/**
 * Configuration class for BoofCV stereo matching operations
 * Mirrors the C++ BoofCVStereoConfig structure for JNI integration
 */
public class BoofCVStereoConfig {

    // Algorithm selection
    public int algorithm = 3;  // SUBPIXEL_SGM for highest precision
    public int errorType = 0;  // CENSUS transform

    // Core parameters
    public int minDisparity = 0;
    public int maxDisparity = 128;
    public int regionRadius = 3;

    // Quality parameters
    public boolean subpixelEnabled = true;
    public boolean leftRightValidation = true;
    public double maxError = 0.15;
    public double textureThreshold = 0.1;

    // Performance parameters
    public int numThreads = 4;
    public boolean useGpuAcceleration = false;

    // Industrial precision settings
    public double baselineMm = 70.017;  // From calibration
    public double focalLengthPx = 1200.0;
    public boolean enforcePrecisionTarget = true;

    public BoofCVStereoConfig() {
        // Default constructor with optimal settings for Unlook 3D Scanner
    }

    @Override
    public String toString() {
        return String.format("BoofCVStereoConfig{algorithm=%d, subpixel=%b, dispRange=[%d,%d]}",
                           algorithm, subpixelEnabled, minDisparity, maxDisparity);
    }
}