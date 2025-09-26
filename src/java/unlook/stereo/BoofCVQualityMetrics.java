package unlook.stereo;

/**
 * Quality metrics for BoofCV stereo matching results
 * Mirrors the C++ BoofCVQualityMetrics structure for JNI integration
 */
public class BoofCVQualityMetrics {

    public double validPixelPercentage = 0.0;
    public double meanError = 0.0;
    public double rmsError = 0.0;
    public double precisionMm = 0.0;
    public double processingTimeMs = 0.0;
    public boolean meetsPrecisionTarget = false;

    public BoofCVQualityMetrics() {
        // Default constructor
    }

    public BoofCVQualityMetrics(double validPixels, double precision, double timeMs) {
        this.validPixelPercentage = validPixels;
        this.precisionMm = precision;
        this.processingTimeMs = timeMs;
        this.meetsPrecisionTarget = (precision <= 0.005);  // 5 micrometers target
    }

    @Override
    public String toString() {
        return String.format("BoofCVQualityMetrics{valid=%.1f%%, precision=%.6fmm, time=%.1fms, target=%b}",
                           validPixelPercentage, precisionMm, processingTimeMs, meetsPrecisionTarget);
    }
}