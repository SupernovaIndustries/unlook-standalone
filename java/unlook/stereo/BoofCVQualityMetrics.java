/**
 * @file BoofCVQualityMetrics.java
 * @brief Quality metrics for BoofCV stereo matching results with precision validation
 * 
 * @author Unlook 3D Scanner Team
 * @date 2025-01-15
 */

package unlook.stereo;

/**
 * Quality metrics and precision assessment for BoofCV stereo matching results
 * Includes validation against 0.005mm industrial precision target
 */
public class BoofCVQualityMetrics {
    
    // Coverage and validity metrics
    public double validPixelPercentage = 0.0;    // Percentage of valid disparity pixels
    
    // Error metrics
    public double meanError = 0.0;               // Mean matching error
    public double rmsError = 0.0;                // RMS error for quality assessment
    
    // Precision metrics (critical for industrial applications)
    public double precisionMm = 0.0;             // Estimated precision in millimeters
    public boolean meetsPrecisionTarget = false; // True if meets 0.005mm target
    
    // Performance metrics
    public double processingTimeMs = 0.0;        // Processing time in milliseconds
    
    // Additional quality indicators
    public double confidenceScore = 0.0;         // Overall confidence in results
    public double noiseLevel = 0.0;              // Estimated noise level
    public double textureQuality = 0.0;          // Texture richness assessment
    
    /**
     * Default constructor
     */
    public BoofCVQualityMetrics() {
        // Initialize with default (invalid) values
        this.validPixelPercentage = 0.0;
        this.meanError = 0.0;
        this.rmsError = 0.0;
        this.precisionMm = 999.0; // High value indicates no valid measurement
        this.meetsPrecisionTarget = false;
        this.processingTimeMs = 0.0;
        this.confidenceScore = 0.0;
        this.noiseLevel = 0.0;
        this.textureQuality = 0.0;
    }
    
    /**
     * Constructor with basic metrics
     */
    public BoofCVQualityMetrics(double validPixelPercentage, double precisionMm, 
                               double processingTimeMs, boolean meetsPrecisionTarget) {
        this();
        this.validPixelPercentage = validPixelPercentage;
        this.precisionMm = precisionMm;
        this.processingTimeMs = processingTimeMs;
        this.meetsPrecisionTarget = meetsPrecisionTarget;
    }
    
    /**
     * Check if the results meet industrial quality standards
     */
    public boolean meetsIndustrialStandards() {
        return validPixelPercentage >= 70.0 &&  // At least 70% coverage
               precisionMm <= 0.005 &&          // Meet precision target
               rmsError <= 0.5 &&               // Low RMS error
               confidenceScore >= 0.8;          // High confidence
    }
    
    /**
     * Get quality grade as string
     */
    public String getQualityGrade() {
        if (meetsIndustrialStandards()) {
            return "INDUSTRIAL";
        } else if (precisionMm <= 0.01 && validPixelPercentage >= 50.0) {
            return "GOOD";
        } else if (precisionMm <= 0.05 && validPixelPercentage >= 30.0) {
            return "ACCEPTABLE";
        } else {
            return "POOR";
        }
    }
    
    /**
     * Calculate overall quality score (0-100)
     */
    public double getQualityScore() {
        double coverageScore = Math.min(validPixelPercentage, 100.0);
        double precisionScore = precisionMm <= 0.005 ? 100.0 : 
                               Math.max(0, 100.0 - (precisionMm - 0.005) * 10000);
        double errorScore = rmsError <= 0.2 ? 100.0 : 
                           Math.max(0, 100.0 - (rmsError - 0.2) * 200);
        
        return (coverageScore + precisionScore + errorScore) / 3.0;
    }
    
    /**
     * Check if processing was fast enough for real-time use
     */
    public boolean isRealTimeCapable(double targetFps) {
        double maxProcessingTime = 1000.0 / targetFps; // ms per frame
        return processingTimeMs <= maxProcessingTime;
    }
    
    /**
     * Get detailed quality report as string
     */
    public String getDetailedReport() {
        StringBuilder report = new StringBuilder();
        report.append("BoofCV Quality Metrics Report\n");
        report.append("============================\n");
        report.append(String.format("Coverage:           %.1f%%\n", validPixelPercentage));
        report.append(String.format("Precision:          %.6f mm\n", precisionMm));
        report.append(String.format("Precision Target:   %s (≤ 0.005mm)\n", 
                                   meetsPrecisionTarget ? "ACHIEVED" : "NOT MET"));
        report.append(String.format("RMS Error:          %.3f pixels\n", rmsError));
        report.append(String.format("Processing Time:    %.1f ms\n", processingTimeMs));
        report.append(String.format("Quality Grade:      %s\n", getQualityGrade()));
        report.append(String.format("Quality Score:      %.1f/100\n", getQualityScore()));
        report.append(String.format("Industrial Standard: %s\n", 
                                   meetsIndustrialStandards() ? "YES" : "NO"));
        
        if (meetsPrecisionTarget) {
            report.append("\n🎯 INDUSTRIAL PRECISION TARGET ACHIEVED!\n");
        } else {
            report.append(String.format("\n⚠️  Precision %.6fmm exceeds target 0.005mm\n", precisionMm));
        }
        
        return report.toString();
    }
    
    /**
     * Get compact summary string
     */
    @Override
    public String toString() {
        return String.format("BoofCVMetrics{coverage=%.1f%%, precision=%.6fmm, " +
                           "target=%s, time=%.1fms, grade=%s}",
                           validPixelPercentage, precisionMm, 
                           meetsPrecisionTarget ? "MET" : "MISSED",
                           processingTimeMs, getQualityGrade());
    }
    
    /**
     * Create metrics for error cases
     */
    public static BoofCVQualityMetrics createErrorMetrics(String errorMessage, double processingTime) {
        BoofCVQualityMetrics metrics = new BoofCVQualityMetrics();
        metrics.validPixelPercentage = 0.0;
        metrics.precisionMm = 999.0;
        metrics.meetsPrecisionTarget = false;
        metrics.processingTimeMs = processingTime;
        metrics.confidenceScore = 0.0;
        return metrics;
    }
    
    /**
     * Create metrics for perfect results (testing purposes)
     */
    public static BoofCVQualityMetrics createPerfectMetrics() {
        BoofCVQualityMetrics metrics = new BoofCVQualityMetrics();
        metrics.validPixelPercentage = 95.0;
        metrics.meanError = 0.1;
        metrics.rmsError = 0.15;
        metrics.precisionMm = 0.003; // Better than target
        metrics.meetsPrecisionTarget = true;
        metrics.processingTimeMs = 50.0;
        metrics.confidenceScore = 0.95;
        metrics.noiseLevel = 0.05;
        metrics.textureQuality = 0.8;
        return metrics;
    }
}