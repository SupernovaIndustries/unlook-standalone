#!/usr/bin/env python3
import re

# Manually fix specific formatting TODOs
replacements = [
    # Pattern metrics (line 150)
    (r'UNLOOK_LOG_DEBUG\("TemporalStereoProcessor"\) << "Pattern metrics: variance=\{:\.1f\}, coverage=\{:\.2%\}" /\* TODO.*?\*/;',
     'UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Pattern metrics: variance=" << result.patternVariance << ", coverage=" << (result.coverageRatio * 100) << "%";'),

    # Pattern variance too low (line 154)
    (r'UNLOOK_LOG_WARNING\("TemporalStereoProcessor"\) << "Pattern variance too low: \{:\.1f\} < \{:\.1f\}" /\* TODO.*?\*/;',
     'UNLOOK_LOG_WARNING("TemporalStereoProcessor") << "Pattern variance too low: " << result.patternVariance << " < " << config_.isolationParams.minPatternVariance;'),

    # Coverage ratio out of range (lines 159-161)
    (r'UNLOOK_LOG_WARNING\("TemporalStereoProcessor"\) << "Coverage ratio out of range: \{:\.2%\} not in \[\{:\.2%\}, \{:\.2%\}\]" /\* TODO.*?\*/;',
     'UNLOOK_LOG_WARNING("TemporalStereoProcessor") << "Coverage ratio out of range: " << (result.coverageRatio * 100) << "% not in [" << (config_.isolationParams.minCoverage * 100) << "%, " << (config_.isolationParams.maxCoverage * 100) << "%]";'),

    # Using fused dual depth (line 238)
    (r'UNLOOK_LOG_DEBUG\("TemporalStereoProcessor"\) << "Using fused dual depth \(\{\}>\{\} valid pixels\)" /\* TODO.*?\*/;',
     'UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Using fused dual depth (" << validFused << ">" << validCombined << " valid pixels)";'),

    # Using combined pattern depth (line 241)
    (r'UNLOOK_LOG_DEBUG\("TemporalStereoProcessor"\) << "Using combined pattern depth \(\{\}>\{\} valid pixels\)" /\* TODO.*?\*/;',
     'UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Using combined pattern depth (" << validCombined << ">" << validFused << " valid pixels)";'),

    # Processing complete (line 277)
    (r'UNLOOK_LOG_INFO\("TemporalStereoProcessor"\) << "Processing complete: \{:\.1f\}ms total \(isolation: \{:\.1f\}ms, matching: \{:\.1f\}ms\)" /\* TODO.*?\*/;',
     'UNLOOK_LOG_INFO("TemporalStereoProcessor") << "Processing complete: " << result.totalTimeMs << "ms total (isolation: " << result.isolationTimeMs << "ms, matching: " << result.matchingTimeMs << "ms)";'),

    # Pattern isolation (line 361)
    (r'UNLOOK_LOG_DEBUG\("TemporalStereoProcessor"\) << "Pattern isolation: variance=\{:\.1f\}, mean=\{:\.1f\}, enhancement=\{:\.2f\}x, adaptive_threshold=\{:\.1f\}" /\* TODO.*?\*/;',
     'UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Pattern isolation: variance=" << variance << ", mean=" << mean[0] << ", enhancement=" << enhancementFactor << "x, adaptive_threshold=" << adaptiveThreshold;'),

    # Pattern combination (line 449)
    (r'UNLOOK_LOG_DEBUG\("TemporalStereoProcessor"\) << "Pattern combination: VCSEL1=\{:\.2%\}, VCSEL2=\{:\.2%\}, combined=\{:\.2%\}, improvement=\{:\.1%\}" /\* TODO.*?\*/;',
     'UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Pattern combination: VCSEL1=" << (coverage1 * 100) << "%, VCSEL2=" << (coverage2 * 100) << "%, combined=" << (coverageCombined * 100) << "%, improvement=" << (coverageImprovement * 100) << "%";'),

    # Debug mode enabled (line 648)
    (r'UNLOOK_LOG_INFO\("TemporalStereoProcessor"\) << "Debug mode enabled, output path: \{\}" /\* TODO.*?\*/;',
     'UNLOOK_LOG_INFO("TemporalStereoProcessor") << "Debug mode enabled, output path: " << outputPath;'),

    # Saved debug images (line 711)
    (r'UNLOOK_LOG_DEBUG\("TemporalStereoProcessor"\) << "Saved debug images to \{\}" /\* TODO.*?\*/;',
     'UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Saved debug images to " << prefix;'),

    # Invalid noise threshold (line 744)
    (r'UNLOOK_LOG_ERROR\("TemporalStereoProcessor"\) << "Invalid noise threshold: \{\}" /\* TODO.*?\*/;',
     'UNLOOK_LOG_ERROR("TemporalStereoProcessor") << "Invalid noise threshold: " << config.isolationParams.noiseThreshold;'),

    # Dual depth generation (line 836-838)
    (r'UNLOOK_LOG_DEBUG\("TemporalStereoProcessor"\)\s*<< "Dual depth generation: VCSEL1=\{:\.2%\} \(\{\}/\{\}\), VCSEL2=\{:\.2%\} \(\{\}/\{\}\)" /\* TODO.*?\*/;',
     'UNLOOK_LOG_DEBUG("TemporalStereoProcessor") << "Dual depth generation: VCSEL1=" << (static_cast<float>(valid1) / totalPixels * 100) << "% (" << valid1 << "/" << totalPixels << "), VCSEL2=" << (static_cast<float>(valid2) / totalPixels * 100) << "% (" << valid2 << "/" << totalPixels << ")";'),

    # Depth fusion complete (line 935-939)
    (r'UNLOOK_LOG_INFO\("TemporalStereoProcessor"\)\s*<< "Depth fusion complete: input1=\{:\.2%\}, input2=\{:\.2%\}, fused=\{:\.2%\}, gain=\{:\.1f\}x" /\* TODO.*?\*/;',
     'UNLOOK_LOG_INFO("TemporalStereoProcessor") << "Depth fusion complete: input1=" << (static_cast<float>(valid1) / totalPixels * 100) << "%, input2=" << (static_cast<float>(valid2) / totalPixels * 100) << "%, fused=" << (static_cast<float>(validFused) / totalPixels * 100) << "%, gain=" << fusionGain << "x";'),
]

# Read the file
with open('/home/alessandro/unlook-standalone/src/stereo/TemporalStereoProcessor.cpp', 'r') as f:
    content = f.read()

# Apply replacements
for pattern, replacement in replacements:
    content = re.sub(pattern, replacement, content, flags=re.MULTILINE | re.DOTALL)

# Write back
with open('/home/alessandro/unlook-standalone/src/stereo/TemporalStereoProcessor.cpp', 'w') as f:
    f.write(content)

print("Fixed all TODO format strings")