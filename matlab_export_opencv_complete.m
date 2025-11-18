%% ========================================================================
%% MATLAB STEREO CALIBRATION EXPORT FOR OPENCV 4.X - COMPLETE SOLUTION
%% ========================================================================
%%
%% Based on MathWorks official documentation:
%% - stereoParametersToOpenCV: https://www.mathworks.com/help/vision/ref/stereoparameterstoopencv.html
%% - rectifyStereoImages: https://www.mathworks.com/help/vision/ref/rectifystereoimages.html
%%
%% USAGE:
%% 1. Load your stereoParams in MATLAB workspace (from Stereo Camera Calibrator)
%% 2. Run this script
%% 3. Output: /unlook_calib/calib-matlab-complete-export.yaml
%%
%% Author: Alessandro (Unlook Project)
%% Date: 2025-11-17
%% ========================================================================

%% Check if stereoParams exists
if ~exist('stereoParams', 'var')
    error('stereoParams not found in workspace! Load calibration first.');
end

%% ========================================================================
%% STEP 1: Export base parameters with stereoParametersToOpenCV
%% ========================================================================
fprintf('==================================================\n');
fprintf('STEP 1: Exporting base parameters (K1, K2, D1, D2, R, T)\n');
fprintf('==================================================\n\n');

[cameraMatrix1, distortionCoefficients1, cameraMatrix2, distortionCoefficients2, ...
 rotationOfCamera2, translationOfCamera2, imageSize] = stereoParametersToOpenCV(stereoParams);

fprintf('✅ Base parameters extracted:\n');
fprintf('   Image size: %dx%d\n', imageSize(2), imageSize(1));
fprintf('   Baseline: %.6f mm\n', norm(translationOfCamera2));
fprintf('\n');

%% ========================================================================
%% STEP 2: Compute rectification with rectifyStereoImages
%% ========================================================================
fprintf('==================================================\n');
fprintf('STEP 2: Computing rectification (R1, R2, P1, P2, Q)\n');
fprintf('==================================================\n\n');

% Load first stereo pair for rectification computation
% NOTE: Replace these paths with your actual calibration images!
leftImagePath = '/unlook_calib_dataset/dataset_20251116_003544/left/0000.png';
rightImagePath = '/unlook_calib_dataset/dataset_20251116_003544/right/0000.png';

if ~isfile(leftImagePath) || ~isfile(rightImagePath)
    warning('Sample images not found! Using dummy images...');
    % Create dummy images with correct size
    leftImage = uint8(zeros(imageSize(1), imageSize(2)));
    rightImage = uint8(zeros(imageSize(1), imageSize(2)));
else
    leftImage = imread(leftImagePath);
    rightImage = imread(rightImagePath);
end

% Rectify images and extract ALL outputs (including Q matrix)
fprintf('Running rectifyStereoImages...\n');
[~, ~, reprojectionMatrix, camMatrix1Rect, camMatrix2Rect, R1, R2] = ...
    rectifyStereoImages(leftImage, rightImage, stereoParams, 'OutputView', 'full');

fprintf('✅ Rectification complete!\n\n');

% Display Q matrix structure (reprojectionMatrix)
fprintf('REPROJECTION MATRIX (Q) from MATLAB:\n');
disp(reprojectionMatrix);
fprintf('\n');

%% ========================================================================
%% STEP 3: Convert to OpenCV YAML format
%% ========================================================================
fprintf('==================================================\n');
fprintf('STEP 3: Exporting to OpenCV YAML format\n');
fprintf('==================================================\n\n');

outputFile = '/unlook_calib/calib-matlab-complete-export.yaml';
fid = fopen(outputFile, 'w');

% Write YAML header
fprintf(fid, '%%YAML:1.0\n');
fprintf(fid, '---\n');
fprintf(fid, 'calibration_date: "%s"\n', datestr(now, 'yyyy-mm-ddTHH:MM:SS'));
fprintf(fid, 'calibration_method: "MATLAB_rectifyStereoImages_COMPLETE_EXPORT"\n');
fprintf(fid, 'note: "Exported with official MATLAB stereoParametersToOpenCV + rectifyStereoImages"\n');
fprintf(fid, '\n');

% Image size
fprintf(fid, 'image_width: %d\n', imageSize(2));
fprintf(fid, 'image_height: %d\n', imageSize(1));
fprintf(fid, '\n');

% Camera matrix left (K1) - TRANSPOSE for OpenCV!
fprintf(fid, 'camera_matrix_left: !!opencv-matrix\n');
fprintf(fid, '   rows: 3\n');
fprintf(fid, '   cols: 3\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
K1_transposed = cameraMatrix1';  % CRITICAL: MATLAB is column-major, OpenCV is row-major!
for i = 1:numel(K1_transposed)
    fprintf(fid, '%.10e', K1_transposed(i));
    if i < numel(K1_transposed), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Distortion coefficients left (D1)
fprintf(fid, 'distortion_coeffs_left: !!opencv-matrix\n');
fprintf(fid, '   rows: %d\n', length(distortionCoefficients1));
fprintf(fid, '   cols: 1\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
for i = 1:length(distortionCoefficients1)
    fprintf(fid, '%.10e', distortionCoefficients1(i));
    if i < length(distortionCoefficients1), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Camera matrix right (K2) - TRANSPOSE!
fprintf(fid, 'camera_matrix_right: !!opencv-matrix\n');
fprintf(fid, '   rows: 3\n');
fprintf(fid, '   cols: 3\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
K2_transposed = cameraMatrix2';
for i = 1:numel(K2_transposed)
    fprintf(fid, '%.10e', K2_transposed(i));
    if i < numel(K2_transposed), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Distortion coefficients right (D2)
fprintf(fid, 'distortion_coeffs_right: !!opencv-matrix\n');
fprintf(fid, '   rows: %d\n', length(distortionCoefficients2));
fprintf(fid, '   cols: 1\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
for i = 1:length(distortionCoefficients2)
    fprintf(fid, '%.10e', distortionCoefficients2(i));
    if i < length(distortionCoefficients2), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Rotation matrix (R) - TRANSPOSE!
fprintf(fid, 'rotation_matrix: !!opencv-matrix\n');
fprintf(fid, '   rows: 3\n');
fprintf(fid, '   cols: 3\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
R_transposed = rotationOfCamera2';
for i = 1:numel(R_transposed)
    fprintf(fid, '%.10e', R_transposed(i));
    if i < numel(R_transposed), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Translation vector (T) - units in MM!
fprintf(fid, 'translation_vector: !!opencv-matrix\n');
fprintf(fid, '   rows: 3\n');
fprintf(fid, '   cols: 1\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
for i = 1:length(translationOfCamera2)
    fprintf(fid, '%.10e', translationOfCamera2(i));
    if i < length(translationOfCamera2), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Baseline
baseline_mm = norm(translationOfCamera2);
fprintf(fid, 'baseline_mm: %.10e\n\n', baseline_mm);

% Rectification transform left (R1) - TRANSPOSE!
fprintf(fid, 'rectification_transform_left: !!opencv-matrix\n');
fprintf(fid, '   rows: 3\n');
fprintf(fid, '   cols: 3\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
R1_transposed = R1';
for i = 1:numel(R1_transposed)
    fprintf(fid, '%.10e', R1_transposed(i));
    if i < numel(R1_transposed), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Rectification transform right (R2) - TRANSPOSE!
fprintf(fid, 'rectification_transform_right: !!opencv-matrix\n');
fprintf(fid, '   rows: 3\n');
fprintf(fid, '   cols: 3\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
R2_transposed = R2';
for i = 1:numel(R2_transposed)
    fprintf(fid, '%.10e', R2_transposed(i));
    if i < numel(R2_transposed), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Projection matrix left (P1) - from camMatrix1Rect
% camMatrix1Rect is already 3x4, TRANSPOSE for OpenCV!
fprintf(fid, 'projection_matrix_left: !!opencv-matrix\n');
fprintf(fid, '   rows: 3\n');
fprintf(fid, '   cols: 4\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
P1_transposed = camMatrix1Rect';
for i = 1:numel(P1_transposed)
    fprintf(fid, '%.10e', P1_transposed(i));
    if i < numel(P1_transposed), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Projection matrix right (P2) - from camMatrix2Rect
fprintf(fid, 'projection_matrix_right: !!opencv-matrix\n');
fprintf(fid, '   rows: 3\n');
fprintf(fid, '   cols: 4\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
P2_transposed = camMatrix2Rect';
for i = 1:numel(P2_transposed)
    fprintf(fid, '%.10e', P2_transposed(i));
    if i < numel(P2_transposed), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% CRITICAL: Disparity-to-depth matrix (Q) from reprojectionMatrix
% TRANSPOSE for OpenCV!
fprintf(fid, 'disparity_to_depth_matrix: !!opencv-matrix\n');
fprintf(fid, '   rows: 4\n');
fprintf(fid, '   cols: 4\n');
fprintf(fid, '   dt: d\n');
fprintf(fid, '   data: [ ');
Q_transposed = reprojectionMatrix';
for i = 1:numel(Q_transposed)
    fprintf(fid, '%.10e', Q_transposed(i));
    if i < numel(Q_transposed), fprintf(fid, ', '); end
end
fprintf(fid, ' ]\n\n');

% Quality metrics
fprintf(fid, 'rms_reprojection_error: %.10e\n', 0.0);  % Add if available
fprintf(fid, 'opencv_version: "4.6.0"\n');

fclose(fid);

fprintf('✅ Calibration exported to: %s\n\n', outputFile);

%% ========================================================================
%% STEP 4: Verification and summary
%% ========================================================================
fprintf('==================================================\n');
fprintf('EXPORT SUMMARY\n');
fprintf('==================================================\n\n');
fprintf('Output file: %s\n\n', outputFile);
fprintf('Q Matrix (reprojectionMatrix) structure:\n');
fprintf('  Q(1,4) = -cx = %.4f\n', reprojectionMatrix(1,4));
fprintf('  Q(2,4) = -cy = %.4f\n', reprojectionMatrix(2,4));
fprintf('  Q(3,4) = f   = %.4f\n', reprojectionMatrix(3,4));
fprintf('  Q(4,3) = -1/baseline = %.8f\n', reprojectionMatrix(4,3));
fprintf('  Baseline computed: %.4f mm\n', -1/reprojectionMatrix(4,3));
fprintf('\n');
fprintf('⚠️  IMPORTANT NOTES:\n');
fprintf('1. ALL matrices were TRANSPOSED (MATLAB col-major → OpenCV row-major)\n');
fprintf('2. Translation vector is in MILLIMETERS (not meters)\n');
fprintf('3. Q matrix from MATLAB rectifyStereoImages (not OpenCV stereoRectify)\n');
fprintf('4. Use with OpenCV reprojectImageTo3D(disparity, points3D, Q, ...)\n');
fprintf('\n');
fprintf('Next steps:\n');
fprintf('1. ln -sf calib-matlab-complete-export.yaml /unlook_calib/default.yaml\n');
fprintf('2. Test scan with: unlook\n');
fprintf('\n');
fprintf('==================================================\n');
