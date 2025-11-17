/**
 * @file recalculate_q_from_matlab.cpp
 * @brief Ricalcola R1, R2, P1, P2, Q da calibrazione MATLAB usando OpenCV stereoRectify
 *
 * PROBLEMA IDENTIFICATO:
 * - La calibrazione MATLAB ha R1, R2, P1, P2, Q pre-calcolati da MATLAB
 * - MATLAB usa CONVENZIONE DIVERSA da OpenCV (Q(2,2)=0, Q(3,2) negativo)
 * - OpenCV reprojectImageTo3D richiede Q con convenzione OpenCV
 *
 * SOLUZIONE:
 * - Carica SOLO parametri base: K1, K2, D1, D2, R, T (da MATLAB)
 * - IGNORA R1, R2, P1, P2, Q pre-calcolati da MATLAB
 * - Ricalcola con OpenCV stereoRectify() → Q matrix corretta!
 *
 * WORKFLOW (da repository JinyongJeong/StereoCalibration_matlab2opencv):
 * 1. MATLAB esporta K1, K2, D1, D2, R, T
 * 2. OpenCV stereoRectify() calcola R1, R2, P1, P2, Q (NATIVO OpenCV!)
 * 3. Export nuovo YAML con Q corretta
 *
 * @author Alessandro (Unlook Project)
 * @date 2025-11-17
 */

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <iomanip>

int main(int argc, char** argv) {
    std::cout << std::fixed << std::setprecision(10);
    std::cout << "========================================" << std::endl;
    std::cout << "RECALCULATE Q MATRIX FROM MATLAB CALIBRATION" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Input file (calibrazione MATLAB)
    std::string inputFile = "/unlook_calib/calib-matlab-final-opencv46.yaml";
    if (argc > 1) {
        inputFile = argv[1];
    }

    std::cout << "📂 Input calibrazione MATLAB: " << inputFile << std::endl;
    std::cout << std::endl;

    // Carica SOLO parametri base (K1, K2, D1, D2, R, T)
    cv::FileStorage fs(inputFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "❌ ERRORE: Impossibile aprire " << inputFile << std::endl;
        return 1;
    }

    cv::Mat K1, K2, D1, D2, R, T;
    int width, height;

    fs["camera_matrix_left"] >> K1;
    fs["camera_matrix_right"] >> K2;
    fs["distortion_coeffs_left"] >> D1;
    fs["distortion_coeffs_right"] >> D2;
    fs["rotation_matrix"] >> R;
    fs["translation_vector"] >> T;
    width = fs["image_width"];
    height = fs["image_height"];

    // IMPORTANTE: Carica anche i parametri MATLAB pre-calcolati per confronto
    cv::Mat R1_matlab, R2_matlab, P1_matlab, P2_matlab, Q_matlab;
    fs["rectification_transform_left"] >> R1_matlab;
    fs["rectification_transform_right"] >> R2_matlab;
    fs["projection_matrix_left"] >> P1_matlab;
    fs["projection_matrix_right"] >> P2_matlab;
    fs["disparity_to_depth_matrix"] >> Q_matlab;

    fs.release();

    std::cout << "✅ Parametri base caricati:" << std::endl;
    std::cout << "   Image size: " << width << "x" << height << std::endl;
    std::cout << "   K1 (3x3): " << K1.at<double>(0,0) << ", " << K1.at<double>(1,1) << ", cx=" << K1.at<double>(0,2) << ", cy=" << K1.at<double>(1,2) << std::endl;
    std::cout << "   K2 (3x3): " << K2.at<double>(0,0) << ", " << K2.at<double>(1,1) << ", cx=" << K2.at<double>(0,2) << ", cy=" << K2.at<double>(1,2) << std::endl;
    std::cout << "   D1 (5x1): " << D1.at<double>(0) << ", " << D1.at<double>(1) << ", " << D1.at<double>(2) << ", " << D1.at<double>(3) << ", " << D1.at<double>(4) << std::endl;
    std::cout << "   D2 (5x1): " << D2.at<double>(0) << ", " << D2.at<double>(1) << ", " << D2.at<double>(2) << ", " << D2.at<double>(3) << ", " << D2.at<double>(4) << std::endl;
    std::cout << "   Baseline: " << cv::norm(T) << " mm" << std::endl;
    std::cout << std::endl;

    // RICALCOLA con OpenCV stereoRectify (NATIVO)
    std::cout << "🔄 Ricalcolo R1, R2, P1, P2, Q con OpenCV stereoRectify()..." << std::endl;
    std::cout << std::endl;

    cv::Mat R1_opencv, R2_opencv, P1_opencv, P2_opencv, Q_opencv;
    cv::Rect validRoi1, validRoi2;

    cv::stereoRectify(
        K1, D1,                    // Camera 1 intrinsics + distortion
        K2, D2,                    // Camera 2 intrinsics + distortion
        cv::Size(width, height),   // Image size
        R,                         // Rotation between cameras
        T,                         // Translation between cameras
        R1_opencv,                 // OUTPUT: Rectification rotation camera 1
        R2_opencv,                 // OUTPUT: Rectification rotation camera 2
        P1_opencv,                 // OUTPUT: Projection matrix camera 1
        P2_opencv,                 // OUTPUT: Projection matrix camera 2
        Q_opencv,                  // OUTPUT: Disparity-to-depth matrix (CRITICAL!)
        cv::CALIB_ZERO_DISPARITY,  // Flag: align principal points
        -1,                        // alpha: -1 = auto scaling
        cv::Size(width, height),   // newImageSize: same as input
        &validRoi1,                // Valid ROI camera 1
        &validRoi2                 // Valid ROI camera 2
    );

    std::cout << "✅ OpenCV stereoRectify() completato!" << std::endl;
    std::cout << std::endl;

    // ========================================
    // CONFRONTO Q MATRIX: MATLAB vs OPENCV
    // ========================================
    std::cout << "========================================" << std::endl;
    std::cout << "CONFRONTO Q MATRIX: MATLAB vs OPENCV" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    std::cout << "Q MATRIX MATLAB (pre-calcolata, CONVENZIONE SBAGLIATA):" << std::endl;
    std::cout << Q_matlab << std::endl;
    std::cout << std::endl;

    std::cout << "Q MATRIX OPENCV (ricalcolata, CONVENZIONE CORRETTA):" << std::endl;
    std::cout << Q_opencv << std::endl;
    std::cout << std::endl;

    std::cout << "DIFFERENZE CRITICHE:" << std::endl;
    std::cout << "   Q(2,2): MATLAB = " << Q_matlab.at<double>(2,2) << ", OpenCV = " << Q_opencv.at<double>(2,2) << " ⚠️" << std::endl;
    std::cout << "   Q(2,3): MATLAB = " << Q_matlab.at<double>(2,3) << ", OpenCV = " << Q_opencv.at<double>(2,3) << std::endl;
    std::cout << "   Q(3,2): MATLAB = " << Q_matlab.at<double>(3,2) << ", OpenCV = " << Q_opencv.at<double>(3,2) << " ⚠️ SEGNO OPPOSTO!" << std::endl;
    std::cout << "   Q(3,3): MATLAB = " << Q_matlab.at<double>(3,3) << ", OpenCV = " << Q_opencv.at<double>(3,3) << std::endl;
    std::cout << std::endl;

    // Salva nuova calibrazione con Q CORRETTA
    std::string outputFile = "/unlook_calib/calib-opencv-native-Q.yaml";
    if (argc > 2) {
        outputFile = argv[2];
    }

    std::cout << "💾 Salvataggio calibrazione con Q matrix OpenCV nativa..." << std::endl;
    std::cout << "   Output file: " << outputFile << std::endl;
    std::cout << std::endl;

    cv::FileStorage fs_out(outputFile, cv::FileStorage::WRITE);

    // Metadata
    fs_out << "calibration_date" << "2025-11-17_Q_RECALCULATED";
    fs_out << "calibration_method" << "OpenCV_stereoRectify_from_MATLAB_base_params";
    fs_out << "note" << "Q matrix recalculated with OpenCV stereoRectify (NOT from MATLAB)";

    // Image size
    fs_out << "image_width" << width;
    fs_out << "image_height" << height;

    // Intrinsics (from MATLAB)
    fs_out << "camera_matrix_left" << K1;
    fs_out << "camera_matrix_right" << K2;
    fs_out << "distortion_coeffs_left" << D1;
    fs_out << "distortion_coeffs_right" << D2;

    // Extrinsics (from MATLAB)
    fs_out << "rotation_matrix" << R;
    fs_out << "translation_vector" << T;
    double baseline = cv::norm(T);
    fs_out << "baseline_mm" << baseline;

    // Rectification (from OpenCV stereoRectify - CORRETTA!)
    fs_out << "rectification_transform_left" << R1_opencv;
    fs_out << "rectification_transform_right" << R2_opencv;
    fs_out << "projection_matrix_left" << P1_opencv;
    fs_out << "projection_matrix_right" << P2_opencv;
    fs_out << "disparity_to_depth_matrix" << Q_opencv;  // ← Q CORRETTA!

    fs_out.release();

    std::cout << "✅ Calibrazione salvata con successo!" << std::endl;
    std::cout << std::endl;

    std::cout << "========================================" << std::endl;
    std::cout << "🎯 PROSSIMI PASSI" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "1. Crea symlink: ln -sf calib-opencv-native-Q.yaml /unlook_calib/default.yaml" << std::endl;
    std::cout << "2. Testa scansione con 'unlook'" << std::endl;
    std::cout << "3. Verifica che il CONO sia SCOMPARSO!" << std::endl;
    std::cout << std::endl;
    std::cout << "Expected result: 🎉 NO MORE CONE with OpenCV native Q matrix!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
