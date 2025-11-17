/**
 * @file verify_principal_point.cpp
 * @brief Verifica che principal point in Q matrix corrisponda a P1
 *
 * PROBLEMA SOSPETTO: CONO RADIALE
 * - Il cono suggerisce che i raggi 3D divergono dal centro sbagliato
 * - Se Q matrix ha principal point (cx, cy) diverso da quello usato per la rettifica
 * - Allora reprojectImageTo3D proietta dal centro sbagliato → CONO!
 *
 * @author Alessandro
 * @date 2025-11-17
 */

#include <opencv2/core.hpp>
#include <iostream>
#include <iomanip>

int main() {
    std::cout << std::fixed << std::setprecision(10);

    cv::FileStorage fs("/unlook_calib/calib-opencv-native-Q.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "ERRORE: Impossibile aprire calibrazione" << std::endl;
        return 1;
    }

    cv::Mat P1, Q;
    int width, height;

    fs["projection_matrix_left"] >> P1;
    fs["disparity_to_depth_matrix"] >> Q;
    fs["image_width"] >> width;
    fs["image_height"] >> height;
    fs.release();

    std::cout << "========================================" << std::endl;
    std::cout << "VERIFICA PRINCIPAL POINT CONSISTENCY" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;

    // Principal point da P1 (projection matrix)
    double cx_P1 = P1.at<double>(0, 2);
    double cy_P1 = P1.at<double>(1, 2);

    // Principal point da Q matrix
    double cx_Q = -Q.at<double>(0, 3);
    double cy_Q = -Q.at<double>(1, 3);

    std::cout << "IMAGE SIZE: " << width << "x" << height << std::endl;
    std::cout << std::endl;

    std::cout << "PRINCIPAL POINT da P1 (projection matrix):" << std::endl;
    std::cout << "  cx = " << cx_P1 << std::endl;
    std::cout << "  cy = " << cy_P1 << std::endl;
    std::cout << std::endl;

    std::cout << "PRINCIPAL POINT da Q matrix:" << std::endl;
    std::cout << "  cx = " << cx_Q << " (da -Q(0,3))" << std::endl;
    std::cout << "  cy = " << cy_Q << " (da -Q(1,3))" << std::endl;
    std::cout << std::endl;

    double diff_cx = std::abs(cx_P1 - cx_Q);
    double diff_cy = std::abs(cy_P1 - cy_Q);

    std::cout << "DIFFERENZA:" << std::endl;
    std::cout << "  Δcx = " << diff_cx << " pixel" << std::endl;
    std::cout << "  Δcy = " << diff_cy << " pixel" << std::endl;
    std::cout << std::endl;

    if (diff_cx > 0.01 || diff_cy > 0.01) {
        std::cout << "⚠️  MISMATCH! Principal point P1 ≠ principal point Q" << std::endl;
        std::cout << "    Questo può causare CONO nella riproiezione 3D!" << std::endl;
    } else {
        std::cout << "✅ Principal points match" << std::endl;
    }
    std::cout << std::endl;

    // Verifica anche il centro dell'immagine
    double img_center_x = width / 2.0;
    double img_center_y = height / 2.0;

    std::cout << "CENTRO IMMAGINE (geometrico):" << std::endl;
    std::cout << "  x = " << img_center_x << std::endl;
    std::cout << "  y = " << img_center_y << std::endl;
    std::cout << std::endl;

    std::cout << "OFFSET PRINCIPAL POINT da centro:" << std::endl;
    std::cout << "  Δx = " << (cx_Q - img_center_x) << " pixel" << std::endl;
    std::cout << "  Δy = " << (cy_Q - img_center_y) << " pixel" << std::endl;
    std::cout << std::endl;

    // Verifica CROP region
    int cropLeft = 316;
    int cropTop = 116;
    int cropWidth = 680;
    int cropHeight = 420;

    double crop_center_x = cropLeft + cropWidth / 2.0;
    double crop_center_y = cropTop + cropHeight / 2.0;

    std::cout << "CENTRO CROP REGION (680x420):" << std::endl;
    std::cout << "  x = " << crop_center_x << " (cropLeft=" << cropLeft << " + width/2=" << cropWidth/2.0 << ")" << std::endl;
    std::cout << "  y = " << crop_center_y << " (cropTop=" << cropTop << " + height/2=" << cropHeight/2.0 << ")" << std::endl;
    std::cout << std::endl;

    std::cout << "PRINCIPAL POINT vs CROP CENTER:" << std::endl;
    std::cout << "  Δx = " << (cx_Q - crop_center_x) << " pixel" << std::endl;
    std::cout << "  Δy = " << (cy_Q - crop_center_y) << " pixel" << std::endl;
    std::cout << std::endl;

    if (std::abs(cx_Q - crop_center_x) < 5 && std::abs(cy_Q - crop_center_y) < 5) {
        std::cout << "✅ Principal point è vicino al centro del CROP" << std::endl;
    } else {
        std::cout << "⚠️  Principal point NON è al centro del CROP!" << std::endl;
        std::cout << "    Se usi CROP per stereo matching, disparità potrebbero essere sbagliate ai bordi" << std::endl;
    }

    return 0;
}
