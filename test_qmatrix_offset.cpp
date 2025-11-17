/**
 * @file test_qmatrix_offset.cpp
 * @brief Test diagnostico per verificare l'offset MATLAB->OpenCV del principal point
 *
 * PROBLEMA SOSPETTO:
 * - MATLAB usa coordinate 1-based: pixel (1,1) è l'angolo top-left
 * - OpenCV usa coordinate 0-based: pixel (0,0) è l'angolo top-left
 * - Quando si esporta da MATLAB a OpenCV, si deve SOTTRARRE 1.0 da cx e cy
 *
 * IPOTESI:
 * Se la calibrazione MATLAB non ha applicato l'offset -1.0, il principal point
 * nella Q matrix è spostato di ~1 pixel, causando:
 * 1. Errore epipolare di ~2 pixel
 * 2. Cono nella nuvola di punti (centro di proiezione errato)
 *
 * QUESTO TEST:
 * - Carica la Q matrix attuale
 * - Applica correzione offset -1.0 pixel a cx e cy
 * - Salva Q_corrected.yaml per confronto
 *
 * @author Alessandro (Unlook Project)
 * @date 2025-11-17
 */

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <iomanip>

int main(int argc, char** argv) {
    std::cout << "=== TEST DIAGNOSTICO: Q Matrix Offset MATLAB->OpenCV ===" << std::endl;
    std::cout << std::endl;

    // Carica calibrazione
    std::string calibPath = "/unlook_calib/calib-matlab-final-opencv46.yaml";
    cv::FileStorage fs(calibPath, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "ERRORE: Impossibile aprire " << calibPath << std::endl;
        return 1;
    }

    cv::Mat P1, Q_original;
    fs["projection_matrix_left"] >> P1;
    fs["disparity_to_depth_matrix"] >> Q_original;
    fs.release();

    if (P1.empty() || Q_original.empty()) {
        std::cerr << "ERRORE: P1 o Q matrix non trovate" << std::endl;
        return 1;
    }

    // Estrai principal point rettificato da P1
    double cx_rect = P1.at<double>(0, 2);
    double cy_rect = P1.at<double>(1, 2);
    double f_rect = P1.at<double>(0, 0);

    std::cout << "=== VALORI ATTUALI (dalla calibrazione MATLAB) ===" << std::endl;
    std::cout << "Principal Point Rettificato (da P1):" << std::endl;
    std::cout << "  cx_rect = " << std::setprecision(10) << cx_rect << std::endl;
    std::cout << "  cy_rect = " << std::setprecision(10) << cy_rect << std::endl;
    std::cout << "  f_rect  = " << std::setprecision(10) << f_rect << std::endl;
    std::cout << std::endl;

    std::cout << "Q Matrix Originale:" << std::endl;
    std::cout << "  Q(0,3) = " << Q_original.at<double>(0, 3) << " (dovrebbe essere -cx_rect)" << std::endl;
    std::cout << "  Q(1,3) = " << Q_original.at<double>(1, 3) << " (dovrebbe essere -cy_rect)" << std::endl;
    std::cout << "  Q(2,3) = " << Q_original.at<double>(2, 3) << " (dovrebbe essere f_rect)" << std::endl;
    std::cout << "  Q(3,2) = " << Q_original.at<double>(3, 2) << " (dovrebbe essere -1/baseline)" << std::endl;
    std::cout << std::endl;

    // VERIFICA: Q(0,3) deve essere -cx_rect, Q(1,3) deve essere -cy_rect
    double Q_cx = -Q_original.at<double>(0, 3);
    double Q_cy = -Q_original.at<double>(1, 3);

    std::cout << "=== VERIFICA COERENZA Q vs P1 ===" << std::endl;
    std::cout << "  P1 cx_rect = " << cx_rect << std::endl;
    std::cout << "  Q  cx      = " << Q_cx << " (da -Q(0,3))" << std::endl;
    std::cout << "  Differenza = " << std::abs(cx_rect - Q_cx) << " pixel" << std::endl;
    std::cout << std::endl;
    std::cout << "  P1 cy_rect = " << cy_rect << std::endl;
    std::cout << "  Q  cy      = " << Q_cy << " (da -Q(1,3))" << std::endl;
    std::cout << "  Differenza = " << std::abs(cy_rect - Q_cy) << " pixel" << std::endl;
    std::cout << std::endl;

    // IPOTESI: Se la calibrazione MATLAB non ha applicato l'offset -1.0,
    // il principal point corretto per OpenCV dovrebbe essere cx-1, cy-1
    double cx_corrected = cx_rect - 1.0;
    double cy_corrected = cy_rect - 1.0;

    std::cout << "=== CORREZIONE PROPOSTA: MATLAB (1-based) -> OpenCV (0-based) ===" << std::endl;
    std::cout << "Se la calibrazione MATLAB non ha sottratto 1.0 pixel:" << std::endl;
    std::cout << "  cx_opencv dovrebbe essere: " << cx_rect << " - 1.0 = " << cx_corrected << std::endl;
    std::cout << "  cy_opencv dovrebbe essere: " << cy_rect << " - 1.0 = " << cy_corrected << std::endl;
    std::cout << std::endl;

    // Crea Q matrix corretta con offset -1.0
    cv::Mat Q_corrected = Q_original.clone();
    Q_corrected.at<double>(0, 3) = -cx_corrected;  // -655.109 invece di -656.109
    Q_corrected.at<double>(1, 3) = -cy_corrected;  // -325.0 invece di -326.0

    std::cout << "=== Q MATRIX CORRETTA (con offset -1.0) ===" << std::endl;
    std::cout << "  Q(0,3) = " << Q_corrected.at<double>(0, 3) << " (era " << Q_original.at<double>(0, 3) << ")" << std::endl;
    std::cout << "  Q(1,3) = " << Q_corrected.at<double>(1, 3) << " (era " << Q_original.at<double>(1, 3) << ")" << std::endl;
    std::cout << "  Delta_cx = " << (Q_corrected.at<double>(0, 3) - Q_original.at<double>(0, 3)) << " pixel" << std::endl;
    std::cout << "  Delta_cy = " << (Q_corrected.at<double>(1, 3) - Q_original.at<double>(1, 3)) << " pixel" << std::endl;
    std::cout << std::endl;

    // Salva Q corretta per test
    cv::FileStorage fs_out("/tmp/Q_corrected.yaml", cv::FileStorage::WRITE);
    fs_out << "Q_original" << Q_original;
    fs_out << "Q_corrected_minus1" << Q_corrected;
    fs_out << "cx_original" << cx_rect;
    fs_out << "cy_original" << cy_rect;
    fs_out << "cx_corrected" << cx_corrected;
    fs_out << "cy_corrected" << cy_corrected;
    fs_out << "offset_explanation" << "MATLAB uses 1-based indexing (1,1), OpenCV uses 0-based (0,0). When converting MATLAB->OpenCV, subtract 1.0 from cx and cy.";
    fs_out.release();

    std::cout << "✅ Q matrix corretta salvata in: /tmp/Q_corrected.yaml" << std::endl;
    std::cout << std::endl;

    // IMPACT ANALYSIS
    std::cout << "=== IMPATTO SULLA RICOSTRUZIONE 3D ===" << std::endl;
    std::cout << "Un offset di 1 pixel nel principal point causa:" << std::endl;
    std::cout << "  1. Errore di allineamento epipolare: ~2 pixel (OSSERVATO!)" << std::endl;
    std::cout << "  2. Shift nella riproiezione 3D: i raggi convergono verso il centro sbagliato" << std::endl;
    std::cout << "  3. Forma CONICA nella nuvola di punti: distorsione radiale dalla Q matrix errata" << std::endl;
    std::cout << std::endl;

    std::cout << "=== PROSSIMI PASSI ===" << std::endl;
    std::cout << "1. Verifica se la calibrazione MATLAB originale ha cx ≈ " << (cx_rect + 1.0) << ", cy ≈ " << (cy_rect + 1.0) << std::endl;
    std::cout << "2. Se SI: conferma che manca l'offset -1.0 → applica correzione" << std::endl;
    std::cout << "3. Se NO: il problema è altrove (distorsione residua, modello inadeguato)" << std::endl;
    std::cout << std::endl;

    return 0;
}
