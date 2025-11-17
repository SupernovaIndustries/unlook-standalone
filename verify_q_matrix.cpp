/**
 * @file verify_q_matrix.cpp
 * @brief Verifica e ricalcola Q matrix da P1 e P2 (MATLAB export vs OpenCV formula)
 *
 * PROBLEMA CRITICO TROVATO:
 * - GitHub Issue #4874: cv::stereoRectify computa Q matrix INCORRETTA
 * - Fix richiesto: Q(3,3) = -Q(3,3) prima di reprojectImageTo3D
 *
 * INOLTRE:
 * - MATLAB stereoParametersToOpenCV NON esporta Q matrix
 * - Q matrix nel file YAML potrebbe essere sbagliata
 * - Dobbiamo ricalcolarla manualmente da P1 e P2
 *
 * FORMULA CORRETTA Q matrix (OpenCV docs):
 * Q = [1  0   0        -cx
 *      0  1   0        -cy
 *      0  0   0         f
 *      0  0  -1/Tx   (cx-cx')/Tx]
 *
 * Dove:
 * - cx, cy = principal point left (da P1)
 * - cx' = principal point right X (da P2)
 * - f = focal length (da P1)
 * - Tx = baseline = P2(0,3) / f
 *
 * @author Alessandro (Unlook Project)
 * @date 2025-11-17
 */

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <iomanip>

int main() {
    std::cout << std::fixed << std::setprecision(10);
    std::cout << "=== VERIFICA E RICALCOLO Q MATRIX ===" << std::endl;
    std::cout << std::endl;

    // Carica calibrazione
    cv::FileStorage fs("/unlook_calib/calib-matlab-final-opencv46.yaml", cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "ERRORE: Impossibile aprire calibrazione" << std::endl;
        return 1;
    }

    cv::Mat P1, P2, Q_matlab;
    fs["projection_matrix_left"] >> P1;
    fs["projection_matrix_right"] >> P2;
    fs["disparity_to_depth_matrix"] >> Q_matlab;
    fs.release();

    std::cout << "=== Q MATRIX DA CALIBRAZIONE MATLAB ===" << std::endl;
    std::cout << Q_matlab << std::endl;
    std::cout << std::endl;

    // Estrai parametri da P1 e P2
    double fx = P1.at<double>(0, 0);
    double fy = P1.at<double>(1, 1);
    double cx = P1.at<double>(0, 2);
    double cy = P1.at<double>(1, 2);
    double cx_prime = P2.at<double>(0, 2);
    double Tx_fx = P2.at<double>(0, 3);  // Tx_fx = -fx * Tx
    double Tx = -Tx_fx / fx;  // Baseline in unità originali

    std::cout << "=== PARAMETRI DA P1 e P2 ===" << std::endl;
    std::cout << "P1 (left projection matrix):" << std::endl << P1 << std::endl;
    std::cout << "P2 (right projection matrix):" << std::endl << P2 << std::endl;
    std::cout << std::endl;
    std::cout << "Estratti:" << std::endl;
    std::cout << "  fx = " << fx << std::endl;
    std::cout << "  fy = " << fy << std::endl;
    std::cout << "  cx = " << cx << " (principal point left X)" << std::endl;
    std::cout << "  cy = " << cy << " (principal point left Y)" << std::endl;
    std::cout << "  cx' = " << cx_prime << " (principal point right X)" << std::endl;
    std::cout << "  Tx = " << Tx << " mm (baseline)" << std::endl;
    std::cout << std::endl;

    // Ricalcola Q matrix secondo formula OpenCV ufficiale
    // Q = [1  0   0        -cx
    //      0  1   0        -cy
    //      0  0   0         f
    //      0  0  -1/Tx   (cx-cx')/Tx]
    cv::Mat Q_computed = cv::Mat::eye(4, 4, CV_64F);
    Q_computed.at<double>(0, 3) = -cx;
    Q_computed.at<double>(1, 3) = -cy;
    Q_computed.at<double>(2, 3) = fx;  // Usa focal length
    Q_computed.at<double>(3, 2) = -1.0 / Tx;
    Q_computed.at<double>(3, 3) = (cx - cx_prime) / Tx;

    std::cout << "=== Q MATRIX RICALCOLATA (formula OpenCV) ===" << std::endl;
    std::cout << Q_computed << std::endl;
    std::cout << std::endl;

    // Confronta elemento per elemento
    std::cout << "=== CONFRONTO MATLAB vs COMPUTED ===" << std::endl;
    std::cout << "Elemento    MATLAB          COMPUTED        DIFFERENZA" << std::endl;
    std::cout << "---------------------------------------------------------------" << std::endl;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            double matlab_val = Q_matlab.at<double>(i, j);
            double computed_val = Q_computed.at<double>(i, j);
            double diff = matlab_val - computed_val;

            if (std::abs(diff) > 1e-6) {
                std::cout << "Q(" << i << "," << j << ")    "
                         << std::setw(15) << matlab_val << "  "
                         << std::setw(15) << computed_val << "  "
                         << std::setw(15) << diff << " ⚠️ DIFFERENTE" << std::endl;
            }
        }
    }
    std::cout << std::endl;

    // CRITICAL: Check Q(3,3) bug
    double Q33_matlab = Q_matlab.at<double>(3, 3);
    double Q33_computed = Q_computed.at<double>(3, 3);

    std::cout << "=== CRITICAL CHECK: Q(3,3) BUG (GitHub Issue #4874) ===" << std::endl;
    std::cout << "Q(3,3) da MATLAB:    " << Q33_matlab << std::endl;
    std::cout << "Q(3,3) ricalcolato:  " << Q33_computed << std::endl;
    std::cout << "Q(3,3) formula:      (cx - cx') / Tx = (" << cx << " - " << cx_prime << ") / " << Tx << std::endl;
    std::cout << std::endl;

    // BUG FIX: Invert Q(3,3)
    cv::Mat Q_bugfixed = Q_computed.clone();
    Q_bugfixed.at<double>(3, 3) = -Q_bugfixed.at<double>(3, 3);

    std::cout << "=== Q MATRIX CON BUG FIX Q(3,3) INVERTITO ===" << std::endl;
    std::cout << "Q(3,3) PRIMA:  " << Q_computed.at<double>(3, 3) << std::endl;
    std::cout << "Q(3,3) DOPO:   " << Q_bugfixed.at<double>(3, 3) << " (invertito)" << std::endl;
    std::cout << std::endl;
    std::cout << "Q matrix completa con bug fix:" << std::endl;
    std::cout << Q_bugfixed << std::endl;
    std::cout << std::endl;

    // Salva Q corretta
    cv::FileStorage fs_out("/tmp/Q_analysis.yaml", cv::FileStorage::WRITE);
    fs_out << "Q_matlab" << Q_matlab;
    fs_out << "Q_computed" << Q_computed;
    fs_out << "Q_bugfixed" << Q_bugfixed;
    fs_out << "parameters" << "{";
    fs_out << "fx" << fx;
    fs_out << "fy" << fy;
    fs_out << "cx" << cx;
    fs_out << "cy" << cy;
    fs_out << "cx_prime" << cx_prime;
    fs_out << "Tx" << Tx;
    fs_out << "Q33_matlab" << Q33_matlab;
    fs_out << "Q33_computed" << Q33_computed;
    fs_out << "Q33_bugfixed" << Q_bugfixed.at<double>(3, 3);
    fs_out << "}";
    fs_out.release();

    std::cout << "✅ Analisi salvata in: /tmp/Q_analysis.yaml" << std::endl;
    std::cout << std::endl;

    // RACCOMANDAZIONE
    std::cout << "=== RACCOMANDAZIONE ===" << std::endl;
    if (std::abs(Q33_matlab - Q33_computed) < 1e-6) {
        std::cout << "⚠️  Q(3,3) MATLAB = Q(3,3) formula OpenCV" << std::endl;
        std::cout << "    Probabilmente serve il BUG FIX: Q(3,3) = -Q(3,3)" << std::endl;
        std::cout << "    Testa con Q_bugfixed!" << std::endl;
    } else if (std::abs(Q33_matlab + Q33_computed) < 1e-6) {
        std::cout << "✅ Q(3,3) MATLAB ha GIÀ il segno invertito rispetto alla formula" << std::endl;
        std::cout << "    MATLAB potrebbe aver già applicato il bug fix" << std::endl;
        std::cout << "    Usa Q_matlab così com'è" << std::endl;
    } else {
        std::cout << "❓ Q(3,3) MATLAB è completamente diverso dalla formula" << std::endl;
        std::cout << "    Verifica la calibrazione MATLAB" << std::endl;
    }
    std::cout << std::endl;

    return 0;
}
