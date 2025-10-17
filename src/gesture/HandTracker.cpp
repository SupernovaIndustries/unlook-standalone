/**
 * @file HandTracker.cpp
 * @brief Implementation of hand tracking with Kalman filtering
 */

#include "unlook/gesture/HandTracker.hpp"
#include <unlook/core/Logger.hpp>
#include <opencv2/video/tracking.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace unlook {
namespace gesture {

/**
 * @brief Single tracked hand with Kalman filter
 */
struct TrackedHandInternal {
    int id;
    cv::KalmanFilter kalman;
    TrackedHand state;
    int frames_missed;
    std::chrono::steady_clock::time_point last_update;

    TrackedHandInternal(int track_id, const cv::Point2f& pos, const cv::Size2f& sz)
        : id(track_id)
        , frames_missed(0) {

        // Initialize Kalman filter
        // State: [x, y, vx, vy, width, height]
        // Measurement: [x, y, width, height]
        kalman.init(6, 4, 0);

        // Transition matrix (constant velocity model)
        kalman.transitionMatrix = (cv::Mat_<float>(6, 6) <<
            1, 0, 1, 0, 0, 0,  // x += vx
            0, 1, 0, 1, 0, 0,  // y += vy
            0, 0, 1, 0, 0, 0,  // vx
            0, 0, 0, 1, 0, 0,  // vy
            0, 0, 0, 0, 1, 0,  // width
            0, 0, 0, 0, 0, 1   // height
        );

        // Measurement matrix
        kalman.measurementMatrix = (cv::Mat_<float>(4, 6) <<
            1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1
        );

        // Initialize state
        kalman.statePost.at<float>(0) = pos.x;
        kalman.statePost.at<float>(1) = pos.y;
        kalman.statePost.at<float>(2) = 0.0f;  // vx
        kalman.statePost.at<float>(3) = 0.0f;  // vy
        kalman.statePost.at<float>(4) = sz.width;
        kalman.statePost.at<float>(5) = sz.height;

        // Initialize tracked state
        state.track_id = track_id;
        state.position = pos;
        state.velocity = cv::Point2f(0, 0);
        state.size = sz;
        state.is_active = true;
        state.frames_since_detection = 0;
        state.confidence = 1.0f;
        state.timestamp = std::chrono::steady_clock::now();
        last_update = state.timestamp;
    }

    void update_noise(float pos_noise, float vel_noise, float meas_noise) {
        // Process noise covariance
        cv::setIdentity(kalman.processNoiseCov, cv::Scalar::all(0.01));
        kalman.processNoiseCov.at<float>(0, 0) = pos_noise;
        kalman.processNoiseCov.at<float>(1, 1) = pos_noise;
        kalman.processNoiseCov.at<float>(2, 2) = vel_noise;
        kalman.processNoiseCov.at<float>(3, 3) = vel_noise;
        kalman.processNoiseCov.at<float>(4, 4) = pos_noise * 0.5f;
        kalman.processNoiseCov.at<float>(5, 5) = pos_noise * 0.5f;

        // Measurement noise covariance
        cv::setIdentity(kalman.measurementNoiseCov, cv::Scalar::all(meas_noise));
    }

    void predict() {
        cv::Mat prediction = kalman.predict();

        state.position.x = prediction.at<float>(0);
        state.position.y = prediction.at<float>(1);
        state.velocity.x = prediction.at<float>(2);
        state.velocity.y = prediction.at<float>(3);
        state.size.width = prediction.at<float>(4);
        state.size.height = prediction.at<float>(5);
    }

    void correct(const cv::Point2f& measured_pos, const cv::Size2f& measured_size) {
        cv::Mat measurement = (cv::Mat_<float>(4, 1) <<
            measured_pos.x,
            measured_pos.y,
            measured_size.width,
            measured_size.height
        );

        cv::Mat corrected = kalman.correct(measurement);

        state.position.x = corrected.at<float>(0);
        state.position.y = corrected.at<float>(1);
        state.velocity.x = corrected.at<float>(2);
        state.velocity.y = corrected.at<float>(3);
        state.size.width = corrected.at<float>(4);
        state.size.height = corrected.at<float>(5);

        state.frames_since_detection = 0;
        frames_missed = 0;
        state.timestamp = std::chrono::steady_clock::now();
        last_update = state.timestamp;
    }

    void mark_missed() {
        frames_missed++;
        state.frames_since_detection++;
        state.confidence *= 0.9f;  // Decay confidence
    }
};

/**
 * @brief PIMPL implementation for HandTracker
 */
class HandTracker::Impl {
public:
    HandTrackerConfig config;
    std::vector<TrackedHandInternal> tracks;
    int next_track_id = 0;

    /**
     * @brief Associate detections with existing tracks
     */
    std::vector<std::pair<int, int>> associate_detections(
        const std::vector<cv::Rect>& detections) {

        std::vector<std::pair<int, int>> associations;

        if (tracks.empty() || detections.empty()) {
            return associations;
        }

        // Compute distance matrix
        std::vector<std::vector<float>> distances(tracks.size(),
                                                  std::vector<float>(detections.size()));

        for (size_t t = 0; t < tracks.size(); ++t) {
            cv::Point2f track_center = tracks[t].state.position;

            for (size_t d = 0; d < detections.size(); ++d) {
                cv::Point2f det_center(
                    detections[d].x + detections[d].width / 2.0f,
                    detections[d].y + detections[d].height / 2.0f
                );

                float dx = track_center.x - det_center.x;
                float dy = track_center.y - det_center.y;
                distances[t][d] = std::sqrt(dx * dx + dy * dy);
            }
        }

        // Simple greedy association (nearest neighbor)
        std::vector<bool> track_matched(tracks.size(), false);
        std::vector<bool> det_matched(detections.size(), false);

        while (true) {
            // Find minimum distance
            float min_dist = config.max_association_distance;
            int min_track = -1;
            int min_det = -1;

            for (size_t t = 0; t < tracks.size(); ++t) {
                if (track_matched[t]) continue;

                for (size_t d = 0; d < detections.size(); ++d) {
                    if (det_matched[d]) continue;

                    if (distances[t][d] < min_dist) {
                        min_dist = distances[t][d];
                        min_track = static_cast<int>(t);
                        min_det = static_cast<int>(d);
                    }
                }
            }

            if (min_track == -1 || min_det == -1) {
                break;  // No more matches
            }

            associations.emplace_back(min_track, min_det);
            track_matched[min_track] = true;
            det_matched[min_det] = true;
        }

        return associations;
    }

    /**
     * @brief Get centroid and size from bounding box
     */
    static void bbox_to_state(const cv::Rect& bbox, cv::Point2f& center, cv::Size2f& size) {
        center.x = bbox.x + bbox.width / 2.0f;
        center.y = bbox.y + bbox.height / 2.0f;
        size.width = static_cast<float>(bbox.width);
        size.height = static_cast<float>(bbox.height);
    }
};

// ===== Public API Implementation =====

HandTracker::HandTracker()
    : pImpl(std::make_unique<Impl>()) {
    pImpl->config = HandTrackerConfig{};
}

HandTracker::HandTracker(const HandTrackerConfig& config)
    : pImpl(std::make_unique<Impl>()) {
    pImpl->config = config;
}

HandTracker::~HandTracker() = default;

void HandTracker::configure(const HandTrackerConfig& config) {
    if (config.is_valid()) {
        pImpl->config = config;

        // Update noise for existing tracks
        for (auto& track : pImpl->tracks) {
            track.update_noise(config.position_noise,
                             config.velocity_noise,
                             config.measurement_noise);
        }
    }
}

void HandTracker::update(const std::vector<cv::Rect>& detections,
                         const std::vector<HandLandmarks>& landmarks) {
    // Predict all tracks
    for (auto& track : pImpl->tracks) {
        track.predict();
    }

    // Associate detections with tracks
    auto associations = pImpl->associate_detections(detections);

    // Update matched tracks
    std::vector<bool> detection_used(detections.size(), false);

    for (const auto& [track_idx, det_idx] : associations) {
        cv::Point2f center;
        cv::Size2f size;
        pImpl->bbox_to_state(detections[det_idx], center, size);

        pImpl->tracks[track_idx].correct(center, size);

        // Update landmarks if available
        if (det_idx < static_cast<int>(landmarks.size())) {
            pImpl->tracks[track_idx].state.landmarks = landmarks[det_idx];
        }

        pImpl->tracks[track_idx].state.confidence = 1.0f;
        detection_used[det_idx] = true;
    }

    // Mark unmatched tracks as missed
    for (size_t t = 0; t < pImpl->tracks.size(); ++t) {
        bool matched = false;
        for (const auto& [track_idx, det_idx] : associations) {
            if (track_idx == static_cast<int>(t)) {
                matched = true;
                break;
            }
        }

        if (!matched) {
            pImpl->tracks[t].mark_missed();
        }
    }

    // Create new tracks for unmatched detections
    int new_tracks_created = 0;
    for (size_t d = 0; d < detections.size(); ++d) {
        if (!detection_used[d]) {
            cv::Point2f center;
            cv::Size2f size;
            pImpl->bbox_to_state(detections[d], center, size);

            TrackedHandInternal new_track(pImpl->next_track_id++, center, size);
            new_track.update_noise(pImpl->config.position_noise,
                                  pImpl->config.velocity_noise,
                                  pImpl->config.measurement_noise);

            if (d < landmarks.size()) {
                new_track.state.landmarks = landmarks[d];
            }

            pImpl->tracks.push_back(std::move(new_track));
            new_tracks_created++;
        }
    }

    // Remove stale tracks
    size_t tracks_before_removal = pImpl->tracks.size();
    pImpl->tracks.erase(
        std::remove_if(pImpl->tracks.begin(), pImpl->tracks.end(),
            [this](const TrackedHandInternal& track) {
                return track.frames_missed > pImpl->config.max_missed_frames ||
                       track.state.confidence < pImpl->config.min_confidence_threshold;
            }),
        pImpl->tracks.end()
    );
    int tracks_removed = static_cast<int>(tracks_before_removal - pImpl->tracks.size());

    // Count valid landmarks
    int valid_landmarks = 0;
    for (const auto& lm : landmarks) {
        if (lm.is_valid()) {
            valid_landmarks++;
        }
    }

    // Production logging
    LOG_INFO("HandTracker: detections=" + std::to_string(detections.size()) +
            ", landmarks=" + std::to_string(landmarks.size()) +
            ", valid_landmarks=" + std::to_string(valid_landmarks) +
            ", associations=" + std::to_string(associations.size()) +
            ", new_tracks=" + std::to_string(new_tracks_created) +
            ", removed=" + std::to_string(tracks_removed) +
            ", active_tracks=" + std::to_string(pImpl->tracks.size()));
}

std::vector<TrackedHand> HandTracker::get_active_tracks() const {
    std::vector<TrackedHand> active_tracks;

    for (const auto& track : pImpl->tracks) {
        if (track.state.is_active && track.frames_missed == 0) {
            active_tracks.push_back(track.state);
        }
    }

    return active_tracks;
}

TrackedHand HandTracker::get_primary_hand() const {
    TrackedHand primary;
    primary.is_active = false;

    float max_confidence = 0.0f;

    for (const auto& track : pImpl->tracks) {
        if (track.state.is_active &&
            track.frames_missed == 0 &&
            track.state.confidence > max_confidence) {
            max_confidence = track.state.confidence;
            primary = track.state;
        }
    }

    return primary;
}

void HandTracker::clear() {
    pImpl->tracks.clear();
    pImpl->next_track_id = 0;
}

size_t HandTracker::get_track_count() const {
    return pImpl->tracks.size();
}

HandTrackerConfig HandTracker::get_config() const {
    return pImpl->config;
}

} // namespace gesture
} // namespace unlook
