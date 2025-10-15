/**
 * @file TemporalBuffer.cpp
 * @brief Implementation of temporal buffer for hand tracking
 */

#include "unlook/gesture/TemporalBuffer.hpp"
#include <deque>
#include <stdexcept>
#include <algorithm>
#include <cmath>

namespace unlook {
namespace gesture {

/**
 * @brief PIMPL implementation for TemporalBuffer
 */
class TemporalBuffer::Impl {
public:
    std::deque<HandFrame> buffer;
    size_t max_capacity;

    explicit Impl(size_t capacity)
        : max_capacity(capacity) {
        // Note: std::deque doesn't have reserve(), but grows efficiently
    }

    void push(const HandFrame& frame) {
        if (buffer.size() >= max_capacity) {
            buffer.pop_front();  // Remove oldest
        }
        buffer.push_back(frame);
    }

    size_t size() const {
        return buffer.size();
    }

    bool is_full() const {
        return buffer.size() >= max_capacity;
    }

    bool is_empty() const {
        return buffer.empty();
    }

    void clear() {
        buffer.clear();
    }

    size_t capacity() const {
        return max_capacity;
    }

    std::vector<HandFrame> get_recent_frames(size_t count) const {
        if (buffer.empty()) {
            return {};
        }

        size_t num_frames = (count == 0 || count > buffer.size()) ? buffer.size() : count;

        std::vector<HandFrame> result;
        result.reserve(num_frames);

        // Return newest first (reverse order)
        for (size_t i = 0; i < num_frames; ++i) {
            result.push_back(buffer[buffer.size() - 1 - i]);
        }

        return result;
    }

    HandFrame get_oldest_frame() const {
        if (buffer.empty()) {
            throw std::runtime_error("TemporalBuffer is empty");
        }
        return buffer.front();
    }

    HandFrame get_newest_frame() const {
        if (buffer.empty()) {
            throw std::runtime_error("TemporalBuffer is empty");
        }
        return buffer.back();
    }

    HandFrame get_frame_at(size_t index) const {
        if (index >= buffer.size()) {
            throw std::out_of_range("Frame index out of range");
        }
        return buffer[index];
    }

    // ===== Motion Analysis =====

    cv::Point2f compute_average_velocity() const {
        if (buffer.empty()) {
            return cv::Point2f(0, 0);
        }

        cv::Point2f sum(0, 0);
        for (const auto& frame : buffer) {
            sum += frame.velocity;
        }

        return sum * (1.0f / static_cast<float>(buffer.size()));
    }

    float compute_total_displacement() const {
        if (buffer.size() < 2) {
            return 0.0f;
        }

        cv::Point2f displacement = compute_displacement_vector();
        return std::sqrt(displacement.x * displacement.x + displacement.y * displacement.y);
    }

    cv::Point2f compute_displacement_vector() const {
        if (buffer.size() < 2) {
            return cv::Point2f(0, 0);
        }

        cv::Point2f oldest = buffer.front().position;
        cv::Point2f newest = buffer.back().position;

        return newest - oldest;
    }

    float compute_scale_change() const {
        if (buffer.size() < 2) {
            return 0.0f;
        }

        cv::Size2f oldest_size = buffer.front().size;
        cv::Size2f newest_size = buffer.back().size;

        // Use average of width and height for scale metric
        float oldest_scale = (oldest_size.width + oldest_size.height) / 2.0f;
        float newest_scale = (newest_size.width + newest_size.height) / 2.0f;

        if (oldest_scale < 1e-6f) {
            return 0.0f;  // Avoid division by zero
        }

        // Return relative change: (new - old) / old
        return (newest_scale - oldest_scale) / oldest_scale;
    }

    cv::Size2f compute_average_size() const {
        if (buffer.empty()) {
            return cv::Size2f(0, 0);
        }

        cv::Size2f sum(0, 0);
        for (const auto& frame : buffer) {
            sum.width += frame.size.width;
            sum.height += frame.size.height;
        }

        float count = static_cast<float>(buffer.size());
        return cv::Size2f(sum.width / count, sum.height / count);
    }

    double compute_duration_ms() const {
        if (buffer.size() < 2) {
            return 0.0;
        }

        auto oldest_time = buffer.front().timestamp;
        auto newest_time = buffer.back().timestamp;

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            newest_time - oldest_time
        );

        return static_cast<double>(duration.count());
    }

    bool has_minimum_frames(size_t min_frames) const {
        return buffer.size() >= min_frames;
    }
};

// ===== Public API Implementation =====

TemporalBuffer::TemporalBuffer(size_t capacity)
    : pImpl(std::make_unique<Impl>(capacity)) {
}

TemporalBuffer::~TemporalBuffer() = default;

TemporalBuffer::TemporalBuffer(TemporalBuffer&&) noexcept = default;
TemporalBuffer& TemporalBuffer::operator=(TemporalBuffer&&) noexcept = default;

void TemporalBuffer::push(const TrackedHand& hand) {
    HandFrame frame(hand);
    pImpl->push(frame);
}

void TemporalBuffer::push(const HandFrame& frame) {
    pImpl->push(frame);
}

size_t TemporalBuffer::size() const {
    return pImpl->size();
}

bool TemporalBuffer::is_full() const {
    return pImpl->is_full();
}

bool TemporalBuffer::is_empty() const {
    return pImpl->is_empty();
}

void TemporalBuffer::clear() {
    pImpl->clear();
}

size_t TemporalBuffer::capacity() const {
    return pImpl->capacity();
}

std::vector<HandFrame> TemporalBuffer::get_recent_frames(size_t count) const {
    return pImpl->get_recent_frames(count);
}

HandFrame TemporalBuffer::get_oldest_frame() const {
    return pImpl->get_oldest_frame();
}

HandFrame TemporalBuffer::get_newest_frame() const {
    return pImpl->get_newest_frame();
}

HandFrame TemporalBuffer::get_frame_at(size_t index) const {
    return pImpl->get_frame_at(index);
}

cv::Point2f TemporalBuffer::compute_average_velocity() const {
    return pImpl->compute_average_velocity();
}

float TemporalBuffer::compute_total_displacement() const {
    return pImpl->compute_total_displacement();
}

cv::Point2f TemporalBuffer::compute_displacement_vector() const {
    return pImpl->compute_displacement_vector();
}

float TemporalBuffer::compute_scale_change() const {
    return pImpl->compute_scale_change();
}

cv::Size2f TemporalBuffer::compute_average_size() const {
    return pImpl->compute_average_size();
}

double TemporalBuffer::compute_duration_ms() const {
    return pImpl->compute_duration_ms();
}

bool TemporalBuffer::has_minimum_frames(size_t min_frames) const {
    return pImpl->has_minimum_frames(min_frames);
}

} // namespace gesture
} // namespace unlook
