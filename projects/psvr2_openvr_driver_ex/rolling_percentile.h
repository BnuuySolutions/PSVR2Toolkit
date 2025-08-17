#pragma once

#include <cstdint>
#include <corecrt_math.h>
#include <corecrt_math_defines.h>
#include <set>
#include <stdexcept>
#include <vector>

namespace psvr2_toolkit {
    /**
     * @class RollingPercentile
     * @brief Calculates the percentile value over a rolling window of data samples.
     *
     * This class is designed to be efficient for calculating percentiles on a stream
     * of data. It maintains a sliding window of a specified maximum size. When a new
     * sample is added and the window is full, the oldest sample is discarded.
     *
     * The implementation uses a std::vector to keep track of the insertion order
     * (acting as a queue) and a std::multiset to keep the samples in the window
     * sorted at all times. This makes both adding new samples and calculating the
     * percentile efficient.
     *
     * @tparam T The numeric data type of the samples (e.g., int, double, float).
     */
    template <typename T>
    class RollingPercentile {
    public:
        /**
         * @brief Constructs a RollingPercentile calculator.
         * @param windowSize The maximum number of samples to keep in the window.
         * @param percentile The percentile to calculate (a value between 0.0 and 100.0).
         * @throw std::invalid_argument if windowSize is 0 or percentile is not in [0, 100].
         */
        RollingPercentile(size_t windowSize, double percentile)
            : maxWindowSize(windowSize), percentileValue(percentile) {
            if (windowSize == 0) {
                throw std::invalid_argument("Window size cannot be zero.");
            }
            if (percentile < 0.0 || percentile > 100.0) {
                throw std::invalid_argument("Percentile must be between 0.0 and 100.0.");
            }
        }

        /**
         * @brief Adds a new data sample to the window.
         *
         * If the window is already full, the oldest sample is removed to make space
         * for the new one.
         *
         * Time Complexity: O(log N), where N is the current window size, due to
         * insertion into the multiset.
         *
         * @param value The data sample to add.
         */
        void add(const T& value) {
            // Add the new value to our sorted data structure
            sortedSamples.insert(value);
            // Add the new value to the queue to track insertion order
            windowQueue.push_back(value);

            // If the window has exceeded its maximum size, remove the oldest element
            if (windowQueue.size() > maxWindowSize) {
                // Find the oldest value from the front of the queue
                T oldestValue = windowQueue.front();
                windowQueue.erase(windowQueue.begin());

                // Find and erase one instance of the oldest value from the multiset
                auto it = sortedSamples.find(oldestValue);
                if (it != sortedSamples.end()) {
                    sortedSamples.erase(it);
                }
            }
        }

        /**
         * @brief Calculates and returns the current percentile value of the samples in the window.
         *
         * It uses the nearest-rank method.
         *
         * Time Complexity: O(N) in the worst case to advance the iterator in the multiset.
         * This is generally more efficient than sorting a vector O(N log N) on each call.
         *
         * @return The calculated percentile value.
         */
        T getPercentile() const {
            if (sortedSamples.empty()) {
                return T();
            }

            // Calculate the 1-based rank of the percentile
            double rank = (percentileValue / 100.0) * (sortedSamples.size());
            size_t index = static_cast<size_t>(ceil(rank));

            // Ensure the index is at least 1 and does not exceed the size
            if (index == 0 && !sortedSamples.empty()) {
                index = 1;
            }
            if (index > sortedSamples.size()) {
                index = sortedSamples.size();
            }

            // The index is 1-based, so we need to move index-1 steps forward
            auto it = sortedSamples.begin();
            std::advance(it, index - 1);

            return *it;
        }

        /**
         * @brief Returns the current number of samples in the window.
         * @return The current size of the window.
         */
        size_t size() const {
            return sortedSamples.size();
        }

        /**
         * @brief Clears all samples from the window.
         */
        void clear() {
            sortedSamples.clear();
            windowQueue.clear();
        }

    private:
        size_t maxWindowSize;
        double percentileValue;
        std::multiset<T> sortedSamples; // Keeps window samples sorted
        std::vector<T> windowQueue;     // Tracks insertion order to find the oldest sample
    };
}