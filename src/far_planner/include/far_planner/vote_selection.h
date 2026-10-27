// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef FAR_PLANNER_VOTE_SELECTION_H
#define FAR_PLANNER_VOTE_SELECTION_H

#include <algorithm>
#include <cstddef>

namespace far_planner {

// Tracks the two highest scores without storing or sorting the full vote set.
// Equal scores occupy separate ranks; every score tied at the cutoff qualifies.
class TopTwoScores {
public:
    void Observe(int score) {
        if (count_ == 0) {
            first_ = score;
            count_ = 1;
        } else if (count_ == 1) {
            second_ = std::min(first_, score);
            first_ = std::max(first_, score);
            count_ = 2;
        } else if (score > first_) {
            second_ = first_;
            first_ = score;
        } else if (score > second_) {
            second_ = score;
        }
    }

    bool Includes(int score) const {
        return count_ != 0 && (count_ == 1 ? score == first_ : score >= second_);
    }

private:
    std::size_t count_ = 0;  // Saturates at two.
    int first_ = 0;
    int second_ = 0;
};

}  // namespace far_planner
#endif
