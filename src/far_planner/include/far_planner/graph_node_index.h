// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT
#ifndef FAR_PLANNER_GRAPH_NODE_INDEX_H
#define FAR_PLANNER_GRAPH_NODE_INDEX_H

#include "far_planner/utility.h"

#include <cmath>

// Parallel point/node arrays preserve exact node identity without encoding an
// integer ID in a floating-point PCL field.
class GraphNodeIndex {
public:
    GraphNodeIndex()
        : cloud_(new PointCloud), tree_(new pcl::KdTreeFLANN<PCLPoint>) {
        tree_->setSortedResults(false);
    }

    void Build(const NodePtrStack& graph) {
        indexed_nodes_.clear();
        cloud_->clear();
        indexed_nodes_.reserve(graph.size());
        cloud_->reserve(graph.size());
        for (const auto& node : graph) {
            if (!node || node->is_odom || FARUtil::IsOutsideGoal(node)) continue;
            cloud_->push_back(FARUtil::Point3DToPCLPoint(node->position));
            indexed_nodes_.push_back(node);
        }
        if (indexed_nodes_.empty()) {
            FARUtil::ClearKdTree(cloud_, tree_);
        } else {
            tree_->setInputCloud(cloud_);
        }
    }

    NavNodePtr Nearest(const Point3D& position, float radius) const {
        if (indexed_nodes_.empty() || !std::isfinite(radius) || radius <= 0.0f ||
            !std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z)) return nullptr;
        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        const auto query = FARUtil::Point3DToPCLPoint(position);
        if (tree_->nearestKSearch(query, 1, indices, squared_distances) == 0) return nullptr;
        if (squared_distances[0] >= radius * radius) return nullptr;
        const auto index = static_cast<std::size_t>(indices[0]);
        return index < indexed_nodes_.size() ? indexed_nodes_[index] : nullptr;
    }

    std::size_t Size() const { return indexed_nodes_.size(); }

private:
    PointCloudPtr cloud_;
    PointKdTreePtr tree_;
    NodePtrStack indexed_nodes_;
};

#endif
