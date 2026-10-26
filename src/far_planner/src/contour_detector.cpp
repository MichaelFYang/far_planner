/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang
 * SPDX-License-Identifier: MIT
 * fanyang2@andrew.cmu.edu,   
 */

#include "far_planner/contour_detector.h"
#include "far_planner/parameter_validation.h"

#include <array>

// const static int BLUR_SIZE = 10;

/***************************************************************************************/

void ContourDetector::Init(const ContourDetectParams& params) {
    cd_params_ = params;
    /* Allocate Pointcloud pointer memory */
    new_corners_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
    // Init projection cv Mat
    far_planner::RequirePositiveFinite(cd_params_.kRatio, "c_detector/resize_ratio");
    if (cd_params_.kBlurSize < 1)
        throw std::invalid_argument("contour blur size must be positive");
    MAT_SIZE = far_planner::CheckedDimension(
        static_cast<double>(cd_params_.sensor_range) * 2.0, cd_params_.voxel_dim,
        "contour image width", 0, true);
    MAT_RESIZE = far_planner::CheckedRoundedDimension(
        static_cast<double>(MAT_SIZE) * cd_params_.kRatio, "resized contour image width");
    CMAT = MAT_SIZE / 2, CMAT_RESIZE = MAT_RESIZE / 2;
    img_mat_ = cv::Mat::zeros(MAT_SIZE, MAT_SIZE, CV_32FC1);
    img_counter_ = 0;
    odom_node_ptr_ = NULL;
    refined_contours_.clear(), refined_hierarchy_.clear();
    DIST_LIMIT = static_cast<float>(MAT_RESIZE) / MAT_SIZE * 1.5f;
    ALIGN_ANGLE_COS = cos(FARUtil::kAcceptAlign / 2.0f);
    VOXEL_DIM_INV = 1.0f / cd_params_.voxel_dim;
}

void ContourDetector::BuildTerrainImgAndExtractContour(const NavNodePtr& odom_node_ptr,
                                                       const PointCloudPtr& surround_cloud,
                                                       std::vector<PointStack>& realworl_contour) {
    this->UpdateOdom(odom_node_ptr);
    this->ResetImgMat(img_mat_);
    this->UpdateImgMatWithCloud(surround_cloud, img_mat_);
    this->ExtractContourFromImg(img_mat_, refined_contours_, realworl_contour);
}

void ContourDetector::UpdateImgMatWithCloud(const PointCloudPtr& pc, cv::Mat& img_mat) {
    int row_idx, col_idx, inf_row, inf_col;
    constexpr std::array<int, 3> inflate_vec{-1, 0, 1};
    for (const auto& pcl_p : pc->points) {
        this->PointToImgSub(pcl_p, odom_pos_, row_idx, col_idx, false, false);
        if (!this->IsIdxesInImg(row_idx, col_idx)) continue;
        for (const auto& dr : inflate_vec) {
            for (const auto& dc : inflate_vec) {
                inf_row = row_idx+dr, inf_col = col_idx+dc;
                if (this->IsIdxesInImg(inf_row, inf_col)) {
                    img_mat.at<float>(inf_row, inf_col) += 1.0;
                }
            }
        }
    }
    if (!FARUtil::IsStaticEnv) {
        cv::threshold(img_mat, img_mat, cd_params_.kThredValue, 1.0, cv::ThresholdTypes::THRESH_BINARY);
    }
    if (cd_params_.is_save_img) this->SaveCurrentImg(img_mat);
}

void ContourDetector::ResizeAndBlurImg(const cv::Mat& img, cv::Mat& Rimg) {
    img.convertTo(Rimg, CV_8UC1, 255);
    cv::resize(Rimg, Rimg, cv::Size(MAT_RESIZE, MAT_RESIZE), 0, 0,
               cv::InterpolationFlags::INTER_LINEAR);
    //cv::morphologyEx(Rimg, Rimg, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
    cv::boxFilter(Rimg, Rimg, -1, cv::Size(cd_params_.kBlurSize, cd_params_.kBlurSize), cv::Point2i(-1, -1), false);
    //cv::morphologyEx(Rimg, Rimg, cv::MORPH_CLOSE, getStructuringElement(cv::MORPH_RECT, cv::Size(cd_params_.kBlurSize+2, cd_params_.kBlurSize+2)));
}

void ContourDetector::ExtractContourFromImg(const cv::Mat& img,
                                            std::vector<CVPointStack>& img_contours, 
                                            std::vector<PointStack>& realworld_contour)
{
    cv::Mat Rimg;
    this->ResizeAndBlurImg(img, Rimg);
    this->ExtractRefinedContours(Rimg, img_contours);
    this->ConvertContoursToRealWorld(img_contours, realworld_contour);
}

void ContourDetector::ConvertContoursToRealWorld(const std::vector<CVPointStack>& ori_contours,
                                                 std::vector<PointStack>& realWorld_contours)
{
    const std::size_t C_N = ori_contours.size();
    realWorld_contours.clear(), realWorld_contours.resize(C_N);
    for (std::size_t i=0; i<C_N; i++) {
        this->ConvertCVToPoint3DVector(ori_contours[i], realWorld_contours[i], true);
    }
}


void ContourDetector::ShowCornerImage(const cv::Mat& img_mat, const PointCloudPtr& pc) {
    cv::Mat resized_img, dst;
    this->ResizeAndBlurImg(img_mat, resized_img);
    cv::cvtColor(resized_img, dst, cv::COLOR_GRAY2BGR);
    const int circle_size = std::max(1, static_cast<int>(static_cast<float>(MAT_RESIZE) / MAT_SIZE * 1.5f));
    for (std::size_t i=0; i<pc->size(); i++) {
        cv::Point2f cv_p = this->ConvertPoint3DToCVPoint(pc->points[i], odom_pos_, true);
        cv::circle(dst, cv_p, circle_size, cv::Scalar(128,128,128), -1);

    }
    // show free odom point
    cv::circle(dst, free_odom_resized_, circle_size, cv::Scalar(0,0,255), -1);
    std::vector<std::vector<cv::Point2i>> round_contours;
    this->RoundContours(refined_contours_, round_contours);
    for(std::size_t idx=0; idx<round_contours.size(); idx++) {
        cv::Scalar color(rand()&255, rand()&255, rand()&255 );
        cv::drawContours(dst, round_contours, idx, color, cv::LineTypes::LINE_4);
    }
    cv::imshow("Obstacle Cloud Image", dst);
    cv::waitKey(30);
}

void ContourDetector::ExtractRefinedContours(const cv::Mat& imgIn,
                                            std::vector<CVPointStack>& refined_contours) 
{ 

    std::vector<std::vector<cv::Point2i>> raw_contours;
    refined_contours.clear(), refined_hierarchy_.clear();
    cv::findContours(imgIn, raw_contours, refined_hierarchy_, 
                     cv::RetrievalModes::RETR_TREE, 
                     cv::ContourApproximationModes::CHAIN_APPROX_TC89_L1);
                     
    refined_contours.resize(raw_contours.size());
    for (std::size_t i=0; i<raw_contours.size(); i++) {
        // using Ramer–Douglas–Peucker algorithm url: https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
        cv::approxPolyDP(raw_contours[i], refined_contours[i], DIST_LIMIT, true);
    }
    this->TopoFilterContours(refined_contours); 
    this->AdjecentDistanceFilter(refined_contours);
}

void ContourDetector::AdjecentDistanceFilter(std::vector<CVPointStack>& contoursInOut) {
    /* filter out vertices that are overlapped with neighbor */
    std::size_t kept = 0;
    for (std::size_t i=0; i<contoursInOut.size(); i++) { 
        auto& contour = contoursInOut[i];
        const std::size_t c_size = contour.size();
        std::size_t refined_idx = 0;
        for (std::size_t j=0; j<c_size; j++) {
            // The write index never exceeds j, so reading in place retains the original next vertex.
            const cv::Point2f p = contour[j];
            if (refined_idx < 1 || FARUtil::PixelDistance(contour[refined_idx-1], p) > DIST_LIMIT) {
                /** Reduce wall nodes */
                RemoveWallConnection(contour, p, refined_idx);
                contour[refined_idx] = p;
                refined_idx ++;
            }
        }
        /** Reduce wall nodes */
        if (refined_idx != 0) RemoveWallConnection(contour, contour[0], refined_idx);
        contour.resize(refined_idx);
        if (refined_idx > 1 && FARUtil::PixelDistance(contour.front(), contour.back()) < DIST_LIMIT) {
            contour.pop_back();
        }
        if (contour.size() >= 3) {
            if (kept != i) contoursInOut[kept] = std::move(contour);
            ++kept;
        }
    }
    contoursInOut.resize(kept);
}

void ContourDetector::TopoFilterContours(std::vector<CVPointStack>& contoursInOut) {
    std::vector<unsigned char> remove(contoursInOut.size(), 0);
    for (std::size_t i=0; i<contoursInOut.size(); i++) {
        if (remove[i]) continue;
        const auto& poly = contoursInOut[i];
        if (poly.size() < 3) {
            remove[i] = 1;
        } else if (!FARUtil::PointInsideAPoly(poly, free_odom_resized_)) {
            InternalContoursIdxs(refined_hierarchy_, i, remove);
        }
    }
    // Keep hierarchy indices valid until every descendant has been marked.
    std::size_t kept = 0;
    for (std::size_t i=0; i<contoursInOut.size(); ++i) {
        if (remove[i]) continue;
        if (kept != i) contoursInOut[kept] = std::move(contoursInOut[i]);
        ++kept;
    }
    contoursInOut.resize(kept);
}



