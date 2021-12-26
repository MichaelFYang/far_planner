#ifndef CONTOUR_DETECTOR_H
#define CONTOUR_DETECTOR_H

#include "utility.h"


struct ContourDetectParams {
    ContourDetectParams() = default;
    float sensor_range;
    float voxel_dim;
    float kRatio;
    int   kThredValue;
    int   kBlurSize;
    bool  is_save_img;
    std::string img_path;
};

class ContourDetector {
private:
    Point3D odom_pos_;
    cv::Point2f free_odom_resized_;
    ContourDetectParams cd_params_;
    PointCloudPtr new_corners_cloud_;
    cv::Mat img_mat_;
    std::size_t img_counter_;
    std::vector<CVPointStack> refined_contours_;
    std::vector<cv::Vec4i> refined_hierarchy_;
    NavNodePtr odom_node_ptr_;

    int MAT_SIZE, CMAT;
    int MAT_RESIZE, CMAT_RESIZE;
    float DIST_LIMIT;
    float ALIGN_ANGLE_COS;
    float VOXEL_DIM_INV;
    

    void UpdateImgMatWithCloud(const PointCloudPtr& pc, cv::Mat& img_mat);

    void ExtractContourFromImg(const cv::Mat& img,
                               std::vector<CVPointStack>& img_contours,
                               std::vector<PointStack>& realworld_contour);

    void ExtractRefinedContours(const cv::Mat& imgIn,
                                std::vector<CVPointStack>& refined_contours);

    void ResizeAndBlurImg(const cv::Mat& img, cv::Mat& Rimg);

    void ConvertContoursToRealWorld(const std::vector<CVPointStack>& ori_contours,
                                    std::vector<PointStack>& realWorld_contours);

    void TopoFilterContours(std::vector<CVPointStack>& contoursInOut);

    void AdjecentDistanceFilter(std::vector<CVPointStack>& contoursInOut);

    /* inline functions */
    inline void UpdateOdom(const NavNodePtr& odom_node_ptr) {
        odom_pos_ = odom_node_ptr->position;
        odom_node_ptr_ = odom_node_ptr;
        free_odom_resized_ = ConvertPoint3DToCVPoint(FARUtil::free_odom_p, odom_pos_, true);
    }

    inline void ConvertCVToPoint3DVector(const CVPointStack& cv_vec,
                                         PointStack& p_vec,
                                         const bool& is_resized_img) {
        const std::size_t vec_size = cv_vec.size();
        p_vec.clear(), p_vec.resize(vec_size);
        for (std::size_t i=0; i<vec_size; i++) {
            cv::Point2f cv_p = cv_vec[i];
            Point3D p = ConvertCVPointToPoint3D(cv_p, odom_pos_, is_resized_img);
            p_vec[i] = p;
        }
    }

    inline void RemoveWallConnection(const CVPointStack& contour,
                                     const cv::Point2f& add_p,
                                     std::size_t& refined_idx)
    {
        if (refined_idx < 2) return;
        if (!IsPrevWallVertex(contour[refined_idx-2], contour[refined_idx-1], add_p)) {
            return;
        } else {
            -- refined_idx;
            RemoveWallConnection(contour, add_p, refined_idx);
        }
    }

    inline void InternalContoursIdxs(const std::vector<cv::Vec4i>& hierarchy,
                                     const std::size_t& high_idx,
                                     std::unordered_set<int>& internal_idxs)
    {
        if (hierarchy[high_idx][2] == -1) return;
        SameAndLowLevelIdxs(hierarchy, hierarchy[high_idx][2], internal_idxs);
    }

    inline void SameAndLowLevelIdxs(const std::vector<cv::Vec4i>& hierarchy,
                                    const std::size_t& cur_idx,
                                    std::unordered_set<int>& remove_idxs)
    {
        if (cur_idx == -1) return;
        int next_idx = cur_idx;
        while (next_idx != -1) {
            remove_idxs.insert(next_idx);
            SameAndLowLevelIdxs(hierarchy, hierarchy[next_idx][2], remove_idxs);
            next_idx = hierarchy[next_idx][0];
        }
    }

    template <typename Point>
    inline cv::Point2f ConvertPoint3DToCVPoint(const Point& p, 
                                               const Point3D& c_pos,
                                               const bool& is_resized_img=false) {
        cv::Point2f cv_p;
        int row_idx, col_idx;
        this->PointToImgSub(p, c_pos, row_idx, col_idx, is_resized_img);
        cv_p.x = col_idx;
        cv_p.y = row_idx;
        return cv_p;
    }


    template <typename Point>
    inline void PointToImgSub(const Point& posIn, const Point3D& c_posIn,
                              int& row_idx, int& col_idx,
                              const bool& is_resized_img=false,
                              const bool& is_crop_idx=true) 
    {
        const float ratio = is_resized_img ? cd_params_.kRatio : 1.0f;
        const int c_idx = is_resized_img ? CMAT_RESIZE : CMAT;
        row_idx = c_idx + (int)std::round((posIn.x - c_posIn.x) * VOXEL_DIM_INV * ratio);
        col_idx = c_idx + (int)std::round((posIn.y - c_posIn.y) * VOXEL_DIM_INV * ratio);
        if (is_crop_idx) {
            CropIdxes(row_idx, col_idx, is_resized_img);
        } 
    }

    inline void CropIdxes(int& row_idx, int& col_idx, const bool& is_resized_img=false) {
        const int max_size = is_resized_img ? MAT_RESIZE : MAT_SIZE;
        row_idx = (int)std::max(std::min(row_idx, max_size-1),0);
        col_idx = (int)std::max(std::min(col_idx, max_size-1),0);
    }

    inline bool IsIdxesInImg(int& row_idx, int& col_idx, const bool& is_resized_img=false) {
        const int max_size = is_resized_img ? MAT_RESIZE : MAT_SIZE;
        if (row_idx < 0 || row_idx > max_size-1 || col_idx < 0 || col_idx > max_size-1) {
            return false;
        }
        return true;
    }

    inline void ResetImgMat(cv::Mat& img_mat) {
        img_mat.release();
        img_mat = cv::Mat::zeros(MAT_SIZE, MAT_SIZE, CV_32FC1);
    }

    inline Point3D ConvertCVPointToPoint3D(const cv::Point2f& cv_p,
                                           const Point3D& c_pos,
                                           const bool& is_resized_img=false) {
        Point3D p;
        const int c_idx = is_resized_img ? CMAT_RESIZE : CMAT;
        const float ratio = is_resized_img ? cd_params_.kRatio : 1.0f;
        p.x = (cv_p.y - c_idx) * cd_params_.voxel_dim / ratio + c_pos.x;
        p.y = (cv_p.x - c_idx) * cd_params_.voxel_dim / ratio + c_pos.y;
        p.z = odom_pos_.z;
        return p;
    }

    inline void SaveCurrentImg(const cv::Mat& img) {
        if (img.empty()) return;
        cv::Mat img_save;
        img.convertTo(img_save, CV_8UC1, 255);
        std::string filename = std::to_string(img_counter_);
        std::string img_name = cd_params_.img_path + filename + ".tiff";
        cv::imwrite(img_name, img_save);
        if (FARUtil::IsDebug) ROS_WARN_THROTTLE(1.0, "CD: image save success!");
        img_counter_ ++;
    }

    inline bool IsPrevWallVertex(const cv::Point2f& first_p,
                                 const cv::Point2f& mid_p,
                                 const cv::Point2f& add_p)
    {
        cv::Point2f diff_p1 = first_p - mid_p;
        cv::Point2f diff_p2 = add_p - mid_p;
        diff_p1 /= std::hypotf(diff_p1.x, diff_p1.y);
        diff_p2 /= std::hypotf(diff_p2.x, diff_p2.y);
        if (abs(diff_p1.dot(diff_p2)) > ALIGN_ANGLE_COS) return true;
        return false;
    }

    inline void CopyContours(const std::vector<std::vector<cv::Point2i>>& raw_contours,
                             std::vector<std::vector<cv::Point2f>>& contours)
    {
        const std::size_t N = raw_contours.size();
        contours.clear(), contours.resize(N);
        for (std::size_t i=0; i<N; i++) {
            const auto c = raw_contours[i];
            const std::size_t c_size = c.size();
            contours[i].resize(c_size);
            for (std::size_t j=0; j<c.size(); j++) {
                contours[i][j] = (cv::Point2f)c[j];
            }
        }
    }

    inline void RoundContours(const std::vector<std::vector<cv::Point2f>>& filtered_contours,
                              std::vector<std::vector<cv::Point2i>>& round_contours) 
    {
        const std::size_t N = filtered_contours.size();
        round_contours.clear(), round_contours.resize(N);
        for (std::size_t i=0; i<N; i++) {
            const auto c = filtered_contours[i];
            const std::size_t c_size = c.size();
            round_contours[i].resize(c_size);
            for (std::size_t j=0; j<c.size(); j++) {
                round_contours[i][j] = (cv::Point2i)c[j];
            }
        }
    }


public:
    ContourDetector() = default;
    ~ContourDetector() = default;

    void Init(const ContourDetectParams& params);
    
    /**
     * Build terrian occupancy image and extract current terrian contour 
     * @param odom_node_ptr current odom node pointer
     * @param surround_cloud surround obstacle cloud used for updating corner image
     * @param real_world_contour [return] current contour in world frame
    */
    void BuildTerrainImgAndExtractContour(const NavNodePtr& odom_node_ptr, 
                                          const PointCloudPtr& surround_cloud,
                                          std::vector<PointStack>& realworl_contour);

    /**
     * Show Corners on Pointcloud projection image
     * @param img_mat pointcloud projection image
     * @param point_vec corners vector detected from cv corner detector
    */
    void ShowCornerImage(const cv::Mat& img_mat,
                         const PointCloudPtr& pc);
    /* Get Internal Values */
    const PointCloudPtr GetNewVertices() const { return new_corners_cloud_;};
    const cv::Mat       GetCloudImgMat() const { return img_mat_;};
};

#endif
