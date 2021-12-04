#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <boost/functional/hash.hpp>


#include "intersection.h"
#include "time_measure.h"

#define CANTORPAIR(X,Y) 0.5*(X+Y)*(X+Y+1)+Y

using namespace cv;
using namespace std;

struct ConnectPair
{
    Point2f start_p;
    Point2f end_p;

    ConnectPair() = default;
    ConnectPair(Point2f p1, Point2f p2):start_p(p1), end_p(p2) {}
    
    bool operator ==(const ConnectPair& pt) const 
    {
        return (this->start_p == pt.start_p && this->end_p == pt.end_p) || (this->start_p == pt.end_p && this->end_p == pt.start_p);
    }
};

const static float RESIZE_RATIO = 5.0;
const static int BLUR_SIZE = 5;
const static float ALIGN_ANGLE = 20.0;
static Point2f ROBOT_POS;

static TimeMeasure TIMER;


void ProcessImg(const Mat& img,
                Mat& processed_img);

void ExtractContours(const Mat& processed_img,
                     vector<vector<Point2f>>& contours,
                     vector<Vec4i>& hierarchy);

float PixelDistance(const Point2f& pre_p,
                    const Point2f& cur_p);

int Mod(const int& a, const int& b) {
  return (b + (a % b)) % b;
}

void CopyContours(const vector<vector<Point2i>>& raw_contours,
                  vector<vector<Point2f>>& contours);

void RoundContours(const vector<vector<Point2f>>& filtered_contours,
                   vector<vector<Point2i>>& round_contours);

bool IsPrevWallVertex(const Point2f& first_p,
                      const Point2f& mid_p,
                      const Point2f& add_p);

void RemoveWallConnection(const vector<Point2f>& contour,
                          const Point2f& add_p,
                          std::size_t& refined_idx);

void InternalContours(const vector<Vec4i>& hierarchy,
                      const size_t high_idx,
                      unordered_set<int>& internal_idxs);

void SameAndLowLevelIdxs(const vector<Vec4i>& hierarchy,
                         const size_t cur_idx,
                         unordered_set<int>& remove_idxs); 
        
void ShowImgAndContours(const Mat& imgIn,
                        const vector<Vec6f> convex_vertices,
                        const vector<Vec6f> concave_vertices,
                        const vector<vector<Point2f>>& contours,
                        const unordered_set<int>& removed_idxs,
                        const vector<ConnectPair>& connect_edges);

void ConvexVerticesAnalysis(const vector<vector<cv::Point2f>>& filtered_contours,
                            vector<Vec6f>& convex_vertices,
                            vector<Vec6f>& concave_vertices);

void ExtractConnectionEdge(const vector<Vec6f>& convex_vertices,
                           vector<ConnectPair>& connect_edges);

bool IsPointInVetexAngleRestriction(const Vec6f& vetex, const Point2f& p);

void RemoveEdgeCollision(const vector<vector<cv::Point2f>>& filtered_contours,
                         vector<ConnectPair>& edges);

bool IsEdgeCollidePoly(const vector<cv::Point2f>& poly, const ConnectPair& edge);

ConnectPair ShorterEdge(const ConnectPair& edge, const float& dist);


void FilterContours(const unordered_set<int>& remove_idxs, 
                    const vector<vector<cv::Point2f>>& contoursIn,
                    vector<vector<cv::Point2f>>& contoursOut);

bool PointInsideAPoly(const vector<Point2f>& poly,
                      const Point2f& p);

Point2f NormalizedCVPoint(const Point2f& cv_p) {
    Point2f return_p = cv_p;
    const float norm = hypotf(return_p.x, return_p.y);
    if (norm > 1e-5) {
        return_p.x /= norm, return_p.y /= norm;
    } else {
        cout<<"Error: normalization fails, vector norm too small."<<endl;
    }
    return return_p;
}



int main() {
    Mat img_mat, processed_img;
    std::string folder_path = "/media/fan.yang/data/IMAGES/nsh_3floor/";
    const std::size_t total_size = 1000;
    for (std::size_t i=0; i<total_size; i++) {
        std::string img_path = folder_path + std::to_string(i) + ".tiff";
        img_mat = imread(img_path, IMREAD_UNCHANGED);
        if (img_mat.empty())
        {
            std::cout << "Could not read the image: " << img_path << std::endl;
            return 1;
        }

        ProcessImg(img_mat, processed_img);
        ROBOT_POS.x = processed_img.size().height / 2.0, ROBOT_POS.y = processed_img.size().width / 2.0;  
        vector<vector<Point2f>> contours;
        vector<vector<Point2f>> filtered_contours;
        vector<Vec6f> convex_vertices;
        vector<Vec6f> concave_vertices;
        vector<Vec4i> hierarchy;
        unordered_set<int> remove_idxs;
        vector<ConnectPair> connect_edge;
        vector<ConnectPair> contour_edge;
        cout<<"------------------------------------------------------"<<endl;
        TIMER.start_time();
        ExtractContours(processed_img, contours, hierarchy);
        TIMER.end_time("Contour Extraction");
        TIMER.start_time();
        for (int i=0; i<contours.size(); i++) {
            if (remove_idxs.find(i) != remove_idxs.end()) continue;
            const auto poly = contours[i];
            if (poly.size() < 3) {
                remove_idxs.insert(i);
            } else if (!PointInsideAPoly(poly, ROBOT_POS)) {
                InternalContours(hierarchy, i, remove_idxs);
            }
        }
        FilterContours(remove_idxs, contours, filtered_contours);
        TIMER.end_time("Topo Contour Filter");
        TIMER.start_time();
        ConvexVerticesAnalysis(filtered_contours, convex_vertices, concave_vertices);
        TIMER.end_time("Convex Vertices Extraction");
        TIMER.start_time();
        ExtractConnectionEdge(convex_vertices, connect_edge);
        const std::size_t raw_contour_size = connect_edge.size();
        RemoveEdgeCollision(filtered_contours, connect_edge);
        TIMER.end_time("Polygon Connection Analysis");
        cout<<"Convex Vertices: "<<convex_vertices.size()<<"; Raw contour size: "<<raw_contour_size<<"; Connect Edge: "<<connect_edge.size()<<endl;
        ShowImgAndContours(processed_img, convex_vertices, concave_vertices, contours, remove_idxs, connect_edge);
    }
    return 0;
}

void ExtractContours(const Mat& processed_img,
                     vector<vector<Point2f>>& contours,
                     vector<Vec4i>& hierarchy) {
    vector<vector<Point2i>> raw_contours;
    findContours(processed_img, raw_contours, hierarchy, 
                 RetrievalModes::RETR_TREE, 
                 ContourApproximationModes::CHAIN_APPROX_TC89_L1);
    const float vertex_dist_limit = RESIZE_RATIO * 2.5;
    CopyContours(raw_contours, contours);
    for (std::size_t i=0; i<raw_contours.size(); i++) {
        const auto c = raw_contours[i];
        const std::size_t c_size = c.size();
        std::size_t refined_idx = 0;
        for (std::size_t j=0; j<c_size; j++) {
            Point2f p = c[j]; 
            if (refined_idx < 1 || PixelDistance(contours[i][refined_idx-1], p) > vertex_dist_limit) {
                /** Reduce wall nodes */
                RemoveWallConnection(contours[i], p, refined_idx);
                contours[i][refined_idx] = p;
                refined_idx ++;
            }
        }
        RemoveWallConnection(contours[i], contours[i][0], refined_idx);
        contours[i].resize(refined_idx);
    }
}

bool PointInsideAPoly(const vector<Point2f>& poly,
                      const Point2f& p) 
{
    // By Randolph Franklin, https://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
    int i, j, c = 0;
    int npol = poly.size();
      for (i = 0, j = npol-1; i < npol; j = i++) {
          const Point2f vetex1 = poly[i];
          const Point2f vetex2 = poly[j];
        if ((((vetex1.y <= p.y) && (p.y < vetex2.y)) ||
             ((vetex2.y <= p.y) && (p.y < vetex1.y))) &&
            (p.x < (vetex2.x - vetex1.x) * (p.y - vetex1.y) / (vetex2.y - vetex1.y) + vetex1.x))
          c = !c;
      }
      return c;
}

void InternalContours(const vector<Vec4i>& hierarchy,
                      const size_t high_idx,
                      unordered_set<int>& internal_idxs)
{
    if (hierarchy[high_idx][2] == -1) return;
    SameAndLowLevelIdxs(hierarchy, hierarchy[high_idx][2], internal_idxs);
}

void SameAndLowLevelIdxs(const vector<Vec4i>& hierarchy,
                         const size_t cur_idx,
                         unordered_set<int>& remove_idxs)
{
    if (cur_idx == -1) return;
    int next_idx = cur_idx;
    while (next_idx != -1) {
        remove_idxs.insert(next_idx);
        SameAndLowLevelIdxs(hierarchy, hierarchy[next_idx][2], remove_idxs);
        next_idx = hierarchy[next_idx][0];
    }
}

void ShowImgAndContours(const Mat& imgIn,
                        const vector<Vec6f> convex_vertices,
                        const vector<Vec6f> concave_vertices,
                        const vector<vector<Point2f>>& contours,
                        const unordered_set<int>& removed_idxs,
                        const vector<ConnectPair>& connect_edges)
{
    TIMER.start_time();
    cv::Mat img_show = cv::Mat::zeros(imgIn.size(), CV_8UC3);
    for (const auto& cv_p : convex_vertices) {
            cv::circle(img_show, Point2i(cv_p[0], cv_p[1]), 7, Scalar(0, 255, 255), -1);
    }
    for (const auto& cv_p : concave_vertices) {
            cv::circle(img_show, Point2i(cv_p[0], cv_p[1]), 5, Scalar(128, 128, 128), -1);
    }
    // visualize robot
    cv::circle(img_show, (cv::Point2i)ROBOT_POS, 5, Scalar(0, 64, 255), -1);
    // visualize contours
    vector<vector<Point2i>> round_contours;
    RoundContours(contours, round_contours);
    for(std::size_t idx=0; idx<round_contours.size(); idx++)
    {   
        cv::Scalar color(200, 200, 200);
        if (removed_idxs.find(idx) != removed_idxs.end()) {
            color = cv::Scalar(255, 20, 255);
        }
        cv::drawContours(img_show, round_contours, idx, color, cv::LineTypes::LINE_4);
    }
    // visual edge
    for (const auto& edge : connect_edges) {
        cv::line(img_show, edge.start_p, edge.end_p, Scalar(0, 128, 255), 1);
    }

    imshow("Display window", img_show);
    TIMER.end_time("Visualization");
    int k = waitKey(0); // Wait for a keystroke in the window
    if(k == 's')
    {
        imwrite("starry_night.png", img_show);
    }
}

void ProcessImg(const Mat& img, Mat& processed_img) {
    img.convertTo(processed_img, CV_8UC1, 255);
    cv::resize(processed_img, processed_img, cv::Size(), RESIZE_RATIO, RESIZE_RATIO, 
               cv::InterpolationFlags::INTER_LINEAR);
    cv::blur(processed_img, processed_img, cv::Size(BLUR_SIZE, BLUR_SIZE));
} 

float PixelDistance(const Point2f& pre_p, const Point2f& cur_p) {
    return std::hypotf(pre_p.x - cur_p.x, pre_p.y - cur_p.y);
}

bool IsPrevWallVertex(const Point2f& first_p,
                      const Point2f& mid_p,
                      const Point2f& add_p)
{
    Point2f diff_p1 = first_p - mid_p;
    Point2f diff_p2 = add_p - mid_p;
    const float vec1_len = std::hypotf(diff_p1.x, diff_p1.y);
    const float vec2_len = std::hypotf(diff_p2.x, diff_p2.y);
    diff_p1 = NormalizedCVPoint(diff_p1);
    diff_p2 = NormalizedCVPoint(diff_p2);
    const float dot_value = diff_p1.dot(diff_p2);
    if (dot_value >= 1.0 || dot_value <= -1.0) return true;
    const float angle = acos(abs(dot_value));
    if (angle < (ALIGN_ANGLE / 180.0 * M_PI)) {
        return true;
    }
    const float height = sin(M_PI - acos(dot_value)) * vec1_len;
    if (height < RESIZE_RATIO * 2.5) {
        return true;
    }
    return false;
}

void FilterContours(const unordered_set<int>& remove_idxs,
                    const vector<vector<cv::Point2f>>& contoursIn,
                    vector<vector<cv::Point2f>>& contoursOut) 
{
    contoursOut.clear();
    for (int i=0; i<contoursIn.size(); i++) {
        if (remove_idxs.find(i) != remove_idxs.end()) continue;
        contoursOut.push_back(contoursIn[i]);
    }
}

void ConvexVerticesAnalysis(const vector<vector<cv::Point2f>>& filtered_contours,
                            vector<Vec6f>& convex_vertices,
                            vector<Vec6f>& concave_vertices)
{
    convex_vertices.clear(), concave_vertices.clear();
    for (const auto& contour : filtered_contours) {
        const bool in_or_out = PointInsideAPoly(contour, ROBOT_POS);
        const int N = contour.size();
        for (int i=0; i<N; i++) {
            const Point2f first_p = contour[Mod(i-1, N)];
            const Point2f center_p = contour[i];
            const Point2f second_p = contour[Mod(i+1, N)];
            const Point2f dir1 = NormalizedCVPoint(first_p - center_p);
            const Point2f dir2 = NormalizedCVPoint(second_p - center_p);
            const Point2f dir_c = NormalizedCVPoint(dir1 + dir2);
            const Point2f ev_p = center_p + dir_c * RESIZE_RATIO;
            if (!PointInsideAPoly(contour, ev_p) == in_or_out) {
                Vec6f vertex(center_p.x, center_p.y, dir1.x, dir1.y, dir2.x, dir2.y);
                convex_vertices.push_back(vertex);
            } else {
                Vec6f vertex(center_p.x, center_p.y, dir1.x, dir1.y, dir2.x, dir2.y);
                concave_vertices.push_back(vertex);
            }
        }
    }
}

void RemoveWallConnection(const vector<Point2f>& contour,
                          const Point2f& add_p,
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

void CopyContours(const vector<vector<Point2i>>& raw_contours,
                  vector<vector<Point2f>>& contours)
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

void RoundContours(const vector<vector<Point2f>>& filtered_contours,
                   vector<vector<Point2i>>& round_contours) 
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

void ExtractConnectionEdge(const vector<Vec6f>& convex_vertices,
                           vector<ConnectPair>& connect_edges)
{
    const int N = convex_vertices.size();
    connect_edges.clear(), connect_edges.resize(N*(N-1)/2);
    std::size_t idx_c = 0;
    for (std::size_t i=0; i<N; i++) {
        for (std::size_t j=0; j<N; j++) {
            if (i == j || j > i) continue;
            const Point2f cv_p1(convex_vertices[i][0], convex_vertices[i][1]);
            const Point2f cv_p2(convex_vertices[j][0], convex_vertices[j][1]);
            ConnectPair edge(cv_p1, cv_p2);
            if (!IsPointInVetexAngleRestriction(convex_vertices[i], cv_p2) ||
                !IsPointInVetexAngleRestriction(convex_vertices[j], cv_p1)) 
            {
                // connection is not in direction restrict
                continue;
            }
            connect_edges[idx_c] = edge;
            idx_c ++;
        }
    }
    connect_edges.resize(idx_c);
}

bool IsPointInVetexAngleRestriction(const Vec6f& vetex, const Point2f& p) {
    const Point2f center_p(vetex[0], vetex[1]);
    const Point2f surf_dir1(vetex[2], vetex[3]);
    const Point2f surf_dir2(vetex[4], vetex[5]);
    if (hypotf(p.x - center_p.x, p.y - center_p.y) < 1e-5) {
        cout<<"The two vertex are too close, connection fails."<<endl;
        return false;
    }
    const Point2f p_dir = NormalizedCVPoint(p - center_p);
    Point2f temp_opposite_dir = -surf_dir2;
    float thred_dot_value = surf_dir1.dot(temp_opposite_dir);
    if (p_dir.dot(surf_dir1) > thred_dot_value && p_dir.dot(temp_opposite_dir) > thred_dot_value) {
        return true;
    }
    temp_opposite_dir = -surf_dir1;
    thred_dot_value = surf_dir2.dot(temp_opposite_dir);
    if (p_dir.dot(surf_dir2) > thred_dot_value && p_dir.dot(temp_opposite_dir) > thred_dot_value) {
        return true;
    }
    return false;
}

void RemoveEdgeCollision(const vector<vector<cv::Point2f>>& filtered_contours,
                         vector<ConnectPair>& edges)
{
    for (auto it = edges.begin(); it != edges.end();) {
        ConnectPair short_edge = ShorterEdge(*it, 1.0);
        bool is_collide = false;
        for (const auto& poly : filtered_contours) {
            if (IsEdgeCollidePoly(poly, short_edge)) {
                it = edges.erase(it);
                is_collide = true;
                break;
            }
        }
        if (!is_collide) {
            ++ it;
        }
    }
}

bool IsEdgeCollidePoly(const vector<cv::Point2f>& poly, const ConnectPair& edge) {
    const int N = poly.size();
    if (N < 3) cout<<"Poly vertex size less than 3."<<endl;
    for (int i=0; i<N; i++) {
        const Point2f start_p = poly[i];
        const Point2f end_p = poly[Mod(i+1, N)];
        if (doIntersect(start_p, end_p, edge.start_p, edge.end_p)) {
            return true;
        }
    }
    return false;
}

ConnectPair ShorterEdge(const ConnectPair& edge, const float& dist) {
    ConnectPair short_edge = edge;
    const float norm = hypotf(edge.end_p.x - edge.start_p.x, edge.end_p.y - edge.start_p.y);
    const float ref_dist = min(norm*(float)0.1, dist);
    Point2f dir = NormalizedCVPoint(edge.end_p - edge.start_p);
    short_edge.start_p += ref_dist * dir;
    short_edge.end_p -= ref_dist * dir;
    return short_edge;
}



/************************* UNUSE CODE ***************************/
// vector<Vec4i> hierarchy = {Vec4i{7, -1, 1, -1},
//                             Vec4i{-1, -1, 2, 0},
//                             Vec4i{-1, -1, 3, 1},
//                             Vec4i{-1, -1, 4, 2},
//                             Vec4i{-1, -1, 5, 3},
//                             Vec4i{6, -1, -1, 4},
//                             Vec4i{-1, 5, -1, 4},
//                             Vec4i{8, 0, -1, -1},
//                             Vec4i{-1, 7, -1, -1}};
// unordered_set<int> remove_idxs;
// InternalContours(hierarchy, 1, remove_idxs);
// std::cout<<"Remove idxs: ";
// for (const auto& idx : remove_idxs) {
//     std::cout<<idx<<", ";
// }
// std::cout<<std::endl;

// vector<Point2i> poly = {Point2i(10,10),
//                         Point2i(15,15),
//                         Point2i(10,20)};
// const Point2f robot_pos(14.9999, 15.0);
// if (PointInsideAPoly(poly, robot_pos)) {
//     std::cout<<"Point in Poly"<<std::endl;
// } else {
//     std::cout<<"Point outside Poly"<<std::endl;
// }

// bool IsAdjacentVertex(const std::vector<cv::Point2i>& contour,
//                       const std::size_t cur_idx,
//                       const std::size_t horizon,
//                       std::size_t& adjacent_id) 
// {
//     const int c_size = contour.size();
//     if (c_size < 3 || horizon < 3) return false;
//     const auto cur_p = contour[cur_idx];
//     for (std::size_t i=2; i<horizon; i++) {
//         const int ref_idx = cur_idx + i;
//         if (ref_idx == c_size) break;
//         const auto ref_p = contour[ref_idx];
//         if (PixelDistance(cur_p, ref_p) < RESIZE_RATIO * 2.5) {
//             adjacent_id = ref_idx;
//             return true;
//         }
//     }
//     return false;
// }
