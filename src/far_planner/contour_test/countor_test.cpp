
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <iostream>

using namespace cv;

typedef std::vector<Point2f> CVPointStack;
const static float RESIZE_RATIO = 5.0;
const static int BLUR_SIZE = 10;
const static float ALIGN_ANGLE = 20.0;


Mat BlurImg(const Mat& imgIn);
void ExtractVertex(const Mat& ImgResize,
                   std::vector<std::vector<cv::Point2i>>& contours,
                   std::vector<cv::Vec4i>& hierarchy,
                   CVPointStack& vertex_stack);

void ConvertContourToVertex(const std::vector<std::vector<cv::Point2i>>& contours,
                            const std::size_t& max_size,
                            CVPointStack& vertex_stack);

void ShowImgAndVertex(const Mat& ImgResize, 
                      const std::vector<std::vector<cv::Point2i>>& contours,
                      const std::vector<cv::Vec4i>& hierarchy, 
                      const CVPointStack& vertex_stack);

float PixelDistance(const Point2f& pre_p,
                    const Point2f& cur_p);

void FilterContours(std::vector<std::vector<cv::Point2i>>& contours);

bool IsAdjacentVertex(const std::vector<cv::Point2i>& contour,
                      const std::size_t cur_idx,
                      const std::size_t horizon,
                      std::size_t& adjecent_id);

bool IsPrevWallVertex(const Point2f& first_p,
                      const Point2f& mid_p,
                      const Point2f& add_p);

int main() {
    std::string folder_path = "/media/fan.yang/data/IMAGES/high_bay/";
    const std::size_t total_size = 750;
    for (std::size_t i=0; i<total_size; i++) {
        std::string img_path = folder_path + std::to_string(i) + ".tiff";
        Mat img = imread(img_path, IMREAD_UNCHANGED);
        if (img.empty())
        {
            std::cout << "Could not read the image: " << img_path << std::endl;
            return 1;
        }
        Mat imgResize = BlurImg(img);
        std::vector<Point2f> vertex_stack;
        std::vector<std::vector<cv::Point2i>> contours;
        std::vector<cv::Vec4i> hierarchy;
        ExtractVertex(imgResize, contours, hierarchy, vertex_stack);
        ShowImgAndVertex(imgResize, contours, hierarchy, vertex_stack);
    }
    return 0;
}

Mat BlurImg(const Mat& imgIn) {
    Mat Rimg;
    imgIn.convertTo(Rimg, CV_8UC1, 255);
    cv::resize(Rimg, Rimg, cv::Size(), RESIZE_RATIO, RESIZE_RATIO, 
               cv::InterpolationFlags::INTER_LINEAR);
    cv::blur(Rimg, Rimg, cv::Size(BLUR_SIZE, BLUR_SIZE));
    return Rimg;
}

void ExtractVertex(const Mat& ImgResize,
                   std::vector<std::vector<cv::Point2i>>& contours,
                   std::vector<cv::Vec4i>& hierarchy, 
                   CVPointStack& vertex_stack) 
{
    findContours(ImgResize, contours, hierarchy, 
                 RetrievalModes::RETR_TREE, 
                 ContourApproximationModes::CHAIN_APPROX_TC89_L1);
    std::size_t idx = 0;
    const float vertex_dist_limit = RESIZE_RATIO * 2.5;
    std::vector<std::vector<cv::Point2i>> refined_contours;
    refined_contours = contours;
    for (std::size_t i=0; i<contours.size(); i++) {
        const auto c = contours[i];
        const std::size_t c_size = c.size();
        std::size_t refined_idx = 0;
        Point2f pre_p;
        bool is_vertex = true;
        for (std::size_t j=0; j<c_size; j++) {
            auto p = c[j]; 
            if (refined_idx == 0 || PixelDistance(pre_p, p) > vertex_dist_limit) {
                if (!is_vertex && PixelDistance(refined_contours[i][refined_idx-1], pre_p) > vertex_dist_limit) {
                    /** Reduce wall nodes */
                    if (refined_idx > 1 && IsPrevWallVertex(refined_contours[i][refined_idx-2], refined_contours[i][refined_idx-1], pre_p)) {
                        refined_contours[i].pop_back();
                        idx --;
                        refined_idx --;
                    }
                    refined_contours[i][refined_idx] = pre_p;
                    idx ++;
                    refined_idx ++;
                }
                std::size_t ad_idx;
                if (IsAdjacentVertex(c, j, 10, ad_idx)) {
                    j = ad_idx;
                    p = (p + c[j]) / 2.0;
                }
                /** Reduce wall nodes */
                if (refined_idx > 1 && IsPrevWallVertex(refined_contours[i][refined_idx-2], refined_contours[i][refined_idx-1], p)) {
                    refined_contours[i].pop_back();
                    idx --;
                    refined_idx --;
                }
                refined_contours[i][refined_idx] = p;
                idx ++;
                refined_idx ++;
                is_vertex = true;
                pre_p = p;
            } else {
                is_vertex = false;
            }
        }
        refined_contours[i].resize(refined_idx);
    }
    contours = refined_contours;
    FilterContours(contours);
    ConvertContourToVertex(contours, idx, vertex_stack);
    std::cout<<"number of raw detected vertices: "<<idx<<std::endl;
}

void ShowImgAndVertex(const Mat& ImgResize,
                      const std::vector<std::vector<cv::Point2i>>& contours,
                      const std::vector<cv::Vec4i>& hierarchy,
                      const CVPointStack& vertex_stack) 
{
    cv::Mat img_show = cv::Mat::zeros(ImgResize.size(), CV_8UC3);
    // Mat img_show = ImgResize.clone();
    for (const auto& vertex : vertex_stack) {
        cv::circle(img_show, vertex, RESIZE_RATIO, Scalar(128, 128, 128), -1);
    }
    // visualize robot
    cv::circle(img_show, Point2i(ImgResize.size().height/2, ImgResize.size().width/2), RESIZE_RATIO, Scalar(0, 64, 255), -1);
    // visualize contours
    for(std::size_t idx=0; idx<contours.size(); idx++)
    {
        cv::Scalar color(rand()&255, rand()&255, rand()&255 );
        cv::drawContours(img_show, contours, idx, color, cv::LineTypes::LINE_4);
    }

    imshow("Display window", img_show);
    int k = waitKey(0); // Wait for a keystroke in the window
    if(k == 's')
    {
        imwrite("starry_night.png", img_show);
    }
}

float PixelDistance(const Point2f& pre_p, const Point2f& cur_p) {
    return std::hypotf(pre_p.x - cur_p.x, pre_p.y - cur_p.y);
}

bool IsAdjacentVertex(const std::vector<cv::Point2i>& contour,
                      const std::size_t cur_idx,
                      const std::size_t horizon,
                      std::size_t& adjacent_id) 
{
    const int c_size = contour.size();
    if (c_size < 3 || horizon < 3) return false;
    const auto cur_p = contour[cur_idx];
    for (std::size_t i=2; i<horizon; i++) {
        const int ref_idx = cur_idx + i;
        if (ref_idx == c_size) break;
        const auto ref_p = contour[ref_idx];
        if (PixelDistance(cur_p, ref_p) < RESIZE_RATIO * 2.5) {
            adjacent_id = ref_idx;
            return true;
        }
    }
    return false;
}

bool IsPrevWallVertex(const Point2f& first_p,
                      const Point2f& mid_p,
                      const Point2f& add_p)
{
    Point2f diff_p1 = first_p - mid_p;
    Point2f diff_p2 = add_p - mid_p;
    const float vec1_len = std::hypotf(diff_p1.x, diff_p1.y);
    const float vec2_len = std::hypotf(diff_p2.x, diff_p2.y);
    diff_p1 /= vec1_len;
    diff_p2 /= vec2_len;
    const float dot_value = diff_p1.x * diff_p2.x + diff_p1.y * diff_p2.y;
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

void FilterContours(std::vector<std::vector<cv::Point2i>>& contours) {
    std::vector<std::size_t> remove_idxs;
    remove_idxs.clear();
    for (std::size_t i=0; i<contours.size(); i++) {
        if (std::find(remove_idxs.begin(), remove_idxs.end(), i) != remove_idxs.end()) continue;
        const std::size_t N = contours[i].size();
        if (N < 3) {
            remove_idxs.push_back(i);
        }
    }
    std::vector<std::vector<cv::Point2i>> temp_contours = contours;
    contours.clear();
    for (std::size_t i=0; i<temp_contours.size(); i++) {
        if (std::find(remove_idxs.begin(), remove_idxs.end(), i) != remove_idxs.end()) continue;
        contours.push_back(temp_contours[i]);
    }
} 

void ConvertContourToVertex(const std::vector<std::vector<cv::Point2i>>& contours,
                            const std::size_t& max_size,
                            CVPointStack& vertex_stack)
{
    vertex_stack.clear(), vertex_stack.resize(max_size);
    std::size_t idx = 0;
    for (const auto& c : contours) {
        for (const auto& p : c) {
            vertex_stack[idx] = p;
            idx ++;
        }
    }
    vertex_stack.resize(idx);
}



