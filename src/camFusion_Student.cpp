
#include <iostream>
#include <algorithm>
#include <numeric>
#include <set>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // Compute mean and std
    std::vector<double> dists;
    for (auto m : kptMatches) {
        auto prevM = m.queryIdx;
        auto curM = m.trainIdx;

        auto prevP = kptsPrev.at(prevM);
        auto curP = kptsCurr.at(curM);

        if (boundingBox.roi.contains(curP.pt)) {
            double dist = cv::norm(curP.pt - prevP.pt);
            dists.push_back(dist);
        }
    }

    if (dists.size() == 0) return;

    double mean_dist = std::accumulate(dists.begin(), dists.end(), 0.0) / dists.size();
    double accum = 0.0;
    std::for_each (std::begin(dists), std::end(dists), [&](const double d) {
        accum += (d - mean_dist) * (d - mean_dist);
    });
    double stddev = sqrt(accum / (dists.size()-1));

    // Filter matches with distantces within 1 sigma
    for (auto m : kptMatches) {
        auto prevM = m.queryIdx;
        auto curM = m.trainIdx;

        auto prevP = kptsPrev.at(prevM);
        auto curP = kptsCurr.at(curM);

        if (boundingBox.roi.contains(curP.pt)) {
            double dist = cv::norm(curP.pt - prevP.pt);
            if (dist <= mean_dist + stddev && dist >= mean_dist - stddev) {
                boundingBox.keypoints.push_back(curP);
                boundingBox.kptMatches.push_back(m);
            }
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer kpt. loop

        // get current keypoint and its matched partner in the prev. frame
        auto& kpOuterCurr = kptsCurr.at(it1->trainIdx);
        auto& kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner kpt.-loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    // Use median of distRatios to compute TTC.
    std::sort(distRatios.begin(), distRatios.end());
    long idx = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[idx - 1] + distRatios[idx]) / 2.0 : distRatios[idx]; // compute median dist. ratio to remove outlier influence

    double dT = 1 / frameRate;
    TTC = -dT / (1 - medDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double dt = 1.0 / frameRate;
    double laneWidth = 4.0;
    double prevX;
    double curX;

    // Containers for x value in previous and current frame.
    // Use them to obtain median, and use median value to compute TTC.
    vector<double> x_prev, x_cur;

    for (auto& lp : lidarPointsPrev) {
        if (abs(lp.y) < laneWidth / 2) {
            x_prev.push_back(lp.x);
        }
    }

    for (auto& lp : lidarPointsCurr) {
        if (abs(lp.y) < laneWidth / 2) {
            x_cur.push_back(lp.x);
        }
    }

    std::sort(x_prev.begin(), x_prev.end());
    std::sort(x_cur.begin(), x_cur.end());

    // Mean
    prevX = std::accumulate(x_prev.begin(), x_prev.end(), 0.0) / x_prev.size();
    curX = std::accumulate(x_cur.begin(), x_cur.end(), 0.0) / x_cur.size();

    // Median
    // prevX = (x_prev.size() % 2)? (x_prev[x_prev.size()/2-1] + x_prev[x_prev.size()/2])/2.0: x_prev[x_prev.size()/2];
    // curX = (x_cur.size() % 2)? (x_cur[x_cur.size()/2-1] + x_cur[x_cur.size()/2])/2.0: x_cur[x_cur.size()/2];

    TTC = curX * dt / (prevX - curX);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    std::multimap<int, int> bbmultimap;

    // Iterate through all matches
    for (auto m : matches) {
        int prevKptIdx = m.queryIdx;
        int curKptIdx = m.trainIdx;

        auto prevKpt = prevFrame.keypoints[prevKptIdx];
        auto curKpt = currFrame.keypoints[curKptIdx];
        
        int prevBBIdx = std::numeric_limits<int>::min();
        int curBBIdx = std::numeric_limits<int>::min();

        vector<int> prevBBIdxs;
        vector<int> curBBIdxs;

        for (auto bbp : prevFrame.boundingBoxes) {
            if (bbp.roi.contains(prevKpt.pt)) {
                prevBBIdxs.push_back(bbp.boxID);
            }
        }

        for (auto bbc : currFrame.boundingBoxes) {
            if (bbc.roi.contains(curKpt.pt)) {
                curBBIdxs.push_back(bbc.boxID);
            }
        }

        if (prevBBIdxs.size() == 0 || curBBIdxs.size() == 0) continue;

        // Insert all possible bounding box combinations into multimap for later processing.
        for (auto ci : curBBIdxs) {
            for (auto pi : prevBBIdxs) {
                bbmultimap.insert(std::pair<int, int>(ci, pi));
            }
        }
    }

    for (auto b: currFrame.boundingBoxes) {
        std::map<int, int> count;
        auto it = bbmultimap.equal_range(b.boxID);

        // Iterate through box IDs in previous frame which map to b.boxID (a box ID in current frame).
        for (auto itr = it.first; itr != it.second; ++itr) {
            count[itr->second]++;
        }

        // Find the box ID in previous frame which has the most count
        using pair_type = decltype(count)::value_type;
        auto prev_box_cnt_ptr = std::max_element
        (
            std::begin(count), std::end(count),
            [] (const pair_type& p1, const pair_type& p2) {
                return p1.second < p2.second;
            }
        );

        bbBestMatches.insert(pair<int,int>(prev_box_cnt_ptr->first, b.boxID));
    }
}
