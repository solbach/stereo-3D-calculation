#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace cv;

void readme();

int main(int argc, char** argv )
{
    // load images
    if ( argc != 3 )
    {
        std::cout << "usage: stereo-3D-calculation.out <Left_Image_Path>" <<
                     "<Right_Image_Path>\n" << std::endl;
        return -1;
    }

    Mat imageLeft, imageRight, grayLeft, grayRight, outputLeft, outputRight;

    imageLeft = imread( argv[1], 1 );
    imageRight = imread( argv[2], 1 );

    if ( !imageLeft.data || !imageRight.data)
    {
        readme();
        return -1;
    }

    std::cout << "[successfull] load images" << std::endl;

    // SIFT
    cvtColor(imageLeft, grayLeft, COLOR_BGR2GRAY, 1);
    cvtColor(imageRight, grayRight, COLOR_BGR2GRAY, 1);

    SiftFeatureDetector detector;
    SiftDescriptorExtractor extractor;

    std::vector<KeyPoint> keypointsLeft, keypointsRight;
    detector.detect(grayLeft, keypointsLeft);
    detector.detect(grayLeft, keypointsRight);

    Mat descriptorsLeft, descriptorsRight;

    extractor.compute(grayLeft, keypointsLeft, descriptorsLeft);
    extractor.compute(grayRight, keypointsRight, descriptorsRight);

    std::cout << "[successfull] SIFT" << std::endl;

    // FLANN
    FlannBasedMatcher matcher;
    std::vector<DMatch> matches;
    matcher.match( descriptorsLeft, descriptorsRight, matches);

    double max_dist = 0;
    double min_dist = 100;

    for (int i = 0; i < descriptorsLeft.rows; ++i) {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    std::cout << "## Max dist.: " << max_dist << std::endl;
    std::cout << "## Min dist.: " << min_dist << std::endl;

    std::vector<DMatch> inliers;
    for (int j = 0; j < descriptorsLeft.rows; ++j) {
        if( matches[j].distance <= max(2*min_dist, 0.01) )
            inliers.push_back( matches[j]);
    }

    // Draw Inliers
    Mat img_inliers;
    drawMatches(imageLeft, keypointsLeft, imageRight, keypointsRight,
                inliers, img_inliers, Scalar::all(-1), Scalar::all(-1),
                std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    drawKeypoints(grayLeft, keypointsLeft, outputLeft);
    drawKeypoints(grayRight, keypointsRight, outputRight);

    std::cout << "[successfull] FLANN" << std::endl;

    // Show Windows <left, right>
    imshow("Stereo Matching", img_inliers);

    for( int i = 0; i < (int)inliers.size(); i++ )
    { printf( "## Inlier [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n",
              i, inliers[i].queryIdx, inliers[i].trainIdx ); }

    std::cout << "[successfull] show windows" << std::endl;

    waitKey(0);

    std::cout << "TERMINATE" << std::endl;

    return 0;
}

void readme()
{
    std::cout << "On or more image data are broken \n" << std::endl;
}
