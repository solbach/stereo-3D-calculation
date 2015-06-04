#include <stdio.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>

using namespace cv;

/*
 * Camera Parameter
 *
 * intrinsic1 = [749.642742046463, 0.0, 539.67454188334; ...
 *               0.0, 718.738253774844, 410.819033898981; 0.0, 0.0, 1.0];
 * radial1     = [-0.305727818014552, 0.125105811097608, 0.0021235435545915];
 * tangential1 = [0.00101183009692414, 0.0];
 *
 * intrinsic2 = [747.473744648049, 0.0, 523.981339714942; ...
 *               0.0, 716.76909875026, 411.218247507688; 0.0, 0.0, 1.0];
 * radial2     = [-0.312470781595577, 0.140416928438558, 0.00187045432179417];
 * tangential2 = [-0.000772438457736498, 0.0];
 */

void readme()
{
    std::cout << "[ERROR] \t On or more provided images are broken \n"
              << std::endl;
}

void sharpenImage(Mat& imageRight, Mat& imageLeft, Mat& rightBlur, Mat& leftBlur)
{
    GaussianBlur(imageLeft, leftBlur, Size(0, 0), 3);
    addWeighted(imageLeft, 3, leftBlur, -2, 0, imageLeft);
    GaussianBlur(imageRight, rightBlur, Size(0, 0), 3);
    addWeighted(imageRight, 3, rightBlur, -2, 0, imageRight);
}

Point3d calc3DPoint( const Point2d leftCameraPoint, const double disparity,
                     const Mat Q )
{
    Point3d calculatedPoint;
    double w;

    calculatedPoint.x = leftCameraPoint.x + Q.at<float>(0, 3);
    calculatedPoint.y = leftCameraPoint.y + Q.at<float>(1, 3);
    calculatedPoint.z = Q.at<float>(2, 3);

    w               = Q.at<float>(3, 2) * disparity + Q.at<float>(3, 3);
    calculatedPoint = calculatedPoint * (1.0 / w);

    return calculatedPoint;
}

Mat getReprojMat()
{
    /*
     *     [ 1  0   0   -Cx ]
     * Q = [ 0  1   0   -Cy ]
     *     [ 0  0   0    Fx ]
     *     [ 0  0 -1/Tx  0  ]
     * parameters taken from the left camera
    */

    double Tx = ( -79.090353246545 * 0.5 ) / ( 749.6427420 * 0.5 );
    double Cx = 509.2988 * 0.5;
    double Cy = 384.118202 * 0.5;
    double Fx = 749.6427420 * 0.5;


    Mat Q             = Mat::zeros(4,4, CV_32F);
    Q.at<float>(0, 0) = 1;
    Q.at<float>(1, 1) = 1;
    Q.at<float>(0, 3) = -Cx;
    Q.at<float>(1, 3) = -Cy;
    Q.at<float>(2, 3) = Fx;
    Q.at<float>(3, 2) = -1/Tx;

    return Q;
}

void detectFeatures(Mat& descriptorsRight, Mat& imageLeft,
                    std::vector<KeyPoint>& keypointsRight,
                    std::vector<KeyPoint>& keypointsLeft, Mat& descriptorsLeft,
                    Mat& grayRight, Mat& imageRight, Mat& grayLeft)
{
    cvtColor(imageLeft, grayLeft, COLOR_BGR2GRAY, 1);
    cvtColor(imageRight, grayRight, COLOR_BGR2GRAY, 1);

    SiftFeatureDetector detector;
    SiftDescriptorExtractor extractor;

    detector.detect(grayLeft, keypointsLeft);
    detector.detect(grayLeft, keypointsRight);


    extractor.compute(grayLeft, keypointsLeft, descriptorsLeft);
    extractor.compute(grayRight, keypointsRight, descriptorsRight);
}

void findNearestNeighbors(Mat& descriptorsRight, Mat& descriptorsLeft,
                          std::vector<KeyPoint>& keypointsLeft,
                          std::vector<DMatch>& inliers,
                          std::vector<Point2d>& rightImagePoints,
                          std::vector<Point2d>& leftImagePoints,
                          std::vector<KeyPoint>& keypointsRight)
{
    std::vector<DMatch> matches;
    FlannBasedMatcher matcher;
    matcher.match( descriptorsLeft, descriptorsRight, matches);

    double max_dist = 0;
    double min_dist = 100;

    for (int i = 0; i < descriptorsLeft.rows; ++i) {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }


    for (int j = 0; j < descriptorsLeft.rows; ++j) {
        if( matches[j].distance <= max(2*min_dist, 0.02) )
        {
            inliers.push_back( matches[j] );
            leftImagePoints.push_back(
                        keypointsLeft[ matches[j].queryIdx ].pt );
            rightImagePoints.push_back(
                        keypointsRight[ matches[j].trainIdx ].pt );
        }
    }
}

void calc3DPoints(std::vector<Point2d>& rightImagePoints, char** argv,
                  std::vector<Point2d>& leftImagePoints)
{
    Mat Q;
    Q = getReprojMat();

    Point2d leftPoint, rightPoint;
    double disparity;

    // prepare to stream data
    std::ofstream output;
    output.open( argv[3], std::ofstream::binary );

    // calculate 3D points for all matches
    for ( std::vector<Point2d>::size_type l = 0; l != leftImagePoints.size();
          l++) {

        leftPoint      = leftImagePoints[l];
        rightPoint     = rightImagePoints[l];

        disparity      = leftPoint.x - rightPoint.x;

        Point3d point3 = calc3DPoint( leftPoint, disparity, Q );

        // stream data to file
        if ( l == 0 )
        {
            if ( output.is_open() )
                output << point3.x << "," << point3.y << ","
                       << point3.z << " \n ";
        }
        else if ( l == leftImagePoints.size() - 1 )
        {
            if ( output.is_open() )
                output << point3.x << "," << point3.y << "," << point3.z;
        }
        else
        {
            if ( output.is_open() )
                output << point3.x << "," << point3.y << "," << point3.z
                       << "\n ";
        }
    }
    // close stream data
    output.close();
}

int main(int argc, char** argv )
{
    // load images
    if ( argc != 4 )
    {
        std::cout << "usage: stereo-3D-calculation.out <Left_Image_Path>"
                  << "<Right_Image_Path> <output.bin>\n" << std::endl;
        return -1;
    }

    Mat imageLeft, imageRight, grayLeft, grayRight, outputLeft, outputRight;
    Mat leftBlur, rightBlur;

    imageLeft = imread( argv[1], 1 );
    imageRight = imread( argv[2], 1 );

    if ( !imageLeft.data || !imageRight.data)
    {
        readme();
        return -1;
    }

    std::cout << "[OK] \t load images" << std::endl;

    // sharpen images
    sharpenImage(imageRight, imageLeft, rightBlur, leftBlur);

    // detect SIFT-features
    std::vector<KeyPoint> keypointsLeft, keypointsRight;
    Mat descriptorsLeft, descriptorsRight;

    detectFeatures(descriptorsRight, imageLeft, keypointsRight, keypointsLeft,
                   descriptorsLeft, grayRight, imageRight, grayLeft);


    std::cout << "[OK] \t detect features" << std::endl;

    // FLANN
    std::vector<DMatch>  inliers;
    std::vector<Point2d> leftImagePoints;
    std::vector<Point2d> rightImagePoints;

    findNearestNeighbors(descriptorsRight, descriptorsLeft, keypointsLeft,
                         inliers, rightImagePoints, leftImagePoints,
                         keypointsRight);

    std::cout << "[OK] \t match features" << std::endl;

    // Draw Inliers
    Mat img_inliers;
    drawMatches(imageLeft, keypointsLeft, imageRight, keypointsRight,
                inliers, img_inliers, Scalar::all(-1), Scalar::all(-1),
                std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    drawKeypoints(grayLeft, keypointsLeft, outputLeft);
    drawKeypoints(grayRight, keypointsRight, outputRight);

    // show images <left, right>
    imshow("Stereo Matching", img_inliers);

    std::cout << "[OK] \t show stereo-matching" << std::endl;

    // 3D Point calculation from 2D-stereo-matches
    // Reprojection-Matrix for used Camera
    // HAS TO BE MODIFIED FOR OTHER SETUP
    calc3DPoints(rightImagePoints, argv, leftImagePoints);

    std::cout << "[OK] \t calculated 3D points <" << leftImagePoints.size()
              << ">"<< std::endl;
    std::cout << "[OK] \t saved to file <" << argv[3] << ">" << std::endl;
    std::cout << "[INFO] \t Hit ANY Button to CLOSE this programm" << std::endl;
    waitKey(0);

    return 0;
}

