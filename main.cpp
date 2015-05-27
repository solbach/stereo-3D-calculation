#include <stdio.h>
#include <opencv2/opencv.hpp>

using namespace cv;

int main(int argc, char** argv )
{
    if ( argc != 3 )
    {
        printf("usage: stereo-3D-calculation.out <Left_Image_Path> <Right_Image_Path>\n");
        return -1;
    }

    Mat imageLeft, imageRight;

    std::cout << "output: " << argv[1] << std::endl;
    std::cout << "output: " << argv[2] << std::endl;

    if ( !imageLeft.data || !imageRight.data)
    {
        printf("On or more image data are missing \n");
        return -1;
    }
    namedWindow("Display left Image", WINDOW_AUTOSIZE );
    namedWindow("Display right Image", WINDOW_AUTOSIZE );
    imshow("Display left Image", imageLeft);
    imshow("Display right Image", imageRight);

    waitKey(0);

    return 0;
}
