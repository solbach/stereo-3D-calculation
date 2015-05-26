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
    imageLeft = imread( argv[1], 1 );
    imageRight = imread( argv[2], 2 );

    if ( !image.data )
    {
        printf("No image data \n");
        return -1;
    }
    namedWindow("Display left Image", WINDOW_AUTOSIZE );
    namedWindow("Display right Image", WINDOW_AUTOSIZE );
    imshow("Display left Image", imageLeft);
    imshow("Display right Image", imageRight);

    waitKey(0);

    return 0;
}