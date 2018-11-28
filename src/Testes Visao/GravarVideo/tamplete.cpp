#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

char* source_window = "Original Video";
char* result_window = "Result Video";

 int main( int argc, char** argv )
 {
    char* nome_video = argv[1];
    VideoCapture cap(1);
    VideoWriter video(nome_video, CV_FOURCC('M','J','P','G'), 10, Size(cap.get(CV_CAP_PROP_FRAME_WIDTH), cap.get(CV_CAP_PROP_FRAME_HEIGHT)));

    if ( !cap.isOpened() )
    {
         cout << "Erro ao abrir o video" << endl;
         return -1;
    }

    /* Janelas */
    namedWindow(source_window, CV_WINDOW_NORMAL);

    Mat src, dst;

    while (waitKey(33) != 27 && cap.isOpened())
    {
        cap.read(src);

        imshow(source_window, src);

        video.write(src);
    }

    waitKey(0);

    destroyWindow(source_window);

    return 0;
 }