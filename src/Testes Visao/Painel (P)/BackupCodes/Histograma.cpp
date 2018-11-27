#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

void colorChanges(Mat&, Mat&);

void histogram(Mat&, Mat&);

int main( int argc, char** argv )
{
    Mat src, imgHSV, imgHist;
     
    src = imread(argv[1]);

    namedWindow("Original", CV_WINDOW_NORMAL);
    namedWindow("Thresholded Image", CV_WINDOW_NORMAL); 
    
    colorChanges(src, imgHSV);
    
    histogram(imgHSV, imgHist);

    imshow("Original", src);
    imshow("Thresholded Image", imgHSV);

    waitKey(0);
    
    return 0;
}

void colorChanges(Mat &in, Mat &out)
{
    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 240;

    cvtColor(in, out, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    inRange(out, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), out); //Threshold the image
        
    //morphological opening (remove small objects from the foreground)
    erode(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
}

void histogram(Mat& in, Mat& hist)
{
    Mat histImage(in.rows, in.cols, CV_8UC3, Scalar(255,255,255) );

    int count[in.cols];
    long int count_Num = 0;
    int count_Den = 0;
    int i_best_col = -1;
    int n_best_col = 0;

    for (int col = 0; col < in.cols; ++col)
    {
        count[col]=0;

        for (int row = 0; row < in.rows; ++row)
        {
            if ( (int)(in.at<uchar>(row,col)) != 0)
            {
                ++count[col];
                count_Num+=(col*count[col]);
                count_Den+=count[col];
            }
        }

        //if (count[col] != 0)
            //cout << "Coluna " << col << " tem " << count[col] << " pixeis" << endl; 

        if (count[col] > n_best_col)
        {
            i_best_col = col;
            n_best_col = count[col];
        }             

        circle(histImage, Point(col, in.rows - count[col]),1,Scalar(0,0,255),3,8,0);
    }

    hist = histImage;

    char* histogram_window = "Histogram 2";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);
}