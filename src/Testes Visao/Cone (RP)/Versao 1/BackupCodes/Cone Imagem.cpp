#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

int x_medio, x_min, x_max;
int y_medio, y_min, y_max;

void colorChanges(Mat&, Mat&);

void histogramH(Mat&, Mat&);

void histogramV(Mat&, Mat&);

void fitRectangle(Mat&, Mat&);

int main( int argc, char** argv )
{
    Mat src, imgHSV, imgHist1, imgHist2, imgCone;
     
    src = imread(argv[1]);

    namedWindow("Original", CV_WINDOW_NORMAL);
    namedWindow("Thresholded Image", CV_WINDOW_NORMAL); 
    namedWindow("Deteccao Cone", CV_WINDOW_NORMAL); 
    
    colorChanges(src, imgHSV);
    
    histogramH(imgHSV, imgHist1);
    histogramV(imgHSV, imgHist2);

    fitRectangle(src, imgCone);

    imshow("Original", src);
    imshow("Thresholded Image", imgHSV);
    imshow("Deteccao Cone", imgCone);

    waitKey(0);
    
    return 0;
}

void colorChanges(Mat &in, Mat &out)
{
    int iLowH = 0;
    //int iHighH = 30; // cone.jpg e cone2.jpg
    int iHighH = 18; // cone3.jpg e cone4.jpg


    //int iLowS = 0; // cone.jpg, cone2.jpg e cone3.jpg
    int iLowS = 125; // cone4.jpg
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    cvtColor(in, out, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    inRange(out, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), out); //Threshold the image
        
    //morphological opening (remove small objects from the foreground)
    erode(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
}

void histogramH(Mat& in, Mat& hist)
{
    Mat histImage(in.rows, in.cols, CV_8UC3, Scalar(255,255,255) );

    int count[in.cols];
    long int count_Num = 0;
    long int count_Den = 0;
    x_min = -1;
    x_max = -1;

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

        if (count[col] != 0)
            //cout << "Coluna " << col << " tem " << count[col] << " pixeis" << endl;

        circle(histImage, Point(col, in.rows - count[col]),1,Scalar(0,0,255),3,8,0);
    }

    x_medio = (int) count_Num / count_Den;

    cout << "x medio: " << x_medio << endl;
    cout << "Pontos no x medio: " << count[x_medio] << endl;
    cout << endl;

    for(int i_min=0; (x_medio-i_min)>=0; i_min++)
    {
        if(count[x_medio-i_min]<(0.15*count[x_medio]))
        {
            x_min = x_medio-i_min;
            break;
        }
    }

    cout << "X minimo: " << x_min << endl;
    cout << "Pontos no x minimo: " << count[x_min] << endl;
    cout << endl;

    for(int i_max=0; (x_medio+i_max)<in.cols; i_max++)
    {
        if(count[x_medio+i_max]<(0.15*count[x_medio]))
        {
            x_max = x_medio+i_max;
            break;
        }
    }

    cout << "X maximo: " << x_max << endl;
    cout << "Pontos no x maximo: " << count[x_max] << endl;
    cout << endl;

    hist = histImage;

    char* histogram_window = "Histogram 1";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);
}

void histogramV(Mat& in, Mat& hist)
{
    Mat histImage(in.rows, in.cols, CV_8UC3, Scalar(255,255,255) );

    int count[in.rows];
    long int count_Num = 0;
    long int count_Den = 0;

    for (int row = 0; row < in.rows; ++row)
    {
        count[row]=0;

        for (int col = 0; col < in.cols; ++col)
        {
            if ( (int)(in.at<uchar>(row,col)) != 0)
            {
                ++count[row];
                count_Num+=(row*count[row]);
                count_Den+=count[row];
            }
        }

        //if (count[row] != 0)
            //cout << "Linha " << row << " tem " << count[row] << " pixeis" << endl;

        circle(histImage, Point(row, in.cols - count[row]),1,Scalar(255,0,255),3,8,0);
    }

    y_medio = (int) count_Num / count_Den;

    cout << "Y medio: " << y_medio << endl;
    cout << "Pontos no y medio: " << count[y_medio] << endl;
    cout << endl;

    for(int j_min=0; j_min<y_medio; j_min++)
    {
        if(count[j_min]>(0.15*count[y_medio]))
        {
            y_min = j_min;
            break;
        }
    }

    cout << "Y minimo: " << y_min << endl;
    cout << "Pontos no y minimo: " << count[y_min] << endl;
    cout << endl;

    for(int j_max=in.rows-1; j_max>y_medio; j_max--)
    {
        if(count[j_max]>(0.15*count[y_medio]))
        {
            y_max = j_max;
            break;
        }
    }

    cout << "Y maximo: " << y_max << endl;
    cout << "Pontos no y maximo: " << count[y_max] << endl;
    cout << endl;

    hist = histImage;

    /*char* histogram_window = "Histogram 2";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);*/
}

void fitRectangle(Mat &in, Mat &out)
{
    out = in.clone();

    Point2d P1, P2;

    P1.x = x_min;
    P1.y = y_min;
    P2.x = x_max;
    P2.y = y_max;

    rectangle(out, P1, P2, Scalar(0,255,180), 4, 8, 0);
}