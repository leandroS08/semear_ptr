#include <iostream>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Int64.h"

using namespace cv;
using namespace std;

int x_medio_hist, x_max_hist; // variaveis da analise do histograma
int x_medio_rec, x_left, x_right; // variaveis boundRect
int delta_x;

void colorChanges(Mat&, Mat&);

void histogramH(Mat&);

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    //image_transport::Publisher image_pub_;

    public:
    ImageConverter()
    : it_(nh_)
    {
        // Recebe a imagem da camera
        image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, &ImageConverter::valveCallback, this);

        // Janelas
        namedWindow("Original", CV_WINDOW_NORMAL);
        namedWindow("Thresholded Image", CV_WINDOW_NORMAL); 
        namedWindow("Deteccao Valvula", CV_WINDOW_NORMAL); 
    }

    ~ImageConverter()
    {
        destroyWindow("Original");
        destroyWindow("Thresholded Image"); 
        destroyWindow("Deteccao Valvula");
    }

    void valveCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat src, imgHSV, imgValvula;
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        RNG rng(12345);
        
        src = cv_ptr->image;
 
        colorChanges(src, imgHSV);

        histogramH(imgHSV);

        findContours( imgHSV, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0) );

        vector<vector<Point> > contours_poly( contours.size() );
        vector<vector<Point> > contours_selected;
        vector<Rect> boundRect( contours.size() );
        vector<Point2f>center( contours.size() );
        vector<float>radius( contours.size() );
        Mat drawing = src.clone();
        float min_area = (src.rows * src.cols) * 0.003;
        for( size_t i = 0; i < contours.size(); i++ )
        {
            approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
            boundRect[i] = boundingRect( Mat(contours_poly[i]) );
            if( (boundRect[i].width * boundRect[i].height)  >= min_area )
            {
                //drawContours( drawing, contours_poly, (int)i, Scalar( 180,255,0 ), 3, 8, vector<Vec4i>(), 0, Point() );
                //rectangle( drawing, boundRect[i].tl(), boundRect[i].br(), Scalar( 255,0,255 ), 3, 8, 0 );
                contours_selected.push_back(contours_poly[i]);
                //cout << "X do contorno: " << boundRect[i].x << endl;
            }
        }

        vector<Rect> rect_selected( contours_selected.size() );
        for( size_t i = 0; i < contours_selected.size(); i++ )
            rect_selected[i] = boundingRect( Mat(contours_selected[i]) );

        int i_max = 0;
        for( size_t i = 1; i < contours_selected.size(); i++ )
        {
            if( (rect_selected[i].width * rect_selected[i].height) > (rect_selected[i_max].width * rect_selected[i_max].height))
                i_max = i;
                
        }

        drawContours( drawing, contours_selected, i_max, Scalar( 180,255,0 ), 3, 8, vector<Vec4i>(), 0, Point() );
        rectangle( drawing, rect_selected[i_max].tl(), rect_selected[i_max].br(), Scalar( 255,0,255 ), 3, 8, 0 );

        x_left = rect_selected[i_max].x;
        x_right = rect_selected[i_max].x + rect_selected[i_max].width;
        x_medio_rec = rect_selected[i_max].x + (rect_selected[i_max].width / 2);
        cout << "> X medio retangulo: " << x_medio_rec << endl;

        delta_x = x_medio_rec - x_medio_hist; 
        cout << "> Diferenca: " << delta_x << endl;

        int error_delta = 0.015 * src.cols;
        cout << "> Erro: " << error_delta << endl;
        //if(delta_x < )
        
        cout << "> Altura: " << rect_selected[i_max].height << endl;
        cout << "> Comprimento: " << rect_selected[i_max].width << endl;
        float rel_hw = ((float)rect_selected[i_max].height/(float)rect_selected[i_max].width);
        cout << "> Relacao altura-comprimento: " << rel_hw << endl;
        cout << endl;
        if(rel_hw >= 2.25)
            cout << "> FECHADA" << endl;
        else
            cout << "> ABERTA" << endl;

        imshow("Original", src);
        imshow("Thresholded Image", imgHSV);
        imshow("Deteccao Valvula", drawing);

        waitKey(3); 
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "valvula_vision");
    cout << "oi" << endl;
    ImageConverter ic;
    ros::spin();
    return 0;
}

void colorChanges(Mat &in, Mat &out)
{
    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0;
    int iHighS = 65;

    int iLowV = 0;
    int iHighV = 110;

    cvtColor(in, out, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

    inRange(out, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), out); //Threshold the image
        
    //morphological opening (remove small objects from the foreground)
    erode(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    dilate(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

    //morphological closing (fill small holes in the foreground)
    dilate(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
    erode(out, out, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
}

void histogramH(Mat& in)
{
    Mat histImage(in.rows, in.cols, CV_8UC3, Scalar(255,255,255) );

    int count[in.cols];
    long int count_Num = 0;
    long int count_Den = 0;

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

    x_medio_hist = count_Num / count_Den;

    //cout << "> Numerador: " << count_Num << endl;
    //cout << "> Denominador: " << count_Den << endl;
    cout << "> x medio histograma: " << x_medio_hist << endl;

    x_max_hist = 0;
    for (int col = 1; col < in.cols; ++col)
    {
        if(count[col]>count[x_max_hist])
            x_max_hist = col;
    }

    cout << "> x maximo histograma: " << x_max_hist << endl;

    char* histogram_window = "Histogram 1";
    namedWindow(histogram_window, CV_WINDOW_NORMAL);
    imshow(histogram_window, histImage);
}