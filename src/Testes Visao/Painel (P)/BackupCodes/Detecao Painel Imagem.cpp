#include <iostream>
#include <stdio.h>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

void ordenaCirculos(vector<Vec3f>& , vector<Vec3f>&);

int main( int argc, char** argv )
{
    Mat src;
    Mat imgGray, imgCircles;
    Mat imgHSV;
    long int count_num;
    int count_den;
    int lum_circulos[6];
    int leitura_painel[3];
     
    src = imread(argv[1]);
    imgCircles = src.clone();

    namedWindow("Imagem Original", CV_WINDOW_NORMAL); 
    namedWindow("Circulos Detectados", CV_WINDOW_NORMAL); 
    namedWindow("Imagem HSV", CV_WINDOW_NORMAL); 
    
    cvtColor( src, imgGray, CV_BGR2GRAY );
    cvtColor( src, imgHSV, COLOR_BGR2HSV); 

    GaussianBlur( imgGray, imgGray, Size(9, 9), 2, 2 );

    vector<Vec3f> circles;
    vector<Vec3f> ordered_circles;
    Vec3b point_aux;

    /// Apply the Hough Transform to find the circles
    HoughCircles( imgGray, circles, CV_HOUGH_GRADIENT, 1, imgGray.rows/8, 50, 30, 0, 0 );

    cout << "-> Numero de circulos detectados: " << circles.size() << endl;
    cout << endl;

    ordenaCirculos(circles, ordered_circles);

    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(ordered_circles[i][0]), cvRound(ordered_circles[i][1]));
        int radius = cvRound(ordered_circles[i][2]);
        // circle center
        circle( imgCircles, center, 3, Scalar(255,0,0), -1, 8, 0 );
        // circle outline
        circle( imgCircles, center, radius, Scalar(100,20,100), 3, 8, 0 );

        cout << "-> Informacoes circulo " << i << endl;
        cout << "     - Posicao: (" << center.x << " , " << center.y << " )" << endl;
        cout << "     - Raio: " << radius << endl;

        count_den = 0;
        count_num = 0;
        for ( int col = center.x - radius ; col <= center.x + radius; ++col )
        {
            for ( int row = center.y - radius; row <= center.y + radius; ++row )
            {
                if ( (pow(col - center.x,2) + pow(row - center.y,2)) <= pow(radius,2) )
                {
                    count_den++;

                    point_aux = imgHSV.at<Vec3b>(row,col);
                    //cout << (int) point_aux.val[2] << "  " << endl;
                    count_num += point_aux.val[2];
                }
            }
        }
        //cout << endl;
        //cout << "     - Numerador: " << count_num << endl;
        //cout << "     - Denominador: " << count_den << endl;
        if (count_den != 0)
            lum_circulos[i] = count_num / count_den;
        cout << "     - Luminosidade: " << lum_circulos[i] << endl;
    }

    cout << "-> Informacoes do painel " << endl;
    for( int i = 0, j = 3; i < 3; i++, j++)
    {
        if( lum_circulos[i] > lum_circulos[j])
            leitura_painel[i] = 1;
        else
            leitura_painel[i] = 0;
        
        if( leitura_painel[i] == 1)
            cout << "     - LIGADO" << endl;
        else
            cout << "     - DESLIGADO" << endl;
    }

    imshow("Imagem Original", src);
    imshow("Circulos Detectados", imgCircles);
    imshow("Imagem HSV", imgHSV);

    waitKey(0);
    
    return 0;
}

void ordenaCirculos(vector<Vec3f>& circles, vector<Vec3f>& ordered_circles)
{
    int mapa[6];
    int y_medio = 0;
    int x_1 = 0;
    int i_min_1 = -1;
    int i_min_2 = -1;
    int x_2 = 0;
    int i_max_1 = -1;
    int i_max_2 = -1;

    for( size_t i = 0; i < circles.size(); i++ )
    {
        y_medio += circles[i][1];

        if(i_min_1 == -1)
            i_min_1 = i;
        else if(i_min_2 == -1)
            i_min_2 = i;
        else if(circles[i][0] < circles[i_min_1][0])
            i_min_1 = i;
        else if(circles[i][0] < circles[i_min_2][0])
            i_min_2 = i;

        if(i_max_1 == -1)
            i_max_1 = i;
        else if(i_max_2 == -1)
            i_max_2 = i;
        else if(circles[i][0] > circles[i_max_1][0])
            i_max_1 = i;
        else if(circles[i][0] > circles[i_max_2][0])
            i_max_2 = i;
    }

    y_medio = y_medio / 6;
    /*cout << "Y Medio: " << y_medio << endl;

    cout << "Minimo X 1: " << circles[i_min_1][0] << endl;
    cout << "Minimo X 2: " << circles[i_min_2][0] << endl;

    cout << "Maximo X 1: " << circles[i_max_1][0] << endl;
    cout << "Maximo X 2: " << circles[i_max_2][0] << endl;*/

    for( size_t i = 0; i < circles.size(); i++ )
    {
        if( circles[i][1] < y_medio )
        {
            if( circles[i][0] <= circles[i_min_2][0] )
                mapa[0] = i;
            else if( circles[i][0] >= circles[i_max_2][0] )
                mapa[2] = i;
            else
                mapa[1] = i;
        }
        else
        {
            if( circles[i][0] <= circles[i_min_2][0] )
                mapa[3] = i;
            else if( circles[i][0] >= circles[i_max_2][0] )
                mapa[5] = i;
            else
                mapa[4] = i;
        }

        //cout << "Mapa [ " << i << " ]: " << mapa[i] << endl;
    }

    Vec3f point_aux;
    for(int j=0; j<6; j++)
        ordered_circles.push_back(circles[mapa[j]]);        
}

