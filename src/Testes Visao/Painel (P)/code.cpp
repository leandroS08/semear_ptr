#include <iostream>
#include <stdio.h>
#include <math.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

void organizaCirculos(vector<Vec3f>& , vector<Vec3f>&);

int main( int argc, char** argv )
{
    VideoCapture capture(0);

    if(!capture.isOpened()) 
    { 
        cerr << " ERR: Unable find input Video source." << endl;
		return -1;
	}
    
    Mat src;
    Mat imgGray, imgCircles;
    Mat imgHSV;
    long int count_num;
    int count_den;
    int lum_circulos[6];
    int leitura_painel[3];
    int upper_threshold = 100;

    namedWindow("Imagem Original", CV_WINDOW_NORMAL); 
    namedWindow("Circulos Detectados", CV_WINDOW_NORMAL); 
    namedWindow("Imagem HSV", CV_WINDOW_NORMAL); 

    while(waitKey(33) != 27)
    {
        capture.read(src);
        imgCircles = src.clone();
        
        cvtColor( src, imgGray, CV_BGR2GRAY );
        cvtColor( src, imgHSV, COLOR_BGR2HSV); 

        GaussianBlur( imgGray, imgGray, Size(9, 9), 2, 2 );

        vector<Vec3f> circles;
        vector<Vec3f> org_circles;
        Vec3b point_aux;

        /// Apply the Hough Transform to find the circles
        HoughCircles( imgGray, circles, CV_HOUGH_GRADIENT, 1, imgGray.rows/8, upper_threshold, 50, 0, 0 );

        if (circles.size()>0)
        {
            cout << "\n> Numero de circulos detectados (threshold = " << upper_threshold << "): " << circles.size() << endl;
            cout << "   -> Informacoes circulos " << endl;
        }
        
        if(circles.size() >= 6)
        {
            organizaCirculos(circles, org_circles);

            for( size_t i = 0; i < org_circles.size(); i++ )
            {
                Point center(cvRound(org_circles[i][0]), cvRound(org_circles[i][1]));
                int radius = cvRound(org_circles[i][2]);
                // circle center
                circle( imgCircles, center, 3, Scalar(255,0,0), -1, 8, 0 );
                // circle outline
                circle( imgCircles, center, radius, Scalar(100,20,100), 3, 8, 0 );

                cout << "        - Circulo " << i << ": posicao (" << center.x << "," << center.y << "), ";
                cout << "raio " << radius;

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
                cout << ", luminosidade " << lum_circulos[i] << endl;
            }

            cout << "   -> Informacoes do painel " << endl;
            for( int i = 0, j = 3; i < 3; i++, j++)
            {
                if( lum_circulos[i] > lum_circulos[j])
                    leitura_painel[i] = 1;
                else
                    leitura_painel[i] = 0;
                
                if( leitura_painel[i] == 1)
                    cout << "        - LIGADO" << endl;
                else
                    cout << "        - DESLIGADO" << endl;
            }

            imshow("Circulos Detectados", imgCircles);
        }
        //else if (circles.size() >= 4)
        //    upper_threshold--;


        imshow("Imagem Original", src);       
        imshow("Imagem HSV", imgHSV); 
    }
    
    return 0;
}

void organizaCirculos(vector<Vec3f>& circles, vector<Vec3f>& org_circles)
{ 
    // Calculo do raio medio dos circulos detectados
    int raio_medio = 0;
    float raio_range = 0.2;
    for( size_t i = 0; i < circles.size(); i++ )
        raio_medio += circles[i][2];
    raio_medio = raio_medio / circles.size();
    cout << "        - Raio medio: " << raio_medio <<  " (" << (1-raio_range)* raio_medio << "," << (1+raio_range)*raio_medio << ") " << endl;

    // Selecao dos seis melhores circulos em caso de deteccao de mais de seis
    vector<Vec3f> aux_circles;
    if(circles.size() > 6)
    {
        for( size_t i = 0; i < circles.size(); i++ )
        {
            if( (circles[i][2] < (1.3 * raio_medio)) && (circles[i][2] > (0.7 * raio_medio)) )
                aux_circles.push_back(circles[i]);  
        }
        cout << "Circulos selecionados: " << aux_circles.size() << endl;
    }
    else
        aux_circles = circles;

    int y_medio = 0;
    int i_min_1 = -1;
    int i_min_2 = -1;
    int i_max_1 = -1;
    int i_max_2 = -1;
    for( int i = 0; i < 6; i++ )
    {
        y_medio += aux_circles[i][1];

        if(i_min_1 == -1)
            i_min_1 = i;
        else if(i_min_2 == -1)
            i_min_2 = i;
        else if(aux_circles[i][0] < aux_circles[i_min_1][0])
        {
            i_min_2 = i_min_1;
            i_min_1 = i;        
        }
        else if(aux_circles[i][0] < aux_circles[i_min_2][0])
            i_min_2 = i;

        if(i_max_1 == -1)
            i_max_1 = i;
        else if(i_max_2 == -1)
            i_max_2 = i;
        else if(aux_circles[i][0] > aux_circles[i_max_1][0])
        {
            i_max_2 = i_max_1;
            i_max_1 = i;
        }
        else if(aux_circles[i][0] > aux_circles[i_max_2][0])
            i_max_2 = i;
    }

    y_medio = y_medio / 6;
    /*cout << "Y Medio: " << y_medio << endl;

    cout << "Minimo X 1: " << aux_circles[i_min_1][0] << endl;
    cout << "Minimo X 2: " << aux_circles[i_min_2][0] << endl;

    cout << "Maximo X 1: " << aux_circles[i_max_1][0] << endl;
    cout << "Maximo X 2: " << aux_circles[i_max_2][0] << endl;*/

    int mapa[6] = {-1,-1,-1,-1,-1,-1};
    for( int i = 0; i < 6; i++ )
    {
        if( aux_circles[i][1] < y_medio )
        {
            if( (aux_circles[i][0] <= aux_circles[i_min_2][0]) && mapa[0]==-1 )
                mapa[0] = i;
            else if( (aux_circles[i][0] == aux_circles[i_max_1][0]) || (aux_circles[i][0] == aux_circles[i_max_2][0]) )
                mapa[2] = i;
            else
                mapa[1] = i;
        }
        else
        {
            if( (aux_circles[i][0] == aux_circles[i_min_1][0]) || (aux_circles[i][0] == aux_circles[i_min_2][0]) )
                mapa[3] = i;
            else if( (aux_circles[i][0] == aux_circles[i_max_1][0]) || (aux_circles[i][0] == aux_circles[i_max_2][0]) )
                mapa[5] = i;
            else
                mapa[4] = i;
        }
    }

    //for( int i = 0; i < 6; i++ )
    //    cout << "Mapa [ " << i << " ]: " << mapa[i] << endl;

    //org_circles = aux_circles;

    for(int j=0; j<aux_circles.size(); j++)
        org_circles.push_back(aux_circles[mapa[j]]);  
}

