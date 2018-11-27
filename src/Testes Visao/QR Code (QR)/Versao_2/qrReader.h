#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class qrReader {
public:
    bool find(const Mat& img);
    void drawFinders(Mat& img);

private:
    bool checkRatio(int* stateCount);
    bool handlePossibleCenter(const Mat& img, int* stateCount, int row, int col);
    bool crossCheckDiagonal(const Mat &img, float centerRow, float centerCol, int maxCount, int stateCountTotal);
    float crossCheckVertical(const Mat& img, int startRow, int centerCol, int stateCount, int stateCountTotal);
    float crossCheckHorizontal(const Mat& img, int centerRow, int startCol, int stateCount, int stateCountTotal);

    inline float centerFromEnd(int* stateCount, int end) {
        return (float)(end-stateCount[4]-stateCount[3])-(float)stateCount[2]/2.0f;
    }

    vector<Point2f> possibleCenters;
    vector<float> estimatedModuleSize;
};