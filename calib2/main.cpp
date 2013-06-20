#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <stdio.h>

using namespace cv;
using namespace std;

const Size imgSize(800, 600);
const Size brdSize(8, 7);
const size_t brds_num = 20;

template<class T> ostream& operator<<(ostream& out, const Mat_<T>& mat)
{
    for(int j = 0; j < mat.rows; ++j)
        for(int i = 0; i < mat.cols; ++i)
            out << mat(j, i) << " ";
    return out;
}



int main()
{
    IplImage* img = cvLoadImage("chess4");
    /* init points */
    vector< vector<Point3f> > objectPoints;//
    vector< vector<Point2f> > imagePoints;

        namedWindow("Current chessboard");
        imshow("Current chessboard", boards[i]);
        waitKey(100);
        bool found = findChessboardCorners(boards[i], cbg.cornersSize(), tmp);
        if (found)
        {
            imagePoints.push_back(tmp);
            objectPoints.push_back(chessboard3D);
            cout<< "-found ";
        }
        else
            cout<< "-not-found ";

        drawChessboardCorners(boards[i], cbg.cornersSize(), Mat(tmp), found);
        imshow("Current chessboard", boards[i]);
        waitKey(1000);
    }
    cout << "Done" << endl;
    cvDestroyAllWindows();

    Mat camMat_est;
    Mat distCoeffs_est;
    vector<Mat> rvecs, tvecs;

    cout << "Calibrating...";
    double rep_err = calibrateCamera(objectPoints, imagePoints, imgSize,
                                     camMat_est, distCoeffs_est, rvecs, tvecs);
    cout << "Done" << endl;

    cout << endl << "Average Reprojection error: " << rep_err/brds_num/cbg.cornersSize().area() << endl;
    cout << "==================================" << endl;
    cout << "Original camera matrix:\n" << camMat << endl;
    cout << "Original distCoeffs:\n" << distCoeffs << endl;
    cout << "==================================" << endl;
    cout << "Estimated camera matrix:\n" << (Mat_<double>&)camMat_est << endl;
    cout << "Estimated distCoeffs:\n" << (Mat_<double>&)distCoeffs_est << endl;

    return 0;
}
