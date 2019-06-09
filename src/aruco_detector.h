#include <iostream>
#include "opencv2/opencv.hpp"
// #include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <opencv2/aruco.hpp>
#include <sensor_msgs/CameraInfo.h>
#include "math.h"
// #include "opencv2/imgcodecs.hpp"
// #include "opencv2/highgui.hpp"

// #include "fast_line_detector.hpp"
using namespace std;
using namespace cv;
using namespace cv::aruco;
static void parseCameraInfo(const sensor_msgs::CameraInfoConstPtr &cinfo,
                     cv::Mat &matrix, cv::Mat &dist)
{
	for (unsigned int i = 0; i < 3; ++i)
		for (unsigned int j = 0; j < 3; ++j)
			matrix.at<double>(i, j) = cinfo->K[3 * i + j];

	for (unsigned int k = 0; k < cinfo->D.size(); k++)
		dist.at<double>(k) = cinfo->D[k];
}

void _getBoardObjectAndImagePoints(const Ptr<aruco::Board> &board, InputArrayOfArrays detectedCorners,
                                   InputArray detectedIds, OutputArray objPoints, OutputArray imgPoints)
{

    CV_Assert(board->ids.size() == board->objPoints.size());
    CV_Assert(detectedIds.total() == detectedCorners.total());

    size_t nDetectedMarkers = detectedIds.total();

    std::vector<Point3f> objPnts;
    objPnts.reserve(nDetectedMarkers);

    std::vector<Point2f> imgPnts;
    imgPnts.reserve(nDetectedMarkers);

    // look for detected markers that belong to the board and get their information
    for (unsigned int i = 0; i < nDetectedMarkers; i++)
    {
        int currentId = detectedIds.getMat().ptr<int>(0)[i];
        for (unsigned int j = 0; j < board->ids.size(); j++)
        {
            if (currentId == board->ids[j])
            {
                for (int p = 0; p < 4; p++)
                {
                    objPnts.push_back(board->objPoints[j][p]);
                    imgPnts.push_back(detectedCorners.getMat(i).ptr<Point2f>(0)[p]);
                }
            }
        }
    }

    // create output
    Mat(objPnts).copyTo(objPoints);
    Mat(imgPnts).copyTo(imgPoints);
}

int _estimatePoseBoard(InputArrayOfArrays _corners, InputArray _ids, const Ptr<aruco::Board> &board,
                       InputArray _cameraMatrix, InputArray _distCoeffs, OutputArray _rvec,
                       OutputArray _tvec, bool useExtrinsicGuess, Mat &objPoints)
{

    CV_Assert(_corners.total() == _ids.total());

    // get object and image points for the solvePnP function
    Mat /*objPoints, */ imgPoints;
    _getBoardObjectAndImagePoints(board, _corners, _ids, objPoints, imgPoints);

    CV_Assert(imgPoints.total() == objPoints.total());

    if (objPoints.total() == 0) // 0 of the detected markers in board
        return 0;

    //    std::cout << "objPoints: " << objPoints << std::endl;
    //    std::cout << "imgPoints: " << imgPoints << std::endl;

    solvePnP(objPoints, imgPoints, _cameraMatrix, _distCoeffs, _rvec, _tvec, useExtrinsicGuess);

    // divide by four since all the four corners are concatenated in the array for each marker
    return (int)objPoints.total() / 4;
}

void _drawPlanarBoard(Board *_board, Size outSize, OutputArray _img, int marginSize,
                      int borderBits)
{

    CV_Assert(outSize.area() > 0);
    CV_Assert(marginSize >= 0);

    _img.create(outSize, CV_8UC1);
    Mat out = _img.getMat();
    out.setTo(Scalar::all(255));
    out.adjustROI(-marginSize, -marginSize, -marginSize, -marginSize);

    // calculate max and min values in XY plane
    CV_Assert(_board->objPoints.size() > 0);
    float minX, maxX, minY, maxY;
    minX = maxX = _board->objPoints[0][0].x;
    minY = maxY = _board->objPoints[0][0].y;

    for (unsigned int i = 0; i < _board->objPoints.size(); i++)
    {
        for (int j = 0; j < 4; j++)
        {
            minX = min(minX, _board->objPoints[i][j].x);
            maxX = max(maxX, _board->objPoints[i][j].x);
            minY = min(minY, _board->objPoints[i][j].y);
            maxY = max(maxY, _board->objPoints[i][j].y);
        }
    }

    float sizeX = maxX - minX;
    float sizeY = maxY - minY;

    // proportion transformations
    float xReduction = sizeX / float(out.cols);
    float yReduction = sizeY / float(out.rows);

    // determine the zone where the markers are placed
    if (xReduction > yReduction)
    {
        int nRows = int(sizeY / xReduction);
        int rowsMargins = (out.rows - nRows) / 2;
        out.adjustROI(-rowsMargins, -rowsMargins, 0, 0);
    }
    else
    {
        int nCols = int(sizeX / yReduction);
        int colsMargins = (out.cols - nCols) / 2;
        out.adjustROI(0, 0, -colsMargins, -colsMargins);
    }

    // now paint each marker
    Dictionary &dictionary = *(_board->dictionary);
    Mat marker;
    Point2f outCorners[3];
    Point2f inCorners[3];
    for (unsigned int m = 0; m < _board->objPoints.size(); m++)
    {
        // transform corners to markerZone coordinates
        for (int j = 0; j < 3; j++)
        {
            Point2f pf = Point2f(_board->objPoints[m][j].x, _board->objPoints[m][j].y);
            // move top left to 0, 0
            pf -= Point2f(minX, minY);
            pf.x = pf.x / sizeX * float(out.cols);
            pf.y = (1.0f - pf.y / sizeY) * float(out.rows);
            outCorners[j] = pf;
        }

        // get marker
        Size dst_sz(outCorners[2] - outCorners[0]);                           // assuming CCW order
        dst_sz.width = dst_sz.height = std::min(dst_sz.width, dst_sz.height); //marker should be square
        dictionary.drawMarker(_board->ids[m], dst_sz.width, marker, borderBits);

        if ((outCorners[0].y == outCorners[1].y) && (outCorners[1].x == outCorners[2].x))
        {
            // marker is aligned to image axes
            marker.copyTo(out(Rect(outCorners[0], dst_sz)));
            continue;
        }

        // interpolate tiny marker to marker position in markerZone
        inCorners[0] = Point2f(-0.5f, -0.5f);
        inCorners[1] = Point2f(marker.cols - 0.5f, -0.5f);
        inCorners[2] = Point2f(marker.cols - 0.5f, marker.rows - 0.5f);

        // remove perspective
        Mat transformation = getAffineTransform(inCorners, outCorners);
        warpAffine(marker, out, transformation, out.size(), INTER_LINEAR,
                   BORDER_TRANSPARENT);
    }
}

Point3d rotationMatrixToEAngles(Mat R)
{
    double sy = sqrtf(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));
    bool singular = sy < 1e-6;
    double x, y, z;
    if (not singular)
    {
        x = atan2f(R.at<double>(2, 1), R.at<double>(2, 2));
        y = atan2f(-R.at<double>(2, 0), sy);
        z = atan2f(R.at<double>(1, 0), R.at<double>(0, 0));
    }
    else
    {
        x = atan2f(-R.at<double>(1, 2), R.at<double>(1, 1));
        y = atan2f(-R.at<double>(2, 0), sy);
        z = 0;
    }
    return Point3d(x, y, z);
}
Mat R_flip_gen(double x = 1.0, double y = -1.0, double z = -1.0)
{
    Mat _R_flip = Mat(3, 3, CV_64F, cvScalar(0.));
    _R_flip.at<double>(0, 0) = x;
    _R_flip.at<double>(1, 1) = y;
    _R_flip.at<double>(2, 2) = z;
    return _R_flip;
}

cv::Ptr<cv::aruco::GridBoard> board_gen(int markers_x, int markers_y, float marker_size, float marker_sep, cv::Ptr<cv::aruco::Dictionary> dictionary, int firstMarker)
{
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markers_x, markers_y, marker_size, marker_sep, dictionary, firstMarker);
    return board;
}

bool estimate_pose(Mat image, cv::Ptr<cv::aruco::GridBoard> board, cv::Ptr<cv::aruco::Dictionary> dictionary, InputArray cameraMatrix, InputArray distCoeffs, Mat _R_flip, bool draw, Point3d &pose, Point3d &angles, Mat &img_out)
{
    Point3d _pose, _angles;
    bool val = false;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary, corners, ids);
    if (ids.size() > 0)
    {
        if (draw == true)
        {
            cv::aruco::drawDetectedMarkers(img_out, corners, ids);
        }
        cv::Vec3d tvec, rvec;
        cv::Mat objPoints;
        int valid = _estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs,
                                       rvec, tvec, false, objPoints);
        if (valid > 0)
        {
            val = true;
            Mat pos_camera = Mat(3, 3, CV_64F, cvScalar(0.));
            Mat R_ct, R_tc;
            // tvec = tvecs[i];
            // rvec = rvecs[i];
            // cout << "pose_raw  x:" << tvec[0] << " y:" << tvec[1] << " z:" << tvec[2] << endl;
            if (draw == true)
            {
                cv::aruco::drawAxis(img_out, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
            }
            cv::Rodrigues(rvec, R_ct);
            R_tc = R_ct.t();
            double roll_marker, pitch_marker, yaw_marker;
            // Point3d marker_angles;
            _angles = rotationMatrixToEAngles(_R_flip * R_tc);
            // roll_marker = marker_angles.x;
            // pitch_marker = marker_angles.y;
            // yaw_marker = marker_angles.z;
            // cout << "marker_angles_raw  roll:" << roll_marker << " pitch:" << pitch_marker << " yaw:" << yaw_marker << endl;
            Mat tvec_mat = Mat(tvec);
            R_tc = -R_tc;
            pos_camera = R_tc * tvec_mat;
            _pose.x = pos_camera.at<double>(0, 0);
            _pose.y = pos_camera.at<double>(0, 1);
            _pose.z = pos_camera.at<double>(0, 2);
            // cout << "pose  x:" << pos_camera.at<double>(0, 0) << " y:" << pos_camera.at<double>(0, 1) << " z:" << pos_camera.at<double>(0, 2) << endl;
        }
    }
    pose = _pose;
    angles = _angles;
    return val;
}
void load_cam(Mat &cameraMatrix, Mat &distCoeffs, String path){
    FileStorage fs2(path, FileStorage::READ);
    // FileStorage fs3();
    // Mat cameraMatrix, distCoeffs;
    fs2["cameraMatrix"] >> cameraMatrix;
    fs2["distCoeffs"] >> distCoeffs;
    fs2.release();
}
/*int main(int argc, char **argv)
{
    FileStorage fs2("./logitech.yml", FileStorage::READ);
    Mat cameraMatrix, distCoeffs;
    fs2["cameraMatrix"] >> cameraMatrix;
    fs2["distCoeffs"] >> distCoeffs;
    fs2.release();

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    Mat _R_flip = Mat(3, 3, CV_64F, cvScalar(0.));
    _R_flip.at<double>(0, 0) = 1.0;
    _R_flip.at<double>(1, 1) = -1.0;
    _R_flip.at<double>(2, 2) = -1.0;
    int id_i = 1;
    float marker_size = 0.1;
    float marker_sep = 0.1;
    int markers_x = 2;
    int markers_y = 2;
    int firstMarker = 0;
    cout << "marker_size: ";
    cin >> marker_size;
    cout << "marker_sep: ";
    cin >> marker_sep;
    cout << "markers_x: ";
    cin >> markers_x;
    cout << "markers_y: ";
    cin >> markers_y;
    cout << "firstMarker: ";
    cin >> firstMarker;
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markers_x, markers_y, marker_size, marker_sep, dictionary, firstMarker);
    // board = cv::aruco::GridBoard:: createCustomGridBoard(
    cv::VideoCapture inputVideo;
    inputVideo.open(1);

    while (inputVideo.grab())
    {
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);
        // std::vector<cv::Vec3d> rvecs, tvecs;
         cv::aruco::estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, distCoeffs, rvecs, tvecs);
        if (ids.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            cv::Vec3d tvec, rvec;
            cv::Mat objPoints;
            // int valid = estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec, tvec);
            int valid = _estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs,
                                       rvec, tvec, false, objPoints);

        // if (valid) {
            // if at least one board marker detected
            if (valid > 0)
            {
                
                Mat pos_camera = Mat(3, 3, CV_64F, cvScalar(0.));
                Mat R_ct, R_tc;
                // tvec = tvecs[i];
                // rvec = rvecs[i];
                // cout << "pose_raw  x:" << tvec[0] << " y:" << tvec[1] << " z:" << tvec[2] << endl;
                cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
                cv::Rodrigues(rvec, R_ct);
                R_tc = R_ct.t();
                double roll_marker, pitch_marker, yaw_marker;
                Point3d marker_angles;
                marker_angles = rotationMatrixToEAngles(_R_flip * R_tc);
                roll_marker = marker_angles.x;
                pitch_marker = marker_angles.y;
                yaw_marker = marker_angles.z;
                // cout << "marker_angles_raw  roll:" << roll_marker << " pitch:" << pitch_marker << " yaw:" << yaw_marker << endl;
                Mat tvec_mat = Mat(tvec);
                R_tc = -R_tc;
                pos_camera = R_tc * tvec_mat;
                cout << "pose  x:" << pos_camera.at<double>(0, 0) << " y:" << pos_camera.at<double>(0, 1) << " z:" << pos_camera.at<double>(0, 2) << endl;
                // cout << corners[i];
            }
        }
        cv::imshow("out", imageCopy);
        char key = (char)cv::waitKey(1);
        if (key == 27)
            break;
    }
    return 0;
}
*/