#ifndef PNP_WRAPPER_H
#define PNP_WRAPPER_H

#include <from_opencv/precomp.hpp>
#include <from_opencv/p3p.h>
#include <opencv2/calib3d.hpp>

#include <iostream>

namespace cv
{
int RANSACUpdateNumIters( double p, double ep, int modelPoints, int maxIters );

class RANSACPointSetRegistrator : public PointSetRegistrator
{
public:
    RANSACPointSetRegistrator(const Ptr<PointSetRegistrator::Callback>& _cb=Ptr<PointSetRegistrator::Callback>(),
                              int _modelPoints=0, double _threshold=0, double _confidence=0.99, int _maxIters=1000);

    int findInliers( const Mat& m1, const Mat& m2, const Mat& model, Mat& err, Mat& mask, double thresh ) const;

    bool getSubset( const Mat& m1, const Mat& m2, Mat& ms1, Mat& ms2, RNG& rng, int maxAttempts=1000 ) const;

    bool run(InputArray _m1, InputArray _m2, OutputArray _model, OutputArray _mask) const override;

    void setCallback(const Ptr<PointSetRegistrator::Callback>& _cb) override { cb = _cb; }

    Ptr<PointSetRegistrator::Callback> cb;
    int modelPoints;
    double threshold;
    double confidence;
    int maxIters;
};
}

class pnp_ransac_callback final : public cv::PointSetRegistrator::Callback
{
public:
    pnp_ransac_callback(cv::Mat _cameraMatrix=cv::Mat(3,3,CV_64F), cv::Mat _distCoeffs=cv::Mat(4,1,CV_64F), int _flags=0,
                        bool _useExtrinsicGuess=false, cv::Mat _rvec=cv::Mat(), cv::Mat _tvec=cv::Mat() )
        : cameraMatrix(_cameraMatrix), distCoeffs(_distCoeffs), flags(_flags), useExtrinsicGuess(_useExtrinsicGuess),
          rvec(_rvec), tvec(_tvec) {}

    int runKernel(cv::InputArray _m1, cv::InputArray _m2, cv::OutputArray _model) const override;

    /* Pre: True */
    /* Post: fill _err with projection errors */
    void computeError( cv::InputArray _m1, cv::InputArray _m2, cv::InputArray _model, cv::OutputArray _err ) const override;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    int flags;
    bool useExtrinsicGuess;
    cv::Mat rvec;
    cv::Mat tvec;
};

bool solve_pnp_ransac(const std::vector<cv::Point3f> &objectPoints, const std::vector<cv::Point2f> &imagePoints,
                      const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs,
                      cv::Mat &rvec, cv::Mat &tvec,
                      bool useExtrinsicGuess = false, int iterationsCount = 100,
                      float reprojectionError = 8.0, double confidence = 0.99,
                      cv::OutputArray inliers = cv::noArray(),
                      int flags =  cv::SOLVEPNP_P3P);

#endif //PNP_WRAPPER_H
