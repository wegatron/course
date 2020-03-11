#include "pnp_wrapper.h"
#include <from_opencv/upnp.h>
#include <from_opencv/dls.h>

int cv::RANSACUpdateNumIters(double p, double ep, int modelPoints, int maxIters)
{
    if( modelPoints <= 0 )
        CV_Error( Error::StsOutOfRange, "the number of model points should be positive" );

    p = MAX(p, 0.);
    p = MIN(p, 1.);
    ep = MAX(ep, 0.);
    ep = MIN(ep, 1.);

    // avoid inf's & nan's
    double num = MAX(1. - p, DBL_MIN);
    double denom = 1. - std::pow(1. - ep, modelPoints);
    if( denom < DBL_MIN )
        return 0;

    num = std::log(num);
    denom = std::log(denom);

    return denom >= 0 || -num >= maxIters*(-denom) ? maxIters : cvRound(num/denom);
}

cv::RANSACPointSetRegistrator::RANSACPointSetRegistrator(const Ptr<cv::PointSetRegistrator::Callback> &_cb, int _modelPoints, double _threshold, double _confidence, int _maxIters)
    : cb(_cb), modelPoints(_modelPoints), threshold(_threshold), confidence(_confidence), maxIters(_maxIters) {}

int cv::RANSACPointSetRegistrator::findInliers(const cv::Mat &m1, const cv::Mat &m2, const cv::Mat &model, cv::Mat &err, cv::Mat &mask, double thresh) const
{
    cb->computeError( m1, m2, model, err );
    mask.create(err.size(), CV_8U);

    CV_Assert( err.isContinuous() && err.type() == CV_32F && mask.isContinuous() && mask.type() == CV_8U);
    const float* errptr = err.ptr<float>();
    uchar* maskptr = mask.ptr<uchar>();
    float t = (float)(thresh*thresh);
    int i, n = (int)err.total(), nz = 0;
    for( i = 0; i < n; i++ )
    {
        int f = errptr[i] <= t;
        maskptr[i] = (uchar)f;
        nz += f;
    }
    return nz;
}

bool cv::RANSACPointSetRegistrator::getSubset(const cv::Mat &m1, const cv::Mat &m2, cv::Mat &ms1, cv::Mat &ms2, cv::RNG &rng, int maxAttempts) const
{
    std::vector<int> _idx(modelPoints);
    int* idx = _idx.data();

    const int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
    const int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;

    int esz1 = (int)m1.elemSize1() * d1;
    int esz2 = (int)m2.elemSize1() * d2;
    CV_Assert((esz1 % sizeof(int)) == 0 && (esz2 % sizeof(int)) == 0);
    esz1 /= sizeof(int);
    esz2 /= sizeof(int);

    const int count = m1.checkVector(d1);
    const int count2 = m2.checkVector(d2);
    CV_Assert(count >= modelPoints && count == count2);

    const int *m1ptr = m1.ptr<int>();
    const int *m2ptr = m2.ptr<int>();

    ms1.create(modelPoints, 1, CV_MAKETYPE(m1.depth(), d1));
    ms2.create(modelPoints, 1, CV_MAKETYPE(m2.depth(), d2));

    int *ms1ptr = ms1.ptr<int>();
    int *ms2ptr = ms2.ptr<int>();

    for( int iters = 0; iters < maxAttempts; ++iters )
    {
        int i;

        for( i = 0; i < modelPoints; ++i )
        {
            int idx_i;

            for ( idx_i = rng.uniform(0, count);
                  std::find(idx, idx + i, idx_i) != idx + i;
                  idx_i = rng.uniform(0, count) )
            {}

            idx[i] = idx_i;

            for( int k = 0; k < esz1; ++k )
                ms1ptr[i*esz1 + k] = m1ptr[idx_i*esz1 + k];

            for( int k = 0; k < esz2; ++k )
                ms2ptr[i*esz2 + k] = m2ptr[idx_i*esz2 + k];
        }

        if( cb->checkSubset(ms1, ms2, i) )
            return true;
    }

    return false;
}

bool cv::RANSACPointSetRegistrator::run(cv::InputArray _m1, cv::InputArray _m2, cv::OutputArray _model, cv::OutputArray _mask) const
{
    bool result = false;
    Mat m1 = _m1.getMat(), m2 = _m2.getMat();
    Mat err, mask, model, bestModel, ms1, ms2;

    int iter, niters = MAX(maxIters, 1);
    int d1 = m1.channels() > 1 ? m1.channels() : m1.cols;
    int d2 = m2.channels() > 1 ? m2.channels() : m2.cols;
    int count = m1.checkVector(d1), count2 = m2.checkVector(d2), maxGoodCount = 0;

    RNG rng((uint64)-1);

    CV_Assert( cb );
    CV_Assert( confidence > 0 && confidence < 1 );

    CV_Assert( count >= 0 && count2 == count );
    if( count < modelPoints )
        return false;

    Mat bestMask0, bestMask;

    if( _mask.needed() )
    {
        _mask.create(count, 1, CV_8U, -1, true);
        bestMask0 = bestMask = _mask.getMat();
        CV_Assert( (bestMask.cols == 1 || bestMask.rows == 1) && (int)bestMask.total() == count );
    }
    else
    {
        bestMask.create(count, 1, CV_8U);
        bestMask0 = bestMask;
    }

    if( count == modelPoints )
    {
        if( cb->runKernel(m1, m2, bestModel) <= 0 )
            return false;
        bestModel.copyTo(_model);
        bestMask.setTo(Scalar::all(1));
        return true;
    }

    for( iter = 0; iter < niters; iter++ )
    {
        int i, nmodels;
        if( count > modelPoints )
        {
            bool found = getSubset( m1, m2, ms1, ms2, rng, 10000 );
            if( !found )
            {
                if( iter == 0 )
                    return false;
                break;
            }
        }

        nmodels = cb->runKernel( ms1, ms2, model );
        if( nmodels <= 0 )
            continue;
        CV_Assert( model.rows % nmodels == 0 );
        Size modelSize(model.cols, model.rows/nmodels);

        for( i = 0; i < nmodels; i++ )
        {
            Mat model_i = model.rowRange( i*modelSize.height, (i+1)*modelSize.height );
            int goodCount = findInliers( m1, m2, model_i, err, mask, threshold );

            if( goodCount > MAX(maxGoodCount, modelPoints-1) )
            {
                std::swap(mask, bestMask);
                model_i.copyTo(bestModel);
                maxGoodCount = goodCount;
                niters = RANSACUpdateNumIters( confidence, (double)(count - goodCount)/count, modelPoints, niters );
            }
        }
    }

    if( maxGoodCount > 0 )
    {
        if( bestMask.data != bestMask0.data )
        {
            if( bestMask.size() == bestMask0.size() )
                bestMask.copyTo(bestMask0);
            else
                transpose(bestMask, bestMask0);
        }
        bestModel.copyTo(_model);
        result = true;
    }
    else
        _model.release();

    return result;
}

int pnp_ransac_callback::runKernel(cv::InputArray _m1, cv::InputArray _m2, cv::OutputArray _model) const
{
    cv::Mat opoints = _m1.getMat(), ipoints = _m2.getMat();
    if(flags == cv::SOLVEPNP_UPNP)
    {
        upnp solver(cameraMatrix, opoints, ipoints);
        cv::Mat r, t;
        solver.compute_pose(r, t);
        Rodrigues(r, rvec);
        cv::Mat _local_model;
        hconcat(rvec, t, _local_model);
        _local_model.copyTo(_model);
        return true;
    }
    if(flags == cv::SOLVEPNP_DLS)
    {
        dls solver(opoints, opoints);
        cv::Mat r, t;
        if(!solver.compute_pose(r, t))
        {
            return false;
        }
        Rodrigues(r, rvec);
        cv::Mat _local_model;
        hconcat(rvec, t, _local_model);
        _local_model.copyTo(_model);
        return true;
    }
    bool correspondence = solvePnP( _m1, _m2, cameraMatrix, distCoeffs,
                                    rvec, tvec, useExtrinsicGuess, flags);
    cv::Mat _local_model;
    hconcat(rvec, tvec, _local_model);
    _local_model.copyTo(_model);
    return correspondence;
}

void pnp_ransac_callback::computeError(cv::InputArray _m1, cv::InputArray _m2, cv::InputArray _model, cv::OutputArray _err) const
{

    cv::Mat opoints = _m1.getMat(), ipoints = _m2.getMat(), model = _model.getMat();

    int i, count = opoints.checkVector(3);
    cv::Mat _rvec = model.col(0);
    cv::Mat _tvec = model.col(1);


    cv::Mat projpoints(count, 2, CV_32FC1);
    projectPoints(opoints, _rvec, _tvec, cameraMatrix, distCoeffs, projpoints);

    const cv::Point2f* ipoints_ptr = ipoints.ptr<cv::Point2f>();
    const cv::Point2f* projpoints_ptr = projpoints.ptr<cv::Point2f>();

    _err.create(count, 1, CV_32FC1);
    float* err = _err.getMat().ptr<float>();

    for ( i = 0; i < count; ++i)
        err[i] = (float)norm( cv::Matx21f(ipoints_ptr[i] - projpoints_ptr[i]), cv::NORM_L2SQR );
}

bool solve_pnp_ransac(const std::vector<cv::Point3f> &objectPoints, const std::vector<cv::Point2f> &imagePoints, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, cv::Mat &rvec, cv::Mat &tvec, bool useExtrinsicGuess, int iterationsCount, float reprojectionError, double confidence, cv::OutputArray inliers, int flags)
{
    const size_t npoints = objectPoints.size();
    cv::Ptr<cv::PointSetRegistrator::Callback> cb
            = cv::makePtr<pnp_ransac_callback>(cameraMatrix, distCoeffs,
                                               flags, useExtrinsicGuess, rvec, tvec);
    int model_points = 4;
    if(flags == cv::SOLVEPNP_EPNP)
    {
        model_points = 5;
    } else if(flags == cv::SOLVEPNP_P3P)
    {

    } else if(flags == cv::SOLVEPNP_UPNP) {
        model_points = 6;
    } else if(flags == cv::SOLVEPNP_DLS) {
        model_points = 3;
    }else {
        throw std::logic_error("should be epnp or p3p!");
    }
    cv::RANSACPointSetRegistrator ransac_pnp(
                cb, // 通过model callback 实现不同种pnp算法, 参考PnPRansacCallback
                model_points, // 不同算法用到的模型点的数量, 比如p3p为4, epnp为5
                reprojectionError, // 像素误差阈值
                confidence, // 置信度
                iterationsCount); // 最大迭代次数
    cv::Mat local_model(3, 2, CV_64F);
    cv::Mat mask_local_inliers(1, objectPoints.size(), CV_8UC1);
    if(!ransac_pnp.run(
                objectPoints, // obj points 3d 点的坐标
                imagePoints, // image points 像素坐标
                local_model, // 3x2的向量, 第一列为rotation vector, 第二列为translation vector
                mask_local_inliers))  // 内点 vector<bool>
    {
        return false;
    }

//    if(flags == cv::SOLVEPNP_P3P) {
//        local_model.col(0).copyTo(rvec);
//        local_model.col(1).copyTo(tvec);
//        return true;
//    }

    size_t npoints1 = cv::countNonZero(mask_local_inliers);
    //std::vector<cv::Point3d> opoints_inliers(npoints1);
    //std::vector<cv::Point2d> ipoints_inliers(npoints1);

    const uchar* mask = mask_local_inliers.ptr<uchar>();
    //    for(size_t i=0, j=0; i<npoints; ++i)
    //    {
    //        if(mask[i] == 0) continue;
    //        opoints_inliers[j] = objectPoints[i];
    //        ipoints_inliers[j] = imagePoints[i];
    //        ++j;
    //    }

    //auto ret = cv::solvePnP(opoints_inliers, ipoints_inliers, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_EPNP);

    if(inliers.needed())
    {
        cv::Mat local_in(1, npoints1, CV_32S);
        int * ptr = local_in.ptr<int>();
        for(size_t i = 0, j=0; i<npoints; ++i)
        {
            if(mask[i]) {
                ptr[j] = i;
                ++j;
            }
        }
        local_in.copyTo(inliers);
    }

    local_model.col(0).copyTo(rvec);
    local_model.col(1).copyTo(tvec);
    return true;
}
