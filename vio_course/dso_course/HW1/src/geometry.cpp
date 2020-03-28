#include "geometry.hpp"
#include <fstream>
#include <sstream>
#include <vector>

hw::GeometryUndistorter::GeometryUndistorter(std::string &path)
{
    w_rect_ = 0;
    h_rect_ = 0;

    std::string data_path = path;

    if(path.find_last_of("/") != path.size()-1)
        data_path = path + "/";

    std::string cam_file = data_path + "camera.txt";

    std::ifstream file(cam_file.c_str());
    assert(file.good());
    std::cout<< "[INFO]: Reading Geometry Calibration from " << cam_file <<std::endl;

    std::string param, size, setting, size_rect;

    std::getline(file, param);
    std::getline(file, size);
    std::getline(file, setting);
    std::getline(file, size_rect);

    std::stringstream ss;
    // line 1
    ss << param;
    double fx, fy, cx, cy, omega;
    ss >> fx >> fy >> cx >> cy >> k1_ >> k2_ >> k3_ >> k4_;
    // line 2
    ss.clear();
    ss << size;
    ss >> w_ >> h_;
    // line 4
    ss.clear();
    ss << size_rect;
    ss >> w_rect_ >> h_rect_;

    if(setting == "crop")
        crop = true;
    else
        crop = false;

    K_.setIdentity();
    if(cx < 1 && cy < 1)
    {
        K_(0,0) = fx*w_;
        K_(0,2) = cx*w_;
        K_(1,1) = fy*h_;
        K_(1,2) = cy*h_;
    }


    remap_.resize(w_rect_*h_rect_);
    makeOptimalK_crop();
    initRectifyMap();
}

// 计算矫正畸变后新的内参
void hw::GeometryUndistorter::makeOptimalK_crop()
{
    if(!crop)
        return;

    K_rect_.setIdentity();
    std::vector<Eigen::Vector2f> tgx; tgx.reserve(100000);
    std::vector<Eigen::Vector2f> tgy; tgy.reserve(100000);
    for(int i=0; i<100000; ++i) {
      tgx.emplace_back((i-50000)*1e-4, 0);
      tgy.emplace_back(0, (i-50000)*1e-4);
    }

    float minx = 0;
    float maxx = 0;
    float miny = 0;
    float maxy = 0;

    // std::cout << "tg in:" << std::endl;
    // std::cout << tgx[0].transpose() << " : " << tgx[99999].transpose() << std::endl;
    // std::cout << tgy[0].transpose() << " : " << tgy[99999].transpose() << std::endl;
    // std::cout << "K_:" << std::endl;
    // std::cout << K_ << std::endl;

    distortCoordinates(tgx);
    for(int i=0; i<100000; ++i) {
      if(tgx[i].x()<0 || tgx[i].x()>w_-1) continue;
      if(minx == 0) minx = (i-50000)*1e-4;
      maxx = (i-50000)*1e-4;
    }

    distortCoordinates(tgy);
    for(int i=0; i<100000; ++i) {
      if(tgy[i].y()<0 || tgy[i].y()>h_-1) continue;
      if(miny == 0) miny = (i-50000)*1e-4;
      maxy = (i-50000)*1e-4;
    }

    // std::cout << "tg out:" << std::endl;
    // std::cout << tgx[0].transpose() << " : " << tgx[99999].transpose() << std::endl;
    // std::cout << tgy[0].transpose() << " : " << tgy[99999].transpose() << std::endl;

    // std::cout << "minx:" << minx << " maxx:" << maxx << std::endl;
    // std::cout << "miny:" << miny << " maxy:" << maxy << std::endl;
    minx *= 1.01;
    maxx *= 1.01;
    miny *= 1.01;
    maxy *= 1.01;
    std::cout << "raw minx:" << minx << " maxx:" << maxx << std::endl;
    std::cout << "raw miny:" << miny << " maxy:" << maxy << std::endl;

    // update
    bool oobLeft = true;
    bool oobRight = true;
    bool oobTop = true;
    bool oobDown = true;
    std::vector<Eigen::Vector2f> buffer; buffer.reserve(2*std::max(h_, w_));
    int iter_times = 0;
    while(oobLeft || oobRight || oobTop || oobDown) {
      oobLeft = oobRight = oobTop = oobDown = false;
      // left right
      buffer.clear();
      for(int y=0; y<h_; ++y) {
        float cy = miny + (maxy-miny)*y/(h_-1);
        buffer.emplace_back(minx, cy);
        buffer.emplace_back(maxx, cy);
      }
      distortCoordinates(buffer);
      for(int y=0; y<h_; ++y) {
        if(buffer[2*y].x()<0) oobLeft=true;
        if(buffer[2*y+1].x()>w_-1) oobRight=true;
      }

      // top down
      buffer.clear();
      for(int x=0; x<w_; ++x) {
        float cx = minx + (maxx - minx)*x/(w_-1);
        buffer.emplace_back(cx, miny);
        buffer.emplace_back(cx, maxy);
      }
      distortCoordinates(buffer);
      for(int x=0; x<w_; ++x) {
        if(buffer[2*x].y()<0) oobTop=true;
        if(buffer[2*x+1].y()>h_-1) oobDown=true;
      }

      // only one direction
      if((oobLeft||oobRight) && (oobTop ||oobDown)) {
        if((maxx-minx) > (maxy-miny)) {
          oobTop = oobDown = false;
        } else { oobLeft = oobRight = false; }
      }

      // do shrink
      if(oobLeft) minx *= 0.995;
      if(oobRight) maxx *= 0.995;
      if(oobTop) miny *= 0.995;
      if(oobDown) maxy *= 0.995;
      if(++iter_times> 500) {
        std::cerr << "Fail to compute good camera matrix." << std::endl;
        exit(1);
      }
    }

    std::cout << "refined iter=" << iter_times << std::endl;
    std::cout << "refined minx:" << minx << " maxx:" << maxx << std::endl;
    std::cout << "refined miny:" << miny << " maxy:" << maxy << std::endl;

    K_rect_(0,0) = (w_-1)/(maxx - minx);
    K_rect_(1,1) = (h_-1)/(maxy - miny);
    K_rect_(0,2) = -minx*K_rect_(0,0);
    K_rect_(1,2) = -miny*K_rect_(0,0);
}

// FoV model
// void hw::GeometryUndistorter::distortCoordinates(std::vector<Eigen::Vector2f> &in)
// {
//     float dist = omega_;
// 	float d2t = 2.0f * tan(dist / 2.0f);
//     int n = in.size();

//     float fx = K_(0, 0);
//     float fy = K_(1, 1);
//     float cx = K_(0, 2);
//     float cy = K_(1, 2);

//     float ofx = K_rect_(0, 0);
//     float ofy = K_rect_(1, 1);
//     float ocx = K_rect_(0, 2);
//     float ocy = K_rect_(1, 2);

//     for(int i=0; i<n; ++i)
//     {
//         float x = in[i][0];
//         float y = in[i][1];
//         float ix = (x - ocx) / ofx;
//         float iy = (y - ocy) / ofy;

//         float r = sqrtf(ix*ix + iy*iy);
// 		float fac = (r==0 || dist==0) ? 1 : atanf(r * d2t)/(dist*r);

// 		ix = fx*fac*ix+cx;
// 		iy = fy*fac*iy+cy;

// 		in[i][0] = ix;
// 		in[i][1] = iy;
//     }
// }

// Equidistant model
void hw::GeometryUndistorter::distortCoordinates(std::vector<Eigen::Vector2f> &in)
{
    float fx = K_(0, 0);
    float fy = K_(1, 1);
    float cx = K_(0, 2);
    float cy = K_(1, 2);
    float k1 = k1_;
    float k2 = k2_;
    float k3 = k3_;
    float k4 = k4_;

    float ofx = K_rect_(0, 0);
    float ofy = K_rect_(1, 1);
    float ocx = K_rect_(0, 2);
    float ocy = K_rect_(1, 2);

    int n = in.size();

    for(int i=0;i<n; i++)
    {
        float &x = in[i][0];
        float &y = in[i][1];

        // // TODO according Equidistant model, to calculate distorted coordinates
        float ix = (x - ocx)/ofx;
        float iy = (y - ocy)/ofy;
        float r = sqrt(ix*ix + iy*iy);
        float theta = atan(r);
        float theta2 = theta * theta;
        float theta4 = theta2 * theta2;
        float theta6 = theta2 * theta4;
        float theta8 = theta4 * theta4;
        float thetad_r = theta/r * (1 + k1*theta2 + k2*theta4 + k3*theta6 + k4*theta8);
        float ix_dis = thetad_r * ix;
        float iy_dis = thetad_r * iy;

        x = fx * ix_dis + cx;
        y = fy * iy_dis + cy;
    }

}


void hw::GeometryUndistorter::initRectifyMap()
{
    for(int y = 0; y < h_rect_; ++y)
        for(int x = 0; x < w_rect_; ++x)
        {
            remap_[x+y*w_rect_][0] = x;
            remap_[x+y*w_rect_][1] = y;
        }

    distortCoordinates(remap_);

    for(int y = 0; y < h_rect_; ++y)
        for(int x = 0; x < w_rect_; ++x)
        {
            // make rounding resistant.
			float ix = remap_[x+y*w_rect_][0];
			float iy = remap_[x+y*w_rect_][1];


			if(ix > 0 && iy > 0 && ix < w_-1 &&  iy < h_-1)
			{
				remap_[x+y*w_rect_][0] = ix;
				remap_[x+y*w_rect_][1] = iy;
			}
			else
			{
				remap_[x+y*w_rect_][0] = -1;
				remap_[x+y*w_rect_][1] = -1;
			}
        }

}


void hw::GeometryUndistorter::processFrame(cv::Mat &src, cv::Mat &dst)
{
    assert(src.type() == CV_8UC1);

    if(src.cols != w_ && src.rows != h_)
    {
        std::cout << "[ERROR]: The solution of input isn't illegal. We need "
                  << w_ << " * " << h_ << "! "<< std::endl;
    }

    dst = cv::Mat(h_rect_, w_rect_, CV_8UC1);

    int cols = w_rect_;
    int rows = h_rect_;

    if(dst.isContinuous())
    {
        cols = cols * rows;
        rows = 1;
    }

    uchar* ptr_src = 0;
    uchar* ptr_dst = 0;
    for(int i = 0; i < rows; ++i)
    {
        ptr_dst = dst.ptr<uchar>(i);
        for(int j = 0; j < cols; ++j)
        {
            int id = i * cols + j;

            float x_f = remap_[id][0];
            float y_f = remap_[id][1];

            if(x_f < 0)
                ptr_dst[j] = 0;
            else
            {
                int x_i = std::floor(x_f);
                int y_i = std::floor(y_f);
                x_f -= x_i;
                y_f -= y_i;
                float xy_f = x_f*y_f;

                ptr_src = src.ptr<uchar>(y_i) + x_i;

                ptr_dst[j] = xy_f * ptr_src[1+w_]
                                + (y_f - xy_f) * ptr_src[w_]
                                + (x_f - xy_f) * ptr_src[1]
                                + (1-x_f-y_f+xy_f) * ptr_src[0];

            }

        }
    }

}
