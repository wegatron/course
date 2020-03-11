#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>


int main(int argc, char *argv[])
{
  //cv::Mat gray_src = cv::imread("../1.png", 0);
  // cv::imshow("src", gray_src);
  // cv::Mat dx, dy;
  // cv::Mat gray_src;
  // gray_src.create(10, 10, CV_8UC1);
  // for(int i=0; i<10; ++i) {
  //   for(int j=0; j<10; ++j) {
  //     gray_src.at<uchar>(i, j) = j;
  //   }
  // }

  // cv::spatialGradient(gray_src, dx, dy);
  // dx = 0.125 * dx;
  //Sobel( gray_src, dx_sobel, CV_16SC1, 1, 0, 3 );
  // std::cout << dx << std::endl;
  //cv::Sobel(gray_src, dx, -1, 1, 0);

  //cv::imshow("dx", dx);
  //cvv::showImage(dx_sobel);
  //cv::imshow("dx_sobel", dx_sobel);
  //cv::waitKey(0);
  return 0;
}
