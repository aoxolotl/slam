#include "pointcloudio.hpp"

template<typename PointT> 
void PointCloudIO<PointT>::saveImage(cv::Mat rgbIm, std::string im_name)
{
	cv::imwrite(im_name, rgbIm);
}
