#include <io.hpp>

namespace october {


void write_image(std::shared_ptr<const image_t> img, const std::string& path) {
	cv::Mat png(img->rows, img->cols, CV_16UC1);
	img->convertTo(png, CV_16UC1, 65535.0);
	cv::imwrite(path, png);
}


} // october
