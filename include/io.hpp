#ifndef _OCTOBER_IO_HPP_
#define _OCTOBER_IO_HPP_

#include <string>

#include "types.hpp"

namespace october {


void write_image(std::shared_ptr<const image_t> img, const std::string& path);


} // october

#endif /* _OCTOBER_IO_HPP_ */
