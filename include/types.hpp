#ifndef _OCTOBER_TYPES_HPP_
#define _OCTOBER_TYPES_HPP_

#include <memory>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace october {


typedef cv::Mat image_t;

typedef Eigen::Vector2f vec2f_t;
typedef Eigen::Vector2i vec2i_t;
typedef Eigen::Vector3f vec3f_t;
typedef Eigen::Vector3i vec3i_t;
typedef Eigen::Vector4f vec4f_t;
typedef Eigen::Vector4i vec4i_t;

typedef Eigen::Matrix3f mat3f_t;
typedef Eigen::Matrix4f mat4f_t;
typedef Eigen::Matrix4f mat23f_t;
typedef Eigen::Affine2f affine2f_t;
typedef Eigen::Affine3f affine3f_t;
typedef Eigen::Rotation2D<float> rotation2f_t;
typedef Eigen::Translation<float, 2> translation2f_t;

typedef Eigen::AlignedBox<float, 2> bbox2f_t;
typedef Eigen::AlignedBox<float, 3> bbox3f_t;


} // october

#endif /* _OCTOBER_TYPES_HPP_ */
