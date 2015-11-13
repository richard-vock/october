#include <e57n_adapter.hpp>
#if defined(WITH_E57PCL) && defined(WITH_PRIMITIVE_DETECTION)

#include <stdexcept>

#include <e57_pcl/read.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <primitive_detection/PrimitiveDetector.h>

namespace october {

typedef pcl::PointNormal          point_t;
typedef pcl::PointCloud<point_t>  cloud_t;


e57n_adapter::e57n_adapter(float epsilon, float bitmap_epsilon, uint32_t min_support, float probability) : adapter(), epsilon_(epsilon), bitmap_epsilon_(bitmap_epsilon), min_support_(min_support), probability_(probability) {
}


e57n_adapter::~e57n_adapter() {
}

bool
e57n_adapter::supports_extension(const std::vector<std::string>& extensions) {
    bool support = true;
    for (const auto& ext : extensions) {
        if (ext != ".e57n") {
            support = false;
            break;
        }
    }
    return support;
}

std::vector<bounded_plane::ptr_t>
e57n_adapter::extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold, std::string& guid) {
    if (file_paths.size() != 1) {
        throw std::runtime_error("Exactly one E57n file must be supplied.");
    }
    
    cloud_t::Ptr cloud = e57_pcl::load_e57_cloud_with_normals(file_paths[0], guid);

    pcshapes::PrimitiveDetector detector;
    detector.setEpsilon(epsilon_);
    detector.setBitmapEpsilon(bitmap_epsilon_);
    detector.setNormalThreshold(1.f - angle_threshold);
    detector.setMinimumSupport(min_support_);
    detector.setProbability(probability_);
    pcshapes::SupportedTypes types;
    types.set(pcshapes::PLANE);
    auto primitives = detector.detectPrimitives<point_t>(cloud, types);

    std::vector<bounded_plane::ptr_t> planes;
    for (auto prim : primitives) {
        auto primPlane = std::dynamic_pointer_cast<pcshapes::PrimitivePlane>(prim);
        if (primPlane->area() < area_threshold) continue;
        std::vector<int> indices = primPlane->indices();
        std::vector<vec3f_t> points(indices.size());
        vec3f_t approx_normal = vec3f_t::Zero();
        for (uint32_t i = 0; i < indices.size(); ++i) {
            points[i] = cloud->points[indices[i]].getVector3fMap();
            approx_normal += cloud->points[indices[i]].getNormalVector3fMap();
        }
        planes.push_back(bounded_plane::from_points(points, approx_normal.normalized()));
    }

    return planes;
}


} // october

#endif // defined(WITH_E57PCL) && defined(WITH_PRIMITIVE_DETECTION)
