#include <pcl_adapter.hpp>
#if defined(WITH_PCL) && defined(WITH_PRIMITIVE_DETECTION)

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <primitive_detection/PrimitiveDetector.h>

namespace october {

typedef pcl::PointNormal          point_t;
typedef pcl::PointCloud<point_t>  cloud_t;


pcl_adapter::pcl_adapter(float epsilon, float bitmap_epsilon, uint32_t min_support, float probability) : adapter(), epsilon_(epsilon), bitmap_epsilon_(bitmap_epsilon), min_support_(min_support), probability_(probability) {
}


pcl_adapter::~pcl_adapter() {
}

bool
pcl_adapter::supports_extension(const std::vector<std::string>& extensions) {
    bool support = true;
    for (const auto& ext : extensions) {
        if (ext != ".pcd") {
            support = false;
            break;
        }
    }
    return support;
}

std::vector<bounded_plane::ptr_t>
pcl_adapter::extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold) {
    cloud_t::Ptr cloud(new cloud_t());
    for (const auto& path : file_paths) {
        cloud_t::Ptr scan(new cloud_t());
        pcl::io::loadPCDFile(path, *scan);
        cloud->insert(cloud->end(), scan->begin(), scan->end());
    }

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
        for (uint32_t i = 0; i < indices.size(); ++i) {
            points[i] = cloud->points[indices[i]].getVector3fMap();
        }
        planes.push_back(bounded_plane::from_points(points));
    }

    return planes;
}


} // october

#endif // defined(WITH_PCL) && defined(WITH_PRIMITIVE_DETECTION)
