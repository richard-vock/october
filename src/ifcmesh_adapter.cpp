#include <ifcmesh_adapter.hpp>

#ifdef WITH_OPENMESH
#include <regex>
#include <ifcopenshell_adapter.hpp>
#include <utils.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace october {

ifcmesh_adapter::ifcmesh_adapter() : adapter() {
}

ifcmesh_adapter::~ifcmesh_adapter() {
}

bool
ifcmesh_adapter::supports_extension(const std::vector<std::string>& extensions) {
    bool support = true;
    for (const auto& ext : extensions) {
        if (ext != ".obj") {
            support = false;
            break;
        }
    }
    return support;
}

std::vector<bounded_plane::ptr_t>
ifcmesh_adapter::extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold) {
    ifc_objects_t objects;
    for (const auto& path : file_paths) {
        fs::path p(path);

        openmesh_t mesh;
        omerr().disable();
        OpenMesh::IO::Options opt;
        opt += OpenMesh::IO::Options::FaceNormal;
        opt += OpenMesh::IO::Options::VertexNormal;

        mesh.request_vertex_colors();
        mesh.request_vertex_normals();
        mesh.request_vertex_texcoords2D();
        mesh.request_face_normals();
        if (!OpenMesh::IO::read_mesh(mesh, p.string(), opt)) {
            throw std::runtime_error("Unable to load mesh file \"" + p.string() + "\"");
        }

        std::vector<std::string> tokens = split_string(p.string(), "__(?:TYPE|GUID)_");
        if (tokens.size() != 3) {
            throw std::runtime_error("Error parsing filename \"" + p.string() + "\": Wrong number of tokens");
        }
        std::string guid = tokens[1];
        std::string type = tokens[2].substr(0, tokens[2].size() - 4);
        objects.push_back(std::make_tuple(mesh, guid, type));
    }
    return ifc_wall_planes(objects, area_threshold, angle_threshold);
}



} // october

#endif // WITH_OPENMESH
