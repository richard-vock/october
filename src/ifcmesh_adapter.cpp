#include <ifcmesh_adapter.hpp>

#ifdef WITH_OPENMESH
#include <regex>
#include <ifcopenshell_adapter.hpp>
#include <utils.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace october {

struct object_info {
  std::string path;
  std::string guid;
  std::string type;

  template <typename Archive> void serialize(Archive &ar) {
    ar(CEREAL_NVP(path));
    ar(CEREAL_NVP(guid));
    ar(CEREAL_NVP(type));
  }
};

ifcmesh_adapter::ifcmesh_adapter() : adapter() {
}

ifcmesh_adapter::~ifcmesh_adapter() {
}

bool
ifcmesh_adapter::supports_extension(const std::vector<std::string>& extensions) {
    bool support = true;
    for (const auto& ext : extensions) {
        if (ext != ".ifcmesh") {
            support = false;
            break;
        }
    }
    return support;
}

std::vector<bounded_plane::ptr_t>
ifcmesh_adapter::extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold, std::string& guid) {
    std::ifstream in(file_paths[0]);
    if (!in.good()) {
        throw std::runtime_error("ifcmesh_adapter::extract_planes: Unable to open file \"" + file_paths[0] + "\" for reading.");
    }
    std::vector<object_info> objs;
    {
        cereal::JSONInputArchive ar(in);
        ar(cereal::make_nvp("guid", guid));
        ar(cereal::make_nvp("objects", objs));
    }
    in.close();

    ifc_objects_t full_objs;
    for (const auto& obj : objs) {
        fs::path mesh_path = fs::path(file_paths[0]).parent_path() / obj.path;
        ifc_object_t ifc_obj;

        openmesh_t mesh;
        omerr().disable();
        OpenMesh::IO::Options opt;
        opt += OpenMesh::IO::Options::FaceNormal;
        opt += OpenMesh::IO::Options::VertexNormal;

        mesh.request_vertex_colors();
        mesh.request_vertex_normals();
        mesh.request_vertex_texcoords2D();
        mesh.request_face_normals();
        if (!OpenMesh::IO::read_mesh(mesh, mesh_path.string(), opt)) {
            throw std::runtime_error("ifcmesh_adapter::extract_planes: Unable to load mesh file \"" + mesh_path.string() + "\"");
        }
        mesh.update_normals();
        std::get<0>(ifc_obj) = mesh;
        std::get<1>(ifc_obj) = obj.guid;
        std::get<2>(ifc_obj) = obj.type;

        full_objs.push_back(ifc_obj);
    }

    return ifc_wall_planes(full_objs, area_threshold, angle_threshold);
}



} // october

#endif // WITH_OPENMESH
