#include <ifcmesh_adapter.hpp>

#ifdef WITH_OPENMESH
#include <regex>
#include <ifcopenshell_adapter.hpp>
#include <utils.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace october {

ifc_objects_t
load_objects_(const std::string& ifc_file);


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
        if (!OpenMesh::IO::read_mesh(mesh, p.string())) {
            throw std::runtime_error("Unable to load mesh file \"" + p.string() + "\"");
        }

        std::vector<std::string> tokens = split_string(p.string(), "__(?:TYPE|GUID)_");
        if (tokens.size() != 3) {
            throw std::runtime_error("Error parsing filename \"" + p.string() + "\": Wrong number of tokens");
        }
        std::string guid = tokens[1];
        std::string type = tokens[2].substr(0, tokens[2].size() - 4);
        objects.push_back(std::make_tuple(mesh, guid, type));

        //std::string filename = p.string();
        //std::regex delim("__(?:TYPE|GUID)_");
        //std::regex_iterator<std::string::iterator> it(filename.begin(), filename.end(), delim);
        //std::regex_iterator<std::string::iterator> end_it;
        //while (it != end_it) {
            //std::cout << it->prefix() << "\n";
            //++it;
        //}


        //std::regex re("\\w+__GUID_(\\w+?)__TYPE_(\\w+)\\.obj");
        //std::smatch m;
        //if (!std::regex_match(p.filename().string(), m, re)) {
            //throw std::runtime_error("Unable to parse GUID/type info for object \"" + p.filename().string() + "\"");
        //} else {
            //std::cout << p.string() << "\n";
            //std::cout << "   " << m[1] << " " << m[2] << "\n";
        //}
        //auto file_objects = load_objects_(path);
        //objects.insert(objects.end(), file_objects.begin(), file_objects.end());
    }
    return ifc_wall_planes(objects, area_threshold, angle_threshold);
}



} // october

#endif // WITH_OPENMESH
