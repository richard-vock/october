#include <ifcopenshell_adapter.hpp>

#if defined(WITH_IFCOPENSHELL) && defined(WITH_OPENMESH)

#include <set>
#include <traverse.hpp>

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <ifcgeom/IfcGeomObjects.h>


namespace october {


struct openmesh_default_traits : public OpenMesh::DefaultTraits {
	typedef OpenMesh::Vec4f  Color;

	VertexAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color );
	FaceAttributes( OpenMesh::Attributes::Normal );
};

typedef ::OpenMesh::TriMesh_ArrayKernelT<openmesh_default_traits> openmesh_t;

typedef std::tuple<openmesh_t, std::string, std::string> ifc_object_t;

typedef std::vector<ifc_object_t> ifc_objects_t;

ifc_objects_t
extract_objects_(const std::string& ifc_file);

std::vector<bounded_plane::ptr_t>
ifc_wall_planes_(const ifc_objects_t& objects, float area_threshold, float angle_threshold);


ifcopenshell_adapter::ifcopenshell_adapter() : adapter() {
}

ifcopenshell_adapter::~ifcopenshell_adapter() {
}

bool
ifcopenshell_adapter::supports_extension(const std::vector<std::string>& extensions) {
    bool support = true;
    for (const auto& ext : extensions) {
        if (ext != ".ifc") {
            support = false;
            break;
        }
    }
    return support;
}

std::vector<bounded_plane::ptr_t>
ifcopenshell_adapter::extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold) {
    ifc_objects_t objects;
    for (const auto& path : file_paths) {
        auto file_objects = extract_objects_(path);
        objects.insert(objects.end(), file_objects.begin(), file_objects.end());
    }
    return ifc_wall_planes_(objects, area_threshold, angle_threshold);
}

std::vector<bounded_plane::ptr_t>
ifc_wall_planes_(const ifc_objects_t& objects, float area_threshold, float angle_threshold) {
    typedef openmesh_t::VertexHandle          vertex_handle_t;
    typedef openmesh_t::FaceHandle            face_handle_t;
    typedef face_handle_t                     entity_t;
    typedef connected_component_t<entity_t>   component_t;
    typedef connected_components_t<entity_t>  components_t;

    std::vector<bounded_plane::ptr_t> planes;
    for (const auto& obj : objects) {
        std::string type = std::get<2>(obj);
        if (type != "ifcwall" && type != "ifcwallstandardcase" && type != "ifcfooting" && type != "ifcslab") {
            continue;
        }
        const openmesh_t& mesh = std::get<0>(obj);

        std::vector<face_handle_t> faces;
        for (auto it = mesh.faces_begin(); it != mesh.faces_end(); ++it) {
            faces.push_back(*it);
        }

        auto get_neighbors = [&] (entity_t face) { return std::vector<face_handle_t>(mesh.cff_begin(face), mesh.cff_end(face)); };

        components_t components = connected_components(
            faces,
            get_neighbors,
            [&] (entity_t face, entity_t start_face) {
                vec3f_t nrm(mesh.normal(face).data());
                vec3f_t start_nrm(mesh.normal(start_face).data());
                if (1.f - fabs(nrm.dot(start_nrm)) > angle_threshold) return false;
                return true;
            },
            [&] (entity_t start_face) {
                return true;
            }
        );

        // extract planes
        for (const auto& component : components) {
            if (!component.size()) continue;
            // get vertex positions constituting points
            std::set<vertex_handle_t> vertex_set;
            float area = 0.f;
            for (const auto& face : component) {
                std::vector<vertex_handle_t> verts(mesh.cfv_begin(face), mesh.cfv_end(face));
                vec3f_t p0(mesh.point(verts[0]).data());
                vec3f_t p1(mesh.point(verts[1]).data());
                vec3f_t p2(mesh.point(verts[2]).data());
                area += 0.5f * (p1 - p0).cross(p2 - p0).norm();
                vertex_set.insert(verts.begin(), verts.end());
            }
            if (area < area_threshold) continue;
            std::vector<vec3f_t> positions(vertex_set.size());
            std::transform(vertex_set.begin(), vertex_set.end(), positions.begin(), [&] (vertex_handle_t vert) { return vec3f_t(mesh.point(vert).data()); });
            planes.push_back(bounded_plane::from_points(positions));
        }
    }

    return planes;
}

ifc_objects_t
extract_objects_(const std::string& ifc_file) {
    typedef typename openmesh_t::Point        point_t;
    typedef typename openmesh_t::Normal       normal_t;
    typedef typename openmesh_t::VertexHandle vertex_t;

    // load ifc file
	IfcGeomObjects::Settings(IfcGeomObjects::USE_WORLD_COORDS, true);
	IfcGeomObjects::Settings(IfcGeomObjects::WELD_VERTICES, true);
	IfcGeomObjects::Settings(IfcGeomObjects::DISABLE_TRIANGULATION, false);
	IfcGeomObjects::Settings(IfcGeomObjects::FORCE_CCW_FACE_ORIENTATION, true);
	//if (!IfcGeomObjects::Init(ifcPath.string(), &std::cout) ) {
	if (!IfcGeomObjects::Init(ifc_file, nullptr, nullptr) ) {
        throw std::runtime_error("Unable to initialize IfcGeomObjects.");
	}

    std::set<std::string> ignore_types;
    ignore_types.insert("ifcopeningelement");
    ignore_types.insert("ifcspace");

    ifc_objects_t objects;
    do { // while (IfcGeomObjects::Next())
        ifc_object_t object;
		std::get<0>(object).request_vertex_normals();
		std::get<0>(object).request_face_normals();

		const IfcGeomObjects::IfcGeomObject* geom_object = IfcGeomObjects::Get();

        // get type
		std::get<2>(object) = geom_object->type();
		for (std::string::iterator c = std::get<2>(object).begin(); c != std::get<2>(object).end(); ++c) {
			*c = tolower(*c);
		}
		if (ignore_types.find(std::get<2>(object)) != ignore_types.end()) continue;
        std::get<1>(object) = geom_object->guid();

        // get polygon data
		auto& ifc_mesh = geom_object->mesh();
		auto& fs = ifc_mesh.faces();
		auto& vs = ifc_mesh.verts();

        uint32_t v_count = vs.size() / 3;
        std::vector<vertex_t> vertex_handles;
        for (uint32_t v = 0; v < v_count; ++v) {
            auto vertex = std::get<0>(object).add_vertex(point_t(vs[3*v + 0], vs[3*v + 1], vs[3*v + 2]));
            vertex_handles.push_back(vertex);
        }

		for (uint32_t f = 0; f < fs.size(); f += 3) {
            std::vector<vertex_t> vertices;
            for (uint32_t v = 0; v < 3; ++v) {
                int idx = fs[f+v];
                vertices.push_back(vertex_handles[idx]);
            }
            std::get<0>(object).add_face(vertices);
		}

        // update mesh-internal normal state
        std::get<0>(object).update_normals();

        objects.push_back(object);
    } while (IfcGeomObjects::Next());

    IfcGeomObjects::CleanUp();

    return objects;
}

} // october

#endif // defined(WITH_IFCOPENSHELL) && defined(WITH_OPENMESH)
