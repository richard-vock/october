#include <ifcopenshell_adapter.hpp>

#if defined(WITH_IFCOPENSHELL) && defined(WITH_OPENMESH)

#include <set>
#include <traverse.hpp>

#include <ifcgeom/IfcGeomIterator.h>


namespace october {


ifc_objects_t
extract_objects_(const std::string& ifc_file, std::string& guid);

std::vector<bounded_plane::ptr_t>
ifc_wall_planes(const ifc_objects_t& objects, float area_threshold, float angle_threshold);


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
ifcopenshell_adapter::extract_planes(const std::vector<std::string>& file_paths, float area_threshold, float angle_threshold, std::string& guid) {
    ifc_objects_t objects;
    for (const auto& path : file_paths) {
        auto file_objects = extract_objects_(path, guid);
        objects.insert(objects.end(), file_objects.begin(), file_objects.end());
    }
    return ifc_wall_planes(objects, area_threshold, angle_threshold);
}

std::vector<bounded_plane::ptr_t>
ifc_wall_planes(const ifc_objects_t& objects, float area_threshold, float angle_threshold) {
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
            vec3f_t approx_normal = vec3f_t::Zero();
            float area = 0.f;
            for (const auto& face : component) {
                std::vector<vertex_handle_t> verts(mesh.cfv_begin(face), mesh.cfv_end(face));
                vec3f_t p0(mesh.point(verts[0]).data());
                vec3f_t p1(mesh.point(verts[1]).data());
                vec3f_t p2(mesh.point(verts[2]).data());
                area += 0.5f * (p1 - p0).cross(p2 - p0).norm();
                vertex_set.insert(verts.begin(), verts.end());
                approx_normal += vec3f_t(mesh.normal(face).data());
            }
            approx_normal.normalize();
            if (area < area_threshold) continue;
            if ((type == "ifcwall" || type == "ifcwallstandardcase") && fabs(approx_normal[2]) > 0.995f) continue; // Ignore horizontal caps of walls.
            std::vector<vec3f_t> positions(vertex_set.size());
            std::transform(vertex_set.begin(), vertex_set.end(), positions.begin(), [&] (vertex_handle_t vert) { return vec3f_t(mesh.point(vert).data()); });
            planes.push_back(bounded_plane::from_points(positions, approx_normal));
        }
    }

    return planes;
}

ifc_objects_t
extract_objects_(const std::string& ifc_file, std::string& guid) {
    typedef typename openmesh_t::Point        point_t;
    typedef typename openmesh_t::Normal       normal_t;
    typedef typename openmesh_t::VertexHandle vertex_t;

    IfcGeom::IteratorSettings settings;

    settings.set(IfcGeom::IteratorSettings::APPLY_DEFAULT_MATERIALS,      true);
    settings.set(IfcGeom::IteratorSettings::USE_WORLD_COORDS,             true);
    settings.set(IfcGeom::IteratorSettings::WELD_VERTICES,                true);
    settings.set(IfcGeom::IteratorSettings::DISABLE_TRIANGULATION,        false);
    //settings.set(IfcGeom::IteratorSettings::SEW_SHELLS,                   sew_shells);
    //settings.set(IfcGeom::IteratorSettings::CONVERT_BACK_UNITS,           convert_back_units);
    //settings.set(IfcGeom::IteratorSettings::FASTER_BOOLEANS,              merge_boolean_operands);
    //settings.set(IfcGeom::IteratorSettings::DISABLE_OPENING_SUBTRACTIONS, disable_opening_subtractions);
    //settings.set(IfcGeom::IteratorSettings::INCLUDE_CURVES,               include_plan);
    //settings.set(IfcGeom::IteratorSettings::EXCLUDE_SOLIDS_AND_SURFACES,  !include_model);
    
    IfcGeom::Iterator<double> context_iterator(settings, ifc_file);

    IfcParse::IfcFile* file_ptr = context_iterator.getFile();
    auto root_entity = file_ptr->entitiesByType<IfcSchema::IfcRoot>();
    guid = (*root_entity->begin())->GlobalId();

    std::set<std::string> ignore_types;
    ignore_types.insert("ifcopeningelement");
    ignore_types.insert("ifcspace");
    
    context_iterator.excludeEntities(ignore_types);
    context_iterator.initialize();

    ifc_objects_t objects;
    
    do {
        const IfcGeom::Element<double>* geom_object = context_iterator.get();
        auto triangulation = static_cast<const IfcGeom::TriangulationElement<double>*>(geom_object);
        const IfcGeom::Representation::Triangulation<double>& mesh = triangulation->geometry();
        
        ifc_object_t object;
		std::get<0>(object).request_vertex_normals();
		std::get<0>(object).request_face_normals();

        //const IfcGeomObjects::IfcGeomObject* geom_object = IfcGeomObjects::Get();

        // get type
		std::get<2>(object) = geom_object->type();
		for (std::string::iterator c = std::get<2>(object).begin(); c != std::get<2>(object).end(); ++c) {
			*c = tolower(*c);
		}
        //if (ignore_types.find(std::get<2>(object)) != ignore_types.end()) continue;
        std::get<1>(object) = geom_object->guid();

        // get polygon data
        //auto& ifc_mesh = geom_object->mesh();
        auto& fs = mesh.faces();
        auto& vs = mesh.verts();

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
    } while (context_iterator.next());

    return objects;
}

} // october

#endif // defined(WITH_IFCOPENSHELL) && defined(WITH_OPENMESH)
