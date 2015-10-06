#ifndef _OCTOBER_IFC_MESH_TYPES_HPP_
#define _OCTOBER_IFC_MESH_TYPES_HPP_
#ifdef WITH_OPENMESH

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>

namespace october {

struct openmesh_default_traits : public OpenMesh::DefaultTraits {
	typedef OpenMesh::Vec4f  Color;

	VertexAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color );
	FaceAttributes( OpenMesh::Attributes::Normal );
};

typedef ::OpenMesh::TriMesh_ArrayKernelT<openmesh_default_traits> openmesh_t;

// (mesh, guid, type)
typedef std::tuple<openmesh_t, std::string, std::string> ifc_object_t;

typedef std::vector<ifc_object_t> ifc_objects_t;

} // october

#endif // WITH_OPENMESH
#endif /* _OCTOBER_IFC_MESH_TYPES_HPP_ */
