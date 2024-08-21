use rapier3d::geometry::{MeshConverter, MeshConverterError, SharedShape};
use rapier3d::math::{Isometry, Point, Real, Vector};
use std::fs::File;
use std::io::{BufReader, Read, Seek};
use std::path::Path;
use stl_io::IndexedMesh;

use std::str::FromStr;
// use dae_parser::*;
use dae_parser::{
    Document, Geom, PhysicsMaterial, PhysicsMaterialCommon, PhysicsModel, RigidBody, RigidBodyCommon, RigidConstraint, RigidConstraintCommon, RigidTransform, Semantic, Technique, TriangleGeom};


/// Error while loading an STL file.
#[derive(thiserror::Error, Debug)]
pub enum StlLoaderError {
    /// An error triggered by rapier’s [`MeshConverter`].
    #[error(transparent)]
    MeshConverter(#[from] MeshConverterError),
    /// A generic IO error.
    #[error(transparent)]
    Io(#[from] std::io::Error),
}

/// The result of loading a shape from an stl mesh.
pub struct StlShape {
    /// The shape loaded from the file and converted by the [`MeshConverter`].
    pub shape: SharedShape,
    /// The shape’s pose.
    pub pose: Isometry<Real>,
    /// The raw mesh read from the stl file without any modification.
    pub raw_mesh: IndexedMesh,
}


pub fn load_from_path(
    file_path: impl AsRef<Path>,
    converter: MeshConverter,
    scale: Vector<Real>,
)
// -> Result<StlShape, StlLoaderError>
{
    let mut reader = BufReader::new(File::open(file_path).unwrap());
    load_from_reader(&mut reader, converter, scale)
}

pub fn load_from_reader<R: Read + Seek>(
    read: &mut R,
    converter: MeshConverter,
    scale: Vector<Real>,
)
// -> Result<StlShape, StlLoaderError>
{
    let mut dae_file = String::new();
    read.read_to_string(&mut dae_file).expect("Cannot read file");

    let document = Document::from_str(&dae_file).unwrap();

    let sources_map = document.local_map::<dae_parser::Source>().unwrap();
    dbg!(&sources_map.0.keys());

    let vertices_map = document.local_map::<dae_parser::Vertices>().unwrap();
    dbg!(&vertices_map);

    let mesh_geometries = document.local_map::<dae_parser::Geometry>().unwrap();

    if let Some(mesh_id) = mesh_geometries.0.keys().next() {
        dbg!(&mesh_id);

        let mesh_geometry = document.local_map::<dae_parser::Geometry>().unwrap().get_str(mesh_id).unwrap();
        dbg!(&mesh_geometry.extra);

        let mesh_geometry_element = &mesh_geometry.element.as_mesh().unwrap();
        dbg!(&mesh_geometry_element.convex);


        let sources = &mesh_geometry_element.sources;
        for s in sources.iter() {
            dbg!(&s.id);
            let elements = s.array.as_ref().unwrap();
            dbg!(&elements.len());
        }

        let vertices = mesh_geometry_element.vertices.as_ref().unwrap();
        let vert_source = &vertices.inputs.first().unwrap().source;
        dbg!(&vert_source);
        dbg!(&vertices);
        for s in sources.iter() {
            let key = if vert_source.to_string().starts_with("#") {
                &vert_source.to_string()[1..]
            } else {
                &vert_source.to_string()
            };

            if s.id.eq(&Some(key.to_string())) {
                let verts_coords = s.array.as_ref().unwrap();
                dbg!("Num vertices: ", &verts_coords.len() / 3);
            }
        }

        // sources.get("Cube-mesh-positions").unwrap();
        // assert_eq!(mesh_geometry.id.as_ref().unwrap(), "Cube-mesh");
        let dae_mesh = mesh_geometry.element.as_mesh().unwrap();
        let tris: &Geom<TriangleGeom> = dae_mesh.elements[0].as_triangles().unwrap();
        // TODO these are the vertices to put into IndexedMesh
        let tris_vertices = tris.data.as_deref().unwrap();
        let tris_vertices: Vec<Vector<f32>> =
            tris_vertices.chunks(3)
                .filter(|chunk| chunk.len() == 3)
                .map(|chunk| {
                    let v: Vector<f32> = Vector::new(chunk[0] as f32, chunk[1] as f32, chunk[2] as f32);
                    v
                    })
                .collect();

        assert_eq!(tris.inputs[0].semantic, Semantic::Vertex);
        dbg!(&tris_vertices.len());



        let rigid_body_common = RigidBodyCommon::default();
        let rigid_body = RigidBody::default();
        



    }
    else {
        println!("The HashMap is empty.");
    }




    // Ok(load_from_raw_mesh(stl_mesh, converter, scale)?)
}



fn main() {
    let file_path = "../../data/house_1.dae";

    // Create a MeshConverter instance.
    let converter = MeshConverter::default();

    // Define the scale factor (1.0 means no scaling).
    let scale = Vector::new(1.0, 1.0, 1.0);

    let _ = load_from_path(file_path, converter, scale);


}
