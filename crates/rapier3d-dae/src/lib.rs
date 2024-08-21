//! ## STL loader for the Rapier physics engine
//!
//! Rapier is a set of 2D and 3D physics engines for games, animation, and robotics. The `rapier3d-stl`
//! crate lets you create a shape compatible with `rapier3d` and `parry3d` (the underlying collision-detection
//! library) from an STL file.

#![warn(missing_docs)]

use rapier3d::geometry::{MeshConverter, MeshConverterError, SharedShape};
use rapier3d::math::{Isometry, Point, Real, Vector};
use std::fs::File;
use std::io::{BufReader, Read, Seek};
use std::path::Path;
use stl_io::IndexedMesh;

use std::str::FromStr;
use dae_parser::*;


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

pub struct DaeShape {
    /// The shape loaded from the file and converted by the [`MeshConverter`].
    pub shape: SharedShape,
    /// The shape’s pose.
    pub pose: Isometry<Real>,
    // The raw mesh read from the stl file without any modification.
    pub raw_mesh: IndexedMesh,
}

/// Loads an STL file as a shape from a file.
///
/// # Parameters
/// - `file_path`: the STL file’s path.
/// - `converter`: controls how the shape is computed from the STL content. In particular, it lets
///                you specify if the computed [`StlShape::shape`] is a triangle mesh, its convex hull,
///                bounding box, etc.
/// - `scale`: the scaling factor applied to the geometry input to the `converter`. This scale will
///            affect at the geometric level the [`StlShape::shape`]. Note that raw mesh value stored
///            in [`StlShape::raw_mesh`] remains unscaled.
pub fn load_from_path(
    file_path: impl AsRef<Path>,
    converter: MeshConverter,
    scale: Vector<Real>,
) -> Result<StlShape, StlLoaderError> {
    let mut reader = BufReader::new(File::open(file_path)?);
    load_from_reader(&mut reader, converter, scale)
}

/// Loads an STL file as a shape from an arbitrary reader.
///
/// # Parameters
/// - `reader`: the reader.
/// - `converter`: controls how the shape is computed from the STL content. In particular, it lets
///                you specify if the computed [`StlShape::shape`] is a triangle mesh, its convex hull,
///                bounding box, etc.
/// - `scale`: the scaling factor applied to the geometry input to the `converter`. This scale will
///            affect at the geometric level the [`StlShape::shape`]. Note that raw mesh value stored
///            in [`StlShape::raw_mesh`] remains unscaled.
pub fn load_from_reader<R: Read + Seek>(
    read: &mut R,
    converter: MeshConverter,
    scale: Vector<Real>,
) -> Result<StlShape, StlLoaderError> {
    let stl_mesh = stl_io::read_stl(read)?;

    let mut dae_file = String::new();
    read.read_to_string(&mut dae_file).expect("Cannot read file");

    let document = Document::from_str(&dae_file).unwrap();

    // let ass = document.get_visual_scene().unwrap();

    // TODO pass mesh names to function
    let mesh_geometry = document.local_map::<dae_parser::Geometry>().unwrap().get_str("Cube-mesh").unwrap();

    // TODO enlist all meshes and extract
    // let mesh_geometries = document.local_map::<dae_parser::Geometry>().unwrap();
    // let mesh_geometries = mesh_geometries.0.keys();
    // for mesh_key in mesh_geometries {
    //     let mesh_geometry = document.local_map::<dae_parser::Geometry>().unwrap().get_str(mesh_key).unwrap();
    // }




    let sources_map = document.local_map::<dae_parser::Source>().unwrap();
    let vertices_map = document.local_map::<dae_parser::Vertices>().unwrap();
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
                // <[u32; 3]>::from(chunk).unwrap()
                let v: Vector<f32> = Vector::new(chunk[0] as f32, chunk[1] as f32, chunk[2] as f32);
                v
                })
            .collect();


    // let res = IndexedMesh {
    //     vertices: tris_vertices,
    //     faces: vec![]
    // };


    assert_eq!(tris.inputs[0].semantic, Semantic::Vertex);
    let vertices = vertices_map.get_raw(&tris.inputs[0].source).unwrap();
    let source = sources_map
                                    .get_raw(&vertices.position_input().source)
                                    .unwrap();

    // TODO return an IndexedMesh here


    Ok(load_from_raw_mesh(stl_mesh, converter, scale)?)
}

/// Loads an STL file as a shape from a preloaded raw stl mesh.
///
/// # Parameters
/// - `raw_mesh`: the raw stl mesh.
/// - `converter`: controls how the shape is computed from the STL content. In particular, it lets
///                you specify if the computed [`StlShape::shape`] is a triangle mesh, its convex hull,
///                bounding box, etc.
/// - `scale`: the scaling factor applied to the geometry input to the `converter`. This scale will
///            affect at the geometric level the [`StlShape::shape`]. Note that raw mesh value stored
///            in [`StlShape::raw_mesh`] remains unscaled.
pub fn load_from_raw_mesh(
    raw_mesh: IndexedMesh,
    converter: MeshConverter,
    scale: Vector<Real>,
) -> Result<StlShape, MeshConverterError> {
    let mut vertices: Vec<_> = raw_mesh
        .vertices
        .iter()
        .map(|xyz| Point::new(xyz[0], xyz[1], xyz[2]))
        .collect();
    vertices
        .iter_mut()
        .for_each(|pt| pt.coords.component_mul_assign(&scale));
    let indices: Vec<_> = raw_mesh
        .faces
        .iter()
        .map(|f| f.vertices.map(|i| i as u32))
        .collect();
    let (shape, pose) = converter.convert(vertices, indices)?;

    Ok(StlShape {
        shape,
        pose,
        raw_mesh,
    })
}
