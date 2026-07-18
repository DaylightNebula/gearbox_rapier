// use std::error::Error;

// use glam::{Quat, Vec3};

// use crate::Collider;

// pub trait IntoRapierCollider {
//     /// Convert this shape to a blaze_physics `Collider` paired with offset position and rotation information.
//     fn as_collider(&self) -> Result<(Vec3, Quat, Collider), Box<dyn Error>>;
// }

// impl IntoRapierCollider for Capsule {
//     fn as_collider(&self) -> Result<(Vec3, Quat, Collider), Box<dyn std::error::Error>> {
//         Ok((
//             self.offset, 
//             self.rotation, 
//             Collider::Capsule { radius: self.radius, half_height: self.height / 2.0 }
//         ))
//     }
// }

// impl IntoRapierCollider for Cone {
//     fn as_collider(&self) -> Result<(Vec3, Quat, Collider), Box<dyn std::error::Error>> {
//         Ok((
//             self.offset,
//             self.rotation,
//             Collider::Cone { radius: self.radius, half_height: self.height / 2.0 }
//         ))
//     }
// }

// impl IntoRapierCollider for Cube {
//     fn as_collider(&self) -> Result<(Vec3, Quat, Collider), Box<dyn std::error::Error>> {
//         Ok((
//             self.offset, 
//             self.rotation, 
//             Collider::Cuboid { 
//                 half_width: self.width / 2.0, 
//                 half_height: self.height / 2.0, 
//                 half_depth: self.depth / 2.0 
//             }
//         ))
//     }
// }

// impl IntoRapierCollider for Cylinder {
//     fn as_collider(&self) -> Result<(Vec3, Quat, Collider), Box<dyn std::error::Error>> {
//         Ok((
//             self.offset,
//             self.rotation,
//             Collider::Cylinder { radius: self.radius, half_height: self.height / 2.0 }
//         ))
//     }
// }

// impl IntoRapierCollider for Sphere {
//     fn as_collider(&self) -> Result<(Vec3, Quat, Collider), Box<dyn std::error::Error>> {
//         Ok((
//             self.offset,
//             self.rotation,
//             Collider::Sphere { radius: self.radius }
//         ))
//     }
// }
