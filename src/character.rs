use anarchy::macros::{Component, Getters, Setters};
use glam::Vec3;

use crate::PhysicsLength;

/// Standard character controller for a `PhysicsObject`.
/// Must be applied to an entity with a `PhysicsObject`.
#[derive(Component, Getters, Debug, Clone, Copy)]
pub struct PhysicsCharacter {
    collider_expansion: Option<PhysicsLength>,
    slope_config: Option<PhysicsCharacterSlopeConfig>,
    step_config: Option<PhysicsCharacterStepConfig>,
    snap_to_ground: Option<PhysicsLength>,
    target_movement: Vec3,
    up: Vec3,
    is_sliding: bool,
    is_grounded: bool
}

#[derive(Getters, Setters, Debug, Clone, Copy)]
pub struct PhysicsCharacterSlopeConfig {
    min_slope_angle_rad: f32,
    max_slope_angle_rad: f32
}

#[derive(Getters, Setters, Debug, Clone, Copy)]
pub struct PhysicsCharacterStepConfig {
    max_height: PhysicsLength,
    min_width: PhysicsLength,
    include_dynamic_bodies: bool
}

impl PhysicsCharacter {
    /// Creates a new `PhysicsCharacter` this produces
    /// the same result as `default`.
    pub fn new() -> Self {
        Self { 
            collider_expansion: None, 
            slope_config: None, 
            step_config: None, 
            snap_to_ground: None, 
            target_movement: Vec3::ZERO, 
            up: Vec3::Y,
            is_sliding: false,
            is_grounded: false
        }
    }

    /// Sets the target movement vector of this character.
    /// This will begin to apply at the next physics tick, 
    /// and the first physics tick after being overriden.
    pub fn set_target_movement(&mut self, movement: Vec3) -> &mut Self { 
        self.target_movement = movement; self
    }

    /// Sets the up vector of this character.
    /// This will begin to apply at the next physics tick, 
    /// and the first physics tick after being overriden.
    pub fn set_up(&mut self, up: Vec3) -> &mut Self { self.up = up; self }

    /// Used to set if the character is grounded by `PhysicsEngine`.
    pub(crate) fn set_is_grounded(&mut self, is_grounded: bool) { self.is_grounded = is_grounded }

    /// Used to set if the character is sliding by `PhysicsEngine`.
    pub(crate) fn set_is_sliding(&mut self, is_sliding: bool) { self.is_sliding = is_sliding }
}

impl Default for PhysicsCharacter {
    fn default() -> Self {
        Self::new()
    }
}
