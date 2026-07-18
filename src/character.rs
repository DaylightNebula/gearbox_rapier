use anarchy::macros::{Component, Getters, Setters};
use glam::Vec3;

use crate::PhysicsLength;

/// Standard character controller for a `PhysicsObject`, backed by rapier's
/// `KinematicCharacterController`. Must be applied to an entity with a
/// `PhysicsObject`; `update_engine_side` creates the underlying controller
/// the first time it sees the pair and moves it every tick using
/// `target_movement`.
///
/// Fields:
/// - `collider_expansion`: Extra offset added around the character's collider
///   (rapier's `offset`), or `None` to use rapier's default.
/// - `slope_config`: Optional min/max slope angles controlling when the
///   character slides or is allowed to climb; `None` uses rapier's defaults.
/// - `step_config`: Optional autostep configuration for climbing small
///   ledges; `None` disables autostepping.
/// - `snap_to_ground`: Optional max distance used to snap the character to
///   the ground when just barely leaving it; `None` disables snapping.
/// - `target_movement`: Desired movement vector for the next physics tick,
///   set via `set_target_movement`.
/// - `up`: The character's up direction, set via `set_up`.
/// - `is_sliding`: Whether the character is currently sliding down a slope,
///   as reported by `PhysicsEngine` after the last move.
/// - `is_grounded`: Whether the character is currently touching the ground,
///   as reported by `PhysicsEngine` after the last move.
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

/// Slope-handling configuration for a `PhysicsCharacter`, mapped to rapier's
/// `KinematicCharacterController::min_slope_slide_angle` and
/// `max_slope_climb_angle`.
///
/// Fields:
/// - `min_slope_angle_rad`: The minimum slope angle, in radians, at which the
///   character starts sliding.
/// - `max_slope_angle_rad`: The maximum slope angle, in radians, the
///   character is allowed to climb.
#[derive(Getters, Setters, Debug, Clone, Copy)]
pub struct PhysicsCharacterSlopeConfig {
    min_slope_angle_rad: f32,
    max_slope_angle_rad: f32
}

/// Autostep configuration for a `PhysicsCharacter`, mapped to rapier's
/// `CharacterAutostep`.
///
/// Fields:
/// - `max_height`: The maximum height of a step the character can climb.
/// - `min_width`: The minimum width a step must have to be climbed.
/// - `include_dynamic_bodies`: Whether dynamic rigidbodies count as steps, in
///   addition to fixed ones.
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
