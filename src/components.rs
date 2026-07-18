use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use anarchy::macros::{Component, Getters};
use glam::Vec3;
use mutual::{CowData, SharedList, SharedMap};
use rapier3d::control::CharacterLength;

use crate::{Collider, NEXT_PHYSICS_WORLD_ID, PhysicsWorldID, RigidBodySnapshot};

#[derive(Component, Getters)]
pub struct PhysicsObject {
    physics_id: PhysicsWorldID,
    last_snapshot: CowData<Option<RigidBodySnapshot>>,
    collider: CowData<Collider>,
    next_velocity: CowData<Vec3>,
    impulses: SharedList<Vec3>,
    torque_impulses: SharedList<Vec3>,
    forces: SharedMap<u32, Vec3>,
    torques: SharedMap<u32, Vec3>,
    force_idx: AtomicU32,
    refresh_rapier: AtomicBool,
    using_empty_collider: AtomicBool,
    mass: CowData<f32>,
    is_fixed: CowData<bool>
}

impl PhysicsObject {
    /// Creates a new fixed `PhysicsObject`.
    pub fn fixed(collider: Collider, mass: f32) -> Self {
        Self {
            physics_id: NEXT_PHYSICS_WORLD_ID.fetch_add(1, std::sync::atomic::Ordering::SeqCst), 
            collider: CowData::new(collider), 
            last_snapshot: CowData::new(None),
            next_velocity: CowData::null(),
            impulses: SharedList::default(),
            torque_impulses: SharedList::default(),
            forces: SharedMap::default(),
            torques: SharedMap::default(),
            force_idx: AtomicU32::new(0),
            refresh_rapier: AtomicBool::new(false),
            using_empty_collider: AtomicBool::new(false),
            mass: CowData::new(mass),
            is_fixed: CowData::new(true)
        }
    }

    /// Creates a new moveable `PhysicsObject`.
    pub fn movable(collider: Collider, mass: f32) -> Self {
        Self { 
            physics_id: NEXT_PHYSICS_WORLD_ID.fetch_add(1, std::sync::atomic::Ordering::SeqCst), 
            collider: CowData::new(collider), 
            last_snapshot: CowData::new(None),
            next_velocity: CowData::null(),
            impulses: SharedList::default(),
            torque_impulses: SharedList::default(),
            forces: SharedMap::default(),
            torques: SharedMap::default(),
            force_idx: AtomicU32::new(0),
            refresh_rapier: AtomicBool::new(false),
            using_empty_collider: AtomicBool::new(false),
            mass: CowData::new(mass),
            is_fixed: CowData::new(false)
        }
    }

    /// Call for the refreshing of rapier colliders and rigidbodies
    pub fn refresh_rapier_counterpart(&self) {
        self.refresh_rapier.store(true, Ordering::Release);
    }

    /// Sets the collider of this object
    pub fn set_collider(&self, collider: Collider) {
        self.collider.set(collider);
        self.refresh_rapier_counterpart();
    }

    /// Sets the mass of the rapier physics counterpart
    pub fn set_mass(&self, mass: f32) {
        self.mass.set(mass);
        self.refresh_rapier_counterpart();
    }

    pub fn set_fixed(&self, fixed: bool) {
        self.is_fixed.set(fixed);
        self.refresh_rapier_counterpart();
    }

    /// Sets velocity of this object to be applied at the next physics tick.
    pub fn set_velocity(&self, velocity: Vec3) {
        self.next_velocity.set(velocity);
    }

    /// Clears the velocity meant to be set at the next physics tick.
    pub fn clear_velocity(&self) {
        self.next_velocity.set_null();
    }

    /// Adds an impulse force to this object.
    /// This will be applied during the next physics tick.
    /// The provided force is in global space, not local space.
    pub fn add_impulse(&self, force: Vec3) {
        self.impulses.push(force);
    }

    /// Adds a torque impulse force to this object.
    /// This will be applied during the next physics tick.
    /// The provided force is in global space, not local space.
    pub fn add_torque_impulse(&self, torque: Vec3) {
        self.torque_impulses.push(torque);
    }

    /// Adds a force to this object.
    /// This will be applied during the next physics tick.
    /// The provided force is in global space, not local space.
    /// The ID returned is the ID assigned to this force as the force is persistent.
    pub fn add_force(&self, force: Vec3) -> u32 {
        let id = self.force_idx.fetch_add(1, Ordering::SeqCst);
        self.forces.insert(id, force);
        id
    }

    /// Sets the force of an ID on this object.  Replacing previous forces of the same ID.
    /// This will be applied during the next physics tick.
    /// The provided force is in global space, not local space.
    /// Returns true if a previous force existed.
    pub fn set_force(&self, id: u32, force: Vec3) -> bool {
        self.forces.insert(id, force)
    }

    /// Removes a force from this object.
    /// This will be applied during the next physics tick.
    /// Returns the removed force, if any.
    pub fn remove_force(&self, id: u32) -> Option<Vec3> {
        self.forces.remove(&id).map(|a| a.clone())
    }

    /// Adds a torque to this object.
    /// This will be applied during the next physics tick.
    /// The provided torque is in global space, not local space.
    /// The ID returned is the ID assigned to this torque as the torque is persistent.
    pub fn add_torque(&self, torque: Vec3) -> u32 {
        let id = self.force_idx.fetch_add(1, Ordering::SeqCst);
        self.torques.insert(id, torque);
        id
    }

    /// Sets the torque of an ID on this object.  Replacing previous forces of the same ID.
    /// This will be applied during the next physics tick.
    /// The provided torque is in global space, not local space.
    /// Returns true if a previous torque existed.
    pub fn set_torque(&self, id: u32, torque: Vec3) -> bool {
        self.torques.insert(id, torque)
    }

    /// Removes a torque from this object.
    /// This will be applied during the next physics tick.
    /// Returns the removed torque, if any.
    pub fn remove_torque(&self, id: u32) -> Option<Vec3> {
        self.torques.remove(&id).map(|a| a.clone())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub enum PhysicsLength {
    Absolute(f32),
    Relative(f32)
}

impl PhysicsLength {
    pub fn to_length(&self) -> CharacterLength {
        match self {
            PhysicsLength::Absolute(length) => 
                CharacterLength::Absolute(*length),
            PhysicsLength::Relative(length) => 
                CharacterLength::Relative(*length)
        }
    }
}
