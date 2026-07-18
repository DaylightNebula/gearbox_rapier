use std::sync::{LazyLock, atomic::Ordering};

use anarchy::{DeltaTime, Query, Res, ResMut, macros::{system, warn}};
use cell::{App, Plugin};
use chrono::{DateTime, Utc};
use gearbox::Transform;
use glam::EulerRot;
use mutual::{CowData, SharedData};
use rapier3d::{control::{CharacterAutostep, KinematicCharacterController}, na::{Unit, Vector3}, prelude::{QueryFilter, RigidBodyBuilder}};

pub mod character;
pub mod colliders;
pub mod components;
pub mod engine;
pub mod queries;
pub mod shapes;

pub use character::*;
pub use colliders::*;
pub use components::*;
pub use engine::*;
pub use queries::*;
pub use rapier3d as rapier3d;

static TIME_TRACKER: LazyLock<CowData<DateTime<Utc>>> = std::sync::LazyLock::new(|| CowData::new(Utc::now()));

pub struct PhysicsPlugin;
impl Plugin for PhysicsPlugin {
    fn build(self, app: App) -> App {
        // let id = ScheduleID { id: "PHYSICS", tick_rate: 60, max_threads: 1 };
        // let mut schedule = Schedule::new_empty();
        // schedule.add_new(ScheduleTile::new(vec![Box::new(update_engine_side)]));
        // app.add_schedule(id, schedule);

        app.world().insert_resource(PhysicsEngine::new());
        app.world().insert_resource(PhysicsQueries::default());

        TIME_TRACKER.get_ref();

        app.add_resource(PhysicsEngine::new())
            .add_resource(PhysicsQueries::default())
            .on_update(update_engine_side)
    }
}

#[system]
fn update_engine_side(
    objects: Query<(&mut Transform, &PhysicsObject, Option<&mut PhysicsCharacter>)>,
    engine: ResMut<PhysicsEngine>,
    queries: Res<PhysicsQueries>,
    delta_time: Res<DeltaTime>
) {
    let delta_time = *delta_time.get_schedule_local(schedule_id).lock_ref();
    let mut bodies = engine.rigid_body_map.clone();

    for (entity_id, _mask, (mut transform, object, character)) in objects.iter_with_id() {
        let force_refresh = object.refresh_rapier().load(Ordering::Acquire);

        if let Some(mut character) = character {
            // create controller if needed
            if !engine.has_character(*object.physics_id()) || force_refresh {
                let up = Unit::new_normalize(Vector3::new(character.up().x, character.up().y, character.up().z));
                object.refresh_rapier().store(false, Ordering::Release);

                // create controller
                let mut kinematic = KinematicCharacterController::default();
                character.slope_config().map(|slope_config| {
                    kinematic.min_slope_slide_angle = *slope_config.min_slope_angle_rad();
                    kinematic.max_slope_climb_angle = *slope_config.max_slope_angle_rad();
                });
                kinematic.autostep = character.step_config().map(|config| CharacterAutostep { 
                    max_height: config.max_height().to_length(), 
                    min_width: config.min_width().to_length(), 
                    include_dynamic_bodies: *config.include_dynamic_bodies()
                });
                character.collider_expansion().map(|length| kinematic.offset = length.to_length());
                kinematic.snap_to_ground = character.snap_to_ground().map(|length| length.to_length());
                kinematic.up = up;

                // create shape and save
                let Ok(shape) = object.collider().get_ref().into_rapier_shared_shape() else { return Ok(()) };
                let Some(shape) = shape else { object.using_empty_collider().store(true, Ordering::Release); return Ok(()) };
                let shape = shape.0.clone_dyn();
                if force_refresh { engine.remove(*object.physics_id()); }
                engine.insert_character(*object.physics_id(), kinematic, shape);

                // stop here as nothing has happened yet
                continue;
            }

            // update character
            let (_scale, rotation, position) = transform.as_matix().to_scale_rotation_translation();
            let result = engine.move_character(
                *object.physics_id(),
                position, rotation,
                character.target_movement() * delta_time,
                *object.mass().get_ref(),
                delta_time
            );
            if let Some(result) = result {
                transform.translation += result.translation;
                character.set_is_grounded(result.is_grounded);
                character.set_is_sliding(result.is_sliding);
            }
        } else {
            let handle = bodies.remove_by_left(object.physics_id());
            let is_collider_empty = object.using_empty_collider().load(Ordering::Acquire);

            // create or refresh collider in engine
            if (handle.is_none() && !is_collider_empty) || force_refresh {
                object.refresh_rapier().store(false, Ordering::Release);

                // compute position and rotation
                let (_scale, rotation, position) = transform.as_matix().to_scale_rotation_translation();
                let position = Vector3::new(position.x, position.y, position.z);
                let rotation = rotation.to_euler(EulerRot::XYZ);
                let rotation = Vector3::new(rotation.0, rotation.1, rotation.2);

                // create rigidbody
                let rigidbody = if *object.is_fixed().get_ref() {
                    RigidBodyBuilder::fixed()
                        .translation(position)
                        .rotation(rotation)
                        .can_sleep(true)
                        .build()
                } else {
                    RigidBodyBuilder::dynamic()
                        .translation(position)
                        .rotation(rotation)
                        .can_sleep(true)
                        .build()
                };

                // create collider
                let collider = object.collider().get_ref().into_rapier_collider();
                let Ok(collider) = collider else {
                    warn!("Failed to convert blaze collider to rapier collider! {:?}", collider.err().unwrap());
                    continue;
                };
                let Some(collider) = collider else { object.using_empty_collider().store(true, Ordering::Release); continue };
                let collider = collider.mass(*object.mass().get_ref()).build();

                if force_refresh { engine.remove(*object.physics_id()); }
                engine.insert(*object.physics_id(), entity_id, collider, rigidbody);

                continue; // no updates to take so stop here
            }

            // get handle, skip if still cannot be found (may not be loaded yet)
            if handle.is_none() { continue }
            let (_, handle) = handle.unwrap();

            if let Some(snapshot) = engine.snapshot_by_rb_handle(handle) {
                // if snapshot.velocity.length_squared() < 0.001 { return }
                let movement = transform.translation().distance(snapshot.translation);
                let diff = (movement - (snapshot.velocity.length() * delta_time)).abs();
                if diff < 0.5 {
                    transform.set_translation(snapshot.translation);
                    transform.set_rotation(snapshot.rotation);
                } else {
                    engine.set_rb_position_rotation(handle, *transform.translation(), *transform.rotation());
                }

                object.last_snapshot().set(Some(snapshot));
            }

            // apply velocity
            let velocity = object.next_velocity().swap(&CowData::null());
            if let Some(velocity) = *velocity {
                engine.apply_velocity(handle, velocity);
            }

            // apply forces and torques
            engine.apply_impulses_torques_forces(
                handle,
                object.impulses().drain().map(|a| a.clone()).collect(),
                object.torque_impulses().drain().map(|a| a.clone()).collect(),
                object.forces().iter().map(|a| a.clone().2).collect(),
                object.torques().iter().map(|a| a.clone().2).collect()
            );
        }
    }

    // anything left in the bodies list, assume has been removed from outside
    bodies.iter().for_each(|(id, _)| engine.remove(*id));

    // update queries
    let query_pipeline = engine.create_query_pipeline(QueryFilter::new());
    queries.execute_queries(&engine, &query_pipeline);

    // step physics engine
    engine.tick();
}

/*
    let delta_time = *delta_time.get_schedule_local(schedule_id).lock_ref();
    // println!("Physics theo fps {}", 1.0 / delta_time);
    let mut bodies = engine.rigid_body_map.clone();

    // update all objects
    objects.iter_with_id().for_each(|(entity_id, _mask, (transform, object, character))| {
        let handle = bodies.remove_by_left(object.physics_id());
        let force_refresh = object.refresh_rapier().load(Ordering::Acquire);

        if (handle.is_none() && !object.using_empty_collider().load(Ordering::Acquire)) || force_refresh {
            if force_refresh { object.refresh_rapier().store(false, Ordering::Release); }

            if let Some(character) = character {
                // create controller if needed
                if !engine.has_character(*object.physics_id()) {
                    let up = Unit::new_normalize(Vector3::new(character.up().x, character.up().y, character.up().z));

                    // create controller
                    let mut kinematic = KinematicCharacterController::default();
                    character.slope_config().map(|slope_config| {
                        kinematic.min_slope_slide_angle = *slope_config.min_slope_angle_rad();
                        kinematic.max_slope_climb_angle = *slope_config.max_slope_angle_rad();
                    });
                    kinematic.autostep = character.step_config().map(|config| CharacterAutostep { 
                        max_height: config.max_height().to_length(), 
                        min_width: config.min_width().to_length(), 
                        include_dynamic_bodies: *config.include_dynamic_bodies()
                    });
                    character.collider_expansion().map(|length| kinematic.offset = length.to_length());
                    kinematic.snap_to_ground = character.snap_to_ground().map(|length| length.to_length());
                    kinematic.up = up;

                    // create shape and save
                    let Ok(shape) = object.collider().get_ref().into_rapier_shared_shape() else { return };
                    let Some(shape) = shape else { object.using_empty_collider().store(true, Ordering::Release); return };
                    let shape = shape.0.clone_dyn();
                    if force_refresh { engine.remove(*object.physics_id()); }
                    engine.insert_character(*object.physics_id(), kinematic, shape);
                }
            } else {
                // compute position and rotation
                let (_scale, rotation, position) = transform.to_matrix().to_scale_rotation_translation();
                let position = Vector3::new(position.x, position.y, position.z);
                let rotation = rotation.to_euler(EulerRot::XYZ);
                let rotation = Vector3::new(rotation.0, rotation.1, rotation.2);

                // create rigidbody
                let rigidbody = if *object.is_fixed().get_ref() {
                    RigidBodyBuilder::fixed()
                        .translation(position)
                        .rotation(rotation)
                        .can_sleep(true)
                        .build()
                } else {
                    RigidBodyBuilder::dynamic()
                        .translation(position)
                        .rotation(rotation)
                        .can_sleep(true)
                        .build()
                };

                // create collider
                let collider = object.collider().get_ref().into_rapier_collider();
                let Ok(collider) = collider else {
                    warn!("Failed to convert blaze collider to rapier collider! {:?}", collider.err().unwrap());
                    return;
                };
                let Some(collider) = collider else { object.using_empty_collider().store(true, Ordering::Release); return };
                let collider = collider.mass(*object.mass().get_ref()).build();

                if force_refresh { engine.remove(*object.physics_id()); }
                engine.insert(*object.physics_id(), entity_id, collider, rigidbody);
            };

            return;
        }

        if handle.is_none() { return }
        let (_, handle) = handle.unwrap();

        if let Some(snapshot) = engine.snapshot_by_rb_handle(handle) {
            // if snapshot.velocity.length_squared() < 0.001 { return }
            let movement = transform.translation().distance(snapshot.translation);
            let diff = (movement - (snapshot.velocity.length() * delta_time)).abs();
            if diff < 0.5 {
                transform.set_translation(snapshot.translation);
                transform.set_rotation(snapshot.rotation);
            } else {
                engine.set_rb_position_rotation(handle, transform.translation(), transform.rotation());
            }

            object.last_snapshot().set(Some(snapshot));
        }

        // update character
        if let Some(character) = character {
            let (_scale, rotation, position) = transform.to_matrix().to_scale_rotation_translation();

            info!("Character move {} {}", position, character.target_movement());

            engine.move_character(
                *object.physics_id(),
                position, rotation,
                character.target_movement() * delta_time,
                *object.mass().get_ref(),
                delta_time
            );
        }

        // apply velocity
        let velocity = object.next_velocity().swap(&CowData::null());
        if let Some(velocity) = *velocity {
            engine.apply_velocity(handle, velocity);
        }

        // apply forces and torques
        engine.apply_impulses_torques_forces(
            handle,
            object.impulses().drain().map(|a| a.clone()).collect(),
            object.torque_impulses().drain().map(|a| a.clone()).collect(),
            object.forces().iter().map(|a| a.clone().2).collect(),
            object.torques().iter().map(|a| a.clone().2).collect()
        );
    });

    // anything left in the bodies list, assume has been removed from outside
    bodies.iter().for_each(|(id, _)| {
        engine.remove(*id);
    });

    // update queries
    let query_pipeline = engine.create_query_pipeline(QueryFilter::new());
    queries.execute_queries(&engine, &query_pipeline);

    // step physics engine
    engine.tick();
*/
