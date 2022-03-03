use rapier3d::prelude::RigidBodySet;

extern crate rapier3d;
use rapier3d::prelude::vector;
use rapier3d::prelude::*;

/// A simple wrapper around rapier3d
///
/// Holds all the important information that relates to the physics engine.
pub struct Physics {
    gravity: [f32; 3],
    integration_parameters: IntegrationParameters,
    physics_pipline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    joint_set: JointSet,
    ccd_solver: CCDSolver,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
}

impl Physics {
    /// Creates the physics pipeline with a specific gravity
    pub fn new(gravity: [f32; 3]) -> Self {
        Self {
            gravity,
            integration_parameters: IntegrationParameters::default(),
            physics_pipline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            joint_set: JointSet::new(),
            ccd_solver: CCDSolver::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
        }
    }

    pub fn get_rigid_body_set(&self) -> &RigidBodySet {
        &self.rigid_body_set
    }

    pub fn get_collider_set(&self) -> &ColliderSet {
        &self.collider_set
    }

    pub fn get_rigid_body_set_mut(&mut self) -> &mut RigidBodySet {
        &mut self.rigid_body_set
    }

    pub fn get_collider_set_mut(&mut self) -> &mut ColliderSet {
        &mut self.collider_set
    }

    /// Get both the rigid_body_set, and the collider_set as mutable
    ///
    /// Borrow checker doesn't like borrowing the same variable twice, so this is a workaround.
    pub fn get_sets_mut(&mut self) -> (&mut RigidBodySet, &mut ColliderSet) {
        let Self {
            rigid_body_set,
            collider_set,
            ..
        } = self;

        (rigid_body_set, collider_set)
    }

    /// Get both the rigid_body_set, and the collider_set as mutable
    ///
    /// Identical to get_sets_mut except that you pass in a closure which will gain access to the
    /// RigidBodySet and ColliderSet. This makes it easier to drop the refrences to RigidBodySet
    /// and ColliderSet. Whatever is returned by the closure is returned by this function
    pub fn modify_sets<T>(
        &mut self,
        func: impl FnOnce(&mut RigidBodySet, &mut ColliderSet) -> T,
    ) -> T {
        func(&mut self.rigid_body_set, &mut self.collider_set)
    }

    /// Update the physics for the frame
    pub fn step(&mut self) {
        let Self {
            gravity,
            integration_parameters,
            physics_pipline,
            island_manager,
            broad_phase,
            narrow_phase,
            joint_set,
            ccd_solver,
            rigid_body_set,
            collider_set,
        } = self;

        physics_pipline.step(
            &vector![gravity[0], gravity[1], gravity[2]],
            integration_parameters,
            island_manager,
            broad_phase,
            narrow_phase,
            rigid_body_set,
            collider_set,
            joint_set,
            ccd_solver,
            &(),
            &(),
        );
    }
}
