use opengl_render::renderer::RenderScene;
use rapier3d::prelude::{ColliderSet, IslandManager, JointSet};
use std::ops::{Index, IndexMut};

use crate::physics::Physics;
use crate::physics_object::PhysicsObject;
use crate::RigidBodySet;

/// Holds physics objects
///
/// The main purpose of this struct is to ensure that objects are removed from the physics engine
/// when you remove a physics object.
pub struct ObjectHolder {
    physics_objects: Vec<PhysicsObject>,
}

impl ObjectHolder {
    pub fn new() -> Self {
        Self {
            physics_objects: Vec::new(),
        }
    }

    /// Adds a physics object to the holder
    pub fn push(&mut self, object: PhysicsObject) {
        self.physics_objects.push(object);
    }

    /// Completely deletes object
    ///
    /// The object at index will be removed from rendering, and from the physics engine
    ///
    /// Panics if out of bounds
    pub fn remove(
        &mut self,
        index: usize,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
        island_manager: &mut IslandManager,
        joint_set: &mut JointSet,
    ) {
        let object = self.physics_objects.remove(index);

        let mut colliders = Vec::new();
        if let Some(rigid_body) = object.get_rigid_body_mut(rigid_body_set) {
            for collider in rigid_body.colliders() {
                colliders.push(*collider);
            }
        }

        for collider in colliders {
            collider_set.remove(collider, island_manager, rigid_body_set, true);
        }

        rigid_body_set.remove(
            object.get_rigid_body_handle(),
            island_manager,
            collider_set,
            joint_set,
        );
    }

    /// Shortcut for [`remove`](Self::remove) where it will automatically get the sets.
    pub fn remove_physics(&mut self, index: usize, physics: &mut Physics) {
        physics.modify_sets(|rigid_body_set, collider_set, island_set, joint_set| {
            self.remove(index, rigid_body_set, collider_set, island_set, joint_set);
        });
    }

    /// Render all the objects associated with this holder
    pub fn render<'a>(&'a mut self, scene: &mut RenderScene<'a>, physics: &mut Physics) {
        physics.modify_sets(|rigid_body_set, _collider_set, _island_set, _joint_set| {
            for object in &mut self.physics_objects {
                object.render(scene, rigid_body_set);
            }
        });
    }

    pub fn get_object(&self, index: usize) -> Option<&PhysicsObject> {
        self.physics_objects.get(index)
    }

    pub fn get_object_mut(&mut self, index: usize) -> Option<&mut PhysicsObject> {
        self.physics_objects.get_mut(index)
    }

    pub fn get_objects(&self) -> &Vec<PhysicsObject> {
        &self.physics_objects
    }

    pub fn get_num_objects(&self) -> usize {
        self.physics_objects.len()
    }
}

impl Index<usize> for ObjectHolder {
    type Output = PhysicsObject;
    fn index(&self, index: usize) -> &Self::Output {
        &self.physics_objects[index]
    }
}

impl IndexMut<usize> for ObjectHolder {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.physics_objects[index]
    }
}
