pub mod object_holder;
pub mod physics;
pub mod physics_object;

use opengl_render::gui::DebugGUI;
use rapier3d::prelude::RigidBodySet;

extern crate rapier3d;
use rapier3d::prelude::*;

/// DebugGUI for Physics Objects
///
/// Since most modifications of a physics object require access to its rigid body, a new trait was
/// created that requires the RigidBodySet to be provided.
pub trait PhysicsDebug {
    fn debug(
        &mut self,
        ui: &mut egui::Ui,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
    );
}

/// Default implementation of DebugGUI
///
/// Might be removed later
impl PhysicsDebug for dyn DebugGUI {
    fn debug(
        &mut self,
        ui: &mut egui::Ui,
        _rigid_body_set: &mut RigidBodySet,
        _collider_set: &mut ColliderSet,
    ) {
        DebugGUI::debug(self, ui);
    }
}
