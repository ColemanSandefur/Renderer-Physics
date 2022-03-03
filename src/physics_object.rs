use crate::nalgebra::Point3;
use crate::PhysicsDebug;
use opengl_render::cgmath::Rad;
use opengl_render::gui::DebugGUI;
use opengl_render::gui::DebugGUIFormat;
use opengl_render::pbr_model::PbrModel;
use opengl_render::renderer::RenderScene;
use rapier3d::na::Matrix3;
use rapier3d::na::Rotation3;
use rapier3d::na::Unit;
use rapier3d::prelude::{RigidBodyBuilder, RigidBodyHandle, RigidBodySet};

extern crate rapier3d;
use rapier3d::prelude::vector;
use rapier3d::prelude::*;
pub struct PhysicsObject {
    shape: PbrModel,
    rigid_body_handle: RigidBodyHandle,
}

impl PhysicsObject {
    /// Render and update physics of object
    ///
    /// It will render the [`PbrModel`] and update the position from the physics engine. This is
    /// the recommended way to render the object.
    ///
    /// Alternatively:
    /// To render without loading new position from physics engine you can use [`render_no_update`]
    /// To update without rendering the object you can use [`update_physics`]
    pub fn render<'a>(
        &'a mut self,
        scene: &mut RenderScene<'a>,
        rigid_body_set: &mut RigidBodySet,
    ) {
        self.update_physics(rigid_body_set);
        self.render_no_update(scene);
    }

    /// Just render the object
    pub fn render_no_update<'a>(&'a mut self, scene: &mut RenderScene<'a>) {
        self.shape.render(scene);
    }

    /// Update position from physics engine
    pub fn update_physics(&mut self, rigid_body_set: &mut RigidBodySet) {
        if let Some(rigid_body) = rigid_body_set.get(self.rigid_body_handle) {
            let trans = rigid_body.translation();
            self.shape.set_translation([trans.x, trans.y, trans.z]);

            //let (axis, angle): ([f32; 3], Rad<f32>) = match rigid_body.rotation().axis_angle() {
            //Some((axis, angle)) => (axis.into_inner().into(), Rad(angle)),
            //None => ([0.0; 3], Rad(rigid_body.rotation().angle())),
            //};

            //self.shape.set_rotation_axis_angle(axis, angle);
            let (roll, pitch, yaw) = rigid_body.rotation().euler_angles();
            self.shape
                .set_rotation_euler(Rad(yaw), Rad(pitch), Rad(roll));
        } else {
            println!("physics body not found");
        }
    }

    pub fn get_model(&self) -> &PbrModel {
        &self.shape
    }

    /// Subscribe to the physics engine
    ///
    /// Subscribes the model to the physics engine. It creates a collider based on the model.
    fn create(
        model: &PbrModel,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
    ) -> RigidBodyHandle {
        let model_pos = model.get_position();
        let model_rot = model.get_euler_angles();
        let na_rotation =
            rapier3d::na::Unit::from_euler_angles(model_rot.x, model_rot.y, model_rot.z);
        let mut rigid_body = RigidBodyBuilder::new_dynamic()
            .translation(vector![model_pos.x, model_pos.y, model_pos.z])
            .rotation(na_rotation.scaled_axis())
            .build();

        // convert mat3 to rotation 3
        let rot_mat: [[f32; 3]; 3] = model.get_rotation_mat3().into();
        let rot_mat: Matrix3<f32> = rot_mat.into();
        let rotation3 = Rotation3::from_matrix(&rot_mat);

        let (rot_x, rot_y, rot_z) = rotation3.euler_angles();
        rigid_body.set_rotation(vector![rot_x, rot_y, rot_z], true);
        let rigid_body_handle = rigid_body_set.insert(rigid_body);

        let mut vertices = Vec::new();
        let mut indices = Vec::new();

        for segment in model.get_segments() {
            let vertex = segment.get_vertex_buffer().read().unwrap();
            let index = segment.get_index_buffer().read().unwrap();

            let mut tmp_vertices: Vec<_> = vertex
                .into_iter()
                .map(|vertex| Point3::from_slice(&vertex.position))
                .collect();

            let mut tmp_indices: Vec<_> = index
                .chunks(3)
                //.map(|chunk| [chunk[0], chunk[1], chunk[2]])
                .map(|chunk| [chunk[2], chunk[1], chunk[0]])
                .collect();

            concat_buffers(
                (&mut vertices, &mut indices),
                (&mut tmp_vertices, &mut tmp_indices),
            );
        }

        // Create a rough outline of the model
        let collider = ColliderBuilder::convex_decomposition(&vertices, &indices)
            .restitution(0.0)
            .friction(1.0)
            //.density(1000.0)
            .build();

        collider_set.insert_with_parent(collider, rigid_body_handle, rigid_body_set);

        let rigid_body = rigid_body_set.get_mut(rigid_body_handle).unwrap();
        println!(
            "{} colliders registered to handle",
            rigid_body.colliders().len()
        );
        rigid_body.set_translation(vector![model_pos.x, model_pos.y, model_pos.z], true);
        rigid_body.set_rotation(na_rotation.scaled_axis(), true);

        rigid_body_handle
    }

    /// Create a PhysicsObject
    ///
    /// Adds physics to a PbrModel.
    pub fn new(
        model: PbrModel,
        rigid_body_set: &mut RigidBodySet,
        collider_set: &mut ColliderSet,
    ) -> Self {
        Self {
            rigid_body_handle: Self::create(&model, rigid_body_set, collider_set),
            shape: model,
        }
    }

    pub fn get_rigid_body<'a>(
        &'a self,
        rigid_body_set: &'a mut RigidBodySet,
    ) -> Option<&'a mut RigidBody> {
        rigid_body_set.get_mut(self.rigid_body_handle)
    }

    pub fn modify_rigid_body<T>(
        &self,
        rigid_body_set: &mut RigidBodySet,
        func: impl FnOnce(Option<&mut RigidBody>) -> T,
    ) -> T {
        func(self.get_rigid_body(rigid_body_set))
    }
}

impl PhysicsDebug for PhysicsObject {
    fn debug(&mut self, ui: &mut egui::Ui, rigid_body_set: &mut RigidBodySet) {
        ui.collapsing("Material Properties", |ui| {
            let material_segments = self.shape.get_segments_mut();

            for i in 0..material_segments.len() {
                let segment = &mut material_segments[i];

                egui::CollapsingHeader::new(format!("Segment {}", i)).show(ui, |ui| {
                    segment.debug(ui);
                });
            }
        });

        if let Some(rigid_body) = rigid_body_set.get_mut(self.rigid_body_handle) {
            egui::CollapsingHeader::new("Physics").show(ui, |ui| {
                let mut rotation_lock = rigid_body.is_rotation_locked();
                let mut translation_lock = rigid_body.is_translation_locked();
                let mut position: [f32; 3] = rigid_body.translation().xyz().into();

                // Roll, pitch, yaw
                let mut rotation: [f32; 3] = {
                    let (roll, pitch, yaw) = rigid_body.rotation().euler_angles();

                    [roll, pitch, yaw]
                };

                ui.group(|ui| {
                    // Lock and unlock rigid_body translations
                    if ui
                        .checkbox(&mut translation_lock, "Lock Translation")
                        .changed()
                    {
                        rigid_body.lock_translations(translation_lock, true);
                    };

                    // add_invisible_ui in egui 0.17
                    // To avoid undefined behavior, translation lock must be enabled
                    // once it is enabled the user will be able to interact with the ui
                    ui.add_enabled_ui(translation_lock, |ui| {
                        if DebugGUIFormat::position(ui, &mut position, -25.0..=25.0) {
                            rigid_body.set_translation(position.into(), true);
                        }
                    });
                });

                // Add sliders to edit/show angles
                if DebugGUIFormat::euler(ui, &mut rotation) {
                    rigid_body.lock_translations(true, true);
                    rigid_body.set_rotation(
                        Unit::from_euler_angles(rotation[0], rotation[1], rotation[2])
                            .scaled_axis(),
                        true,
                    );
                    rigid_body.lock_translations(translation_lock, true);
                }

                egui::CollapsingHeader::new("Rotation Lock").show(ui, |ui| {
                    let mut ui_changed = false;
                    if ui.checkbox(&mut rotation_lock[0], "x").changed() {
                        ui_changed = true
                    }
                    if ui.checkbox(&mut rotation_lock[1], "y").changed() {
                        ui_changed = true
                    }
                    if ui.checkbox(&mut rotation_lock[2], "z").changed() {
                        ui_changed = true
                    }

                    if ui_changed {
                        rigid_body.restrict_rotations(
                            rotation_lock[0],
                            rotation_lock[1],
                            rotation_lock[2],
                            true,
                        );
                    }
                });
            });
        }
    }
}
fn concat_buffers(
    initial: (&mut Vec<Point3<f32>>, &mut Vec<[u32; 3]>),
    append: (&mut Vec<Point3<f32>>, &mut Vec<[u32; 3]>),
) {
    for i in append.1 {
        let offset = initial.0.len() as u32;

        initial
            .1
            .push([i[0] + offset, i[1] + offset, i[2] + offset]);
    }
    initial.0.append(append.0);
}
