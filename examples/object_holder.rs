use opengl_render::camera::Camera;
use opengl_render::cgmath::Rad;
use opengl_render::cubemap_loader::CubemapLoader;
use opengl_render::glium::backend::{Context, Facade};
use opengl_render::gui::DebugGUIFormat;
use opengl_render::ibl::Ibl;
use opengl_render::ibl::{IrradianceConverter, Prefilter, BRDF};
use opengl_render::material::{Equirectangle, SkyboxMat, PBR};
use opengl_render::pbr_model::PbrModel;
use opengl_render::skybox::Skybox;
use opengl_render::support::System;
use opengl_render::{glium::Surface, renderer::Renderer};
use physics_render::object_holder::ObjectHolder;
use physics_render::physics::Physics;
use physics_render::physics_object::PhysicsObject;
use physics_render::PhysicsDebug;
use rapier3d::prelude::*;
use std::path::{Path, PathBuf};
use std::rc::Rc;

fn main() {
    let ground_dir = PathBuf::from("./examples/models/primitives/ground2.glb");
    let model_dir = PathBuf::from("./examples/models/primitives/cube.glb");

    let skybox_file = PathBuf::from("./examples/ibl/Summi_Pool/Summi_Pool_3k.hdr");
    let ibl_dir = PathBuf::from("./examples/ibl/Summi_Pool/");

    let light_pos = [0.0, 0.4, -10.0];
    let light_color = [300.0, 300.0, 300.0];

    let (display, facade, mut renderer) = set_up_renderer();
    let skybox = load_skybox(&facade, &skybox_file, &ibl_dir);

    // Load the Physically Based Rendering shader from the file system
    let pbr = PBR::load_from_fs(&facade);

    //
    // Here we will load the model that will be rendered
    // and make it a physics object
    //

    // Set gravity to be -9.81 m/s
    let mut physics = Physics::new([0.0, -9.81, 0.0]);
    let quarter = std::f32::consts::FRAC_PI_4;

    let mut models = ObjectHolder::new();
    models.push(
        physics.modify_sets(|rigid_body_set, collider_set, _islands, _joints| {
            PhysicsObject::new(
                PbrModel::load_from_fs(model_dir.clone(), &facade, pbr.clone()).unwrap(),
                rigid_body_set,
                collider_set,
            )
        }),
    );
    models.push(
        physics.modify_sets(|rigid_body_set, collider_set, _islands, _joints| {
            let ground = PbrModel::load_from_fs(ground_dir.clone(), &facade, pbr.clone()).unwrap();
            let ground = PhysicsObject::new(ground, rigid_body_set, collider_set);
            let na_rotation =
                rapier3d::na::Unit::from_euler_angles(quarter * 0.25, 0.0, quarter * 0.25);

            ground.modify_rigid_body(rigid_body_set, |rigid_body| {
                let ground_rigid_body = rigid_body.unwrap();

                ground_rigid_body.set_body_type(RigidBodyType::Static);
                ground_rigid_body.set_translation(vector![0.0, -2.0, 0.0], false);
                ground_rigid_body.set_rotation(na_rotation.scaled_axis(), false);
            });

            ground
        }),
    );

    // Move the cube upwards
    models[0].modify_rigid_body(physics.get_rigid_body_set_mut(), |rigid_body| {
        if let Some(rigid_body) = rigid_body {
            rigid_body.set_translation(vector![0.0, 5.0, 10.0], true);
        }
    });

    let mut camera_pos = [0.0, 0.0, 0.0];
    let mut camera_rot = [Rad(0.0), Rad(0.0), Rad(0.0)];

    // Will hold new models that will be added to models next frame
    let mut new_models = Vec::new();

    display.main_loop(
        // Event loop
        move |_, _| {},
        // Render loop
        move |frame, delta_time, egui_ctx| {
            // Time between frames should be used when moving or rotating objects
            let _delta_ms = delta_time.as_micros() as f32 / 1000.0;

            // To render a frame, we must begin a new scene.
            // The scene will keep track of variables that apply to the whole scene, like the
            // camera, and skybox.
            let mut scene = renderer.begin_scene();

            // Create a camera with a 60 degree field of view
            let (width, height) = frame.get_dimensions();
            let camera = Camera::new(Rad(std::f32::consts::PI / 3.0), width, height);

            // Update physics simulation
            physics.step();

            // Set scene variables
            scene.set_camera(camera.get_matrix().into());
            scene.set_camera_pos(camera_pos);
            scene.set_camera_rot(camera_rot);
            scene.set_skybox(Some(&skybox));
            scene
                .get_scene_data_mut()
                .get_raw_lights_mut()
                .add_light(light_pos, light_color);

            // new_models is a buffer of new objects to be rendered
            while new_models.len() > 0 {
                models.push(new_models.pop().unwrap());
            }

            // Render models
            models.render(&mut scene, &mut physics);

            scene.finish(&mut frame.into());

            // Add menu bar to the screen
            egui::TopBottomPanel::top("title_bar").show(egui_ctx, |ui| {
                // Open model
                if ui.button("open").clicked() {
                    if let Some(files) = rfd::FileDialog::new().pick_files() {
                        for path in files {
                            if let Ok(mut model) =
                                PbrModel::load_from_fs(path, &facade, pbr.clone())
                            {
                                // Move the model off of the camera so you can actually see it
                                model.relative_move([0.0, 0.0, 4.0]);
                                let object = physics.modify_sets(
                                    |rigid_body_set, collider_set, _islands, _joints| {
                                        let phys_obj =
                                            PhysicsObject::new(model, rigid_body_set, collider_set);

                                        // Make the object static
                                        phys_obj.modify_rigid_body(rigid_body_set, |rigid_body| {
                                            if let Some(rigid_body) = rigid_body {
                                                rigid_body.set_body_type(RigidBodyType::Static);
                                            }
                                        });

                                        phys_obj
                                    },
                                );
                                models.push(object);
                            }
                        }
                    }
                }
            });

            // List all models in the side panel
            egui::SidePanel::new(egui::panel::Side::Left, "Models").show(egui_ctx, |ui| {
                egui::ScrollArea::new([false, true]).show(ui, |ui| {
                    // Holds indices for models to be removed
                    let mut removed = Vec::new();

                    ui.group(|ui| {
                        ui.heading("Camera");

                        ui.label("Position");
                        if DebugGUIFormat::position(ui, &mut camera_pos, -25.0..=25.0) {}

                        ui.label("Rotation");
                        let mut camera_rotation =
                            [camera_rot[0].0, camera_rot[1].0, camera_rot[2].0];
                        if DebugGUIFormat::euler(ui, &mut camera_rotation) {
                            camera_rot[0] = Rad(camera_rotation[0]);
                            camera_rot[1] = Rad(camera_rotation[1]);
                            camera_rot[2] = Rad(camera_rotation[2]);
                        }
                    });

                    for i in 0..models.get_num_objects() {
                        let model = &mut models.get_object_mut(i).unwrap();

                        egui::CollapsingHeader::new(format!("Object {}", i)).show(ui, |ui| {
                            let (rigid_body_set, collider_set, _islands, _joints) =
                                physics.get_sets_mut();
                            model.debug(ui, rigid_body_set, collider_set);

                            // mark item for removal
                            if ui.button("delete").clicked() {
                                removed.push(i);
                            }

                            if ui.button("clone").clicked() {
                                //new_models.push(model.clone());
                            }
                        });
                    }

                    // Remove items from vec (from back to front to not mess up indexing)
                    for i in removed.len() - 1..=0 {
                        models.remove_physics(removed[i], &mut physics);
                    }
                });
            });
        },
        // Gui loop
        move |_egui_ctx| {},
    );
}

fn set_up_renderer() -> (System, Rc<Context>, Renderer) {
    // Create the window and opengl instance
    let display = System::init("renderer");

    let facade = display.display.get_context().clone();
    let renderer = Renderer::new((*display.display).clone());

    (display, facade, renderer)
}

fn load_skybox(facade: &impl Facade, skybox_dir: &Path, ibl_dir: &Path) -> Skybox {
    let ibl_dir = ibl_dir.to_path_buf();
    let skybox_file = skybox_dir.to_path_buf();
    // Convert an equirectangular image into a cubemap and store it to the file system
    // This generated cubemap will be used as the skybox
    let compute = Equirectangle::load_from_fs(facade);
    compute
        .compute_from_fs_hdr(
            skybox_file,
            ibl_dir.join("cubemap/"),
            "png",
            facade,
            Camera::new(Rad(std::f32::consts::PI * 0.5), 1024, 1024).into(),
        )
        .unwrap();

    //
    // Here we will generate the irradiance map, prefilter map, and brdf texture
    //
    // The irradiance map maps the light output from the skybox to use as ambient light,
    // The prefilter map is used for reflections
    // The brdf is the same for all skyboxes, we just generate it to make sure that it exists
    //

    // Load the necessary shaders from the file system
    let irradiance_converter = IrradianceConverter::load(facade);
    let prefilter_shader = Prefilter::load(facade);
    let brdf_shader = BRDF::new(facade);

    // Load the skybox again to generate the maps
    let ibl_cubemap = CubemapLoader::load_from_fs(ibl_dir.join("cubemap/"), "png", facade).unwrap();

    // Generate the maps and store them to the file system
    opengl_render::ibl::generate_ibl_from_cubemap(
        facade,
        &ibl_cubemap,
        ibl_dir.clone(),
        irradiance_converter,
        prefilter_shader,
        brdf_shader,
    )
    .unwrap();
    let skybox_mat = SkyboxMat::load_from_fs(facade, ibl_dir.join("cubemap/"), "png").unwrap();
    // Will hold the generated maps
    let mut skybox = Skybox::new(facade, skybox_mat);

    // Load prefilter, irradiance, and brdf from file system
    let Ibl {
        prefilter,
        irradiance_map: ibl,
        brdf,
    } = opengl_render::ibl::load_ibl_fs(facade, ibl_dir).unwrap();

    // Assign irradiance map, prefilter map and brdf to the skybox wrapper
    skybox.set_ibl(Some(ibl));
    skybox.set_prefilter(Some(prefilter));
    skybox.set_brdf(Some(brdf));

    skybox
}
