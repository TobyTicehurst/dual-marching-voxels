//! A minimal fly camera: hold right mouse to look, WASD + QE to move,
//! scroll wheel to change speed. No cursor grabbing needed.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;

use crate::scene;

#[derive(Component)]
pub struct FlyCamera {
    pub speed: f32,
    pub sensitivity: f32,
    yaw: f32,
    pitch: f32,
}

pub fn plugin(app: &mut App) {
    app.add_systems(Startup, spawn_camera)
        .add_systems(Update, fly);
}

fn spawn_camera(mut commands: Commands) {
    let (eye, target) = scene::camera_start();
    let transform = Transform::from_translation(eye).looking_at(target, Vec3::Y);
    let (yaw, pitch, _) = transform.rotation.to_euler(EulerRot::YXZ);
    commands.spawn((
        Camera3d::default(),
        transform,
        // brighten the scene-wide ambient (this component on the camera
        // overrides the dimmer GlobalAmbientLight default)
        AmbientLight {
            color: Color::WHITE,
            brightness: 300.0,
            ..default()
        },
        FlyCamera {
            speed: 40.0,
            sensitivity: 0.003,
            yaw,
            pitch,
        },
    ));
}

fn fly(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
    buttons: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    mut cameras: Query<(&mut Transform, &mut FlyCamera)>,
) {
    let Ok((mut transform, mut cam)) = cameras.single_mut() else {
        return;
    };

    if scroll.delta.y != 0.0 {
        cam.speed = (cam.speed * 1.15f32.powf(scroll.delta.y)).clamp(1.0, 500.0);
    }

    if buttons.pressed(MouseButton::Right) && motion.delta != Vec2::ZERO {
        cam.yaw -= motion.delta.x * cam.sensitivity;
        cam.pitch = (cam.pitch - motion.delta.y * cam.sensitivity).clamp(-1.54, 1.54);
        transform.rotation = Quat::from_euler(EulerRot::YXZ, cam.yaw, cam.pitch, 0.0);
    }

    let mut dir = Vec3::ZERO;
    let forward = transform.forward().as_vec3();
    let right = transform.right().as_vec3();
    if keys.pressed(KeyCode::KeyW) {
        dir += forward;
    }
    if keys.pressed(KeyCode::KeyS) {
        dir -= forward;
    }
    if keys.pressed(KeyCode::KeyD) {
        dir += right;
    }
    if keys.pressed(KeyCode::KeyA) {
        dir -= right;
    }
    if keys.pressed(KeyCode::KeyE) || keys.pressed(KeyCode::Space) {
        dir += Vec3::Y;
    }
    if keys.pressed(KeyCode::KeyQ) || keys.pressed(KeyCode::ShiftLeft) {
        dir -= Vec3::Y;
    }
    if dir != Vec3::ZERO {
        let step = dir.normalize() * cam.speed * time.delta_secs();
        transform.translation += step;
    }
}
