//! A minimal fly camera: the cursor is captured and mouse movement looks
//! around; WASD + QE to move, scroll wheel to change speed, Esc to quit.

use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::prelude::*;
use bevy::window::{CursorGrabMode, CursorOptions, WindowFocused};

use crate::{config, scene};

#[derive(Component)]
pub struct FlyCamera {
    pub speed: f32,
    pub sensitivity: f32,
    yaw: f32,
    pitch: f32,
}

pub fn plugin(app: &mut App) {
    app.add_systems(Startup, (spawn_camera, grab_cursor))
        .add_systems(Update, (fly, regrab_on_focus, exit_on_esc));
}

fn spawn_camera(mut commands: Commands) {
    let start = config::CAMERA.unwrap_or_else(|| scene::camera_start(config::SCENE));
    let transform = Transform::from_translation(start.eye).looking_at(start.target, Vec3::Y);
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
            speed: start.speed,
            sensitivity: 0.003,
            yaw,
            pitch,
        },
    ));
}

fn grab_cursor(mut windows: Query<&mut CursorOptions>) {
    for mut cursor in &mut windows {
        cursor.grab_mode = CursorGrabMode::Locked;
        cursor.visible = false;
    }
}

/// The OS releases the cursor when the window loses focus; re-apply the
/// grab when focus returns (touching the component re-triggers the lock).
fn regrab_on_focus(
    mut focus_events: MessageReader<WindowFocused>,
    mut windows: Query<&mut CursorOptions>,
) {
    for event in focus_events.read() {
        if event.focused
            && let Ok(mut cursor) = windows.get_mut(event.window)
        {
            cursor.set_changed();
            cursor.grab_mode = CursorGrabMode::Locked;
            cursor.visible = false;
        }
    }
}

fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut commands: Commands) {
    if keys.just_pressed(KeyCode::Escape) {
        commands.write_message(AppExit::Success);
    }
}

fn fly(
    time: Res<Time>,
    keys: Res<ButtonInput<KeyCode>>,
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

    if motion.delta != Vec2::ZERO {
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
