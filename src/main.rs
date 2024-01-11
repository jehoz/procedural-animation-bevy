use bevy::input::common_conditions::input_toggle_active;
use bevy::prelude::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use creature::CreaturePlugin;

mod creature;

#[derive(Component)]
struct OrbitingCamera {
    pub target: Vec3,
    pub speed: f32,
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(
            WorldInspectorPlugin::default().run_if(input_toggle_active(true, KeyCode::Escape)),
        )
        .add_plugins(CreaturePlugin)
        .add_systems(Startup, setup_scene)
        .add_systems(Update, update_orbiting_camera)
        .run();
}

fn setup_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn(PointLightBundle {
        point_light: PointLight {
            intensity: 9000.0,
            range: 100.,
            shadows_enabled: true,
            ..default()
        },
        transform: Transform::from_xyz(5.0, 15.0, 5.0),
        ..default()
    });

    commands.spawn((
        Camera3dBundle {
            transform: Transform::from_xyz(5.0, 2.0, 0.0).looking_at(Vec3::ZERO, Vec3::Y),
            ..default()
        },
        OrbitingCamera {
            target: Vec3::ZERO,
            speed: 0.2,
        },
    ));

    // ground plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(shape::Plane::from_size(10.0).into()),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    });
}

fn update_orbiting_camera(mut query: Query<(&OrbitingCamera, &mut Transform)>, time: Res<Time>) {
    for (orbit, mut transform) in query.iter_mut() {
        transform.rotate_around(
            orbit.target,
            Quat::from_rotation_y(time.delta_seconds() * orbit.speed),
        );
        transform.look_at(orbit.target, Vec3::Y);
    }
}
