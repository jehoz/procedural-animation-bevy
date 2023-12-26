use bevy::prelude::*;
use rand::prelude::*;

#[derive(Component)]
struct Creature {
    move_speed: f32,
    turn_speed: f32,
    target_position: Vec3,
}

impl Creature {
    fn new() -> Self {
        Creature {
            move_speed: 2.0,
            turn_speed: 3.0,
            target_position: Vec3::ZERO,
        }
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_systems(Startup, setup)
        .add_systems(Update, move_creatures)
        .run();
}

fn setup(
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

    commands.spawn(Camera3dBundle {
        transform: Transform::from_xyz(10.0, 15.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        ..default()
    });

    // ground plane
    commands.spawn(PbrBundle {
        mesh: meshes.add(shape::Plane::from_size(10.0).into()),
        material: materials.add(Color::rgb(0.3, 0.5, 0.3).into()),
        ..default()
    });

    // creature (sphere)
    commands.spawn((
        PbrBundle {
            mesh: meshes.add(Mesh::from(shape::Box {
                min_x: -0.25,
                max_x: 0.25,
                min_y: -0.25,
                max_y: 0.25,
                min_z: -0.5,
                max_z: 0.5,
            })),
            material: materials.add(Color::WHITE.into()),
            transform: Transform::IDENTITY,
            ..default()
        },
        Creature::new(),
    ));
}

fn move_creatures(
    mut gizmos: Gizmos,
    mut query: Query<(&mut Transform, &mut Creature)>,
    time: Res<Time>,
) {
    for (mut transform, mut creature) in &mut query {
        if transform.translation.distance(creature.target_position) < 0.5 {
            creature.target_position = Vec3 {
                x: (random::<f32>() * 10.0) - 5.0,
                y: transform.translation.y,
                z: (random::<f32>() * 10.0) - 5.0,
            };
        }

        gizmos.circle(creature.target_position, Vec3::Y, 0.25, Color::RED);

        let target_dir = (creature.target_position - transform.translation).normalize();
        let rquat = Quat::from_rotation_arc(transform.forward(), target_dir);
        let (_, mut y_rot, _) = rquat.to_euler(EulerRot::XYZ);
        if y_rot < 0.0 {
            y_rot = f32::max(y_rot, -1.0 * creature.turn_speed * time.delta_seconds());
        } else {
            y_rot = f32::min(y_rot, creature.turn_speed * time.delta_seconds());
        }

        transform.rotate(Quat::from_rotation_y(y_rot));

        let movement = transform.forward() * creature.move_speed * time.delta_seconds();
        transform.translation += movement;
    }
}
