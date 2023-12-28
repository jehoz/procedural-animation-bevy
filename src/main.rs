use bevy::prelude::*;
use rand::prelude::*;

#[derive(Component)]
struct BodySegment {
    entity: Entity,
    distance_to_parent: f32,
}

impl BodySegment {
    fn new(e: Entity) -> Self {
        BodySegment {
            entity: e,
            distance_to_parent: 0.3,
        }
    }
}

#[derive(Component)]
struct Creature {
    move_speed: f32,
    turn_speed: f32,
    target_position: Vec3,
    body_segments: Vec<BodySegment>,
}

impl Creature {
    fn new() -> Self {
        Creature {
            move_speed: 2.0,
            turn_speed: 3.0,
            target_position: Vec3::ZERO,
            body_segments: Vec::new(),
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

    // creature
    let mut creature = Creature::new();
    for i in 0..8 {
        let segment = BodySegment::new(
            commands
                .spawn(Transform::IDENTITY.with_translation(Vec3::new(
                    0.0,
                    0.25,
                    (i as f32) * 0.25,
                )))
                .id(),
        );
        creature.body_segments.push(segment);
    }
    commands.spawn(creature);
}

fn move_creatures(
    mut gizmos: Gizmos,
    mut creatures: Query<&mut Creature>,
    mut transforms: Query<&mut Transform>,
    time: Res<Time>,
) {
    for mut creature in &mut creatures {
        let mut head = transforms
            .get_mut(creature.body_segments[0].entity)
            .unwrap();

        // if reached target position, choose a new one randomly
        if head.translation.distance(creature.target_position) < 0.5 {
            creature.target_position = Vec3 {
                x: (random::<f32>() * 10.0) - 5.0,
                y: head.translation.y,
                z: (random::<f32>() * 10.0) - 5.0,
            };
        }

        gizmos.sphere(head.translation, head.rotation, 0.15, Color::WHITE);
        gizmos.circle(creature.target_position, Vec3::Y, 0.25, Color::RED);

        // turn head towards target
        let target_dir = (creature.target_position - head.translation).normalize();
        let rot = Quat::from_rotation_arc(head.forward(), target_dir);
        let (_, mut y_rot, _) = rot.to_euler(EulerRot::XYZ);
        let max_turn = creature.turn_speed * time.delta_seconds();
        y_rot = y_rot.clamp(-max_turn, max_turn);
        head.rotate(Quat::from_rotation_y(y_rot));

        // move head forward
        let movement = head.forward() * creature.move_speed * time.delta_seconds();
        head.translation += movement;

        // move each body segment towards the one ahead of it
        for i in 1..creature.body_segments.len() {
            let segment = &creature.body_segments[i];
            let [parent, mut current] =
                transforms.many_mut([creature.body_segments[i - 1].entity, segment.entity]);
            gizmos.sphere(current.translation, current.rotation, 0.15, Color::WHITE);
            gizmos.line(parent.translation, current.translation, Color::BLUE);

            current.look_at(parent.translation, Vec3::Y);

            let dist = parent.translation.distance(current.translation);
            let movement = current.forward() * (dist - segment.distance_to_parent);
            current.translation += movement;
        }
    }
}
