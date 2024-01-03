use bevy::prelude::*;
use rand::prelude::*;

#[derive(Component)]
struct Leg {
    length: f32,
}

impl Leg {
    fn new() -> Self {
        Leg { length: 0.5 }
    }
}

#[derive(Component)]
struct BodySegment {
    radius: f32,
    distance_to_parent: f32,
    legs: Option<(Entity, Entity)>,
}

impl BodySegment {
    fn new() -> Self {
        BodySegment {
            radius: 0.2,
            distance_to_parent: 0.3,
            legs: None,
        }
    }
}

#[derive(Component)]
struct Creature {
    move_speed: f32,
    turn_speed: f32,
    target_position: Vec3,
    body_segments: Vec<Entity>,
}

impl Creature {
    fn new() -> Self {
        Creature {
            move_speed: 2.0,
            turn_speed: std::f32::consts::PI,
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
        let transform =
            Transform::IDENTITY.with_translation(Vec3::new(0.0, 0.5, 0.25 * (i as f32)));

        let mut segment = BodySegment::new();
        if i == 1 || i == 6 {
            let leg_l = commands
                .spawn((
                    Leg::new(),
                    transform.with_translation(Vec3::new(-0.1, 0.0, 0.0)),
                ))
                .id();
            let leg_r = commands
                .spawn((
                    Leg::new(),
                    transform.with_translation(Vec3::new(-0.1, 0.0, 0.0)),
                ))
                .id();
            segment.legs = Some((leg_l, leg_r));
        }
        let segment_ent = commands.spawn((segment, transform)).id();
        creature.body_segments.push(segment_ent);
    }

    commands.spawn(creature);
}

fn move_creatures(
    mut gizmos: Gizmos,
    mut creatures: Query<&mut Creature>,
    mut body_segments: Query<(&BodySegment, &mut Transform)>,
    mut legs: Query<(&mut Leg, &mut Transform), Without<BodySegment>>,
    time: Res<Time>,
) {
    for mut creature in &mut creatures {
        let (head, mut head_transform) = body_segments.get_mut(creature.body_segments[0]).unwrap();

        // if reached target position, choose a new one randomly
        if head_transform
            .translation
            .xz()
            .distance(creature.target_position.xz())
            < 0.5
        {
            creature.target_position = Vec3 {
                x: (random::<f32>() * 10.0) - 5.0,
                y: head_transform.translation.y,
                z: (random::<f32>() * 10.0) - 5.0,
            };
        }

        gizmos.circle(creature.target_position, Vec3::Y, 0.25, Color::RED);

        // turn head towards target
        let target_dir =
            (head_transform.translation.xz() - creature.target_position.xz()).normalize();
        let (_, _, mut y_rot) =
            Quat::from_rotation_arc_2d(head_transform.forward().xz(), target_dir)
                .to_euler(EulerRot::XYZ);
        let max_turn = creature.turn_speed * time.delta_seconds();
        y_rot = y_rot.clamp(-max_turn, max_turn);
        head_transform.rotate(Quat::from_rotation_y(y_rot));

        // move head forward
        let movement = head_transform.forward() * creature.move_speed * time.delta_seconds();
        head_transform.translation += movement;

        gizmos.sphere(
            head_transform.translation,
            head_transform.rotation,
            head.radius,
            Color::WHITE,
        );

        // move each trailing body segment towards the one ahead of it
        for i in 1..creature.body_segments.len() {
            let [(current, mut current_transform), (_, parent_transform)] =
                body_segments.many_mut([creature.body_segments[i], creature.body_segments[i - 1]]);

            current_transform.look_at(parent_transform.translation, Vec3::Y);

            let dist = parent_transform
                .translation
                .distance(current_transform.translation);
            let movement = current_transform.forward() * (dist - current.distance_to_parent);
            current_transform.translation += movement;

            gizmos.sphere(
                current_transform.translation,
                current_transform.rotation,
                current.radius,
                Color::WHITE,
            );
            gizmos.line(
                current_transform.translation,
                parent_transform.translation,
                Color::BLUE,
            );

            if let Some((leg_l_ent, leg_r_ent)) = current.legs {
                let [l, r] = legs.many_mut([leg_l_ent, leg_r_ent]);
                update_leg_pair(&mut gizmos, l, r, (current, &current_transform));
            }
        }
    }
}

fn update_leg_pair<'a>(
    gizmos: &mut Gizmos,
    left: (Mut<'a, Leg>, Mut<'a, Transform>),
    right: (Mut<'a, Leg>, Mut<'a, Transform>),
    body: (&BodySegment, &Transform),
) {
    let shoulder_l = body.1.translation + body.1.left() * body.0.radius;
    let shoulder_r = body.1.translation + body.1.right() * body.0.radius;

    let (leg_l, mut leg_l_transform) = left;
    let (leg_r, mut leg_r_transform) = right;
    if leg_l_transform.translation.distance(shoulder_l) >= leg_l.length {
        let front_left =
            Quat::from_rotation_y(std::f32::consts::FRAC_PI_4).mul_vec3(body.1.forward());
        leg_l_transform.translation = shoulder_l + front_left * leg_l.length * 0.9;
    }

    if leg_r_transform.translation.distance(shoulder_r) >= leg_r.length {
        let front_right =
            Quat::from_rotation_y(-std::f32::consts::FRAC_PI_4).mul_vec3(body.1.forward());
        leg_r_transform.translation = shoulder_r + front_right * leg_r.length * 0.9;
    }

    let seg_len = leg_l.length * 0.5;
    let elbow_l = elbow_position(shoulder_l, leg_l_transform.translation, seg_len);
    let elbow_r = elbow_position(shoulder_r, leg_r_transform.translation, seg_len);

    draw_limb_segment(gizmos, shoulder_l, elbow_l, seg_len);
    draw_limb_segment(gizmos, elbow_l, leg_l_transform.translation, seg_len);

    draw_limb_segment(gizmos, shoulder_r, elbow_r, seg_len);
    draw_limb_segment(gizmos, elbow_r, leg_r_transform.translation, seg_len);
}

fn draw_limb_segment(gizmos: &mut Gizmos, a: Vec3, b: Vec3, length: f32) {
    if a.distance(b) < length * 0.99 {
        gizmos.line(a, b, Color::CYAN);
    } else if a.distance(b) > length * 1.01 {
        gizmos.line(a, b, Color::RED);
    } else {
        gizmos.line(a, b, Color::PURPLE);
    }
}

fn elbow_position(shoulder: Vec3, hand: Vec3, segment_length: f32) -> Vec3 {
    let alpha = {
        let hyp = shoulder.distance(hand);
        // simplifying here because a and b sides always same length
        f32::acos(hyp / (2.0 * segment_length))
    };

    let theta = {
        let mut delta = hand - shoulder;
        let y = delta.y.abs();
        delta.y = 0.0;
        let xz = delta.length();
        (y / xz).atan()
    };

    let rotation = Quat::from_axis_angle(
        (hand - shoulder).normalize().cross(Vec3::Y),
        std::f32::consts::FRAC_PI_4 - (alpha + theta),
    );

    let mut offset = Vec3::new(0.0, segment_length, 0.0);
    offset = rotation.mul_vec3(offset);

    return hand + offset;
}
