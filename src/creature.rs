use bevy::prelude::*;
use rand::prelude::*;

pub struct CreaturePlugin;

impl Plugin for CreaturePlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(Startup, spawn_creatures)
            // .add_systems(Update, move_creatures)
            .add_systems(Update, move_legs);
    }
}

#[derive(Component)]
struct Oscillator {
    frequency: f32,
    phase: f32,
}

impl Oscillator {
    fn sin(&self, t: &Time) -> f32 {
        (self.frequency * t.elapsed_seconds() + self.phase).sin()
    }

    fn cos(&self, t: &Time) -> f32 {
        (self.frequency * t.elapsed_seconds() + self.phase).cos()
    }
}

#[derive(Component)]
struct Leg {
    femur_length: f32,
    tibia_length: f32,
    metatarsal_length: f32,
    toe_length: f32,
    ankle_lift: f32,
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
            move_speed: 1.0,
            turn_speed: std::f32::consts::PI,
            target_position: Vec3::ZERO,
            body_segments: Vec::new(),
        }
    }
}

fn spawn_creatures(mut commands: Commands) {
    let mut creature = Creature::new();

    for i in 0..1 {
        let transform =
            Transform::IDENTITY.with_translation(Vec3::new(0.0, 0.5, 0.25 * (i as f32)));

        let mut segment = BodySegment::new();
        let leg_l = {
            let oscillator = Oscillator {
                frequency: 5.0,
                phase: 0.0 + i as f32 * std::f32::consts::FRAC_PI_3 * 2.0,
            };
            let t = transform.with_translation(Vec3::new(-segment.radius, 0.0, 0.0));
            let leg = Leg {
                femur_length: 0.25,
                tibia_length: 0.225,
                metatarsal_length: 0.175,
                ankle_lift: std::f32::consts::PI * 0.4,
            };
            commands.spawn((leg, t, oscillator)).id()
        };
        let leg_r = {
            let oscillator = Oscillator {
                frequency: 5.0,
                phase: std::f32::consts::PI + i as f32 * std::f32::consts::FRAC_PI_3 * 2.0,
            };
            let t = transform.with_translation(Vec3::new(segment.radius, 0.0, 0.0));
            let leg = Leg {
                femur_length: 0.25,
                tibia_length: 0.225,
                metatarsal_length: 0.175,
                ankle_lift: std::f32::consts::PI * 0.4,
            };
            commands.spawn((leg, t, oscillator)).id()
        };
        segment.legs = Some((leg_l, leg_r));
        let segment_ent = commands.spawn((segment, transform)).id();
        creature.body_segments.push(segment_ent);
    }

    commands.spawn(creature);
}

fn move_creatures(
    mut gizmos: Gizmos,
    mut creatures: Query<&mut Creature>,
    mut body_segments: Query<(&BodySegment, &mut Transform)>,
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
        }
    }
}

fn move_legs(
    mut gizmos: Gizmos,
    mut body_segments: Query<(&BodySegment, &Transform)>,
    mut legs: Query<(&mut Leg, &mut Transform, &Oscillator), Without<BodySegment>>,
    time: Res<Time>,
) {
    for (body, body_transform) in &mut body_segments {
        gizmos.sphere(
            body_transform.translation,
            body_transform.rotation,
            body.radius,
            Color::WHITE,
        );
        if let Some((ent_l, ent_r)) = body.legs {
            let [(leg_l, mut toe_l, osc_l), (leg_r, mut toe_r, osc_r)] =
                legs.many_mut([ent_l, ent_r]);

            let hip_l = body_transform.translation + body_transform.left() * body.radius;
            let hip_r = body_transform.translation + body_transform.right() * body.radius;

            toe_l.translation = {
                let scale = (leg_l.femur_length + leg_l.tibia_length) * 0.25;
                let mut pos = hip_l + body_transform.forward() * osc_l.sin(&time) * scale;
                pos.y = f32::max(0.0, osc_l.cos(&time)) * scale;
                pos
            };
            gizmos.circle(toe_l.translation, Vec3::Y, 0.025, Color::RED);

            toe_r.translation = {
                let scale = (leg_r.femur_length + leg_r.tibia_length) * 0.25;
                let mut pos = hip_r + body_transform.forward() * osc_r.sin(&time) * scale;
                pos.y = f32::max(0.0, osc_r.cos(&time)) * scale;
                pos
            };
            gizmos.circle(toe_r.translation, Vec3::Y, 0.025, Color::RED);

            let (knee_l, ankle_l) =
                solve_leg_ik(&leg_l, hip_l, toe_l.translation, body_transform.forward());
            let (knee_r, ankle_r) =
                solve_leg_ik(&leg_r, hip_r, toe_r.translation, body_transform.forward());

            draw_limb_segment(&mut gizmos, hip_l, knee_l, leg_l.femur_length);
            draw_limb_segment(&mut gizmos, knee_l, ankle_l, leg_l.tibia_length);
            draw_limb_segment(
                &mut gizmos,
                ankle_l,
                toe_l.translation,
                leg_l.metatarsal_length,
            );
            gizmos.ray(
                toe_l.translation,
                body_transform.forward() * leg_l.toe_length,
                Color::WHITE,
            );

            draw_limb_segment(&mut gizmos, hip_r, knee_r, leg_r.femur_length);
            draw_limb_segment(&mut gizmos, knee_r, ankle_r, leg_r.tibia_length);
            draw_limb_segment(
                &mut gizmos,
                ankle_r,
                toe_r.translation,
                leg_r.metatarsal_length,
            );
            gizmos.ray(
                toe_r.translation,
                body_transform.forward() * leg_r.toe_length,
                Color::WHITE,
            );
        }
    }
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

fn solve_leg_ik(leg: &Leg, hip: Vec3, toe: Vec3, forward: Vec3) -> (Vec3, Vec3) {
    let ankle_lift = {
        let hip_to_toe = toe - hip;
        let xz = hip_to_toe.xz().length() * forward.xz().dot(hip_to_toe.xz()).signum();
        leg.ankle_lift + (xz / hip_to_toe.y).atan()
    };

    let ankle = {
        let mut offset = -forward * leg.metatarsal_length;
        let (axis, _) = Quat::from_rotation_arc(-forward, Vec3::Y).to_axis_angle();
        offset = Quat::from_axis_angle(axis, ankle_lift).mul_vec3(offset);
        toe + offset
    };

    let gamma = {
        if hip.distance(ankle) >= leg.femur_length + leg.tibia_length {
            0.0
        } else {
            let l = hip.distance(ankle);
            f32::acos(
                (l.powi(2) + leg.femur_length.powi(2) - leg.tibia_length.powi(2))
                    / (2.0 * l * leg.femur_length),
            )
        }
    };

    let (rot_axis, _) = Quat::from_rotation_arc(-Vec3::Y, forward).to_axis_angle();

    let rotation = Quat::from_axis_angle(rot_axis, gamma);

    let mut offset = (ankle - hip).normalize() * leg.femur_length;
    offset = rotation.mul_vec3(offset);

    return (hip + offset, ankle);
}
