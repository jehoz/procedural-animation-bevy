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
    length: f32,
    oscillator: Oscillator,
    last_position: Vec3,
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
                frequency: 8.0,
                phase: 0.0 + i as f32 * std::f32::consts::FRAC_PI_3 * 2.0,
            };
            let length = 0.5;
            let t = transform.with_translation(Vec3::new(-segment.radius, 0.0, 0.0));
            let leg = Leg {
                length,
                oscillator,
                last_position: t.translation,
            };
            commands.spawn((leg, t)).id()
        };
        let leg_r = {
            let oscillator = Oscillator {
                frequency: 8.0,
                phase: std::f32::consts::PI + i as f32 * std::f32::consts::FRAC_PI_3 * 2.0,
            };
            let length = 0.5;
            let t = transform.with_translation(Vec3::new(segment.radius, 0.0, 0.0));
            let leg = Leg {
                length,
                oscillator,
                last_position: t.translation,
            };
            commands.spawn((leg, t)).id()
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
    mut body_segments: Query<(&BodySegment, &mut Transform)>,
    mut legs: Query<(&mut Leg, &mut Transform), Without<BodySegment>>,
    time: Res<Time>,
) {
    for (body, mut body_transform) in &mut body_segments {
        gizmos.sphere(
            body_transform.translation,
            body_transform.rotation,
            body.radius,
            Color::WHITE,
        );
        if let Some((ent_l, ent_r)) = body.legs {
            let [(leg_l, mut foot_l), (leg_r, mut foot_r)] = legs.many_mut([ent_l, ent_r]);

            let hip_l = body_transform.translation + body_transform.left() * body.radius;
            let hip_r = body_transform.translation + body_transform.right() * body.radius;

            foot_l.translation = {
                let scale = leg_l.length * 0.25;
                let mut offset = body_transform.forward() * leg_l.oscillator.sin(&time) * scale;
                offset.y = -leg_l.length + f32::max(0.0, leg_l.oscillator.cos(&time)) * scale;
                hip_l + offset
            };
            gizmos.circle(foot_l.translation, Vec3::Y, 0.1, Color::RED);

            foot_r.translation = {
                let scale = leg_r.length * 0.25;
                let mut offset = body_transform.forward() * leg_r.oscillator.sin(&time) * scale;
                offset.y = -leg_r.length + f32::max(0.0, leg_r.oscillator.cos(&time)) * scale;
                hip_r + offset
            };
            gizmos.circle(foot_r.translation, Vec3::Y, 0.1, Color::RED);

            body_transform.translation.y = {
                let mut delta_l = foot_l.translation - hip_l;
                delta_l.y = 0.0;
                let l = leg_l.length * f32::cos(f32::asin(delta_l.length() / leg_l.length));

                let mut delta_r = foot_r.translation - hip_r;
                delta_r.y = 0.0;
                let r = leg_r.length * f32::cos(f32::asin(delta_r.length() / leg_r.length));

                f32::max(l, r)
            };

            let seg_len = leg_l.length * 0.5;

            let knee_l =
                knee_position(hip_l, foot_l.translation, seg_len, body_transform.forward());
            let knee_r =
                knee_position(hip_r, foot_r.translation, seg_len, body_transform.forward());

            draw_limb_segment(&mut gizmos, hip_l, knee_l, seg_len);
            draw_limb_segment(&mut gizmos, knee_l, foot_l.translation, seg_len);

            draw_limb_segment(&mut gizmos, hip_r, knee_r, seg_len);
            draw_limb_segment(&mut gizmos, knee_r, foot_r.translation, seg_len);
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

fn knee_position(hip: Vec3, foot: Vec3, segment_length: f32, forward: Vec3) -> Vec3 {
    let alpha = {
        if hip.distance(foot) >= 2.0 * segment_length {
            0.0
        } else {
            let hyp = hip.distance(foot);
            // simplifying here because a and b sides always same length
            f32::acos(hyp / (2.0 * segment_length))
        }
    };

    let theta = {
        let mut delta = foot - hip;
        let y = delta.y.abs();
        delta.y = 0.0;
        let xz = delta.length() * forward.dot(delta.normalize()).signum();
        (xz / y).atan()
    };

    let (rot_axis, _) = Quat::from_rotation_arc(-Vec3::Y, forward).to_axis_angle();

    let rotation = Quat::from_axis_angle(rot_axis, alpha + theta);

    let mut offset = Vec3::new(0.0, -segment_length, 0.0);
    offset = rotation.mul_vec3(offset);

    return hip + offset;
}
