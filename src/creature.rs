use bevy::prelude::*;
use rand::prelude::*;
use std::f32::consts::*;

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
    fn with_offset(&self, offset: f32) -> Self {
        Oscillator {
            frequency: self.frequency,
            phase: self.phase + offset,
        }
    }

    /// Samples the oscillator as a normal sine wave.
    fn sin(&self, t: &Time) -> f32 {
        self.frequency * (t.elapsed_seconds() + self.phase).sin()
    }

    /// Samples the oscillator as a skewed sine wave.
    ///
    /// The `skew` parameter should be a value in the range [-1, 1] and shifts the peak of the sine
    /// wave to the left or the right.
    /// When `skew` is 0, the wave is a normal sine wave.  At a `skew` of 1, the sine wave is
    /// skewed all the way to the right, resulting in an ascending sawtooth wave, and at -1 it
    /// becomes a _descending_ sawtooth wave.
    fn skewed(&self, t: &Time, skew: f32) -> f32 {
        // equation is undefined for skew=0, but the limit as it approaches 0 is a normal sine wave
        if skew == 0.0 {
            return self.sin(t);
        }
        let x = self.frequency * (t.elapsed_seconds() + self.phase);
        let skew = skew.clamp(-1.0, 1.0);
        (1.0 / skew) * f32::atan((skew * f32::sin(x)) / (1.0 - skew * f32::cos(x)))
    }

    /// Samples the oscillator as sine wave whose positive and negative portions are asymmetric.
    ///
    /// The bias parameter is any real number in the range (-inf, inf) and controls how much the
    /// wave is shifted negatively or positively.
    fn asymmetric(&self, t: &Time, bias: f32) -> f32 {
        // equation is undefined for bias=0, but the limit as it approaches 0 is a normal sine wave
        if bias == 0.0 {
            return self.sin(t);
        }
        let x = self.frequency * (t.elapsed_seconds() + self.phase);
        let k = 2_f32.powf(-bias);
        (2.0 * (k.powf(x.sin() + 1.0) - 1.0)) / (k.powi(2) - 1.0) - 1.0
    }
}

#[derive(Component, Clone)]
struct Leg {
    femur_length: f32,
    tibia_length: f32,
    metatarsal_length: f32,
    toe_length: f32,
    ankle_lift: f32,
    ik_type: LegIKType,
}

impl Leg {
    fn max_length(&self) -> f32 {
        self.femur_length + self.tibia_length + self.metatarsal_length * self.ankle_lift.cos()
    }
}

#[derive(Clone)]
enum LegIKType {
    Front,
    Rear,
}

const PLANTIGRADE_LEG_FRONT: Leg = Leg {
    femur_length: 0.48,
    tibia_length: 0.48,
    metatarsal_length: 0.15,
    toe_length: 0.1,
    ankle_lift: 1.15,
    ik_type: LegIKType::Front,
};

const PLANTIGRADE_LEG_REAR: Leg = Leg {
    femur_length: 0.5,
    tibia_length: 0.51,
    metatarsal_length: 0.1,
    toe_length: 0.1,
    ankle_lift: 0.0,
    ik_type: LegIKType::Rear,
};

const DIGITIGRADE_LEG_FRONT: Leg = Leg {
    femur_length: 0.37,
    tibia_length: 0.57,
    metatarsal_length: 0.21,
    toe_length: 0.06,
    ankle_lift: 1.5,
    ik_type: LegIKType::Front,
};

const DIGITIGRADE_LEG_REAR: Leg = Leg {
    femur_length: 0.5,
    tibia_length: 0.45,
    metatarsal_length: 0.3,
    toe_length: 0.2,
    ankle_lift: 0.8,
    ik_type: LegIKType::Rear,
};

const UNGULIGRADE_LEG_FRONT: Leg = Leg {
    femur_length: 0.5,
    tibia_length: 0.5,
    metatarsal_length: 0.5,
    toe_length: 0.0,
    ankle_lift: 1.28,
    ik_type: LegIKType::Front,
};

const UNGULIGRADE_LEG_REAR: Leg = Leg {
    femur_length: 0.5,
    tibia_length: 0.5,
    metatarsal_length: 0.5,
    toe_length: 0.0,
    ankle_lift: 1.28,
    ik_type: LegIKType::Rear,
};

#[derive(Component)]
struct BodySegment {
    radius: f32,
    legs: Option<(Entity, Entity)>,
}

impl BodySegment {
    fn new() -> Self {
        BodySegment {
            radius: 0.2,
            legs: None,
        }
    }
}

#[derive(Component)]
struct Creature {
    move_speed: f32,
    body_segments: Vec<Entity>,
}

impl Creature {
    fn new() -> Self {
        Creature {
            move_speed: 1.0,
            body_segments: Vec::new(),
        }
    }
}

fn spawn_creatures(mut commands: Commands) {
    for (i, (front_leg_type, rear_leg_type)) in vec![
        (PLANTIGRADE_LEG_FRONT, PLANTIGRADE_LEG_REAR),
        (DIGITIGRADE_LEG_FRONT, DIGITIGRADE_LEG_REAR),
        (UNGULIGRADE_LEG_FRONT, UNGULIGRADE_LEG_REAR),
    ]
    .iter()
    .enumerate()
    {
        let mut creature = Creature::new();

        for s in 0..3 {
            let transform = Transform::IDENTITY.with_translation(Vec3::new(
                -1.5 + i as f32 * 1.5,
                1.0,
                -0.75 + 0.5 * s as f32,
            ));
            let mut segment = BodySegment::new();
            if s == 0 || s == 2 {
                let leg_type = if s == 0 {
                    front_leg_type
                } else {
                    rear_leg_type
                };

                let leg_l = {
                    let oscillator = Oscillator {
                        frequency: 5.0,
                        phase: 0.0,
                    };
                    let t = transform.with_translation(Vec3::new(-segment.radius, 0.0, 0.0));
                    commands.spawn((leg_type.clone(), t, oscillator)).id()
                };
                let leg_r = {
                    let oscillator = Oscillator {
                        frequency: 5.0,
                        phase: PI,
                    };
                    let t = transform.with_translation(Vec3::new(segment.radius, 0.0, 0.0));
                    commands.spawn((leg_type.clone(), t, oscillator)).id()
                };
                segment.legs = Some((leg_l, leg_r));
            }
            let s_name = Name::new(format!("Creature {} - Segment {}", i, s));
            let segment_ent = commands.spawn((s_name, segment, transform)).id();
            creature.body_segments.push(segment_ent);
        }

        let c_name = Name::new(format!("Creature {}", i));
        commands.spawn((c_name, creature));
    }
}

fn move_legs(
    mut gizmos: Gizmos,
    mut body_segments: Query<(&BodySegment, &mut Transform)>,
    mut legs: Query<(&mut Leg, &mut Transform, &Oscillator), Without<BodySegment>>,
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
            let [(leg_l, mut target_l, osc_l), (leg_r, mut target_r, osc_r)] =
                legs.many_mut([ent_l, ent_r]);

            let b_osc = Oscillator {
                frequency: osc_l.frequency * 2.0,
                phase: 0.9306,
            };
            body_transform.translation.y =
                0.9 * leg_l.max_length() - 0.025 * b_osc.asymmetric(&time, 1.0);

            let hip_l = body_transform.translation + body_transform.left() * body.radius;
            let hip_r = body_transform.translation + body_transform.right() * body.radius;

            target_l.translation = {
                let step_height = leg_l.max_length() * 0.125;
                let step_distance = step_height * 2.0;
                let mut pos =
                    hip_l + body_transform.forward() * osc_l.skewed(&time, 0.5) * step_distance;
                pos.y = f32::max(0.0, osc_l.with_offset(FRAC_PI_2).asymmetric(&time, -1.0))
                    * step_height;
                pos
            };
            gizmos.circle(target_l.translation, Vec3::Y, 0.025, Color::RED);

            target_r.translation = {
                let step_height = leg_r.max_length() * 0.125;
                let step_distance = step_height * 2.0;
                let mut pos =
                    hip_r + body_transform.forward() * osc_r.skewed(&time, 0.5) * step_distance;
                pos.y = f32::max(0.0, osc_r.with_offset(FRAC_PI_2).asymmetric(&time, -1.0))
                    * step_height;
                pos
            };
            gizmos.circle(target_r.translation, Vec3::Y, 0.025, Color::RED);

            let (knee_l, ankle_l, ball_l, toe_l) = solve_leg_ik(
                &leg_l,
                hip_l,
                target_l.translation,
                body_transform.forward(),
            );
            let (knee_r, ankle_r, ball_r, toe_r) = solve_leg_ik(
                &leg_r,
                hip_r,
                target_r.translation,
                body_transform.forward(),
            );

            draw_limb_segment(&mut gizmos, hip_l, knee_l, leg_l.femur_length);
            draw_limb_segment(&mut gizmos, knee_l, ankle_l, leg_l.tibia_length);
            draw_limb_segment(&mut gizmos, ankle_l, ball_l, leg_l.metatarsal_length);
            draw_limb_segment(&mut gizmos, ball_l, toe_l, leg_l.toe_length);

            draw_limb_segment(&mut gizmos, hip_r, knee_r, leg_r.femur_length);
            draw_limb_segment(&mut gizmos, knee_r, ankle_r, leg_r.tibia_length);
            draw_limb_segment(&mut gizmos, ankle_r, ball_r, leg_r.metatarsal_length);
            draw_limb_segment(&mut gizmos, ball_r, toe_r, leg_r.toe_length);
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

fn solve_leg_ik(
    leg: &Leg,
    hip: Vec3,
    foot_target: Vec3,
    forward: Vec3,
) -> (Vec3, Vec3, Vec3, Vec3) {
    let ball = {
        let ankle_lift_percent = (1.0 - (leg.ankle_lift / FRAC_PI_2)).clamp(0.0, 1.0);
        let offset = ankle_lift_percent * (leg.ankle_lift.cos() * leg.metatarsal_length) * forward;
        foot_target + offset
    };

    let ankle_lift = {
        let hip_to_ball = ball - hip;
        let xz = hip_to_ball.xz().length() * forward.xz().dot(hip_to_ball.xz()).signum();
        leg.ankle_lift + (xz / hip_to_ball.y).atan()
    };

    let ankle = {
        let mut offset = -forward * leg.metatarsal_length;
        let (axis, _) = Quat::from_rotation_arc(-forward, Vec3::Y).to_axis_angle();
        offset = Quat::from_axis_angle(axis, ankle_lift).mul_vec3(offset);
        ball + offset
    };

    let toe = {
        let ankle_to_ball = ball - ankle;
        let xz = ankle_to_ball.xz().length() * forward.xz().dot(ankle_to_ball.xz()).signum();
        let toe_angle = (ankle_to_ball.y / xz).atan().max(0.0);
        let (axis, _) = Quat::from_rotation_arc(forward, Vec3::Y).to_axis_angle();
        let mut offset = forward * leg.toe_length;
        offset = Quat::from_axis_angle(axis, toe_angle).mul_vec3(offset);
        ball + offset
    };

    let mut gamma = {
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
    if let LegIKType::Front = leg.ik_type {
        gamma *= -1.0;
    }

    let (rot_axis, _) = Quat::from_rotation_arc(-Vec3::Y, forward).to_axis_angle();

    let rotation = Quat::from_axis_angle(rot_axis, gamma);

    let mut offset = (ankle - hip).normalize() * leg.femur_length;
    offset = rotation.mul_vec3(offset);

    return (hip + offset, ankle, ball, toe);
}
