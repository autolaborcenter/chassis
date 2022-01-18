/// 里程计模型，表示当前机器人位姿
///
/// 采用两轮差动模型，轨迹为圆弧，给定单步弧长、转角 `(s，theta)`，累计得到当前位置和姿态 `pose`
///
/// ## NOTICE
///
/// 里程计初始化，可设为默认原点 `Odometry::ZERO`
/// 里程计增量，借用 `Velocity` 结构体，给定位移 `s` 和角度 `theta` ，得到 `delta_odometry = Odometry::from(Velocity{v: s, w: theta})`
/// 累加增量，可直接用 `+=` 运算，即 `Odometry += delta_Odometry::from(Velocity)`

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Odometry {
    /// 机器人行驶总里程，单位 m
    ///
    /// 这个量是单调增的
    pub s: f32,
    /// 机器人行驶总转角，单位 rad
    ///
    /// 这个量是单调增的
    pub a: f32,
    /// 机器人当前位置及角度，采用刚体变换 SE(2) 来表示位姿，即一个 2 维平移和一个 2 维旋转
    pub pose: crate::Isometry2<f32>,
}

impl Odometry {
    /// 零里程，用于初始化
    pub const ZERO: Self = Self {
        s: 0.0,
        a: 0.0,
        pose: crate::isometry(0.0, 0.0, 1.0, 0.0),
    };
}

/// 里程原地求和
impl std::ops::AddAssign for Odometry {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.s += rhs.s;
        self.a += rhs.a;
        // 位姿的叠加在 SE(2) 中用乘法表示
        self.pose *= rhs.pose;
    }
}

/// 里程求和
///
/// `self` + `rhs` 即从 `self` 出发再行驶 `rhs`
impl std::ops::Add for Odometry {
    type Output = Self;

    #[inline]
    fn add(mut self, rhs: Self) -> Self::Output {
        self += rhs;
        self
    }
}

impl std::fmt::Display for Odometry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "Odometry: {{ s: {:.3}, a: {:.1}°, x: {:.3}, y: {:.3}, theta: {:.1}° }}",
            self.s,
            self.a.to_degrees(),
            self.pose.translation.vector[0],
            self.pose.translation.vector[1],
            self.pose.rotation.angle().to_degrees()
        )
    }
}

#[test]
fn odometry_test() {
    use crate::Velocity;
    use std::{
        f32::{consts::PI, EPSILON},
        time::Duration,
    };

    #[inline]
    fn pose_equal(a: crate::Isometry2<f32>, b: crate::Isometry2<f32>) -> bool {
        (a.translation.vector[0] - b.translation.vector[0]).abs() <= EPSILON
            || (a.translation.vector[1] - b.translation.vector[1]).abs() <= EPSILON
            || (a.rotation.angle() - b.rotation.angle()).abs() <= EPSILON
    }

    //测试里程计原点及输出是否正确
    let mut d = Odometry::ZERO;
    assert_eq!(
        format!("{d}"),
        "Odometry: { s: 0.000, a: 0.0°, x: 0.000, y: 0.000, theta: 0.0° }"
    );
    //测试机器人从原点出发，行进一整个圆是否会回到原点
    let radius = 100.0;
    let circumference = 2.0 * PI * radius;
    let step_num = 10.0;
    let delta_vel = Velocity {
        v: (circumference / step_num),
        w: (PI * 2.0 / step_num),
    };
    let dd = delta_vel * Duration::from_secs(1);

    for _ in 0..10 {
        d += dd;
    }
    assert!(
        pose_equal(d.pose, Odometry::ZERO.pose),
        "{} != {}",
        d.pose,
        Odometry::ZERO.pose
    );
}
