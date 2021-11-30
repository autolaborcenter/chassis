use crate::Velocity;
use nalgebra::{ArrayStorage, Complex, Isometry2, SVector, Translation, Unit, Vector2};
use std::fmt::Display;

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
    pub pose: Isometry2<f32>,
}

impl Odometry {
    /// 里程计零，常用于初始化
    pub const ZERO: Self = Self {
        s: 0.0,
        a: 0.0,
        pose: Isometry2 {
            translation: Translation {
                vector: SVector::from_array_storage(ArrayStorage([[0.0, 0.0]])),
            },
            rotation: Unit::new_unchecked(Complex { re: 1.0, im: 0.0 }),
        },
    };
}

/// 从单位时间内 Velocity 表示的圆轨迹生成里程增量
impl From<Velocity> for Odometry {
    fn from(vel: Velocity) -> Self {
        let Velocity { v: s, w: theta } = vel;
        let a = theta.abs();

        Self {
            s: s.abs(),
            a,
            pose: Isometry2::new(
                if a < f32::EPSILON {
                    Vector2::new(s, 0.0)
                } else {
                    Vector2::new(theta.sin(), 1.0 - theta.cos()) * (s / theta)
                },
                theta,
            ),
        }
    }
}

/// 重载 `+=`。
impl std::ops::AddAssign for Odometry {
    fn add_assign(&mut self, rhs: Self) {
        self.s += rhs.s;
        self.a += rhs.a;
        // 位姿的叠加在 SE(2) 中用乘法表示
        self.pose *= rhs.pose;
    }
}

/// 重载 +。
///
/// `self` + `rhs` 即从 `self` 出发再行驶 `rhs`。
impl std::ops::Add for Odometry {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        let mut copy = self.clone();
        copy += rhs;
        copy
    }
}

impl Display for Odometry {
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

#[cfg(test)]
mod tests {
    use super::*;
    use std::f32::{consts::PI, EPSILON};

    #[inline]
    fn pose_equal(a: Isometry2<f32>, b: Isometry2<f32>) -> bool {
        let mut result = (a.translation.vector[0] - b.translation.vector[0]).abs() <= EPSILON;
        result = result || (a.translation.vector[1] - b.translation.vector[1]).abs() <= EPSILON;
        result = result || (a.rotation.angle() - b.rotation.angle()).abs() <= EPSILON;
        result
    }

    #[test]
    fn odometry_test() {
        //测试里程计原点及输出是否正确
        let mut od = Odometry::ZERO;
        assert_eq!(
            format!("{}", od),
            "Odometry: { s: 0.000, a: 0.0°, x: 0.000, y: 0.000, theta: 0.0° }"
        );
        //测试机器人从原点出发，行进一整个圆是否会回到原点
        let radius = 100.0;
        let circumference = 2.0 * PI * radius;
        let step_num = 10.0 as f32;
        let delta_vel = Velocity {
            v: (circumference / step_num),
            w: (PI * 2.0 / step_num),
        };
        let delta_od = Odometry::from(delta_vel);

        for _ in 0..10 {
            od += delta_od;
        }
        assert!(
            pose_equal(od.pose, Odometry::ZERO.pose),
            "{} != {}",
            od.pose,
            Odometry::ZERO.pose
        );
    }
}
