mod odometry;
mod predict;

use std::time::Duration;

pub use nalgebra::Isometry2;
pub use odometry::Odometry;
pub use predict::{StatusPredictor, TrajectoryPredictor};

/// 底盘模型
pub trait ChassisModel {
    /// `State` 应该完备地表征底盘运动状态
    type State;

    /// 根据一个状态估计底盘转动中心相对地面的速度
    fn volocity_from(&self, s: &Self::State) -> Velocity;
}

/// 刚体速度模型
#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Velocity {
    /// 旋转中心相对地面线速度 m/s
    pub v: f32,
    /// 旋转中心相对地面角速度 rad/s
    pub w: f32,
}

impl std::ops::Mul<Duration> for Velocity {
    type Output = Odometry;

    fn mul(self, rhs: Duration) -> Self::Output {
        let Velocity {
            v: mut s,
            w: mut theta,
        } = self;
        let seconds = rhs.as_secs_f32();
        s *= seconds;
        theta *= seconds;
        let a = theta.abs();
        let (sin, cos) = theta.sin_cos();
        Self::Output {
            s: s.abs(),
            a,
            pose: if a < f32::EPSILON {
                isometry(s, 0.0, cos, sin)
            } else {
                let radius = s / theta;
                isometry(radius * sin, radius * (1.0 - cos), cos, sin)
            },
        }
    }
}

#[inline]
pub const fn isometry(x: f32, y: f32, cos: f32, sin: f32) -> Isometry2<f32> {
    use nalgebra::{ArrayStorage, Complex, SVector, Translation, Unit};

    Isometry2 {
        translation: Translation {
            vector: SVector::from_array_storage(ArrayStorage([[x, y]])),
        },
        rotation: Unit::new_unchecked(Complex { re: cos, im: sin }),
    }
}
