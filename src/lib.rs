mod odometry;
mod predict;

pub use odometry::Odometry;

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
