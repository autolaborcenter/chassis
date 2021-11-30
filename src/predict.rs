use crate::{ChassisModel, Odometry};
use std::time::Duration;

/// 状态预测器
///
/// 利用给定的机器人参数，根据当前状态和目标状态预测下一周期机器人的状态
pub trait StatusPredictor<S>: Clone + Iterator<Item = S> {}

/// 轨迹预测器
///
/// 利用给定的机器人参数，根据当前状态和目标状态预测一个周期中机器人里程的增量
#[derive(Clone)]
pub struct TrajectoryPredictor<M, P> {
    pub period: Duration, // 控制周期
    pub model: M,         // 机器人模型
    pub predictor: P,     // 状态预测器
}

impl<M, P> Iterator for TrajectoryPredictor<M, P>
where
    M: ChassisModel,
    P: StatusPredictor<M::State>,
{
    type Item = Odometry;

    /// 预测下一周期以 [`Duration`] 表示的时间**增量**和以 [`Odometry`] 表示的里程**增量**
    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        self.predictor
            .next()
            .map(|s| self.model.volocity_from(&s) * self.period)
    }
}
