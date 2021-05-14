using MathNet.Numerics.LinearAlgebra;
using ModelEntities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    /// <summary>
    /// <para>仿真控制器接口，创建的控制器都需要实现下面的函数</para>
    /// <para>用于控制一下控制器格式</para>
    /// </summary>
    interface IControlModule
    {
        /// <summary>
        /// 仿真配置
        /// </summary>
        Configuration Configuration { get; }

        /// <summary>
        /// 计算状态变量和误差
        /// </summary>
        /// <param name="dt"></param>
        /// <param name="input"></param>
        void CalculateState(double dt, Vector<double> input);

        /// <summary>
        /// 计算反步法参数
        /// </summary>
        void CalculateObservation();

        /// <summary>
        /// 计算非线性观测器参数
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        /// <param name="disturbance">风场扰动</param>
        void CalculateNonlinearObserver(double dt, Disturbance disturbance);

        /// <summary>
        /// 计算输出变量
        /// </summary>
        void CalculateOutput();

        /// <summary>
        /// 计算输出变量限幅
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        void CalculateLimiter(double dt);

        /// <summary>
        /// 计算输出变量滤波
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        void CalculateFilter(double dt);

        /// <summary>
        /// 更新状态变量
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        /// <param name="disturbance">仿真配置对象</param>
        [Obsolete("这个函数本是用于移植旧版本的更新状态，请用OnUpdateState", true)]
        void UpdateState(double dt, Disturbance disturbance);

        /// <summary>
        /// 记录变量
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        [Obsolete("这个函数本是用于移植旧版本的记录，现在发现控制器变量暂时不用记录")]
        void Record(double dt);

        /// <summary>
        /// 重置控制器，将相关变量恢复至初始状态
        /// </summary>
        void Reset();
    }
}
