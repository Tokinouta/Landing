using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Trig;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using ModelEntities.Enumerations;
using ModelEntities;

namespace CsharpVersion
{
    /// <summary>
    /// 姿态环
    /// </summary>
    public class AttitudeLoop : IControlModule
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;

        /// <summary>
        /// 仿真配置
        /// </summary>
        public Configuration Configuration { get; }
        readonly Plane plane;
        readonly Ship ship;

        /// <summary>
        /// 输入变量，舰载机的α、β和μ
        /// </summary>
        Vector<double> U2;

        /// <summary>
        /// 状态变量，舰载机的α、β和μ
        /// </summary>
        public Vector<double> X3;

        /// <summary>
        /// 输出变量，舰载机的p、q和r
        /// </summary>
        public Vector<double> U3 = vb.Dense(3, 0);

        // Interior Variable
        readonly Matrix<double> epsilonX3 = mb.DenseDiagonal(3, 0.707);
        readonly Matrix<double> omegaX3 = mb.DenseDiagonal(3, 40);

        // 反步法参数
        readonly Matrix<double> k3_backstepping =
            mb.DenseOfDiagonalArray(new[] { 0.9, 1, 0.7 }); // 1.0

        // 滤波器参数
        readonly int sampleNumber = 1;
        int U3FilterBufferIndex = 1;
        Matrix<double> U3FilterBuffer; // p q r

        // 观测器输出变量
        Vector<double> F3;
        Matrix<double> B3;

        // 中间变量
        Vector<double> e3;
        Vector<double> filteredU2;
        Vector<double> deriveX3 = vb.Dense(3, 0); //[theta,beta,miu]'
        Vector<double> previousU3;

        //event RecordAttitudeLoopEvent;

        /// <summary>
        /// 创建位置环类型的实例
        /// </summary>
        /// <param name="plane">舰载机对象引用</param>
        /// <param name="ship">航母对象引用</param>
        /// <param name="config">仿真配置对象引用</param>
        public AttitudeLoop(Plane plane, Ship ship, Configuration config)
        {
            Configuration = config;
            this.plane = plane;
            this.ship = ship;

            U2 = vb.Dense(new[] { plane.DesiredParameter.Alpha, 0, 0 });
            U3FilterBuffer = mb.Dense(sampleNumber, 3, 0); // p q r
            X3 = vb.Dense(new[]
                { plane.Alpha, plane.Beta, plane.Miu });
            previousU3 = U3;
            filteredU2 = U2;
            //addlistener(plane, 'X3ChangedEvent', @updateState);
        }

        /// <summary>
        /// 计算输出变量滤波
        /// </summary>
        /// <remarks>目前未使用</remarks>
        /// <param name="dt">仿真时间步长</param>
        public void CalculateFilter(double dt)
        {
            U3FilterBuffer.SetRow(U3FilterBufferIndex, U3);
            U3FilterBufferIndex++;

            if (U3FilterBufferIndex >= sampleNumber)
            {
                U3FilterBufferIndex = 1;
            }
            U3 = U3FilterBuffer.ColumnSums() / sampleNumber;
        }

        /// <summary>
        /// 计算输出变量限幅
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        public void CalculateLimiter(double dt)
        {
            if (U3[0] < plane.PRange[0])
            {
                U3[0] = plane.PRange[0];
            }
            if (U3[0] > plane.PRange[1])
            {
                U3[0] = plane.PRange[1];
            }
            if (U3[1] < plane.QRange[0])
            {
                U3[1] = plane.QRange[0];
            }
            if (U3[1] > plane.QRange[1])
            {
                U3[1] = plane.QRange[1];
            }
            if (U3[2] < plane.RRange[0])
            {
                U3[2] = plane.RRange[0];
            }
            if (U3[2] > plane.RRange[1])
            {
                U3[2] = plane.RRange[1];
            }

            // Description    : p q r变化速率限制
            var derive_u3 = (U3 - previousU3) / dt;

            if (derive_u3[0] < plane.PRateRange[0])
            {
                U3[0] = previousU3[0] + plane.PRateRange[0] * dt;
            }
            if (derive_u3[0] > plane.PRateRange[1])
            {
                U3[0] = previousU3[0] + plane.PRateRange[1] * dt;
            }
            if (derive_u3[1] < plane.QRateRange[0])
            {
                U3[1] = previousU3[1] + plane.QRateRange[0] * dt;
            }
            if (derive_u3[1] > plane.QRateRange[1])
            {
                U3[1] = previousU3[1] + plane.QRateRange[1] * dt;
            }
            if (derive_u3[2] < plane.RRateRange[0])
            {
                U3[2] = previousU3[2] + plane.RRateRange[0] * dt;
            }
            if (derive_u3[2] > plane.RRateRange[1])
            {
                U3[2] = previousU3[2] + plane.RRateRange[1] * dt;
            }
        }

        /// <summary>
        /// 计算非线性观测器参数
        /// </summary>
        /// <remarks>目前未使用</remarks>
        /// <param name="dt">仿真时间步长</param>
        /// <param name="disturbance">风场扰动</param>
        public void CalculateNonlinearObserver(double dt, Disturbance disturbance)
        {
            return;
        }

        /// <summary>
        /// 计算反步法参数
        /// </summary>
        public void CalculateObservation()
        {
            double current_alpha = plane.Alpha;
            double current_beta = plane.Beta;
            double current_gamma = plane.Gamma;
            double current_miu = plane.Miu;
            double derive_kai = plane.ChiDerive;
            double derive_gamma = plane.GammaDerive;

            F3 = vb.Dense(new[] {
                derive_gamma + (-derive_gamma * Cos(current_miu) - derive_kai * Sin(current_miu) * Cos(current_gamma)) / (Cos(current_beta)),
                -derive_gamma * Sin(current_miu) + derive_kai * Cos(current_miu) * Cos(current_gamma),
                 (derive_gamma * Sin(current_beta) * Cos(current_miu) + derive_kai * (Sin(current_gamma) * Cos(current_beta)
                     + Sin(current_beta) * Sin(current_miu) * Cos(current_gamma))) / (Cos(current_beta))});
            // 保持迎角
            B3 = mb.DenseOfArray(new[,] {
                {-Cos(current_alpha) * Tan(current_beta), 1 / Cos(current_beta), -Sin(current_alpha) * Tan(current_beta) },
                { Sin(current_alpha), 0, -Cos(current_alpha) },
                { Cos(current_alpha) / Cos(current_beta), 0, Sin(current_alpha) / Cos(current_beta) }});
        }

        /// <summary>
        /// 计算输出变量
        /// </summary>
        public void CalculateOutput()
        {
            if (Configuration.AttitudeController == AttitudeConfig.IDLC)
            {
                // 直接升力控制
                switch (Configuration.DisturbanceObserver)
                {
                    case DisturbanceObserverConfig.NDO:
                        U3 = B3.Inverse() * (-F3 + k3_backstepping * e3 + deriveX3); //[p, q, r] nonlinear disturbance observer attitude环不使用DO
                        break;
                    case DisturbanceObserverConfig.FTO:
                        break;
                    case DisturbanceObserverConfig.HOSMO:
                        break;
                    case DisturbanceObserverConfig.ESO:
                        break;
                    case DisturbanceObserverConfig.NONE:
                        U3 = B3.Inverse() * (-F3 + k3_backstepping * e3 + deriveX3); //[p, q, r] 不使用DO
                        break;
                    default:
                        Console.WriteLine("请指定干扰观测器种类 id 34");
                        break;
                }
            }
            else
            {
                Console.WriteLine("请指定控制器种类 id 32");
                return;
            }
        }

        /// <summary>
        /// 计算状态变量和误差
        /// </summary>
        /// <param name="dt"></param>
        /// <param name="input"></param>
        [Obsolete("旧版控制器变量计算函数", true)]
        public void CalculateState(double dt, Vector<double> input)
        { }
        /// <summary>
        /// 计算状态变量和误差
        /// </summary>
        /// <param name="dt"></param>
        /// <param name="input"></param>
        /// <param name="derive"></param>
        public void CalculateState(double dt, Vector<double> input, double derive = 0)
        {
            U2 = input;

            switch (Configuration.AttitudeFilter)// 判断使用何种滤波器
            {
                case AttitudeFilters.Command:  // 使用指令滤波器
                    var derive2_X3 = -2 * epsilonX3 * omegaX3 * deriveX3 - omegaX3.Power(2) * (filteredU2 - U2);
                    deriveX3 += derive2_X3 * dt;
                    filteredU2 += deriveX3 * dt;
                    deriveX3[0] = derive;
                    e3 = filteredU2 - X3;
                    previousU3 = U3;
                    break;
                default:
                    Console.WriteLine("请指定滤波器种类 id 31");
                    break;
            }
        }

        /// <summary>
        /// 记录位置环变量
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        [Obsolete("这个函数本是用于移植旧版本的记录，现在发现控制器变量暂时不用记录")]
        public void Record(double dt)
        {
            //ev = XChangedEventArgs(dt, e3, derive_X3, previous_u3);
            //notify(obj, "RecordAttitudeLoopEvent", ev);
        }

        /// <summary>
        /// 重置控制器，将相关变量恢复至初始状态
        /// </summary>
        public void Reset()
        {
            U3FilterBufferIndex = 1;
            U2 = vb.Dense(new[] { plane.DesiredParameter.Alpha, 0, 0 });
            U3 = vb.Dense(new double[] { 0, 0, 0 });
            U3FilterBuffer = mb.Dense(sampleNumber, 3, 0); // p q r
            X3 = vb.Dense(new[]
                { plane.Alpha, plane.Beta, plane.Miu });
            previousU3 = U3;
            filteredU2 = U2;

            deriveX3 = vb.Dense(3, 0); //[theta,beta,miu]'
        }

        /// <summary>
        /// 更新位置环状态变量
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        /// <param name="disturbance">仿真配置对象</param>
        [Obsolete("这个函数本是用于移植旧版本的更新状态，请用OnUpdateState", true)]
        public void UpdateState(double dt, Disturbance disturbance)
        {
            // plane.current_v = [Sin(plane.current_miu), plane.current_alpha * Cos(plane.current_miu)]';
            //dt = e.data{ 1};
            //current_X3_dot = e.data{ 2};
            //current_X3 = current_X3 + current_X3_dot * dt;
        }

        /// <summary>
        /// 更新位置环状态变量
        /// </summary>
        /// <param name="sender">发送数据的对象</param>
        /// <param name="e">接受的数据，用于计算更新</param>
        public void OnUpdateState(object sender, XChangedEventArgs e)
        {
            X3 += e.Data * e.Dt;
        }
    }
}
