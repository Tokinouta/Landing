using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Trig;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    public class AttitudeLoop : IControlModule
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;
        Plane plane;
        Ship ship;
        
        // Input Variable
        Vector<double> U2;

        // State Variable
        public Vector<double> X3;

        // Output Variable
        public Vector<double> U3 = vb.Dense(3, 0);

        // Interior Variable
        Matrix<double> epsilonX3 = mb.DenseDiagonal(3, 0.707);
        Matrix<double> omegaX3 = mb.DenseDiagonal(3, 40);

        // 反步法参数
        Matrix<double> k3_backstepping =
            mb.DenseOfDiagonalArray(new[] { 0.9, 1, 0.7 }); // 1.0

        // 滤波器参数
        int sampleNumber = 1;
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

        public AttitudeLoop(Plane plane, Ship ship)
        {
            this.plane = plane;
            this.ship = ship;

            U2 = vb.Dense(new[] { plane.DesiredParameter.AlphaDesired, 0, 0 });
            U3FilterBuffer = mb.Dense(sampleNumber, 3, 0); // p q r
            X3 = vb.Dense(new[]
                { plane.Alpha, plane.Beta, plane.Miu });
            previousU3 = U3;
            filteredU2 = U2;
            //addlistener(plane, 'X3ChangedEvent', @updateState);
        }

        public void CalculateFilter(double dt)
        {
            U3FilterBuffer.SetRow(U3FilterBufferIndex, U3);
            U3FilterBufferIndex++;

            if (U3FilterBufferIndex >= sampleNumber)
            {
                U3FilterBufferIndex = 1;
            }

            //current_u3(1) = sum(current_u3_index(:, 1)) / sample_num_u3; // theta
            //current_u3(2) = sum(current_u3_index(:, 2)) / sample_num_u3; // beta
            //current_u3(3) = sum(current_u3_index(:, 3)) / sample_num_u3; // miu
            U3 = U3FilterBuffer.ColumnSums() / sampleNumber;
        }

        public void CalculateLimiter(double dt)
        {
            //p_range = plane.p_range;
            //q_range = plane.q_range;
            //r_range = plane.r_range;
            //p_rate_range = plane.p_rate_range;
            //q_rate_range = plane.q_rate_range;
            //r_rate_range = plane.r_rate_range;

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
            // p q r变化速率限制
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

        public void CalculateNonlinearObserver(double dt, Disturbance disturbance)
        {
            return;
        }

        public void CalculateObservation()
        {
            double current_alpha = plane.Alpha;
            double current_beta = plane.Beta;
            double current_gamma = plane.Gamma;
            double current_miu = plane.Miu;
            double derive_kai = plane.KaiDerive;
            double derive_gamma = plane.GammaDerive;

            F3 = vb.Dense(new[] {
                derive_gamma + (-derive_gamma * Cos(current_miu) - derive_kai * Sin(current_miu) * Cos(current_gamma)) / (Cos(current_beta)),
                -derive_gamma * Sin(current_miu) + derive_kai * Cos(current_miu) * Cos(current_gamma),
                 (derive_gamma * Sin(current_beta) * Cos(current_miu) + derive_kai * (Sin(current_gamma) * Cos(current_beta)
                     + Sin(current_beta) * Sin(current_miu) * Cos(current_gamma))) / (Cos(current_beta))});
            // 保持迎角
            B3 = mb.DenseOfArray(new[,] {
                {-Cos(current_alpha) * Tan(current_beta), 1, -Sin(current_alpha) * Tan(current_beta) },
                { Sin(current_alpha), 0, -Cos(current_alpha) },
                { Cos(current_alpha) / Cos(current_beta), 0, Sin(current_alpha) / Cos(current_beta) }});
        }

        public void CalculateOutput()
        {
            if (Configuration.AttitudeController == AttitudeConfig.IDLC)
            {
                // 直接升力控制
                if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NDO)
                {
                    U3 = B3.Inverse() * (-F3 + k3_backstepping * e3 + deriveX3); //[p, q, r] nonlinear disturbance observer attitude环不使用DO
                }
                else if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NONE)
                {
                    U3 = B3.Inverse() * (-F3 + k3_backstepping * e3 + deriveX3); //[p, q, r] 不使用DO
                }
                else
                {
                    Console.WriteLine("请指定干扰观测器种类 id 34");
                    return;
                }
            }
            else
            {
                Console.WriteLine("请指定控制器种类 id 32");
                return;
            }
        }

        public void CalculateState(double dt, Vector<double> input)
        {
            U2 = input;

            if (Configuration.attitude_command_filter_flag)// 判断使用何种滤波器
            {
                // 使用指令滤波器
                var derive2_X3 = -2 * epsilonX3 * omegaX3 * deriveX3 - omegaX3.Power(2) * (filteredU2 - U2);
                deriveX3 += derive2_X3 * dt;
                filteredU2 += deriveX3 * dt;
                e3 = filteredU2 - X3;
                previousU3 = U3;
            }
            else
            {
                Console.WriteLine("请指定滤波器种类 id 31");
            }
        }

        public void calculateState(double dt, Vector<double> input, Plane plane)
        {
            throw new NotImplementedException();
        }

        public void Record(double dt)
        {
            //ev = XChangedEventArgs(dt, e3, derive_X3, previous_u3);
            //notify(obj, "RecordAttitudeLoopEvent", ev);
        }

        public void Reset()
        {
            U3FilterBufferIndex = 1;
            U2 = vb.Dense(new[] { plane.DesiredParameter.AlphaDesired, 0, 0 });
            U3FilterBuffer = mb.Dense(sampleNumber, 3, 0); // p q r
            X3 = vb.Dense(new[]
                { plane.Alpha, plane.Beta, plane.Miu });
            previousU3 = U3;
            filteredU2 = U2;

            deriveX3 = vb.Dense(3, 0); //[theta,beta,miu]'
        }

        public void UpdateState(double dt, Disturbance disturbance)
        {
            // plane.current_v = [Sin(plane.current_miu), plane.current_alpha * Cos(plane.current_miu)]';
            //dt = e.data{ 1};
            //current_X3_dot = e.data{ 2};
            //current_X3 = current_X3 + current_X3_dot * dt;
        }

        public void OnUpdateState(object sender, XChangedEventArgs e)
        {
            X3 += e.Data * e.Dt;
        }
    }
}
