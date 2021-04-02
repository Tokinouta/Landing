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
        Vector<double> current_u2;

        // State Variable
        Vector<double> current_X3;

        // Output Variable
        Vector<double> current_u3 = vb.Dense(3, 0);

        // Interior Variable
        Matrix<double> epsilon_X3 = mb.DenseDiagonal(3, 0.7);
        Matrix<double> omega_X3 = mb.DenseDiagonal(3, 40);

        // 反步法参数
        Matrix<double> k3_backstepping = mb.DenseIdentity(3); // 1.0

        // 滤波器参数
        int sample_num_u3 = 1;
        int current_u3_index_count = 1;
        Matrix<double> current_u3_index; // p q r

        // 观测器输出变量
        Vector<double> F3;
        Matrix<double> B3;

        // 中间变量
        Vector<double> e3;
        Vector<double> filter_u2;
        Vector<double> derive_X3 = vb.Dense(3, 0); //[theta,beta,miu]'
        Vector<double> previous_u3;

        //event RecordAttitudeLoopEvent;

        public AttitudeLoop(Plane plane, Ship ship)
        {
            this.plane = plane;
            this.ship = ship;

            current_u2 = vb.Dense(new[] { plane.desired_alpha, 0, 0 });
            current_u3_index = mb.Dense(sample_num_u3, 3, 0); // p q r
            current_X3 = vb.Dense(new[]
                { plane.current_alpha, plane.current_beta, plane.current_miu });
            previous_u3 = current_u3;
            filter_u2 = current_u2;
            //addlistener(plane, 'X3ChangedEvent', @updateState);
        }

        public void calculateFilter(double dt)
        {
            current_u3_index.SetRow(current_u3_index_count, current_u3);
            current_u3_index_count++;

            if (current_u3_index_count >= sample_num_u3)
            {
                current_u3_index_count = 1;
            }

            //current_u3(1) = sum(current_u3_index(:, 1)) / sample_num_u3; // theta
            //current_u3(2) = sum(current_u3_index(:, 2)) / sample_num_u3; // beta
            //current_u3(3) = sum(current_u3_index(:, 3)) / sample_num_u3; // miu
            current_u3 = current_u3_index.ColumnSums() / sample_num_u3;
        }

        public void calculateLimiter(double dt)
        {
            //p_range = plane.p_range;
            //q_range = plane.q_range;
            //r_range = plane.r_range;
            //p_rate_range = plane.p_rate_range;
            //q_rate_range = plane.q_rate_range;
            //r_rate_range = plane.r_rate_range;

            if (current_u3[0] < plane.p_range[0])
            {
                current_u3[0] = plane.p_range[0];
            }
            if (current_u3[0] > plane.p_range[1])
            {
                current_u3[0] = plane.p_range[1];
            }
            if (current_u3[1] < plane.q_range[0])
            {
                current_u3[1] = plane.q_range[0];
            }
            if (current_u3[1] > plane.q_range[1])
            {
                current_u3[1] = plane.q_range[1];
            }
            if (current_u3[2] < plane.r_range[0])
            {
                current_u3[2] = plane.r_range[0];
            }
            if (current_u3[2] > plane.r_range[1])
            {
                current_u3[2] = plane.r_range[1];
            }

            // Description    : p q r变化速率限制
            // p q r变化速率限制
            var derive_u3 = (current_u3 - previous_u3) / dt;

            if (derive_u3[0] < plane.p_rate_range[0])
            {
                current_u3[0] = previous_u3[0] + plane.p_rate_range[0] * dt;
            }
            if (derive_u3[0] > plane.p_rate_range[1])
            {
                current_u3[0] = previous_u3[0] + plane.p_rate_range[1] * dt;
            }
            if (derive_u3[1] < plane.q_rate_range[0])
            {
                current_u3[1] = previous_u3[1] + plane.q_rate_range[0] * dt;
            }
            if (derive_u3[1] > plane.q_rate_range[1])
            {
                current_u3[1] = previous_u3[1] + plane.q_rate_range[1] * dt;
            }
            if (derive_u3[2] < plane.r_rate_range[0])
            {
                current_u3[2] = previous_u3[2] + plane.r_rate_range[0] * dt;
            }
            if (derive_u3[2] > plane.r_rate_range[1])
            {
                current_u3[2] = previous_u3[2] + plane.r_rate_range[1] * dt;
            }
        }

        public void calculateNonlinearObserver(double dt, Disturbance disturbance)
        {
            return;
        }

        public void calculateObservation()
        {
            double current_alpha = plane.current_alpha;
            double current_beta = plane.current_beta;
            double current_gamma = plane.current_gamma;
            double current_miu = plane.current_miu;
            double derive_kai = plane.derive_kai;
            double derive_gamma = plane.derive_gamma;

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

        public void calculateOutput()
        {
            if (Configuration.AttitudeController == AttitudeConfig.IDLC)
            {
                // 直接升力控制
                if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NDO)
                {
                    current_u3 = B3.Inverse() * (-F3 + k3_backstepping * e3 + derive_X3); //[p, q, r] nonlinear disturbance observer attitude环不使用DO
                }
                else if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NONE)
                {
                    current_u3 = B3.Inverse() * (-F3 + k3_backstepping * e3 + derive_X3); //[p, q, r] 不使用DO
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

        public void calculateState(double dt, Vector<double> input)
        {
            current_u2 = input;

            if (Configuration.attitude_command_filter_flag)// 判断使用何种滤波器
            {
                // 使用指令滤波器
                var derive2_X3 = -2 * epsilon_X3 * omega_X3 * derive_X3 - omega_X3.Power(2) * (filter_u2 - current_u2);
                derive_X3 += derive2_X3 * dt;
                filter_u2 += derive_X3 * dt;
                e3 = filter_u2 - current_X3;
                previous_u3 = current_u3;
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

        public void record()
        {
            //ev = XChangedEventArgs(dt, e3, derive_X3, previous_u3);
            //notify(obj, "RecordAttitudeLoopEvent", ev);
        }

        public void reset()
        {
            current_u3_index_count = 1;
            current_u2 = vb.Dense(new[] { plane.desired_alpha, 0, 0 });
            current_u3_index = mb.Dense(sample_num_u3, 3, 0); // p q r
            current_X3 = vb.Dense(new[]
                { plane.current_alpha, plane.current_beta, plane.current_miu });
            previous_u3 = current_u3;
            filter_u2 = current_u2;

            derive_X3 = vb.Dense(3, 0); //[theta,beta,miu]'
        }

        public void updateState(double dt, Disturbance disturbance)
        {
            // plane.current_v = [Sin(plane.current_miu), plane.current_alpha * Cos(plane.current_miu)]';
            //dt = e.data{ 1};
            //current_X3_dot = e.data{ 2};
            //current_X3 = current_X3 + current_X3_dot * dt;
        }
    }
}
