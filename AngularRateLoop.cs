using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Constants;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    public class AngularRateLoop : IControlModule
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;
        Plane plane;
        Ship ship;

        // Input Variable
        Vector<double> current_u3 = vb.Dense(3, 0);

        // State Variable
        public Vector<double> current_X4;

        // Output Variable
        public Vector<double> current_Uact;

        // Interior Variable
        Matrix<double> epsilon_X4 = mb.DiagonalIdentity(3) * 0.7; // 增大阻尼同时减小频率
        Matrix<double> omega_X4 = mb.DiagonalIdentity(3) * 40;

        // 反步法参数
        Matrix<double> k4_backstepping = mb.DiagonalIdentity(3) * 2; // 1.0

        // 滤波器参数
        int sample_num_rudder = 1; // 3
        int current_Uact_index_count = 0;
        Matrix<double> current_Uact_index; // delta_a delta_e delta_r
        public Vector<double> filter_Uact;
        Vector<double> filter_Uact_previous;

        // 观测器输出变量
        Vector<double> F4;
        Matrix<double> B4;

        // Nonlinear Observer
        Vector<double> current_NDO_p_omega = vb.Dense(3, 0);
        Vector<double> NDO_d_omega_output;

        // 中间变量
        Vector<double> e4;
        Vector<double> filter_u3;
        Vector<double> derive_X4 = vb.Dense(3, 0); //[p,q,r]'
        Vector<double> previous_Uact;

        //event RecordAngularRateLoopEvent;
        //event RecordAngularRateLoopVarEvent

        public AngularRateLoop(Plane plane, Ship ship)
        {
            this.plane = plane;
            this.ship = ship;

            current_X4 = vb.Dense(new[]
                { plane.P, plane.Q, plane.R });
            current_Uact = vb.Dense(new[]
                { plane.DeltaA, plane.DeltaE, plane.DeltaR });

            current_Uact_index = mb.Dense(sample_num_rudder, 3, 0);
            filter_Uact = current_Uact;
            filter_Uact_previous = filter_Uact;
            filter_u3 = current_u3;
            previous_Uact = current_Uact;
            //addlistener(plane, 'X4ChangedEvent', @updateState);
        }

        public void calculateFilter(double dt)
        {
            filter_Uact_previous = filter_Uact;
            //filter_Uact = 1 / 48 / (1 / 48 + dt * sample_num_rudder) * filter_Uact_previous
            //    + dt * sample_num_rudder / (1 / 48 + dt * sample_num_rudder) * current_Uact;
            filter_Uact[0] = 1 / 48.0 / (1 / 48.0 + dt * sample_num_rudder) * filter_Uact_previous[0]
                + dt * sample_num_rudder / (1 / 48.0 + dt * sample_num_rudder) * current_Uact[0];
            filter_Uact[1] = 1 / 30.0 / (1 / 30.0 + dt * sample_num_rudder) * filter_Uact_previous[1]
                + dt * sample_num_rudder / (1 / 30.0 + dt * sample_num_rudder) * current_Uact[1];
            filter_Uact[2] = 1 / 40.0 / (1 / 40.0 + dt * sample_num_rudder) * filter_Uact_previous[2]
                + dt * sample_num_rudder / (1 / 40.0 + dt * sample_num_rudder) * current_Uact[2];

            current_Uact_index.SetRow(current_Uact_index_count, filter_Uact);
            current_Uact_index_count++;

            if (current_Uact_index_count >= sample_num_rudder)
            {
                current_Uact_index_count = 0;
            }

            filter_Uact = current_Uact_index.ColumnSums() / sample_num_rudder;
            //filter_Uact(1) = sum(current_Uact_index(:, 1)) / sample_num_rudder;
            //filter_Uact(2) = sum(current_Uact_index(:, 2)) / sample_num_rudder;
            //filter_Uact(3) = sum(current_Uact_index(:, 3)) / sample_num_rudder;
            plane.DeltaA = filter_Uact[0];
            plane.DeltaE = filter_Uact[1];
            plane.DeltaR = filter_Uact[2];
        }

        public void calculateLimiter(double dt)
        {
            //delta_e_range = plane.delta_e_range;
            //delta_e_rate_range = plane.delta_e_rate_range;
            //delta_a_range = plane.delta_a_range;
            //delta_a_rate_range = plane.delta_a_rate_range;
            //delta_r_range = plane.delta_r_range;
            //delta_r_rate_range = plane.delta_r_rate_range;
            int step_count = 0;
            // 舵面偏转角度限幅
            if (current_Uact[0] < plane.DeltaARange[0])
            {
                Console.WriteLine($"Aileron over range bottom, {step_count}");
                current_Uact[0] = plane.DeltaARange[0];
            }
            if (current_Uact[0] > plane.DeltaARange[1])
            {
                Console.WriteLine($"Aileron over range top, {step_count}");
                current_Uact[0] = plane.DeltaARange[1];
            }
            if (current_Uact[1] < plane.DeltaERange[0])
            {
                Console.WriteLine($"Elevator over range bottom, {step_count}");
                current_Uact[1] = plane.DeltaERange[0];
            }
            if (current_Uact[1] > plane.DeltaERange[1])
            {
                Console.WriteLine($"Elevator over range top, {step_count}");
                current_Uact[1] = plane.DeltaERange[1];
            }
            if (current_Uact[2] < plane.DeltaRRange[0])
            {
                Console.WriteLine($"Rudder over range bottom, {step_count}");
                current_Uact[2] = plane.DeltaRRange[0];
            }
            if (current_Uact[2] > plane.DeltaRRange[1])
            {
                Console.WriteLine($"Rudder over range top, {step_count}");
                current_Uact[2] = plane.DeltaRRange[1];
            }

            // 舵面偏转角速度限制
            var derive_Uact = (current_Uact - previous_Uact) / dt;

            if (derive_Uact[0] < plane.DeltaARateRange[0])
            {
                current_Uact[0] = previous_Uact[0] + plane.DeltaARateRange[0] * dt;
            }
            if (derive_Uact[0] > plane.DeltaARateRange[1])
            {
                current_Uact[0] = previous_Uact[0] + plane.DeltaARateRange[1] * dt;
            }
            if (derive_Uact[1] < plane.DeltaERateRange[0])
            {
                current_Uact[1] = previous_Uact[1] + plane.DeltaERateRange[0] * dt;
            }
            if (derive_Uact[1] > plane.DeltaERateRange[1])
            {
                current_Uact[1] = previous_Uact[1] + plane.DeltaERateRange[1] * dt;
            }
            if (derive_Uact[2] < plane.DeltaRRateRange[0])
            {
                current_Uact[2] = previous_Uact[2] + plane.DeltaRRateRange[0] * dt;
            }
            if (derive_Uact[2] > plane.DeltaRRateRange[1])
            {
                current_Uact[2] = previous_Uact[2] + plane.DeltaRRateRange[1] * dt;
            }
        }

        public void calculateLimiter(double dt, int step_count)
        {
            //delta_e_range = plane.delta_e_range;
            //delta_e_rate_range = plane.delta_e_rate_range;
            //delta_a_range = plane.delta_a_range;
            //delta_a_rate_range = plane.delta_a_rate_range;
            //delta_r_range = plane.delta_r_range;
            //delta_r_rate_range = plane.delta_r_rate_range;
            // 舵面偏转角度限幅
            if (current_Uact[0] < plane.DeltaARange[0])
            {
                Console.WriteLine($"Aileron over range bottom, {step_count}");
                current_Uact[0] = plane.DeltaARange[0];
            }
            if (current_Uact[0] > plane.DeltaARange[1])
            {
                Console.WriteLine($"Aileron over range top, {step_count}");
                current_Uact[0] = plane.DeltaARange[1];
            }
            if (current_Uact[1] < plane.DeltaERange[0])
            {
                Console.WriteLine($"Elevator over range bottom, {step_count}");
                current_Uact[1] = plane.DeltaERange[0];
            }
            if (current_Uact[1] > plane.DeltaERange[1])
            {
                Console.WriteLine($"Elevator over range top, {step_count}");
                current_Uact[1] = plane.DeltaERange[1];
            }
            if (current_Uact[2] < plane.DeltaRRange[0])
            {
                Console.WriteLine($"Rudder over range bottom, {step_count}");
                current_Uact[2] = plane.DeltaRRange[0];
            }
            if (current_Uact[2] > plane.DeltaRRange[1])
            {
                Console.WriteLine($"Rudder over range top, {step_count}");
                current_Uact[2] = plane.DeltaRRange[1];
            }

            // 舵面偏转角速度限制
            var derive_Uact = (current_Uact - previous_Uact) / dt;

            if (derive_Uact[0] < plane.DeltaARateRange[0])
            {
                current_Uact[0] = previous_Uact[0] + plane.DeltaARateRange[0] * dt;
            }
            if (derive_Uact[0] > plane.DeltaARateRange[1])
            {
                current_Uact[0] = previous_Uact[0] + plane.DeltaARateRange[1] * dt;
            }
            if (derive_Uact[1] < plane.DeltaERateRange[0])
            {
                current_Uact[1] = previous_Uact[1] + plane.DeltaERateRange[0] * dt;
            }
            if (derive_Uact[1] > plane.DeltaERateRange[1])
            {
                current_Uact[1] = previous_Uact[1] + plane.DeltaERateRange[1] * dt;
            }
            if (derive_Uact[2] < plane.DeltaRRateRange[0])
            {
                current_Uact[2] = previous_Uact[2] + plane.DeltaRRateRange[0] * dt;
            }
            if (derive_Uact[2] > plane.DeltaRRateRange[1])
            {
                current_Uact[2] = previous_Uact[2] + plane.DeltaRRateRange[1] * dt;
            }

        }

        public void calculateNonlinearObserver(double dt, Disturbance disturbance)
        {
            Matrix<double> NDO_l_omega = mb.DenseOfDiagonalArray(new double[] { 10, 20, 10 });

            Vector<double> previous_NDO_p_omega = current_NDO_p_omega;
            Vector<double> derive_NDO_p_omega = -NDO_l_omega * (NDO_l_omega * current_X4 + current_NDO_p_omega + F4 + B4 * current_Uact);
            current_NDO_p_omega = previous_NDO_p_omega + derive_NDO_p_omega * dt;
            Vector<double> NDO_d_omega = current_NDO_p_omega + NDO_l_omega * current_X4;

            if (disturbance.wind_disturbance_start)
            {
                // NDO_d_omega_output = NDO_d_gamma_b2f;
                NDO_d_omega_output = NDO_d_omega;
            }
            else
            {
                NDO_d_omega_output = vb.Dense(3, 0);
            }
            //ev = XChangedEventArgs(NDO_d_omega);
            //notify(obj, "RecordAngularRateLoopVarEvent", ev);
        }

        public void calculateObservation()
        {
            double F4_1 = 1 / (Plane.Ixx * Plane.Izz - Math.Pow(Plane.Ixz, 2))
                * ((Plane.Iyy * Plane.Izz - Math.Pow(Plane.Izz, 2) - Math.Pow(Plane.Ixz, 2)) * plane.R * plane.Q
                + (Plane.Ixx * Plane.Ixz + Plane.Izz * Plane.Ixz - Plane.Iyy * Plane.Ixz) * plane.P * plane.Q
                + Plane.Izz * (plane.L - plane.Flow * Plane.WingS * Plane.WingL
                * (plane.CL_delta_a * plane.DeltaA + plane.CL_delta_r * plane.DeltaR))
                + Plane.Ixz * (plane.N - plane.Flow * Plane.WingS * Plane.WingL
                * (plane.CN_delta_r * plane.DeltaR + plane.CN_delta_a * plane.DeltaA)));
            double F4_2 = 1 / Plane.Iyy * ((Plane.Izz - Plane.Ixx) * plane.P * plane.R
                - Plane.Ixz * Math.Pow(plane.P, 2) + Plane.Ixz * Math.Pow(plane.R, 2)
                + (plane.M - plane.Flow * Plane.WingS * Plane.WingC
                * (plane.CM_delta_e * plane.DeltaE)));
            double F4_3 = 1 / (Plane.Ixx * Plane.Izz - Math.Pow(Plane.Ixz, 2))
                * ((Math.Pow(Plane.Ixx, 2) + Math.Pow(Plane.Ixz, 2) - Plane.Ixx * Plane.Iyy) * plane.P * plane.Q
                + (Plane.Iyy * Plane.Ixz - Plane.Ixx * Plane.Ixz - Plane.Izz * Plane.Ixz) * plane.Q * plane.R
                + Plane.Ixz * (plane.L - plane.Flow * Plane.WingS * Plane.WingL
                * (plane.CL_delta_a * plane.DeltaA + plane.CL_delta_r * plane.DeltaR))
                + Plane.Ixx * (plane.N - plane.Flow * Plane.WingS * Plane.WingL
                * (plane.CN_delta_r * plane.DeltaR + plane.CN_delta_a * plane.DeltaA)));
            F4 = vb.Dense(new[] { F4_1, F4_2, F4_3 });
            B4 = plane.Flow * Plane.WingS * mb.DenseOfArray(new[,] {
                { Plane.WingL * (Plane.Izz * plane.CL_delta_a + Plane.Ixz * plane.CN_delta_a) / (Plane.Ixx * Plane.Izz - Math.Pow(Plane.Ixz, 2)), 0,
                    Plane.WingL * (Plane.Izz * plane.CL_delta_r + Plane.Ixz * plane.CN_delta_r) / (Plane.Ixx * Plane.Izz - Math.Pow(Plane.Ixz, 2))},
                { 0, Plane.WingC* plane.CM_delta_e / Plane.Iyy, 0 },
                { Plane.WingL * (Plane.Ixz * plane.CL_delta_a + Plane.Ixx * plane.CN_delta_a) / (Plane.Ixx * Plane.Izz - Math.Pow(Plane.Ixz, 2)), 0,
                    Plane.WingL * (Plane.Ixz * plane.CL_delta_r + Plane.Ixx * plane.CN_delta_r) / (Plane.Ixx * Plane.Izz - Math.Pow(Plane.Ixz, 2)) } });
        }

        public void calculateOutput()
        {
            if (Configuration.AttitudeController == AttitudeConfig.IDLC)
            {
                // 直接升力控制
                if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NDO)// 判断使用何种干扰观测器
                {
                    current_Uact = B4.Inverse() * (-F4 + k4_backstepping * e4 + derive_X4 - NDO_d_omega_output); // 使用NDO
                }
                else if(Configuration.DisturbanceObserver == DisturbanceObserverConfig.NONE)
                {
                    current_Uact = B4.Inverse() * (-F4 + k4_backstepping * e4 + derive_X4); // 不使用DO
                }
                else
                {
                    Console.WriteLine("请指定干扰观测器种类 id 44");
                }
            }
            else
            {
                Console.WriteLine("请指定控制器种类 id 12");
            }
        }

        public void calculateState(double dt, Vector<double> input)
        {
            // 可能需要添加检查向量维度的代码
            current_u3 = input;

            if (Configuration.attitude_command_filter_flag)// 判断使用何种滤波器
            {
                // 角速度环使用指令滤波器
                Vector<double> derive2_X4 = -2 * epsilon_X4 * omega_X4 * derive_X4 - omega_X4.Power(2) * (filter_u3 - current_u3);
                derive_X4 += derive2_X4 * dt;
                filter_u3 += derive_X4 * dt;
                e4 = filter_u3 - current_X4;
                previous_Uact = current_Uact;
            }
            else
            {
                Console.WriteLine("请指定滤波器种类 id 41");
                return;
            }
        }

        public void record(double dt)
        {
            //ev = XChangedEventArgs(dt, e4, filter_Uact, derive_X4);
            //notify(obj, "RecordAngularRateLoopEvent", ev);
        }

        public void reset()
        {
            //current_p = plane.current_p;
            //current_q = plane.current_q;
            //current_r = plane.current_r;
            //current_delta_a = plane.current_delta_a;
            //current_delta_e = plane.current_delta_e;
            //current_delta_r = plane.current_delta_r;

            current_X4 = vb.Dense(new[]
                { plane.P, plane.Q, plane.R });
            current_Uact = vb.Dense(new[]
                { plane.DeltaA, plane.DeltaE, plane.DeltaR });

            current_Uact_index_count = 0;
            current_Uact_index = mb.Dense(sample_num_rudder, 3, 0);
            filter_Uact = current_Uact;
            filter_Uact_previous = filter_Uact;
            filter_u3 = current_u3;
            previous_Uact = current_Uact;


            current_u3 = vb.Dense(3, 0);
            filter_Uact = current_Uact;
            filter_Uact_previous = filter_Uact;
            filter_u3 = current_u3;
            previous_Uact = current_Uact;
            derive_X4 = vb.Dense(3, 0); //[p,q,r]'
            current_NDO_p_omega = vb.Dense(3, 0);
        }

        public void updateState(double dt, Disturbance disturbance)
        {
            //dt = e.data{ 1};
            //current_X4_dot = e.data{ 2};
            //current_X4 = current_X4 + current_X4_dot * dt;
        }

        public void OnUpdateState(object sender, XChangedEventArgs e)
        {
            current_X4 += e.Data * e.Dt;
        }
    }
}
