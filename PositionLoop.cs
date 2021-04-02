using CsharpVersion.PositionController;
using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Trig;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    public class PositionLoop : IControlModule
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;

        Plane plane;
        Ship ship;
        // Input Variable
        public Vector<double> current_desired_X1;

        // State Variable
        Vector<double> current_X1;

        // Output Variable
        public Vector<double> current_u1 = vb.Dense(2, 0);

        // Interior Variable
        Matrix<double> epsilon_X1 = mb.DenseDiagonal(2, 2);
        Matrix<double> omega_X1 = mb.DenseDiagonal(2, 40);

        // 反步法参数
        Matrix<double> k1_backstepping = mb.DenseOfDiagonalArray(new[] { 0.9, 0.2 });

        // 滤波器参数
        int sample_num_u1 = 1; // 3
        int current_u1_index_count = 1;
        Matrix<double> current_u1_index; //


        // 观测器输出变量
        Vector<double> F1;
        Matrix<double> B1;

        // 中间变量
        Vector<double> epp;
        Vector<double> epc;
        // filter_u3;
        Vector<double> derive_X1 = vb.Dense(2, 0); //[y,z]'
        Vector<double> previous_u1;
        Vector<double> previous_desired_X1;
        Vector<double> filter_desired_X1;

        IPositionController controller;

        public event EventHandler<EventArgs> RecordPositionLoopEvent;
        public event EventHandler<EventArgs> RecordPositionLoopVarEvent;

        public PositionLoop(Plane plane, Ship ship)
        {
            plane.
            this.ship = ship;
            var current_position = plane.current_position;
            var current_desired_position = plane.current_desired_position;
            current_X1 = current_position.SubVector(1, 2);
            current_desired_X1 = current_desired_position.SubVector(1, 2); ;
            filter_desired_X1 = current_desired_X1;

            // 判断使用何种控制器
            switch (Configuration.GuidanceController)
            {
                case GuidanceConfig.Backstepping:
                    // 使用反步法控制器
                    //controller = new Backstepping(ship, current_time);
                    break;
                case GuidanceConfig.BacksteppingPPC:
                    // Description    : 基于预设性能的反步法制导律
                    //controller = new BacksteppingPpc(ship, current_time);
                    break;
                case GuidanceConfig.PPCVectorTimevarying:
                    // Description    : 基于预设性能的时变向量场制导律
                    //controller = new PpcVectorTimeVarying(ship, current_time);
                    break;
                case GuidanceConfig.NoPPCVectorNoVarying:
                    // Description    : 非时变向量场制导律
                    //controller = new NoPpcVectorNoVarying(ship, current_time);
                    break;
                case GuidanceConfig.NoPPCVectorTimeVarying:
                    // Description    : 时变向量场制导律
                    //controller = new NoPpcVectorTimeVarying(ship, current_time);
                    break;
                case GuidanceConfig.G3dMPF:
                    // Description    : 3D移动路径跟踪制导律
                    controller = new G3dMPF(plane, ship);
                    break;
                default:
                    Console.WriteLine("请指定控制器种类 id 12");
                    break;
            }
            //addlistener(plane, 'X1ChangedEvent', @updateState);
        }

        public void calculateFilter(double dt)
        {
            current_u1_index.SetRow(current_u1_index_count,current_u1);
            current_u1_index_count++;

            if (current_u1_index_count >= sample_num_u1)
            {
                current_u1_index_count = 0;
            }
            current_u1 = current_u1_index.ColumnSums() / sample_num_u1;

            //current_u1(1) = sum(current_u1_index(:, 1)) / sample_num_u1; // kai
            //current_u1(2) = sum(current_u1_index(:, 2)) / sample_num_u1; // gamma
        }

        public void calculateLimiter(double dt)
        {
            //kai_range = plane.kai_range;
            //gamma_range = plane.gamma_range;
            //kai_rate_range = plane.kai_rate_range;
            //gamma_rate_range = plane.gamma_rate_range;
            // kai gamma变化幅度限制
            if (current_u1[0] < plane.kai_range[0])
            {
                current_u1[0] = plane.kai_range[0];
            }
            if (current_u1[0] > plane.kai_range[1])
            {
                current_u1[0] = plane.kai_range[1];
            }
            if (current_u1[1] < plane.gamma_range[0])
            {
                current_u1[1] = plane.gamma_range[0];
            }
            if (current_u1[1] > plane.gamma_range[1])
            {
                current_u1[1] = plane.gamma_range[1];
            }

            // ********************************************************************************************************

            // Description    : kai gamma变化速率限制
            //********************************************************************************************************//
            // kai gamma变化速率限制
            var derive_u1 = (current_u1 - previous_u1) / dt;

            if (derive_u1[0] < plane.kai_rate_range[0])
            {
                current_u1[0] = previous_u1[0] + plane.kai_rate_range[0] * dt;
            }
            if (derive_u1[0] > plane.kai_rate_range[1])
            {
                current_u1[0] = previous_u1[0] + plane.kai_rate_range[1] * dt;
            }
            if (derive_u1[1] < plane.gamma_rate_range[0])
            {
                current_u1[1] = previous_u1[1] + plane.gamma_rate_range[0] * dt;
            }
            if (derive_u1[1] > plane.gamma_rate_range[1])
            {
                current_u1[1] = previous_u1[1] + plane.gamma_rate_range[1] * dt;
            }
        }

        public void calculateNonlinearObserver(double dt, Disturbance disturbance)
        {
            throw new NotImplementedException();
        }

        public void calculateObservation()
        {
            //current_Vk = plane.current_Vk;
            //current_gamma = plane.current_gamma;
            //current_kai = plane.current_kai;

            // 位置控制环
            F1 = vb.Dense(new[] {
                plane.current_Vk * (Cos(plane.current_gamma) * Sin(plane.current_kai) - plane.current_kai),
                -plane.current_Vk * (Sin(plane.current_gamma) - plane.current_gamma)});
            B1 = mb.DenseDiagonal(2, plane.current_Vk);
            B1[1, 1] *= -1;
        }

        public void calculateOutput()
        {
            throw new NotImplementedException();
        }

        public void calculateOutput(double dt, double current_time, int step_count)
        {
            current_u1 = controller.CalculateOutput(dt, current_time, step_count);
            controller.InvokeRecordEvent();
        }

        public void calculateState(double dt, Vector<double> input)
        {
            if (Configuration.guidance_command_filter_flag)// 判断使用何种滤波器
            {                                                         // 使用指令滤波器
                var derive2_X1 = -2 * epsilon_X1 * omega_X1 * derive_X1
                    - omega_X1.Power(2) * (filter_desired_X1 - current_desired_X1);
                derive_X1 += derive2_X1 * dt;
                filter_desired_X1 += derive_X1 * dt;
                epc = filter_desired_X1 - current_X1;
                previous_desired_X1 = current_desired_X1;
                previous_u1 = current_u1;

                epc /= Cos(-ship.theta_s + ship.psi_s);
            }
            else
            {
                Console.WriteLine("请指定滤波器种类 id 11");
            }
        }

        public void calculateState(double dt, Vector<double> input, Plane plane)
        {
            throw new NotImplementedException();
        }

        public void record(double dt)
        {
            //ev = XChangedEventArgs(epc, derive_X1, previous_desired_X1, ...
            //    previous_u1, dt);
            //notify(obj, "RecordPositionLoopEvent", ev);
        }

        public void reset()
        {
            current_X1 = plane.current_position.SubVector(1, 2);
            current_desired_X1 = plane.current_desired_position.SubVector(1, 2); ;
            filter_desired_X1 = current_desired_X1;
            derive_X1 = vb.Dense(2, 0); //[y,z]'
            controller.Reset();
        }

        public void updateState(double dt, Disturbance disturbance)
        {
            //dt = e.data{ 1};
            //current_X1_dot = e.data{ 2};
            //current_X1 = current_X1 + current_X1_dot * dt;
        }

        public void OnUpdateState(object sender, XChangedEventArgs e)
        {
            current_X1 += e.Data * e.Dt;
        }

        public void calculatePrescribedParameter()
        {
            //current_position = plane.current_position;

            //current_position_ship = ship.current_position_ship;
            //theta_s = ship.theta_s;
            //psi_s = ship.psi_s;
            previous_desired_X1 = current_desired_X1;

            var current_desired_position = vb.Dense(3, plane.current_position[0]);
            current_desired_position.SetSubVector(
                1, 2, HelperFunction.ideal_path(plane.current_position, ship.current_position_ship, ship.theta_s, ship.psi_s));
            current_desired_X1 = current_desired_position.SubVector(1, 2);
            epp = current_desired_X1 - plane.current_position.SubVector(1, 2); // 不使用滤波器得到的横向、纵向追踪误差，结果更准确 特别注意，此值用于反步法可能导致未考虑甲板运动补偿

            epp /= Cos(-ship.theta_s + ship.psi_s);
        }
    }
}
