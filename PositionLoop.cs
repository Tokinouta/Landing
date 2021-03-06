using CsharpVersion.Controllers;
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

        Plane _plane;
        Ship _ship;
        // Input Variable
        public Vector<double> X1Desired;

        // State Variable
        public Vector<double> X1;

        // Output Variable
        public Vector<double> U1 = vb.Dense(2, 0);

        // Interior Variable
        readonly Matrix<double> epsilonX1 = mb.DenseDiagonal(2, 0.707);
        readonly Matrix<double> omegaX1 = mb.DenseDiagonal(2, 50);

        // 反步法参数
        Matrix<double> k1_backstepping = mb.DenseOfDiagonalArray(new[] { 0.9, 0.2 });

        // 滤波器参数
        readonly int sampleNumber = 1; // 3
        int U1FilterBufferIndex = 1;
        Matrix<double> U1FilterBuffer; //


        // 观测器输出变量
        Vector<double> F1;
        Matrix<double> B1;

        // 中间变量
        Vector<double> epp;
        Vector<double> epc;
        // filter_u3;
        public Vector<double> deriveX1 = vb.Dense(2, 0); //[y,z]'
        Vector<double> previousU1;
        Vector<double> previousX1Desired;
        Vector<double> filterdX1Desired;
        readonly IController controller;

        public event EventHandler<EventArgs> RecordPositionLoopEvent;
        public event EventHandler<EventArgs> RecordPositionLoopVarEvent;

        public PositionLoop(Plane plane, Ship ship)
        {
            _plane = plane;
            _ship = ship;
            X1 = plane.Position.SubVector(1, 2);
            X1Desired = plane.DesiredPosition.SubVector(1, 2);
            filterdX1Desired = X1Desired;

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
                    controller = new G3dMPF(plane, ship, this);
                    break;
                default:
                    Console.WriteLine("请指定控制器种类 id 12");
                    break;
            }
            //addlistener(plane, 'X1ChangedEvent', @updateState);
        }

        public void CalculateFilter(double dt)
        {
            U1FilterBuffer.SetRow(U1FilterBufferIndex, U1);
            U1FilterBufferIndex++;

            if (U1FilterBufferIndex >= sampleNumber)
            {
                U1FilterBufferIndex = 0;
            }
            U1 = U1FilterBuffer.ColumnSums() / sampleNumber;
        }

        public void CalculateLimiter(double dt)
        {
            // chi gamma变化幅度限制
            if (U1[0] < _plane.ChiRange[0])
            {
                U1[0] = _plane.ChiRange[0];
            }
            if (U1[0] > _plane.ChiRange[1])
            {
                U1[0] = _plane.ChiRange[1];
            }
            if (U1[1] < _plane.GammaRange[0])
            {
                U1[1] = _plane.GammaRange[0];
            }
            if (U1[1] > _plane.GammaRange[1])
            {
                U1[1] = _plane.GammaRange[1];
            }

            // kai gamma变化速率限制
            var derive_u1 = (U1 - previousU1) / dt;

            if (derive_u1[0] < _plane.ChiRateRange[0])
            {
                U1[0] = previousU1[0] + _plane.ChiRateRange[0] * dt;
            }
            if (derive_u1[0] > _plane.ChiRateRange[1])
            {
                U1[0] = previousU1[0] + _plane.ChiRateRange[1] * dt;
            }
            if (derive_u1[1] < _plane.GammaRateRange[0])
            {
                U1[1] = previousU1[1] + _plane.GammaRateRange[0] * dt;
            }
            if (derive_u1[1] > _plane.GammaRateRange[1])
            {
                U1[1] = previousU1[1] + _plane.GammaRateRange[1] * dt;
            }
        }

        public void CalculateNonlinearObserver(double dt, Disturbance disturbance)
        {
            throw new NotImplementedException();
        }

        public void CalculateObservation()
        {
            // 位置控制环
            F1 = vb.Dense(new[] {
                _plane.Vk * (Cos(_plane.Gamma) * Sin(_plane.Chi) - _plane.Chi),
                -_plane.Vk * (Sin(_plane.Gamma) - _plane.Gamma)});
            B1 = mb.DenseOfDiagonalArray(new double[] { _plane.Vk, -_plane.Vk });
        }

        public void CalculateOutput()
        {
            throw new NotImplementedException();
        }

        public void CalculateOutput(double dt, double current_time, int step_count)
        {
            U1 = controller.CalculateOutput(dt, current_time, step_count);
            controller.InvokeRecordEvent();
        }

        public void CalculateState(double dt, Vector<double> input)
        {
            switch (Configuration.GuidanceFilter)// 判断使用何种滤波器
            {
                case GuidanceFilters.Command: // 使用指令滤波器
                    var derive2_X1 = -2 * epsilonX1 * omegaX1 * deriveX1
                        - omegaX1.Power(2) * (filterdX1Desired - X1Desired);
                    deriveX1 += derive2_X1 * dt;
                    filterdX1Desired += deriveX1 * dt;
                    epc = filterdX1Desired - X1;
                    previousX1Desired = X1Desired;
                    previousU1 = U1;

                    epc /= Cos(-_ship.Theta + _ship.Psi);
                    break;
                default:
                    Console.WriteLine("请指定滤波器种类 id 11");
                    break;
            }
        }

        public void Record(double dt)
        {
            //ev = XChangedEventArgs(epc, derive_X1, previous_desired_X1, ...
            //    previous_u1, dt);
            //notify(obj, "RecordPositionLoopEvent", ev);
        }

        public void Reset()
        {
            X1 = _plane.Position.SubVector(1, 2);
            X1Desired = _plane.DesiredPosition.SubVector(1, 2); ;
            filterdX1Desired = X1Desired;
            deriveX1 = vb.Dense(2, 0); //[y,z]'
            controller.Reset();
        }

        public void UpdateState(double dt, Disturbance disturbance)
        {
            //dt = e.data{ 1};
            //current_X1_dot = e.data{ 2};
            //current_X1 = current_X1 + current_X1_dot * dt;
        }

        public void OnUpdateState(object sender, XChangedEventArgs e)
        {
            X1 += e.Data * e.Dt;
        }

        public void CalculatePrescribedParameter()
        {
            previousX1Desired = X1Desired;

            var current_desired_position = vb.Dense(3, _plane.Position[0]);
            current_desired_position.SetSubVector(
                1, 2, HelperFunction.ideal_path(_plane.Position, _ship.Position, _ship.Theta, _ship.Psi));
            X1Desired = current_desired_position.SubVector(1, 2);
            epp = X1Desired - _plane.Position.SubVector(1, 2); // 不使用滤波器得到的横向、纵向追踪误差，结果更准确 特别注意，此值用于反步法可能导致未考虑甲板运动补偿

            epp /= Cos(-_ship.Theta + _ship.Psi);
        }
    }
}
