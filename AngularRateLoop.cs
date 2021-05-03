using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Constants;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using CsharpVersion.Controllers;
using ModelEntities.Enumerations;
using ModelEntities;

namespace CsharpVersion
{
    public class AngularRateLoop : IControlModule
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;
        public Configuration Configuration { get; }
        Plane plane;
        Ship ship;

        // Input Variable
        Vector<double> U3 = vb.Dense(3, 0);

        // State Variable
        public Vector<double> X4;

        // Output Variable
        public Vector<double> Uact;

        // Interior Variable
        Matrix<double> epsilonX4 = mb.DenseDiagonal(3, 0.707); // 增大阻尼同时减小频率
        Matrix<double> omegaX4 = mb.DenseDiagonal(3, 40);

        // 反步法参数
        Matrix<double> k4_backstepping = mb.DiagonalIdentity(3) * 2; // 1.0

        // 滤波器参数
        int sampleNumber = 1; // 3
        int UactFilterBufferIndex = 0;
        Matrix<double> UactFilterBuffer; // delta_a delta_e delta_r
        public Vector<double> filteredUact;
        Vector<double> filteredUactPrevious;

        // 观测器输出变量
        Vector<double> F4;
        Matrix<double> B4;
        double _Fq;
        double _Gq;
        Vector<double> _Fpr;
        Matrix<double> _Gpr;

        // Nonlinear Observer
        Vector<double> current_NDO_p_omega = vb.Dense(3, 0);
        Vector<double> NDO_d_omega_output;

        // 中间变量
        Vector<double> e4;
        Vector<double> filteredU3;
        Vector<double> deriveX4 = vb.Dense(3, 0); //[p,q,r]'
        Vector<double> previousUact;

        IController controller;

        public Vector<double> FilteredU3 { get => filteredU3; }
        public Vector<double> DeriveX4 { get => deriveX4; }
        public double Fq { get => _Fq; }
        public double Gq { get => _Gq; }
        public Matrix<double> Gpr { get => _Gpr; }
        public Vector<double> Fpr { get => _Fpr; }

        //event RecordAngularRateLoopEvent;
        //event RecordAngularRateLoopVarEvent

        public AngularRateLoop(Plane plane, Ship ship, Configuration config)
        {
            this.plane = plane;
            this.ship = ship;
            Configuration = config;

            X4 = vb.Dense(new[]
                { plane.P, plane.Q, plane.R });
            Uact = vb.Dense(new[]
                { plane.DeltaA, plane.DeltaE, plane.DeltaR });

            UactFilterBuffer = mb.Dense(sampleNumber, 3, 0);
            filteredUact = Uact;
            filteredUactPrevious = filteredUact;
            filteredU3 = U3;
            previousUact = Uact;
            switch (Configuration.AngularRateController)
            {
                case AngularRateConfig.BS:
                    break;
                case AngularRateConfig.NDI:
                    controller = new L1Adaptive(this.ship, this.plane, this, Configuration);
                    break;
                default:
                    break;
            }
            //addlistener(plane, 'X4ChangedEvent', @updateState);
        }

        public void CalculateFilter(double dt)
        {
            filteredUactPrevious = filteredUact;
            filteredUact[0] = 1 / 48.0 / (1 / 48.0 + dt * sampleNumber) * filteredUactPrevious[0]
                + dt * sampleNumber / (1 / 48.0 + dt * sampleNumber) * Uact[0];
            filteredUact[1] = 1 / 30.0 / (1 / 30.0 + dt * sampleNumber) * filteredUactPrevious[1]
                + dt * sampleNumber / (1 / 30.0 + dt * sampleNumber) * Uact[1];
            filteredUact[2] = 1 / 40.0 / (1 / 40.0 + dt * sampleNumber) * filteredUactPrevious[2]
                + dt * sampleNumber / (1 / 40.0 + dt * sampleNumber) * Uact[2];

            UactFilterBuffer.SetRow(UactFilterBufferIndex, filteredUact);
            UactFilterBufferIndex++;

            if (UactFilterBufferIndex >= sampleNumber)
            {
                UactFilterBufferIndex = 0;
            }

            filteredUact = UactFilterBuffer.ColumnSums() / sampleNumber;
            plane.DeltaA = filteredUact[0];
            plane.DeltaE = filteredUact[1];
            plane.DeltaR = filteredUact[2];
        }

        public void CalculateLimiter(double dt)
        {
            // 舵面偏转角度限幅
            if (Uact[0] < plane.DeltaARange[0])
            {
                Uact[0] = plane.DeltaARange[0];
            }
            if (Uact[0] > plane.DeltaARange[1])
            {
                Uact[0] = plane.DeltaARange[1];
            }
            if (Uact[1] < plane.DeltaERange[0])
            {
                Uact[1] = plane.DeltaERange[0];
            }
            if (Uact[1] > plane.DeltaERange[1])
            {
                Uact[1] = plane.DeltaERange[1];
            }
            if (Uact[2] < plane.DeltaRRange[0])
            {
                Uact[2] = plane.DeltaRRange[0];
            }
            if (Uact[2] > plane.DeltaRRange[1])
            {
                Uact[2] = plane.DeltaRRange[1];
            }

            // 舵面偏转角速度限制
            var derive_Uact = (Uact - previousUact) / dt;

            if (derive_Uact[0] < plane.DeltaARateRange[0])
            {
                Uact[0] = previousUact[0] + plane.DeltaARateRange[0] * dt;
            }
            if (derive_Uact[0] > plane.DeltaARateRange[1])
            {
                Uact[0] = previousUact[0] + plane.DeltaARateRange[1] * dt;
            }
            if (derive_Uact[1] < plane.DeltaERateRange[0])
            {
                Uact[1] = previousUact[1] + plane.DeltaERateRange[0] * dt;
            }
            if (derive_Uact[1] > plane.DeltaERateRange[1])
            {
                Uact[1] = previousUact[1] + plane.DeltaERateRange[1] * dt;
            }
            if (derive_Uact[2] < plane.DeltaRRateRange[0])
            {
                Uact[2] = previousUact[2] + plane.DeltaRRateRange[0] * dt;
            }
            if (derive_Uact[2] > plane.DeltaRRateRange[1])
            {
                Uact[2] = previousUact[2] + plane.DeltaRRateRange[1] * dt;
            }
        }

        public void CalculateLimiter(double dt, int step_count)
        {
            // 舵面偏转角度限幅
            if (Uact[0] < plane.DeltaARange[0])
            {
                Console.WriteLine($"Aileron over range bottom, {step_count}");
                Uact[0] = plane.DeltaARange[0];
            }
            if (Uact[0] > plane.DeltaARange[1])
            {
                Console.WriteLine($"Aileron over range top, {step_count}");
                Uact[0] = plane.DeltaARange[1];
            }
            if (Uact[1] < plane.DeltaERange[0])
            {
                Console.WriteLine($"Elevator over range bottom, {step_count}");
                Uact[1] = plane.DeltaERange[0];
            }
            if (Uact[1] > plane.DeltaERange[1])
            {
                Console.WriteLine($"Elevator over range top, {step_count}");
                Uact[1] = plane.DeltaERange[1];
            }
            if (Uact[2] < plane.DeltaRRange[0])
            {
                Console.WriteLine($"Rudder over range bottom, {step_count}");
                Uact[2] = plane.DeltaRRange[0];
            }
            if (Uact[2] > plane.DeltaRRange[1])
            {
                Console.WriteLine($"Rudder over range top, {step_count}");
                Uact[2] = plane.DeltaRRange[1];
            }

            // 舵面偏转角速度限制
            var derive_Uact = (Uact - previousUact) / dt;

            if (derive_Uact[0] < plane.DeltaARateRange[0])
            {
                Uact[0] = previousUact[0] + plane.DeltaARateRange[0] * dt;
            }
            if (derive_Uact[0] > plane.DeltaARateRange[1])
            {
                Uact[0] = previousUact[0] + plane.DeltaARateRange[1] * dt;
            }
            if (derive_Uact[1] < plane.DeltaERateRange[0])
            {
                Uact[1] = previousUact[1] + plane.DeltaERateRange[0] * dt;
            }
            if (derive_Uact[1] > plane.DeltaERateRange[1])
            {
                Uact[1] = previousUact[1] + plane.DeltaERateRange[1] * dt;
            }
            if (derive_Uact[2] < plane.DeltaRRateRange[0])
            {
                Uact[2] = previousUact[2] + plane.DeltaRRateRange[0] * dt;
            }
            if (derive_Uact[2] > plane.DeltaRRateRange[1])
            {
                Uact[2] = previousUact[2] + plane.DeltaRRateRange[1] * dt;
            }

        }

        public void CalculateNonlinearObserver(double dt, Disturbance disturbance)
        {
            Matrix<double> NDO_l_omega = mb.DenseOfDiagonalArray(new double[] { 10, 20, 10 });

            Vector<double> previous_NDO_p_omega = current_NDO_p_omega;
            Vector<double> derive_NDO_p_omega = -NDO_l_omega * (NDO_l_omega * X4 + current_NDO_p_omega + F4 + B4 * Uact);
            current_NDO_p_omega = previous_NDO_p_omega + derive_NDO_p_omega * dt;
            Vector<double> NDO_d_omega = current_NDO_p_omega + NDO_l_omega * X4;
            NDO_d_omega_output = disturbance.IsWindDisturbanceStarted ? NDO_d_omega : vb.Dense(3, 0);
            //ev = XChangedEventArgs(NDO_d_omega);
            //notify(obj, "RecordAngularRateLoopVarEvent", ev);
        }

        public void CalculateObservation()
        {
            double Ixx = plane.PlaneInertia.Ixx;
            double Iyy = plane.PlaneInertia.Iyy;
            double Izz = plane.PlaneInertia.Izz;
            double Ixz = plane.PlaneInertia.Ixz;
            double WingC = plane.PlaneInertia.WingC;
            double WingS = plane.PlaneInertia.WingS;
            double WingL = plane.PlaneInertia.WingL;

            double F4_1 = 1 / (Ixx * Izz - Math.Pow(Ixz, 2))
                * ((Iyy * Izz - Math.Pow(Izz, 2) - Math.Pow(Ixz, 2)) * plane.R * plane.Q
                + (Ixx * Ixz + Izz * Ixz - Iyy * Ixz) * plane.P * plane.Q
                + Izz * (plane.L - plane.Flow * WingS * WingL
                * (plane.CL_delta_a * plane.DeltaA + plane.CL_delta_r * plane.DeltaR))
                + Ixz * (plane.N - plane.Flow * WingS * WingL
                * (plane.CN_delta_r * plane.DeltaR + plane.CN_delta_a * plane.DeltaA)));
            double F4_2 = 1 / Iyy * ((Izz - Ixx) * plane.P * plane.R
                - Ixz * Math.Pow(plane.P, 2) + Ixz * Math.Pow(plane.R, 2)
                + (plane.M - plane.Flow * WingS * WingC
                * (plane.CM_delta_e * plane.DeltaE)));
            double F4_3 = 1 / (Ixx * Izz - Math.Pow(Ixz, 2))
                * ((Math.Pow(Ixx, 2) + Math.Pow(Ixz, 2) - Ixx * Iyy) * plane.P * plane.Q
                + (Iyy * Ixz - Ixx * Ixz - Izz * Ixz) * plane.Q * plane.R
                + Ixz * (plane.L - plane.Flow * WingS * WingL
                * (plane.CL_delta_a * plane.DeltaA + plane.CL_delta_r * plane.DeltaR))
                + Ixx * (plane.N - plane.Flow * WingS * WingL
                * (plane.CN_delta_r * plane.DeltaR + plane.CN_delta_a * plane.DeltaA)));
            F4 = vb.Dense(new[] { F4_1, F4_2, F4_3 });
            B4 = plane.Flow * WingS * mb.DenseOfArray(new[,] {
                { WingL * (Izz * plane.CL_delta_a + Ixz * plane.CN_delta_a) / (Ixx * Izz - Math.Pow(Ixz, 2)), 0,
                    WingL * (Izz * plane.CL_delta_r + Ixz * plane.CN_delta_r) / (Ixx * Izz - Math.Pow(Ixz, 2))},
                { 0, WingC* plane.CM_delta_e / Iyy, 0 },
                { WingL * (Ixz * plane.CL_delta_a + Ixx * plane.CN_delta_a) / (Ixx * Izz - Math.Pow(Ixz, 2)), 0,
                    WingL * (Ixz * plane.CL_delta_r + Ixx * plane.CN_delta_r) / (Ixx * Izz - Math.Pow(Ixz, 2)) } });

            _Fq = 1 / Iyy * ((Izz - Ixx) * plane.P * plane.R - Ixz * Math.Pow(plane.P, 2)
                + Ixz * Math.Pow(plane.R, 2)
                + (plane.M - plane.Flow * WingS * WingC * (plane.CM_delta_e * plane.DeltaE)));
            _Gq = plane.Flow * WingS * WingC * plane.CM_delta_e / Iyy;

            double Fpr_0 = 1 / (Ixx * Izz - Math.Pow(Ixz, 2))
                * ((Iyy * Izz - Math.Pow(Izz, 2) - Math.Pow(Ixz, 2)) * plane.R * plane.Q
                + (Ixx * Ixz + Izz * Ixz - Iyy * Ixz) * plane.P * plane.Q
                + Izz * (plane.L - plane.Flow * WingS * WingL * (plane.CL_delta_a * plane.DeltaA + plane.CL_delta_r * plane.DeltaR))
                + Ixz * (plane.N - plane.Flow * WingS * WingL * (plane.CN_delta_r * plane.DeltaR + plane.CN_delta_a * plane.DeltaA)));
            double Fpr_1 = 1 / (Ixx * Izz - Math.Pow(Ixz, 2))
                * ((Math.Pow(Ixx, 2) + Math.Pow(Ixz, 2) - Ixx * Iyy) * plane.P * plane.Q
                + (Iyy * Ixz - Ixx * Ixz - Izz * Ixz) * plane.Q * plane.R
                + Ixz * (plane.L - plane.Flow * WingS * WingL * (plane.CL_delta_a * plane.DeltaA + plane.CL_delta_r * plane.DeltaR))
                + Ixx * (plane.N - plane.Flow * WingS * WingL * (plane.CN_delta_r * plane.DeltaR + plane.CN_delta_a * plane.DeltaA)));

            _Fpr = vb.Dense(new[] { Fpr_0, Fpr_1 });
            _Gpr = plane.Flow * WingS * WingL / (Ixx * Izz - Math.Pow(Ixz, 2)) * mb.DenseOfArray(new[,] {
                {Izz * plane.CL_delta_a + Ixz * plane.CN_delta_a, Izz * plane.CL_delta_r + Ixz * plane.CN_delta_r },
                {Ixz * plane.CL_delta_a + Ixx * plane.CN_delta_a, Ixz * plane.CL_delta_r + Ixx * plane.CN_delta_r }
            });
        }

        public void CalculateOutput()
        {
            throw new NotImplementedException();
        }
        public void CalculateOutput(double dt, double current_time, int step_count)
        {
            if (Configuration.AttitudeController == AttitudeConfig.IDLC)
            {
                switch (Configuration.AngularRateController)
                {
                    case AngularRateConfig.BS:
                        // 直接升力控制
                        switch (Configuration.DisturbanceObserver)// 判断使用何种干扰观测器
                        {
                            case DisturbanceObserverConfig.NDO:
                                Uact = B4.Inverse() * (-F4 + k4_backstepping * e4 + deriveX4 - NDO_d_omega_output); // 使用NDO
                                break;
                            case DisturbanceObserverConfig.FTO:
                                break;
                            case DisturbanceObserverConfig.HOSMO:
                                break;
                            case DisturbanceObserverConfig.ESO:
                                break;
                            case DisturbanceObserverConfig.NONE:
                                Uact = B4.Inverse() * (-F4 + k4_backstepping * e4 + deriveX4); // 不使用DO
                                break;
                            default:
                                Console.WriteLine("请指定干扰观测器种类 id 44");
                                break;
                        }
                        break;
                    case AngularRateConfig.NDI:
                        Uact = controller.CalculateOutput(dt, current_time, step_count);
                        controller.InvokeRecordEvent();
                        break;
                    default:
                        break;
                }
            }
            else
            {
                Console.WriteLine("请指定控制器种类 id 12");
            }
        }

        public void CalculateState(double dt, Vector<double> input)
        {
            U3 = input;

            switch (Configuration.AttitudeFilter)// 判断使用何种滤波器
            {
                case AttitudeFilters.Command: // 角速度环使用指令滤波器
                    Vector<double> derive2_X4 = -2 * epsilonX4 * omegaX4 * deriveX4
                        - omegaX4.Power(2) * (filteredU3 - U3);
                    deriveX4 += derive2_X4 * dt;
                    filteredU3 += deriveX4 * dt;
                    e4 = filteredU3 - X4;
                    previousUact = Uact;
                    break;
                default:
                    Console.WriteLine("请指定滤波器种类 id 41");
                    break;
            }
        }

        public void Record(double dt)
        {
            //ev = XChangedEventArgs(dt, e4, filter_Uact, derive_X4);
            //notify(obj, "RecordAngularRateLoopEvent", ev);
        }

        public void Reset()
        {
            X4 = vb.Dense(new[]
                { plane.P, plane.Q, plane.R });
            Uact = vb.Dense(new[]
                { plane.DeltaA, plane.DeltaE, plane.DeltaR });

            UactFilterBufferIndex = 0;
            UactFilterBuffer = mb.Dense(sampleNumber, 3, 0);
            filteredUact = Uact;
            filteredUactPrevious = filteredUact;
            filteredU3 = U3;
            previousUact = Uact;


            U3 = vb.Dense(3, 0);
            filteredUact = Uact;
            filteredUactPrevious = filteredUact;
            filteredU3 = U3;
            previousUact = Uact;
            deriveX4 = vb.Dense(3, 0); //[p,q,r]'
            current_NDO_p_omega = vb.Dense(3, 0);
            controller?.Reset();
        }

        public void UpdateState(double dt, Disturbance disturbance)
        {
            //dt = e.data{ 1};
            //current_X4_dot = e.data{ 2};
            //current_X4 = current_X4 + current_X4_dot * dt;
        }

        public void OnUpdateState(object sender, XChangedEventArgs e)
        {
            X4 += e.Data * e.Dt;
        }
    }
}
