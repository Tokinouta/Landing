using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Constants;
using static MathNet.Numerics.Trig;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    public struct InertiaParameter
    {
        public double Ixx; // 转动惯量 kg/m2
        public double Iyy;
        public double Izz;
        public double Ixz;
        public double WingS; // 翼面积
        public double WingC;  // 平均气动弦长
        public double WingL;  // 翼展
        public double Mass; // 空重 kg
        public double TMax;  // 军用推力（两台）
        public double Rou;  // 空气密度
        public double G;
    }

    public struct DesiredParameter
    {
        public double AlphaDesired; // 期望迎角 9.1
        public double ChiDesired; // 期望航向角
        public double GammaDesired; // 期望爬升角
        public double VkDesired; // 期望速度 71
        public double EngineDelta; // 发动机安装角
    }

    public class Plane
    {
        public InertiaParameter PlaneInertia;
        public DesiredParameter DesiredParameter;

        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;
        // 舵面角度及角速度限幅
        // 所有角度、角速度均以弧度为单位
        public Vector<double> DeltaERange = vb.Dense(new double[] { -24, 10.5 }) * Pi / 180; // 升降舵角度范围
        public Vector<double> DeltaERateRange = vb.Dense(new double[] { -40, 40 }) * Pi / 180; // 升降舵角速度范围
        public Vector<double> DeltaARange = vb.Dense(new double[] { -25, 45 }) * Pi / 180;
        public Vector<double> DeltaARateRange = vb.Dense(new double[] { -100, 100 }) * Pi / 180;
        public Vector<double> DeltaRRange = vb.Dense(new double[] { -30, 30 }) * Pi / 180;
        public Vector<double> DeltaRRateRange = vb.Dense(new double[] { -61, 61 }) * Pi / 180;

        public Vector<double> DeltaTEFRange = vb.Dense(new double[] { -8, 45 }) * Pi / 180;
        public Vector<double> DeltaTEFRateRange = vb.Dense(new double[] { -40, 40 }) * Pi / 180; // 调整后缘襟翼偏转速率，改善纵向轨迹跟踪性能

        public Vector<double> DeltaPRange = vb.Dense(new double[] { 0.01, 1.0 });
        public Vector<double> DeltaPRateRange = vb.Dense(new double[] { -0.01, 0.01 }) * 100000;
        public Vector<double> ThrustRange;
        public Vector<double> ThrustRateRange;

        public Vector<double> PRange = vb.Dense(new double[] { -179, 179 }) * Pi / 180 * 10;
        public Vector<double> QRange = vb.Dense(new double[] { -179, 179 }) * Pi / 180 * 10;
        public Vector<double> RRange = vb.Dense(new double[] { -179, 179 }) * Pi / 180 * 10;
        public Vector<double> GammaRange = vb.Dense(new double[] { -89, 89 }) * Pi / 180;
        public Vector<double> ChiRange = vb.Dense(new double[] { -89, 89 }) * Pi / 180;
        public Vector<double> ThetaRange = vb.Dense(new double[] { -89, 89 }) * Pi / 180;
        public Vector<double> MiuRange = vb.Dense(new double[] { -89, 89 }) * Pi / 180;

        public Vector<double> PRateRange = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> QRateRange = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> RRateRange = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> GammaRateRange = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> ChiRateRange = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> ThetaRateRange = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> MiuRateRange = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;

        // 飞机气动参数
        public double CY; // 升力系数 弧度制
        public double CD; // 阻力系数 弧度制 drag coefficient
        public double CC; // 侧力系数 弧度制
        public double CL; // 滚转力矩 弧度制 rolling moment coefficient
        public double CM; // 俯仰力矩 弧度制 pitching moment coefficient
        public double CN; // 偏航力矩 弧度制 yawing moment coefficient


        // 飞机状态
        public Vector<double> Position;
        public double Chi;
        public double Gamma;
        public double Alpha = 6.0 * Pi / 180; // 5.0
        public double Miu = 0 * Pi / 180;
        public double Beta = 0 * Pi / 180;
        public double Theta;
        public double Phi = 0; // 欧拉角
        public double Psi = 0;
        public double P = 0 * Pi / 180;
        public double Q = 0 * Pi / 180;
        public double R = 0 * Pi / 180;
        public double Vk = 72; // 70
        public double DeltaA = 0 * Pi / 180;
        public double DeltaE = 0 * Pi / 180;
        public double DeltaR = 0 * Pi / 180;
        public double DeltaP = 0.100; // 0.10
        public double DeltaLEF = 33 * Pi / 180 * 1; // leading-edge flap
        public double DeltaTEF = 45 * Pi / 180 * 1; // tailing-edge flap
        public double DeltaTEFDesired;
        public double Flow;
        public double T;
        public Vector<double> DesiredPosition;
        public double GammaDerive = 0;
        public double KaiDerive = 0;

        public double omega_fx_2f;
        public double omega_fy_2f;
        public double omega_fz_2f;
        public double x_b_2f;
        public double y_b_2f;
        public double z_b_2f;
        public double kai_b2f;
        public double gamma_b2f;

        // 气动力和力矩
        public double Y;
        public double D;
        public double C;
        public double L;
        public double M;
        public double N;

        //properties(SetAccess = private)
        public double CY_alpha;
        public double CY_delta_tef;
        public double CL_delta_a;
        public double CL_delta_r;
        public double CM_delta_e;
        public double CN_delta_a;
        public double CN_delta_r;
        public double CC_beta;
        public double CD_alpha;
        public double CL_beta;
        public double CM_alpha1;
        public double CM_alpha2;
        public double CN_beta;


        // 着舰过程中期望参量

        //event RecordPlaneStateEvent;
        public event EventHandler<XChangedEventArgs> X1ChangedEvent;
        public event EventHandler<XChangedEventArgs> X2ChangedEvent;
        public event EventHandler<XChangedEventArgs> X3ChangedEvent;
        public event EventHandler<XChangedEventArgs> X4ChangedEvent;
        public event EventHandler RecordPlaneStateEvent;
        //event X1ChangedEvent;
        //event X2ChangedEvent;
        //event X3ChangedEvent;
        //event X4ChangedEvent;

        public Plane(Ship ship)
        {
            PlaneInertia = new()
            {
                Ixx = 31183.40,// 转动惯量 kg/m2
                Iyy = 205123.05,
                Izz = 230411.43,
                Ixz = 4028.08,
                WingS = 37.16, // 翼面积
                WingC = 3.51, // 平均气动弦长
                WingL = 11.41, // 翼展
                Mass = 15097.39, // 空重 kg
                TMax = 130.6 * 1000, // 军用推力（两台）
                Rou = 1.225, // 空气密度
                G = 9.8
            };
            DesiredParameter = new()
            {
                AlphaDesired = 9.1 * Pi / 180, // 期望迎角 9.1
                ChiDesired = 0 * Pi / 180, // 期望航向角
                GammaDesired = -3.5 * Pi / 180, // 期望爬升角
                VkDesired = 75, // 期望速度 71
                EngineDelta = 0 * Pi / 180// 发动机安装角
            };

            Flow = 0.5 * PlaneInertia.Rou * Math.Pow(Vk, 2);
            T = PlaneInertia.TMax * DeltaP;
            ThrustRange = PlaneInertia.TMax * DeltaPRange;
            ThrustRateRange = PlaneInertia.TMax * DeltaPRateRange;
            CalculatePneumaticParameters();
            CalculateForceAndMoment();
            Initialize(ship);
            Theta = Gamma + Alpha;
            DeltaTEFDesired = DeltaTEF;
            DesiredPosition = vb.Dense(3, Position[0]);
            DesiredPosition.SetSubVector(
                1, 2, HelperFunction.ideal_path(Position, ship.Position, ship.Theta, ship.Psi));
        }

        void AddListeners()
        {
            //addlistener(varargin{ 1}, 'X1ChangedEvent', @X1ChangedEventHandler);
            //addlistener(varargin{ 2}, 'X2ChangedEvent', @X2ChangedEventHandler);
            //addlistener(varargin{ 3}, 'X3ChangedEvent', @X3ChangedEventHandler);
            //addlistener(varargin{ 4}, 'X4ChangedEvent', @X4ChangedEventHandler);
        }

        void Initialize(Ship ship)
        {
            //double ship.Theta = ship.Theta;
            //double ship.Psi = ship.Psi;
            //double ship.Gamma = ship.Gamma;

            double l_path_0 = 1620; // 期望路径参数，初始路径长度
            double l_path = 0; // 期望路径参数，路径长度参数->特别注意，l_path初始值必须为0
            double x_d_2p = -(l_path_0 - l_path) * Cos(ship.Theta) * Cos(ship.Gamma); // 期望点坐标 P系下表示
            double y_d_2p = (l_path_0 - l_path) * Sin(ship.Theta) * Cos(ship.Gamma);
            double z_d_2p = (l_path_0 - l_path) * Sin(ship.Gamma);
            Vector<double> p_d_2p = vb.Dense(new double[] { x_d_2p, y_d_2p, z_d_2p }); // 期望点坐标 P系下表示

            Matrix<double> R_i2p = mb.DenseOfArray(new double[,] {
                { Cos(ship.Psi), Sin(ship.Psi), 0 }, { -Sin(ship.Psi), Cos(ship.Psi), 0 }, { 0, 0, 1 } });
            Matrix<double> R_p2i = R_i2p.Transpose();
            Vector<double> p_d_2i = R_p2i * p_d_2p + ship.Position; // 期望点坐标 I系下表示

            double kai_f = -ship.Theta + ship.Psi;
            double gamma_f = ship.Gamma;

            Matrix<double> R_i2f = mb.DenseOfArray(new double[,] {
                { Cos(gamma_f) * Cos(kai_f), Cos(gamma_f) * Sin(kai_f), -Sin(gamma_f) },
                { -Sin(kai_f), Cos(kai_f), 0 },
                { Sin(gamma_f) * Cos(kai_f), Sin(gamma_f) * Sin(kai_f), Cos(gamma_f) } });
            Matrix<double> R_f2i = R_i2f.Transpose();
            Vector<double> p_b_2f = vb.Dense(new double[] { 0, 5, 10 }); // 飞机在F系中初始位置
            x_b_2f = p_b_2f[0];
            y_b_2f = p_b_2f[1];
            z_b_2f = p_b_2f[2];

            Position = R_f2i * p_b_2f + p_d_2i; // 飞机在I系中初始位置
            Gamma = 0; // 初始爬升角
            Chi = -ship.Theta; // 初始航向角
            Vector<double> omega_d_2f = R_i2f * ship.omega_d_2i;
            double omega_dx_2f = omega_d_2f[0];
            double omega_dy_2f = omega_d_2f[1];
            double omega_dz_2f = omega_d_2f[2];

            Vector<double> omega_f_2f = vb.Dense(new double[] { omega_dx_2f, omega_dy_2f, omega_dz_2f });
            omega_fx_2f = omega_f_2f[0];
            omega_fy_2f = omega_f_2f[1];
            omega_fz_2f = omega_f_2f[2];

            kai_b2f = Chi - kai_f;
            gamma_b2f = Gamma - gamma_f;
        }

        public void Record()
        {
            //notify(obj, "RecordPlaneStateEvent
            RecordPlaneStateEvent?.Invoke(this, null);
        }


        void CalculatePneumaticParameters()
        {
            double WingC = PlaneInertia.WingC;
            double WingS = PlaneInertia.WingS;
            double WingL = PlaneInertia.WingL;

            // flap coefficient
            double CM_delta_tef = 0.001 * 180 / Pi;
            double CM_delta_lef = 0;
            double Cnormal_delta_tef = 0.009 * 180 / Pi;
            double Cnormal_delta_lef = Cnormal_delta_tef * 0.5; // 推测数据，缺少实测数据
            double Caxis_delta_tef = 0;
            double Caxis_delta_lef = 0;

            // 升力系数相关参数 弧度制
            double CY_alpha3 = 1.1645;
            double CY_alpha2 = -5.4246;
            double CY_alpha1 = 5.6770;
            double CY_alpha0 = -0.0204;
            double CY_delta_e3 = 2.1852;
            double CY_delta_e2 = -2.6975;
            double CY_delta_e1 = 0.4055;
            double CY_delta_e0 = 0.5725;
            double CY_0 = CY_alpha0 * Cos(2 * Beta / 3);
            CY_alpha = Cos(2 * Beta / 3) *
                (CY_alpha3 * Math.Pow(Alpha, 2) + CY_alpha2 * Alpha + CY_alpha1);
            double CY_delta_e = CY_delta_e3 * Math.Pow(Alpha, 3) + CY_delta_e2 * Math.Pow(Alpha, 2) + CY_delta_e1 * Alpha + CY_delta_e0;
            double CY_delta_lef = Cnormal_delta_lef * Cos(Alpha) - Caxis_delta_lef * Sin(Alpha);
            CY_delta_tef = Cnormal_delta_tef * Cos(Alpha) - Caxis_delta_tef * Sin(Alpha);
            CY = CY_0
                + CY_alpha * Alpha
                + CY_delta_e * DeltaE
                + CY_delta_lef * DeltaLEF // leading - egde flap && tailing - edge flap
                + CY_delta_tef * DeltaTEF; // lift coefficient

            // 阻力系数相关参数 弧度制
            double CD_alpha4 = 1.4610;
            double CD_alpha3 = -5.7341;
            double CD_alpha2 = 6.3971;
            double CD_alpha1 = -0.1995;
            double CD_alpha0 = -1.4994;
            double CD_delta_e3 = -3.8578;
            double CD_delta_e2 = 4.2360;
            double CD_delta_e1 = -0.2739;
            double CD_delta_e0 = 0.0366;
            double CD_0 = 1.5036 + CD_alpha0 * Cos(Beta);
            CD_alpha = Cos(Beta) * (CD_alpha4 * Math.Pow(Alpha, 3) + CD_alpha3 * Math.Pow(Alpha, 2) + CD_alpha2 * Alpha + CD_alpha1);
            double CD_delta_e = CD_delta_e3 * Math.Pow(Alpha, 3) + CD_delta_e2 * Math.Pow(Alpha, 2) + CD_delta_e1 * Alpha + CD_delta_e0;
            double CD_delta_lef = (Caxis_delta_lef * Cos(Alpha) + Cnormal_delta_lef * Sin(Alpha));
            double CD_delta_tef = (Caxis_delta_tef * Cos(Alpha) + Cnormal_delta_tef * Sin(Alpha));
            CD = CD_0
                + CD_alpha * Alpha
                + CD_delta_e * DeltaE
                + CD_delta_lef * DeltaLEF // leading - egde flap && tailing - edge flap
                + CD_delta_tef * DeltaTEF; // drag coefficient

            // 侧力系数 弧度制
            double CC_beta = -0.012 * 180 / Pi;
            CC = CC_beta * Beta;
            // CC_beta2 = -0.1926;
            // CC_beta1 = 0.2654;
            // CC_beta0 = -0.7344;
            // CC_delta_a3 = -0.8500;
            // CC_delta_a2 = 1.5317;
            // CC_delta_a1 = -0.2403;
            // CC_delta_a0 = -0.1656;
            // CC_delta_r3 = 0.9351;
            // CC_delta_r2 = -1.6921;
            // CC_delta_r1 = 0.4082;
            // CC_delta_r0 = 0.2054;
            // CC_beta = CC_beta2 * Math.Pow(current_alpha, 2) + CC_beta1 * current_alpha + CC_beta0;
            // CC_delta_a = CC_delta_a3 * Math.Pow(current_alpha, 3) + CC_delta_a2 * Math.Pow(current_alpha, 2) + CC_delta_a1 * current_alpha + CC_delta_a0;
            // CC_delta_r = CC_delta_r3 * Math.Pow(current_alpha, 3) + CC_delta_r2 * Math.Pow(current_alpha, 2) + CC_delta_r1 * current_alpha + CC_delta_r0;
            // CC = CC_beta * current_beta...
            // +CC_delta_a * DeltaA...
            // +CC_delta_r * current_delta_r; // sidefroce coefficient

            // 滚转力矩相关参数 弧度制
            double CL_beta4 = -1.6196;
            double CL_beta3 = 2.3843;
            double CL_beta2 = -0.3620;
            double CL_beta1 = -0.4153;
            double CL_beta0 = -0.0556;
            double CL_delta_a3 = 0.1989;
            double CL_delta_a2 = -0.2646;
            double CL_delta_a1 = -0.0516;
            double CL_delta_a0 = 0.1424;
            double CL_delta_r3 = -0.0274;
            double CL_delta_r2 = 0.0083;
            double CL_delta_r1 = 0.0014;
            double CL_delta_r0 = 0.0129;
            double CL_p1 = 0.2377;
            double CL_p0 = -0.3540;
            double CL_r2 = -1.0871;
            double CL_r1 = 0.7804;
            double CL_r0 = 0.1983;
            CL_beta = CL_beta4 * Math.Pow(Alpha, 4) + CL_beta3 * Math.Pow(Alpha, 3) + CL_beta2 * Math.Pow(Alpha, 2) + CL_beta1 * Alpha + CL_beta0;
            CL_delta_a = CL_delta_a3 * Math.Pow(Alpha, 3) + CL_delta_a2 * Math.Pow(Alpha, 2) + CL_delta_a1 * Alpha + CL_delta_a0;
            CL_delta_r = CL_delta_r3 * Math.Pow(Alpha, 3) + CL_delta_r2 * Math.Pow(Alpha, 2) + CL_delta_r1 * Alpha + CL_delta_r0;
            double CL_p = CL_p1 * Alpha + CL_p0;
            double CL_r = CL_r2 * Math.Pow(Alpha, 2) + CL_r1 * Alpha + CL_r0;
            CL = CL_beta * Beta
                + CL_delta_a * DeltaA
                + CL_delta_r * DeltaR
                + WingL / 2 / Vk * CL_p * P
                + WingL / 2 / Vk * CL_r * R; // rolling moment coefficient

            // 俯仰力矩相关参数 弧度制
            CM_alpha2 = -1.2897;
            CM_alpha1 = 0.5110;
            double CM_alpha0 = -0.0866;
            double CM_delta_e2 = 0.9338;
            double CM_delta_e1 = -0.3245;
            double CM_delta_e0 = -0.9051;
            double CM_q3 = 64.7190;
            double CM_q2 = -68.5641;
            double CM_q1 = 10.9921;
            double CM_q0 = -4.1186;
            double CM_0 = CM_alpha2 * Math.Pow(Alpha, 2) + CM_alpha1 * Alpha + CM_alpha0;
            CM_delta_e = CM_delta_e2 * Math.Pow(Alpha, 2) + CM_delta_e1 * Alpha + CM_delta_e0;
            double CM_q = CM_q3 * Math.Pow(Alpha, 3) + CM_q2 * Math.Pow(Alpha, 2) + CM_q1 * Alpha + CM_q0;
            CM_0 = CM_0 * Pi / 180;
            CM = CM_0
                + CM_delta_e * DeltaE
                + WingC / 2 / Vk * CM_q * Q
                + CM_delta_lef * DeltaLEF + CM_delta_tef * DeltaTEF; // pitching moment coefficient

            // 偏航力矩相关参数 弧度制
            double CN_beta2 = -0.3816;
            double CN_beta1 = 0.0329;
            double CN_beta0 = 0.0885;
            double CN_delta_a3 = 0.2694;
            double CN_delta_a2 = -0.3413;
            double CN_delta_a1 = 0.0584;
            double CN_delta_a0 = 0.0104;
            double CN_delta_r4 = 0.3899;
            double CN_delta_r3 = -0.8980;
            double CN_delta_r2 = 0.5564;
            double CN_delta_r1 = -0.0176;
            double CN_delta_r0 = -0.0780;
            double CN_p1 = -0.0881;
            double CN_p0 = 0.0792;
            double CN_r1 = -0.1307;
            double CN_r0 = -0.4326;
            CN_beta = CN_beta2 * Math.Pow(Alpha, 2) + CN_beta1 * Alpha + CN_beta0;
            CN_delta_r = CN_delta_r4 * Math.Pow(Alpha, 4) + CN_delta_r3 * Math.Pow(Alpha, 3) + CN_delta_r2 * Math.Pow(Alpha, 2) + CN_delta_r1 * Alpha + CN_delta_r0;
            CN_delta_a = CN_delta_a3 * Math.Pow(Alpha, 3) + CN_delta_a2 * Math.Pow(Alpha, 2) + CN_delta_a1 * Alpha + CN_delta_a0;
            double CN_p = CN_p1 * Alpha + CN_p0;
            double CN_r = CN_r1 * Alpha + CN_r0;
            CN = CN_beta * Beta
                + CN_delta_r * DeltaR
                + CN_delta_a * DeltaA
                + WingL / 2 / Vk * CN_p * P
                + WingL / 2 / Vk * CN_r * R; // yawing moment coefficient

            //CY_alpha = CY_alpha;
            //CY_delta_tef = CY_delta_tef;
            //CL_delta_a = CL_delta_a;
            //CL_delta_r = CL_delta_r;
            //CM_delta_e = CM_delta_e;
            //CN_delta_a = CN_delta_a;
            //CN_delta_r = CN_delta_r;
            //CC_beta = CC_beta;
            //CD_alpha = CD_alpha;
            //CL_beta = CL_beta;
            //CM_alpha1 = CM_alpha1;
            //CM_alpha2 = CM_alpha2;
            //CN_beta = CN_beta;

        }

        void CalculateForceAndMoment()
        {
            Y = Flow * PlaneInertia.WingS * CY;
            D = Flow * PlaneInertia.WingS * CD;
            C = Flow * PlaneInertia.WingS * CC;
            L = Flow * PlaneInertia.WingS * PlaneInertia.WingL * CL;
            M = Flow * PlaneInertia.WingS * PlaneInertia.WingC * CM;
            N = Flow * PlaneInertia.WingS * PlaneInertia.WingL * CN;
        }

        public void UpdateState(double dt, Disturbance disturbance)
        {
            CalculatePneumaticParameters();
            CalculateForceAndMoment();

            disturbance.updateState(this);

            double Ixx = PlaneInertia.Ixx;
            double Iyy = PlaneInertia.Iyy;
            double Izz = PlaneInertia.Izz;
            double Ixz = PlaneInertia.Ixz;
            double Mass = PlaneInertia.Mass;
            double TMax = PlaneInertia.TMax;
            double Rou = PlaneInertia.Rou;
            double G = PlaneInertia.G;

            double AlphaDesired = DesiredParameter.AlphaDesired; // 期望迎角 9.1
            double ChiDesired = DesiredParameter.ChiDesired; // 期望航向角
            double GammaDesired = DesiredParameter.GammaDesired; // 期望爬升角
            double VkDesired = DesiredParameter.VkDesired; // 期望速度 71
            double EngineDelta = DesiredParameter.EngineDelta; // 发动机安装角


            double current_p_dot = 1 / (Ixx * Izz - Math.Pow(Ixz, 2)) * ((Iyy * Izz - Math.Pow(Izz, 2) - Math.Pow(Ixz, 2)) * Q * R
                        + (Ixx * Ixz + Izz * Ixz - Iyy * Ixz) * P * Q
                        + Izz * L
                        + Ixz * N)
                        + disturbance.disturbance_p;

            double current_q_dot = 1 / Iyy * ((Izz - Ixx) * P * R - Ixz * Math.Pow(P, 2)
                + Ixz * Math.Pow(R, 2) + M)
                + disturbance.disturbance_q;

            double current_r_dot = 1 / (Ixx * Izz - Math.Pow(Ixz, 2)) * ((Math.Pow(Ixx, 2) + Math.Pow(Ixz, 2) - Ixx * Iyy) * P * Q
                + (Iyy * Ixz - Ixx * Ixz - Izz * Ixz) * Q * R
                + Ixz * L
                + Ixx * N)
                + disturbance.disturbance_r;
            Vector<double> current_X4_dot = vb.Dense(new[] { current_p_dot, current_q_dot, current_r_dot });

            P += current_p_dot * dt;
            Q += current_q_dot * dt;
            R += current_r_dot * dt;

            X4ChangedEvent?.Invoke(this, new XChangedEventArgs(dt, current_X4_dot));
            //ev = XChangedEventArgs(dt, current_X4_dot);
            //notify(obj, "X4ChangedEvent", ev);

            // 欧拉角
            double current_phi_dot = P + Tan(Theta) * (Q * Sin(Phi) + R * Cos(Phi));
            double current_theta_dot = Q * Cos(Phi) - R * Sin(Phi);
            double current_psi_dot = 1 / Cos(Theta) * (Q * Sin(Phi) + R * Cos(Phi));
            Phi += current_phi_dot * dt;
            Theta += current_theta_dot * dt;
            Psi += current_psi_dot * dt;

            // 绕质心运动学方程
            double current_beta_dot = P * Sin(Alpha) - R * Cos(Alpha) - GammaDerive * Sin(Miu) + KaiDerive * Cos(Miu) * Cos(Gamma);
            double current_miu_dot = (P * Cos(Alpha) + R * Sin(Alpha) + GammaDerive * Sin(Beta) * Cos(Miu) + KaiDerive * (Sin(Gamma)
                * Cos(Beta) + Sin(Beta) * Sin(Miu) * Cos(Gamma))) / Cos(Beta);
            double current_alpha_dot = (Q * Cos(Beta) - (P * Cos(Alpha) + R * Sin(Alpha)) * Sin(Beta) - GammaDerive * Cos(Miu)
                    - KaiDerive * Sin(Miu) * Cos(Gamma)) / Cos(Beta);

            Vector<double> current_X3_dot = vb.Dense(new[] { current_alpha_dot, current_beta_dot, current_miu_dot });

            // update state parameters
            Alpha += current_alpha_dot * dt;
            Beta += current_beta_dot * dt;
            Miu += current_miu_dot * dt;

            X3ChangedEvent?.Invoke(this, new XChangedEventArgs(dt, current_X3_dot));
            //ev = XChangedEventArgs(dt, current_X3_dot);
            //notify(obj, "X3ChangedEvent", ev);

            // 质心动力方程
            double current_kai_dot = 1 / (Mass * Vk * Cos(Gamma))
                * (T * (Sin(Alpha + EngineDelta) * Sin(Miu)
                - Cos(Alpha + EngineDelta)
                * Sin(Beta) * Cos(Miu))
                + C * Cos(Miu)
                + Y * Sin(Miu))
                + disturbance.disturbance_kai;
            double current_gamma_dot = -1 / (Mass * Vk)
                * (T * (-Sin(Alpha + EngineDelta) * Cos(Miu)
                - Cos(Alpha + EngineDelta) * Sin(Beta) * Sin(Miu))
                + C * Sin(Miu)
                - Y * Cos(Miu)
                + Mass * G * Cos(Gamma))
                + disturbance.disturbance_gamma;
            Vector<double> current_X2_dot = vb.Dense(new[] { current_kai_dot, current_gamma_dot });

            double current_Vk_dot = 1 / Mass * (T * Cos(Alpha + EngineDelta) * Cos(Beta)
                - D
                - Mass * G * Sin(Gamma))
                + disturbance.disturbance_Vk;

            // update state parameters
            Chi += current_kai_dot * dt;
            Gamma += current_gamma_dot * dt;

            // previous_delta_p = current_delta_p;
            DeltaP = T / TMax;
            Vk += current_Vk_dot * dt;
            Flow = 0.5 * Rou * Math.Pow(Vk, 2);

            GammaDerive = current_gamma_dot;
            KaiDerive = current_kai_dot;

            X2ChangedEvent?.Invoke(this, new XChangedEventArgs(dt, current_X2_dot));
            //ev = XChangedEventArgs(dt, current_X2_dot);
            //notify(obj, "X2ChangedEvent", ev);

            // 质心运动方程
            double current_x_dot = Vk * Cos(Gamma) * Cos(Chi);
            double current_y_dot = Vk * Cos(Gamma) * Sin(Chi);
            double current_z_dot = -Vk * Sin(Gamma);
            Vector<double> current_X1_dot = vb.Dense(new[] { current_y_dot, current_z_dot });
            // update state parameters
            Position[0] += current_x_dot * dt;
            Position[1] += current_y_dot * dt;
            Position[2] += current_z_dot * dt;

            X1ChangedEvent?.Invoke(this, new XChangedEventArgs(dt, current_X1_dot));
            //ev = XChangedEventArgs(dt, current_X1_dot);
            //notify(obj, "X1ChangedEvent", ev);
        }

        public void Reset(Ship ship)
        {
            //var current_position_ship = ship.Position;
            //double ship.Theta = ship.Theta;
            //double ship.Psi = ship.Psi;

            Alpha = 6.0 * Pi / 180; // 5.0
            Miu = 0 * Pi / 180;
            Beta = 0 * Pi / 180;
            Phi = 0; // 欧拉角
            Psi = 0;
            P = 0 * Pi / 180;
            Q = 0 * Pi / 180;
            R = 0 * Pi / 180;
            Vk = 72; // 70
            DeltaA = 0 * Pi / 180;
            DeltaE = 0 * Pi / 180;
            DeltaR = 0 * Pi / 180;
            DeltaP = 0.100; // 0.10
            DeltaLEF = 33 * Pi / 180 * 1; // leading - edge flap
            DeltaTEF = 45 * Pi / 180 * 1; // tailing - edge flap

            GammaDerive = 0;
            KaiDerive = 0;
            Flow = 0.5 * PlaneInertia.Rou * Math.Pow(Vk, 2);
            T = PlaneInertia.TMax * DeltaP;

            CalculatePneumaticParameters();
            CalculateForceAndMoment();
            Initialize(ship);
            Theta = Gamma + Alpha;
            DesiredPosition = vb.Dense(3, Position[0]);
            DesiredPosition.SetSubVector(
                1, 2, HelperFunction.ideal_path(Position, ship.Position, ship.Theta, ship.Psi));
            DeltaTEFDesired = DeltaTEF;
        }
    }
}
