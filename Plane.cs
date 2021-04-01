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
    public class Plane
    {
        public const double Ixx = 31183.40; // 转动惯量 kg/m2
        public const double Iyy = 205123.05;
        public const double Izz = 230411.43;
        public const double Ixz = 4028.08;
        public const double wing_S = 37.16; // 翼面积
        public const double wing_C = 3.51; // 平均气动弦长
        public const double wing_L = 11.41; // 翼展
        public const double m = 15097.39; // 空重 kg
        public const double T_max = 130.6 * 1000; // 军用推力（两台）
        public const double rou = 1.225; // 空气密度
        public const double g = 9.8;

        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;
        // 舵面角度及角速度限幅
        // 所有角度、角速度均以弧度为单位
        public Vector<double> delta_e_range = vb.Dense(new double[] { 2.0, 3.0 }) * Pi / 180; // 升降舵角度范围
        public Vector<double> delta_e_rate_range = vb.Dense(new double[] { -40, 40 }) * Pi / 180; // 升降舵角速度范围
        public Vector<double> delta_a_range = vb.Dense(new double[] { -25, 45 }) * Pi / 180;
        public Vector<double> delta_a_rate_range = vb.Dense(new double[] { -100, 100 }) * Pi / 180;
        public Vector<double> delta_r_range = vb.Dense(new double[] { -30, 30 }) * Pi / 180;
        public Vector<double> delta_r_rate_range = vb.Dense(new double[] { -61, 61 }) * Pi / 180;

        public Vector<double> delta_tef_range = vb.Dense(new double[] { -8, 45 }) * Pi / 180;
        public Vector<double> delta_tef_rate_range = vb.Dense(new double[] { -40, 40 }) * Pi / 180; // 调整后缘襟翼偏转速率，改善纵向轨迹跟踪性能

        public Vector<double> delta_p_range = vb.Dense(new double[] { 0.01, 1.0 });
        public Vector<double> delta_p_rate_range = vb.Dense(new double[] { -0.01, 0.01 }) * 100000;
        public Vector<double> thrust_range;
        public Vector<double> thrust_rate_range;

        public Vector<double> p_range = vb.Dense(new double[] { -179, 179 }) * Pi / 180 * 10;
        public Vector<double> q_range = vb.Dense(new double[] { -179, 179 }) * Pi / 180 * 10;
        public Vector<double> r_range = vb.Dense(new double[] { -179, 179 }) * Pi / 180 * 10;
        public Vector<double> gamma_range = vb.Dense(new double[] { -89, 89 }) * Pi / 180;
        public Vector<double> kai_range = vb.Dense(new double[] { -89, 89 }) * Pi / 180;
        public Vector<double> theta_range = vb.Dense(new double[] { -89, 89 }) * Pi / 180;
        public Vector<double> miu_range = vb.Dense(new double[] { -89, 89 }) * Pi / 180;

        public Vector<double> p_rate_range = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> q_rate_range = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> r_rate_range = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> gamma_rate_range = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> kai_rate_range = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> theta_rate_range = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;
        public Vector<double> miu_rate_range = vb.Dense(new double[] { -20, 20 }) * Pi / 180 * 100;

        // 飞机气动参数
        public double CY; // 升力系数 弧度制
        public double CD; // 阻力系数 弧度制 drag coefficient
        public double CC; // 侧力系数 弧度制
        public double CL; // 滚转力矩 弧度制 rolling moment coefficient
        public double CM; // 俯仰力矩 弧度制 pitching moment coefficient
        public double CN; // 偏航力矩 弧度制 yawing moment coefficient


        // 飞机状态
        public Vector<double> current_position;
        public double current_kai;
        public double current_gamma;
        public double current_alpha = 6.0 * Pi / 180; // 5.0
        public double current_miu = 0 * Pi / 180;
        public double current_beta = 0 * Pi / 180;
        public double current_theta;
        public double current_phi = 0; // 欧拉角
        public double current_psi = 0;
        public double current_p = 0 * Pi / 180;
        public double current_q = 0 * Pi / 180;
        public double current_r = 0 * Pi / 180;
        public double current_Vk = 72; // 70
        public double current_delta_a = 0 * Pi / 180;
        public double current_delta_e = 0 * Pi / 180;
        public double current_delta_r = 0 * Pi / 180;
        public double current_delta_p = 0.100; // 0.10
        public double current_delta_lef = 33 * Pi / 180 * 1; // leading-edge flap
        public double current_delta_tef = 45 * Pi / 180 * 1; // tailing-edge flap
        public double current_delta_tef_desired;
        public double current_Q;
        public double current_T;
        public Vector<double> current_desired_position;
        public double derive_gamma = 0;
        public double derive_kai = 0;

        public double omega_fx_2f;
        public double omega_fy_2f;
        public double omega_fz_2f;
        public double x_b_2f;
        public double y_b_2f;
        public double z_b_2f;
        public double kai_b2f;
        public double gamma_b2f;



        // 气动力和力矩

        public double current_Y;
        public double current_D;
        public double current_C;
        public double current_L;
        public double current_M;
        public double current_N;

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

        public double desired_alpha = 9.1 * Pi / 180; // 期望迎角 9.1
        public double desired_kai = 0 * Pi / 180; // 期望航向角
        public double desired_gamma = -3.5 * Pi / 180; // 期望爬升角
        public double desired_Vk = 75; // 期望速度 71
        public double engine_delta = 0 * Pi / 180; // 发动机安装角


        //event RecordPlaneStateEvent;
        //event X1ChangedEvent;
        //event X2ChangedEvent;
        //event X3ChangedEvent;
        //event X4ChangedEvent;

        public Plane(Ship ship)
        {
            //current_position_ship = ship.current_position_ship;
            //theta_s = ship.theta_s;
            //psi_s = ship.psi_s;

            current_Q = 0.5 * rou * Math.Pow(current_Vk, 2);
            current_T = T_max * current_delta_p;
            thrust_range = T_max * delta_p_range;
            thrust_rate_range = T_max * delta_p_rate_range;
            calculatePneumaticParameters();
            calculateForceAndMoment();
            initialize(ship);
            current_theta = current_gamma + current_alpha;
            current_desired_position = [current_position(1), ideal_path(current_position, current_position_ship, theta_s, psi_s)]';
            current_delta_tef_desired = current_delta_tef;

        }

        void addListeners()
        {
            //addlistener(varargin{ 1}, 'X1ChangedEvent', @X1ChangedEventHandler);
            //addlistener(varargin{ 2}, 'X2ChangedEvent', @X2ChangedEventHandler);
            //addlistener(varargin{ 3}, 'X3ChangedEvent', @X3ChangedEventHandler);
            //addlistener(varargin{ 4}, 'X4ChangedEvent', @X4ChangedEventHandler);
        }

        void initialize(Ship ship)
        {
            double theta_s = ship.theta_s;
            double psi_s = ship.psi_s;
            double gamma_s = ship.gamma_s;

            double l_path_0 = 1620; // 期望路径参数，初始路径长度
            double l_path = 0; // 期望路径参数，路径长度参数->特别注意，l_path初始值必须为0
            double x_d_2p = -(l_path_0 - l_path) * Cos(theta_s) * Cos(gamma_s); // 期望点坐标 P系下表示
            double y_d_2p = (l_path_0 - l_path) * Sin(theta_s) * Cos(gamma_s);
            double z_d_2p = (l_path_0 - l_path) * Sin(gamma_s);
            Vector<double> p_d_2p = vb.Dense(new double[] { x_d_2p, y_d_2p, z_d_2p }); // 期望点坐标 P系下表示

            Matrix<double> R_i2p = mb.DenseOfArray(new double[,] {
                { Cos(psi_s), Sin(psi_s), 0 }, { -Sin(psi_s), Cos(psi_s), 0 }, { 0, 0, 1 } });
            Matrix<double> R_p2i = R_i2p.Transpose();
            Vector<double> p_d_2i = R_p2i * p_d_2p + ship.current_position_ship; // 期望点坐标 I系下表示

            double kai_f = -theta_s + psi_s;
            double gamma_f = gamma_s;

            Matrix<double> R_i2f = mb.DenseOfArray(new double[,] {
                { Cos(gamma_f) * Cos(kai_f), Cos(gamma_f) * Sin(kai_f), -Sin(gamma_f) },
                { -Sin(kai_f), Cos(kai_f), 0 },
                { Sin(gamma_f) * Cos(kai_f), Sin(gamma_f) * Sin(kai_f), Cos(gamma_f) } });
            Matrix<double> R_f2i = R_i2f.Transpose();
            Vector<double> p_b_2f = vb.Dense(new double[] { 0, 5, 10 }); // 飞机在F系中初始位置
            x_b_2f = p_b_2f[0];
            y_b_2f = p_b_2f[1];
            z_b_2f = p_b_2f[2];

            current_position = R_f2i * p_b_2f + p_d_2i; // 飞机在I系中初始位置
            current_gamma = 0; // 初始爬升角
            current_kai = -theta_s; // 初始航向角
            Vector<double> omega_d_2f = R_i2f * ship.omega_d_2i;
            double omega_dx_2f = omega_d_2f[0];
            double omega_dy_2f = omega_d_2f[1];
            double omega_dz_2f = omega_d_2f[2];

            Vector<double> omega_f_2f = vb.Dense(new double[] { omega_dx_2f, omega_dy_2f, omega_dz_2f });
            omega_fx_2f = omega_f_2f[0];
            omega_fy_2f = omega_f_2f[1];
            omega_fz_2f = omega_f_2f[2];

            kai_b2f = current_kai - kai_f;
            gamma_b2f = current_gamma - gamma_f;
        }

        public void record()
        {
            //notify(obj, "RecordPlaneStateEvent");
        }


        void calculatePneumaticParameters()
        {
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
            double CY_0 = CY_alpha0 * Cos(2 * current_beta / 3);
            CY_alpha = Cos(2 * current_beta / 3) *
                (CY_alpha3 * Math.Pow(current_alpha, 2) + CY_alpha2 * current_alpha + CY_alpha1);
            double CY_delta_e = CY_delta_e3 * Math.Pow(current_alpha, 3) + CY_delta_e2 * Math.Pow(current_alpha, 2) + CY_delta_e1 * current_alpha + CY_delta_e0;
            double CY_delta_lef = Cnormal_delta_lef * Cos(current_alpha) - Caxis_delta_lef * Sin(current_alpha);
            CY_delta_tef = Cnormal_delta_tef * Cos(current_alpha) - Caxis_delta_tef * Sin(current_alpha);
            CY = CY_0
                + CY_alpha * current_alpha
                + CY_delta_e * current_delta_e
                + CY_delta_lef * current_delta_lef // leading - egde flap && tailing - edge flap
                + CY_delta_tef * current_delta_tef; // lift coefficient

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
            double CD_0 = 1.5036 + CD_alpha0 * Cos(current_beta);
            CD_alpha = Cos(current_beta) * (CD_alpha4 * Math.Pow(current_alpha, 3) + CD_alpha3 * Math.Pow(current_alpha, 2) + CD_alpha2 * current_alpha + CD_alpha1);
            double CD_delta_e = CD_delta_e3 * Math.Pow(current_alpha, 3) + CD_delta_e2 * Math.Pow(current_alpha, 2) + CD_delta_e1 * current_alpha + CD_delta_e0;
            double CD_delta_lef = (Caxis_delta_lef * Cos(current_alpha) + Cnormal_delta_lef * Sin(current_alpha));
            double CD_delta_tef = (Caxis_delta_tef * Cos(current_alpha) + Cnormal_delta_tef * Sin(current_alpha));
            CD = CD_0
                + CD_alpha * current_alpha
                + CD_delta_e * current_delta_e
                + CD_delta_lef * current_delta_lef // leading - egde flap && tailing - edge flap
                + CD_delta_tef * current_delta_tef; // drag coefficient

            // 侧力系数 弧度制
            double CC_beta = -0.012 * 180 / Pi;
            CC = CC_beta * current_beta;
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
            // +CC_delta_a * current_delta_a...
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
            CL_beta = CL_beta4 * Math.Pow(current_alpha, 4) + CL_beta3 * Math.Pow(current_alpha, 3) + CL_beta2 * Math.Pow(current_alpha, 2) + CL_beta1 * current_alpha + CL_beta0;
            CL_delta_a = CL_delta_a3 * Math.Pow(current_alpha, 3) + CL_delta_a2 * Math.Pow(current_alpha, 2) + CL_delta_a1 * current_alpha + CL_delta_a0;
            CL_delta_r = CL_delta_r3 * Math.Pow(current_alpha, 3) + CL_delta_r2 * Math.Pow(current_alpha, 2) + CL_delta_r1 * current_alpha + CL_delta_r0;
            double CL_p = CL_p1 * current_alpha + CL_p0;
            double CL_r = CL_r2 * Math.Pow(current_alpha, 2) + CL_r1 * current_alpha + CL_r0;
            CL = CL_beta * current_beta
                + CL_delta_a * current_delta_a
                + CL_delta_r * current_delta_r
                + wing_L / 2 / current_Vk * CL_p * current_p
                + wing_L / 2 / current_Vk * CL_r * current_r; // rolling moment coefficient

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
            double CM_0 = CM_alpha2 * Math.Pow(current_alpha, 2) + CM_alpha1 * current_alpha + CM_alpha0;
            CM_delta_e = CM_delta_e2 * Math.Pow(current_alpha, 2) + CM_delta_e1 * current_alpha + CM_delta_e0;
            double CM_q = CM_q3 * Math.Pow(current_alpha, 3) + CM_q2 * Math.Pow(current_alpha, 2) + CM_q1 * current_alpha + CM_q0;
            CM_0 = CM_0 * Pi / 180;
            CM = CM_0
                + CM_delta_e * current_delta_e
                + wing_C / 2 / current_Vk * CM_q * current_q
                + CM_delta_lef * current_delta_lef + CM_delta_tef * current_delta_tef; // pitching moment coefficient

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
            CN_beta = CN_beta2 * Math.Pow(current_alpha, 2) + CN_beta1 * current_alpha + CN_beta0;
            CN_delta_r = CN_delta_r4 * Math.Pow(current_alpha, 4) + CN_delta_r3 * Math.Pow(current_alpha, 3) + CN_delta_r2 * Math.Pow(current_alpha, 2) + CN_delta_r1 * current_alpha + CN_delta_r0;
            CN_delta_a = CN_delta_a3 * Math.Pow(current_alpha, 3) + CN_delta_a2 * Math.Pow(current_alpha, 2) + CN_delta_a1 * current_alpha + CN_delta_a0;
            double CN_p = CN_p1 * current_alpha + CN_p0;
            double CN_r = CN_r1 * current_alpha + CN_r0;
            CN = CN_beta * current_beta
                +CN_delta_r * current_delta_r
                +CN_delta_a * current_delta_a
                +wing_L / 2 / current_Vk * CN_p * current_p
                +wing_L / 2 / current_Vk * CN_r * current_r; // yawing moment coefficient

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

        void calculateForceAndMoment()
        {
            current_Y = current_Q * wing_S * CY;
            current_D = current_Q * wing_S * CD;
            current_C = current_Q * wing_S * CC;
            current_L = current_Q * wing_S * wing_L * CL;
            current_M = current_Q * wing_S * wing_C * CM;
            current_N = current_Q * wing_S * wing_L * CN;
        }

        void updateState(double dt, disturbance, config)
        {
            calculatePneumaticParameters();
            calculateForceAndMoment();

            disturbance.updateState(obj, config);

            current_p_dot = 1 / (Ixx * Izz - Math.Pow(Ixz, 2)) * ((Iyy * Izz - Math.Pow(Izz, 2) - Math.Pow(Ixz, 2)) * current_q * current_r + (Ixx * Ixz + Izz * Ixz - Iyy * Ixz) * current_p * current_q...
                + Izz * current_L...
                +Ixz * current_N) ...
                +disturbance.disturbance_p;

            current_q_dot = 1 / Iyy * ((Izz - Ixx) * current_p * current_r - Ixz * Math.Pow(current_p, 2) + Ixz * Math.Pow(current_r, 2) + current_M)...
                +disturbance.disturbance_q;

            current_r_dot = 1 / (Ixx * Izz - Math.Pow(Ixz, 2)) * ((Math.Pow(Ixx, 2) + Math.Pow(Ixz, 2) - Ixx * Iyy) * current_p * current_q + (Iyy * Ixz - Ixx * Ixz - Izz * Ixz) * current_q * current_r...
                + Ixz * current_L...
                +Ixx * current_N) ...
                +disturbance.disturbance_r;
            current_X4_dot = [current_p_dot, current_q_dot, current_r_dot]';

            current_p = current_p + current_p_dot * dt;
            current_q = current_q + current_q_dot * dt;
            current_r = current_r + current_r_dot * dt;

            ev = XChangedEventArgs(dt, current_X4_dot);
            notify(obj, "X4ChangedEvent", ev);

            // 欧拉角
            current_phi_dot = current_p + tan(current_theta) * (current_q * Sin(current_phi) + current_r * Cos(current_phi));
            current_theta_dot = current_q * Cos(current_phi) - current_r * Sin(current_phi);
            current_psi_dot = 1 / Cos(current_theta) * (current_q * Sin(current_phi) + current_r * Cos(current_phi));
            current_phi = current_phi + current_phi_dot * dt;
            current_theta = current_theta + current_theta_dot * dt;
            current_psi = current_psi + current_psi_dot * dt;

            // 绕质心运动学方程
            current_beta_dot = current_p * Sin(current_alpha) - current_r * Cos(current_alpha) - derive_gamma * Sin(current_miu) + derive_kai * Cos(current_miu) * Cos(current_gamma);
            current_miu_dot = (current_p * Cos(current_alpha) + current_r * Sin(current_alpha) + derive_gamma * Sin(current_beta) * Cos(current_miu) + derive_kai * (Sin(current_gamma) * ...
                    Cos(current_beta) + Sin(current_beta) * Sin(current_miu) * Cos(current_gamma))) / Cos(current_beta);
            current_alpha_dot = (current_q * Cos(current_beta) - (current_p * Cos(current_alpha) + current_r * Sin(current_alpha)) * Sin(current_beta) - derive_gamma * Cos(current_miu)...


                    - derive_kai * Sin(current_miu) * Cos(current_gamma)) / Cos(current_beta);

            current_X3_dot = [current_alpha_dot, current_beta_dot, current_miu_dot]';

                // update state parameters
            current_alpha = current_alpha + current_alpha_dot * dt;
            current_beta = current_beta + current_beta_dot * dt;
            current_miu = current_miu + current_miu_dot * dt;

            ev = XChangedEventArgs(dt, current_X3_dot);
            notify(obj, "X3ChangedEvent", ev);

            // 质心动力方程
            current_kai_dot = 1 / (m * current_Vk * Cos(current_gamma)) * (current_T * (Sin(current_alpha + engine_delta) * Sin(current_miu) - Cos(current_alpha + engine_delta)...
                * Sin(current_beta) * Cos(current_miu))...
                +current_C * Cos(current_miu)...
                +current_Y * Sin(current_miu)) ...
                +disturbance.disturbance_kai;
            current_gamma_dot = -1 / (m * current_Vk) * (current_T * (-Sin(current_alpha + engine_delta) * Cos(current_miu) - Cos(current_alpha + engine_delta) * Sin(current_beta) * Sin(current_miu))...
                + current_C * Sin(current_miu)...
                -current_Y * Cos(current_miu)...
                +m * g * Cos(current_gamma)) ...
                +disturbance.disturbance_gamma;
            current_X2_dot = [current_kai_dot, current_gamma_dot]';

            current_Vk_dot = 1 / m * (current_T * Cos(current_alpha + engine_delta) * Cos(current_beta)...
                - current_D...
                -m * g * Sin(current_gamma)) +disturbance.disturbance_Vk;

            // update state parameters
            current_kai = current_kai + current_kai_dot * dt;
            current_gamma = current_gamma + current_gamma_dot * dt;

            // previous_delta_p = current_delta_p;
            current_delta_p = current_T / T_max;
            current_Vk = current_Vk + current_Vk_dot * dt;
            current_Q = 0.5 * rou * (current_Vk) ^ 2;

            derive_gamma = current_gamma_dot;
            derive_kai = current_kai_dot;

            ev = XChangedEventArgs(dt, current_X2_dot);
            notify(obj, "X2ChangedEvent", ev);

            // 质心运动方程
            current_x_dot = current_Vk * Cos(current_gamma) * Cos(current_kai);
            current_y_dot = current_Vk * Cos(current_gamma) * Sin(current_kai);
            current_z_dot = -current_Vk * Sin(current_gamma);
            current_X1_dot = [current_y_dot, current_z_dot]';
                // update state parameters
            current_position(1) = current_position(1) + current_x_dot * dt;
            current_position(2) = current_position(2) + current_y_dot * dt;
            current_position(3) = current_position(3) + current_z_dot * dt;
            ev = XChangedEventArgs(dt, current_X1_dot);
            notify(obj, "X1ChangedEvent", ev);
        }

        void reset(Ship ship)
        {
            var current_position_ship = ship.current_position_ship;
            double theta_s = ship.theta_s;
            double psi_s = ship.psi_s;

            current_alpha = 6.0 * Pi / 180; // 5.0
            current_miu = 0 * Pi / 180;
            current_beta = 0 * Pi / 180;
            current_phi = 0; // 欧拉角
            current_psi = 0;
            current_p = 0 * Pi / 180;
            current_q = 0 * Pi / 180;
            current_r = 0 * Pi / 180;
            current_Vk = 72; // 70
            current_delta_a = 0 * Pi / 180;
            current_delta_e = 0 * Pi / 180;
            current_delta_r = 0 * Pi / 180;
            current_delta_p = 0.100; // 0.10
            current_delta_lef = 33 * Pi / 180 * 1; // leading - edge flap
            current_delta_tef = 45 * Pi / 180 * 1; // tailing - edge flap

            derive_gamma = 0;
            derive_kai = 0;
            current_Q = 0.5 * rou * Math.Pow(current_Vk, 2);
            current_T = T_max * current_delta_p;

            calculatePneumaticParameters();
            calculateForceAndMoment();
            initialize(ship);
            current_theta = current_gamma + current_alpha;
            current_desired_position = [current_position(1), ideal_path(current_position, current_position_ship, theta_s, psi_s)]';
            current_delta_tef_desired = current_delta_tef;
        }
    }
}
