using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Constants;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    class Plane
    {
        private const double Ixx = 31183.40; // 转动惯量 kg/m2
        private const double Iyy = 205123.05;
        private const double Izz = 230411.43;
        private const double Ixz = 4028.08;
        private const double wing_S = 37.16; // 翼面积
        private const double wing_C = 3.51; // 平均气动弦长
        private const double wing_L = 11.41; // 翼展
        private const double m = 15097.39; // 空重 kg
        private const double T_max = 130.6 * 1000; // 军用推力（两台）
        private const double rou = 1.225; // 空气密度
        private const double g = 9.8;

        static readonly VectorBuilder<double> v = Vector<double>.Build;
        // 舵面角度及角速度限幅
        // 所有角度、角速度均以弧度为单位
        Vector<double> delta_e_range = v.Dense(new double[] { 2.0, 3.0 }) * Pi / 180; // 升降舵角度范围
        Vector<double> delta_e_rate_range = v.Dense(new double[] { -40, 40 }) * Pi / 180; // 升降舵角速度范围
        Vector<double> delta_a_range = v.Dense(new double[] { -25, 45 }) * Pi / 180;
        Vector<double> delta_a_rate_range = v.Dense(new double[] { -100, 100 }) * Pi / 180;
        Vector<double> delta_r_range = v.Dense(new double[] { -30, 30 }) * Pi / 180;
        Vector<double> delta_r_rate_range = v.Dense(new double[] { -61, 61 }) * Pi / 180;

        Vector<double> delta_tef_range = v.Dense(new double[] { -8, 45 }) * Pi / 180;
        Vector<double> delta_tef_rate_range = v.Dense(new double[] { -40, 40 }) *Pi / 180; // 调整后缘襟翼偏转速率，改善纵向轨迹跟踪性能

        Vector<double> delta_p_range = v.Dense(new double[] { 0.01, 1.0 });
        Vector<double> delta_p_rate_range = v.Dense(new double[] { -0.01, 0.01 }) * 100000;
        thrust_range;
        thrust_rate_range;

        p_range = [-179, 179] * pi / 180 * 10;
        q_range = [-179, 179] * pi / 180 * 10;
        r_range = [-179, 179] * pi / 180 * 10;
        gamma_range = [-89, 89] * pi / 180;
        kai_range = [-89, 89] * pi / 180;
        theta_range = [-89, 89] * pi / 180;
        miu_range = [-89, 89] * pi / 180;

        p_rate_range = [-20, 20] * pi / 180 * 100;
        q_rate_range = [-20, 20] * pi / 180 * 100;
        r_rate_range = [-20, 20] * pi / 180 * 100;
        gamma_rate_range = [-20, 20] * pi / 180 * 100;
        kai_rate_range = [-20, 20] * pi / 180 * 100;
        theta_rate_range = [-20, 20] * pi / 180 * 100;
        miu_rate_range = [-20, 20] * pi / 180 * 100;
        end

    % 飞机气动参数
    properties
        CY; % 升力系数 弧度制
        CD; % 阻力系数 弧度制 drag coefficient
        CC; % 侧力系数 弧度制
        CL; % 滚转力矩 弧度制 rolling moment coefficient
        CM; % 俯仰力矩 弧度制 pitching moment coefficient
        CN; % 偏航力矩 弧度制 yawing moment coefficient
    end

    % 飞机状态
    properties(SetObservable)
        current_position;
        current_kai;
        current_gamma;
        current_alpha = 6.0 * pi / 180; % 5.0
        current_miu = 0 * pi / 180;
        current_beta = 0 * pi / 180;
        current_theta;
        current_phi = 0; % 欧拉角
        current_psi = 0;
        current_p = 0 * pi / 180;
        current_q = 0 * pi / 180;
        current_r = 0 * pi / 180;
        current_Vk = 72; % 70
        current_delta_a = 0 * pi / 180;
        current_delta_e = 0 * pi / 180;
        current_delta_r = 0 * pi / 180;
        current_delta_p = 0.100; % 0.10
        current_delta_lef = 33 * pi / 180 * 1; % leading-edge flap
        current_delta_tef = 45 * pi / 180 * 1; % tailing-edge flap
        current_delta_tef_desired;
        current_Q;
        current_T;
        current_desired_position;
        derive_gamma = 0;
        derive_kai = 0;

        omega_fx_2f;
        omega_fy_2f;
        omega_fz_2f;
        x_b_2f;
        y_b_2f;
        z_b_2f;
        kai_b2f;
        gamma_b2f;

    end

    % 气动力和力矩
    properties
        current_Y;
        current_D;
        current_C;
        current_L;
        current_M;
        current_N;
    end

    properties(SetAccess = private)
        CY_alpha;
        CY_delta_tef;
        CL_delta_a;
        CL_delta_r;
        CM_delta_e;
        CN_delta_a;
        CN_delta_r;
        CC_beta;
        CD_alpha;
        CL_beta;
        CM_alpha1;
        CM_alpha2;
        CN_beta;
    end

    % 着舰过程中期望参量
    properties
        desired_alpha = 9.1 * pi / 180; % 期望迎角 9.1
        desired_kai = 0 * pi / 180; % 期望航向角
        desired_gamma = -3.5 * pi / 180; % 期望爬升角
        desired_Vk = 75; % 期望速度 71
        engine_delta = 0 * pi / 180; % 发动机安装角
    end

    events
        RecordPlaneStateEvent
    end

    events
        X1ChangedEvent;
        X2ChangedEvent;
        X3ChangedEvent;
        X4ChangedEvent;
    end

    methods

        function obj = Plane(dt, ship)
            current_position_ship = ship.current_position_ship;
            theta_s = ship.theta_s;
            psi_s = ship.psi_s;

            obj.current_Q = 0.5 * obj.rou* (obj.current_Vk)^2;
            obj.current_T = obj.T_max* obj.current_delta_p;
        obj.thrust_range = obj.T_max* obj.delta_p_range;
        obj.thrust_rate_range = obj.T_max* obj.delta_p_rate_range;
        obj = calculatePneumaticParameters(obj);
        obj = calculateForceAndMoment(obj);
        obj = obj.initialize(ship);
            obj.current_theta = obj.current_gamma + obj.current_alpha;
            obj.current_desired_position = [obj.current_position(1), ideal_path(obj.current_position, current_position_ship, theta_s, psi_s)]';
            obj.current_delta_tef_desired = obj.current_delta_tef;

        end

        function obj = addListeners(obj, varargin)
            %addListeners - Description
            %
            % Syntax: obj = addListeners(obj, vara)
            %
            % Long description
            addlistener(varargin{ 1}, 'X1ChangedEvent', @obj.X1ChangedEventHandler);
            addlistener(varargin{ 2}, 'X2ChangedEvent', @obj.X2ChangedEventHandler);
            addlistener(varargin{ 3}, 'X3ChangedEvent', @obj.X3ChangedEventHandler);
            addlistener(varargin{ 4}, 'X4ChangedEvent', @obj.X4ChangedEventHandler);

        end

        function obj = initialize(obj, ship)
            theta_s = ship.theta_s;
            psi_s = ship.psi_s;
            gamma_s = ship.gamma_s;

            l_path_0 = 1620; % 期望路径参数，初始路径长度
            l_path = 0; % 期望路径参数，路径长度参数->特别注意，l_path初始值必须为0
            x_d_2p = -(l_path_0 - l_path) * cos(theta_s) * cos(gamma_s); % 期望点坐标 P系下表示
            y_d_2p = (l_path_0 - l_path) * sin(theta_s) * cos(gamma_s);
        z_d_2p = (l_path_0 - l_path) * sin(gamma_s);
        p_d_2p = [x_d_2p, y_d_2p, z_d_2p]'; % 期望点坐标 P系下表示

            R_i2p = [cos(psi_s), sin(psi_s), 0; -sin(psi_s), cos(psi_s), 0; 0, 0, 1];
            R_p2i = R_i2p';
            p_d_2i = R_p2i* p_d_2p + ship.current_position_ship; % 期望点坐标 I系下表示

            kai_f = -theta_s + psi_s;
            gamma_f = gamma_s;

            R_i2f = [cos(gamma_f) * cos(kai_f), cos(gamma_f) * sin(kai_f), -sin(gamma_f); ...
                    -sin(kai_f), cos(kai_f), 0;
                sin(gamma_f) * cos(kai_f), sin(gamma_f) * sin(kai_f), cos(gamma_f)];
            R_f2i = R_i2f';
            p_b_2f = [0, 5, 10]'; % 飞机在F系中初始位置
            obj.x_b_2f = p_b_2f(1);
        obj.y_b_2f = p_b_2f(2);
        obj.z_b_2f = p_b_2f(3);

        obj.current_position = R_f2i* p_b_2f + p_d_2i; % 飞机在I系中初始位置
       obj.current_gamma = 0; % 初始爬升角
       obj.current_kai = -theta_s; % 初始航向角
       omega_d_2f = R_i2f * ship.omega_d_2i;
        omega_dx_2f = omega_d_2f(1);
        omega_dy_2f = omega_d_2f(2);
        omega_dz_2f = omega_d_2f(3);

        omega_f_2f = [omega_dx_2f, omega_dy_2f, omega_dz_2f]';
            obj.omega_fx_2f = omega_f_2f(1);
        obj.omega_fy_2f = omega_f_2f(2);
        obj.omega_fz_2f = omega_f_2f(3);

        obj.kai_b2f = obj.current_kai - kai_f;
            obj.gamma_b2f = obj.current_gamma - gamma_f;
        end

        function obj = record(obj)
            notify(obj, "RecordPlaneStateEvent");
        end

    end

    methods

        function obj = calculatePneumaticParameters(obj)
            % flap coefficient
            CM_delta_tef = 0.001 * 180 / pi;
            CM_delta_lef = 0;
            Cnormal_delta_tef = 0.009 * 180 / pi;
            Cnormal_delta_lef = Cnormal_delta_tef* 0.5; % 推测数据，缺少实测数据
           Caxis_delta_tef = 0;
        Caxis_delta_lef = 0;

            % 升力系数相关参数 弧度制
            CY_alpha3 = 1.1645;
            CY_alpha2 = -5.4246;
            CY_alpha1 = 5.6770;
            CY_alpha0 = -0.0204;
            CY_delta_e3 = 2.1852;
            CY_delta_e2 = -2.6975;
            CY_delta_e1 = 0.4055;
            CY_delta_e0 = 0.5725;
            CY_0 = CY_alpha0* cos(2 * obj.current_beta / 3);
        CY_alpha = cos(2 * obj.current_beta / 3) * (CY_alpha3* obj.current_alpha^2 + CY_alpha2* obj.current_alpha + CY_alpha1);
        CY_delta_e = CY_delta_e3* obj.current_alpha^3 + CY_delta_e2* obj.current_alpha^2 +CY_delta_e1* obj.current_alpha + CY_delta_e0;
        CY_delta_lef = Cnormal_delta_lef* cos(obj.current_alpha) - Caxis_delta_lef* sin(obj.current_alpha);
        CY_delta_tef = Cnormal_delta_tef* cos(obj.current_alpha) - Caxis_delta_tef* sin(obj.current_alpha);
        obj.CY = CY_0...
                + CY_alpha* obj.current_alpha ...
                + CY_delta_e* obj.current_delta_e...
                + CY_delta_lef* obj.current_delta_lef... % leading-egde flap && tailing-edge flap
                + CY_delta_tef* obj.current_delta_tef; % lift coefficient

            % 阻力系数相关参数 弧度制
            CD_alpha4 = 1.4610;
            CD_alpha3 = -5.7341;
            CD_alpha2 = 6.3971;
            CD_alpha1 = -0.1995;
            CD_alpha0 = -1.4994;
            CD_delta_e3 = -3.8578;
            CD_delta_e2 = 4.2360;
            CD_delta_e1 = -0.2739;
            CD_delta_e0 = 0.0366;
            CD_0 = 1.5036 + CD_alpha0* cos(obj.current_beta);
        CD_alpha = cos(obj.current_beta) * (CD_alpha4* obj.current_alpha^3 + CD_alpha3* obj.current_alpha^2 + CD_alpha2* obj.current_alpha + CD_alpha1);
        CD_delta_e = CD_delta_e3* obj.current_alpha^3 + CD_delta_e2* obj.current_alpha^2 + CD_delta_e1* obj.current_alpha + CD_delta_e0;
        CD_delta_lef = (Caxis_delta_lef* cos(obj.current_alpha) + Cnormal_delta_lef* sin(obj.current_alpha));
            CD_delta_tef = (Caxis_delta_tef* cos(obj.current_alpha) + Cnormal_delta_tef* sin(obj.current_alpha));
            obj.CD = CD_0...
                + CD_alpha* obj.current_alpha ...
                + CD_delta_e* obj.current_delta_e...
                + CD_delta_lef* obj.current_delta_lef... % leading-egde flap && tailing-edge flap
                + CD_delta_tef* obj.current_delta_tef; % drag coefficient

            % 侧力系数 弧度制
            CC_beta = -0.012 * 180 / pi;
            obj.CC = CC_beta* obj.current_beta;
            % CC_beta2 = -0.1926;
            % CC_beta1 = 0.2654;
            % CC_beta0 = -0.7344;
            % CC_delta_a3 = -0.8500;
            % CC_delta_a2 = 1.5317;
            % CC_delta_a1 = -0.2403;
            % CC_delta_a0 = -0.1656;
            % CC_delta_r3 = 0.9351;
            % CC_delta_r2 = -1.6921;
            % CC_delta_r1 = 0.4082;
            % CC_delta_r0 = 0.2054;
            % CC_beta = CC_beta2* obj.current_alpha^2 + CC_beta1* obj.current_alpha + CC_beta0;
            % CC_delta_a = CC_delta_a3* obj.current_alpha^3 + CC_delta_a2* obj.current_alpha^2 + CC_delta_a1* obj.current_alpha + CC_delta_a0;
            % CC_delta_r = CC_delta_r3* obj.current_alpha^3 + CC_delta_r2* obj.current_alpha^2 + CC_delta_r1* obj.current_alpha + CC_delta_r0;
            % CC = CC_beta* obj.current_beta ...
            %     + CC_delta_a* obj.current_delta_a...
            %     + CC_delta_r* obj.current_delta_r; % sidefroce coefficient

            % 滚转力矩相关参数 弧度制
            CL_beta4 = -1.6196;
            CL_beta3 = 2.3843;
            CL_beta2 = -0.3620;
            CL_beta1 = -0.4153;
            CL_beta0 = -0.0556;
            CL_delta_a3 = 0.1989;
            CL_delta_a2 = -0.2646;
            CL_delta_a1 = -0.0516;
            CL_delta_a0 = 0.1424;
            CL_delta_r3 = -0.0274;
            CL_delta_r2 = 0.0083;
            CL_delta_r1 = 0.0014;
            CL_delta_r0 = 0.0129;
            CL_p1 = 0.2377;
            CL_p0 = -0.3540;
            CL_r2 = -1.0871;
            CL_r1 = 0.7804;
            CL_r0 = 0.1983;
            CL_beta = CL_beta4* obj.current_alpha^4 + CL_beta3* obj.current_alpha^3 + CL_beta2* obj.current_alpha^2 + CL_beta1* obj.current_alpha + CL_beta0;
        CL_delta_a = CL_delta_a3* obj.current_alpha^3 + CL_delta_a2* obj.current_alpha^2 + CL_delta_a1* obj.current_alpha + CL_delta_a0;
        CL_delta_r = CL_delta_r3* obj.current_alpha^3 + CL_delta_r2* obj.current_alpha^2 + CL_delta_r1* obj.current_alpha + CL_delta_r0;
        CL_p = CL_p1* obj.current_alpha + CL_p0;
        CL_r = CL_r2* obj.current_alpha^2 + CL_r1* obj.current_alpha + CL_r0;
        obj.CL = CL_beta* obj.current_beta ...
                + CL_delta_a* obj.current_delta_a...
                + CL_delta_r* obj.current_delta_r...
                + obj.wing_L / 2 / obj.current_Vk* CL_p * obj.current_p...
                + obj.wing_L / 2 / obj.current_Vk* CL_r * obj.current_r; % rolling moment coefficient

            % 俯仰力矩相关参数 弧度制
            CM_alpha2 = -1.2897;
            CM_alpha1 = 0.5110;
            CM_alpha0 = -0.0866;
            CM_delta_e2 = 0.9338;
            CM_delta_e1 = -0.3245;
            CM_delta_e0 = -0.9051;
            CM_q3 = 64.7190;
            CM_q2 = -68.5641;
            CM_q1 = 10.9921;
            CM_q0 = -4.1186;
            CM_0 = CM_alpha2* obj.current_alpha^2 + CM_alpha1* obj.current_alpha + CM_alpha0;
        CM_delta_e = CM_delta_e2* obj.current_alpha^2 + CM_delta_e1* obj.current_alpha + CM_delta_e0;
        CM_q = CM_q3* obj.current_alpha^3 + CM_q2* obj.current_alpha^2 + CM_q1* obj.current_alpha + CM_q0;
        CM_0 = CM_0* pi / 180;
        obj.CM = CM_0...
                + CM_delta_e* obj.current_delta_e ...
                + obj.wing_C / 2 / obj.current_Vk* CM_q * obj.current_q...
                + CM_delta_lef* obj.current_delta_lef + CM_delta_tef* obj.current_delta_tef; % pitching moment coefficient

            % 偏航力矩相关参数 弧度制
            CN_beta2 = -0.3816;
            CN_beta1 = 0.0329;
            CN_beta0 = 0.0885;
            CN_delta_a3 = 0.2694;
            CN_delta_a2 = -0.3413;
            CN_delta_a1 = 0.0584;
            CN_delta_a0 = 0.0104;
            CN_delta_r4 = 0.3899;
            CN_delta_r3 = -0.8980;
            CN_delta_r2 = 0.5564;
            CN_delta_r1 = -0.0176;
            CN_delta_r0 = -0.0780;
            CN_p1 = -0.0881;
            CN_p0 = 0.0792;
            CN_r1 = -0.1307;
            CN_r0 = -0.4326;
            CN_beta = CN_beta2* obj.current_alpha^2 + CN_beta1* obj.current_alpha + CN_beta0;
        CN_delta_r = CN_delta_r4* obj.current_alpha^4 + CN_delta_r3* obj.current_alpha^3 + CN_delta_r2* obj.current_alpha^2 + CN_delta_r1* obj.current_alpha + CN_delta_r0;
        CN_delta_a = CN_delta_a3* obj.current_alpha^3 + CN_delta_a2* obj.current_alpha^2 + CN_delta_a1* obj.current_alpha + CN_delta_a0;
        CN_p = CN_p1* obj.current_alpha + CN_p0;
        CN_r = CN_r1* obj.current_alpha + CN_r0;
        obj.CN = CN_beta* obj.current_beta ...
                + CN_delta_r* obj.current_delta_r...
                + CN_delta_a* obj.current_delta_a...
                + obj.wing_L / 2 / obj.current_Vk* CN_p * obj.current_p...
                + obj.wing_L / 2 / obj.current_Vk* CN_r * obj.current_r; % yawing moment coefficient

       obj.CY_alpha = CY_alpha;
        obj.CY_delta_tef = CY_delta_tef;
            obj.CL_delta_a = CL_delta_a;
            obj.CL_delta_r = CL_delta_r;
            obj.CM_delta_e = CM_delta_e;
            obj.CN_delta_a = CN_delta_a;
            obj.CN_delta_r = CN_delta_r;
            obj.CC_beta = CC_beta;
            obj.CD_alpha = CD_alpha;
            obj.CL_beta = CL_beta;
            obj.CM_alpha1 = CM_alpha1;
            obj.CM_alpha2 = CM_alpha2;
            obj.CN_beta = CN_beta;

        end

        function obj = calculateForceAndMoment(obj)
            obj.current_Y = obj.current_Q* obj.wing_S * obj.CY;
        obj.current_D = obj.current_Q* obj.wing_S * obj.CD;
        obj.current_C = obj.current_Q* obj.wing_S * obj.CC;
        obj.current_L = obj.current_Q* obj.wing_S * obj.wing_L* obj.CL;
        obj.current_M = obj.current_Q* obj.wing_S * obj.wing_C* obj.CM;
        obj.current_N = obj.current_Q* obj.wing_S * obj.wing_L* obj.CN;
        end

        function obj = updateState(obj, dt, disturbance, config)
            obj.calculatePneumaticParameters();
            obj.calculateForceAndMoment();

            disturbance.updateState(obj, config);

            current_p_dot = 1 / (obj.Ixx* obj.Izz - obj.Ixz^2) * ((obj.Iyy* obj.Izz - obj.Izz^2 - obj.Ixz^2) * obj.current_q* obj.current_r + (obj.Ixx* obj.Ixz + obj.Izz* obj.Ixz - obj.Iyy* obj.Ixz) * obj.current_p* obj.current_q ...
                + obj.Izz* obj.current_L...
                + obj.Ixz* obj.current_N) ...
                + disturbance.disturbance_p;

            current_q_dot = 1 / obj.Iyy* ((obj.Izz - obj.Ixx) * obj.current_p* obj.current_r - obj.Ixz* obj.current_p^2 + obj.Ixz* obj.current_r^2 + obj.current_M) ...
                + disturbance.disturbance_q;

            current_r_dot = 1 / (obj.Ixx* obj.Izz - obj.Ixz^2) * ((obj.Ixx^2 + obj.Ixz^2 - obj.Ixx* obj.Iyy) * obj.current_p* obj.current_q + (obj.Iyy* obj.Ixz - obj.Ixx* obj.Ixz - obj.Izz* obj.Ixz) * obj.current_q* obj.current_r ...
                + obj.Ixz* obj.current_L...
                + obj.Ixx* obj.current_N) ...
                + disturbance.disturbance_r;
            current_X4_dot = [current_p_dot, current_q_dot, current_r_dot]';

            obj.current_p = obj.current_p + current_p_dot* dt;
        obj.current_q = obj.current_q + current_q_dot* dt;
        obj.current_r = obj.current_r + current_r_dot* dt;

        ev = XChangedEventArgs(dt, current_X4_dot);
        notify(obj, "X4ChangedEvent", ev);

            % 欧拉角
            current_phi_dot = obj.current_p + tan(obj.current_theta) * (obj.current_q * sin(obj.current_phi) + obj.current_r * cos(obj.current_phi));
        current_theta_dot = obj.current_q* cos(obj.current_phi) - obj.current_r* sin(obj.current_phi);
        current_psi_dot = 1 / cos(obj.current_theta) * (obj.current_q* sin(obj.current_phi) + obj.current_r* cos(obj.current_phi));
            obj.current_phi = obj.current_phi + current_phi_dot* dt;
        obj.current_theta = obj.current_theta + current_theta_dot* dt;
        obj.current_psi = obj.current_psi + current_psi_dot* dt;

            % 绕质心运动学方程
            current_beta_dot = obj.current_p * sin(obj.current_alpha) - obj.current_r * cos(obj.current_alpha) - obj.derive_gamma * sin(obj.current_miu) + obj.derive_kai * cos(obj.current_miu) * cos(obj.current_gamma);
        current_miu_dot = (obj.current_p* cos(obj.current_alpha) + obj.current_r* sin(obj.current_alpha) + obj.derive_gamma* sin(obj.current_beta) * cos(obj.current_miu) + obj.derive_kai* (sin(obj.current_gamma)* ...
                cos(obj.current_beta) + sin(obj.current_beta) * sin(obj.current_miu) * cos(obj.current_gamma))) / cos(obj.current_beta);
        current_alpha_dot = (obj.current_q* cos(obj.current_beta) - (obj.current_p* cos(obj.current_alpha) + obj.current_r* sin(obj.current_alpha)) * sin(obj.current_beta) - obj.derive_gamma* cos(obj.current_miu) ...
                - obj.derive_kai* sin(obj.current_miu) * cos(obj.current_gamma)) / cos(obj.current_beta);

        current_X3_dot = [current_alpha_dot, current_beta_dot, current_miu_dot]';

            % update state parameters
            obj.current_alpha = obj.current_alpha + current_alpha_dot* dt;
        obj.current_beta = obj.current_beta + current_beta_dot* dt;
        obj.current_miu = obj.current_miu + current_miu_dot* dt;

        ev = XChangedEventArgs(dt, current_X3_dot);
        notify(obj, "X3ChangedEvent", ev);

            % 质心动力方程
            current_kai_dot = 1 / (obj.m * obj.current_Vk * cos(obj.current_gamma)) * (obj.current_T * (sin(obj.current_alpha + obj.engine_delta) * sin(obj.current_miu) - cos(obj.current_alpha + obj.engine_delta)...
                * sin(obj.current_beta) * cos(obj.current_miu))...
                + obj.current_C* cos(obj.current_miu) ...
                + obj.current_Y* sin(obj.current_miu)) ...
                + disturbance.disturbance_kai;
            current_gamma_dot = -1 / (obj.m* obj.current_Vk) * (obj.current_T* (-sin(obj.current_alpha + obj.engine_delta) * cos(obj.current_miu) - cos(obj.current_alpha + obj.engine_delta) * sin(obj.current_beta) * sin(obj.current_miu)) ...
                + obj.current_C* sin(obj.current_miu) ...
                - obj.current_Y* cos(obj.current_miu) ...
                + obj.m* obj.g * cos(obj.current_gamma)) ...
                + disturbance.disturbance_gamma;
            current_X2_dot = [current_kai_dot, current_gamma_dot]';

            current_Vk_dot = 1 / obj.m* (obj.current_T* cos(obj.current_alpha + obj.engine_delta) * cos(obj.current_beta) ...
                - obj.current_D...
                - obj.m* obj.g * sin(obj.current_gamma)) + disturbance.disturbance_Vk;

            % update state parameters
            obj.current_kai = obj.current_kai + current_kai_dot* dt;
        obj.current_gamma = obj.current_gamma + current_gamma_dot* dt;

            % previous_delta_p = current_delta_p;
            obj.current_delta_p = obj.current_T / obj.T_max;
            obj.current_Vk = obj.current_Vk + current_Vk_dot* dt;
        obj.current_Q = 0.5 * obj.rou* (obj.current_Vk)^2;

            obj.derive_gamma = current_gamma_dot;
            obj.derive_kai = current_kai_dot;

            ev = XChangedEventArgs(dt, current_X2_dot);
        notify(obj, "X2ChangedEvent", ev);

            % 质心运动方程
            current_x_dot = obj.current_Vk * cos(obj.current_gamma) * cos(obj.current_kai);
        current_y_dot = obj.current_Vk* cos(obj.current_gamma) * sin(obj.current_kai);
        current_z_dot = -obj.current_Vk* sin(obj.current_gamma);
        current_X1_dot = [current_y_dot, current_z_dot]';
            %update state parameters
            obj.current_position(1) = obj.current_position(1) + current_x_dot* dt;
        obj.current_position(2) = obj.current_position(2) + current_y_dot* dt;
        obj.current_position(3) = obj.current_position(3) + current_z_dot* dt;
        ev = XChangedEventArgs(dt, current_X1_dot);
        notify(obj, "X1ChangedEvent", ev);

        end

        function obj = reset(obj, ship)
            current_position_ship = ship.current_position_ship;
            theta_s = ship.theta_s;
            psi_s = ship.psi_s;

            obj.current_alpha = 6.0 * pi / 180; % 5.0
            obj.current_miu = 0 * pi / 180;
            obj.current_beta = 0 * pi / 180;
            obj.current_phi = 0; % 欧拉角
            obj.current_psi = 0;
        obj.current_p = 0 * pi / 180;
            obj.current_q = 0 * pi / 180;
            obj.current_r = 0 * pi / 180;
            obj.current_Vk = 72; % 70
            obj.current_delta_a = 0 * pi / 180;
            obj.current_delta_e = 0 * pi / 180;
            obj.current_delta_r = 0 * pi / 180;
            obj.current_delta_p = 0.100; % 0.10
            obj.current_delta_lef = 33 * pi / 180 * 1; % leading-edge flap
            obj.current_delta_tef = 45 * pi / 180 * 1; % tailing-edge flap

            obj.derive_gamma = 0;
            obj.derive_kai = 0;
            obj.current_Q = 0.5 * obj.rou* (obj.current_Vk)^2;
            obj.current_T = obj.T_max* obj.current_delta_p;

        obj = calculatePneumaticParameters(obj);
        obj = calculateForceAndMoment(obj);
        obj = obj.initialize(ship);
            obj.current_theta = obj.current_gamma + obj.current_alpha;
            obj.current_desired_position = [obj.current_position(1), ideal_path(obj.current_position, current_position_ship, theta_s, psi_s)]';
            obj.current_delta_tef_desired = obj.current_delta_tef;
        end
    end

end

    }
}
