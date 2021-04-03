// 要不让每个控制模块维护一个plane和ship的引用吧，用得很多
using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Trig;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    public class FlightPathLoop : IControlModule
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;

        Plane plane;
        Ship ship;

        // Input Variable
        Vector<double> current_u1 = vb.Dense(2, 0);

        // State Variable
        public Vector<double> current_X2;

        // Output Variable
        public Vector<double> current_u2;

        // Interior Variable
        Matrix<double> epsilon_X2 = mb.DenseDiagonal(2, 0.7);
        Matrix<double> omega_X2 = mb.DenseDiagonal(2, 40);

        // 反步法参数
        Matrix<double> k2_backstepping = mb.DenseDiagonal(2, 0.5); // 0.5
        double k_alpha_backstepping = 5; // 自动油门系数 5
        double k_Vk_backstepping = 3.0;

        // 有DMC有omega
        double k_kai_mpf = 0.6; // 用于移动路径跟踪的控制参数
        double k_gamma_mpf = 3;

        // 无3D移动路径跟踪 直接升力控制参数
        double k_kai = 1.0;
        double k_gamma = 30;

        // 滤波器参数
        int sample_num_T = 1;
        int current_T_index_count = 0; // 推力
        Vector<double> current_T_index;

        // 观测器输出变量
        Vector<double> F2;
        Matrix<double> B2;
        double F_alpha;
        double B_alpha;
        double F_Vk;
        double B_Vk;
        double F_kai;
        double B_kai;
        double F_gamma;
        double B_gamma;
        double F_kai_b2f;
        double B_kai_b2f;
        double F_gamma_b2f;
        double B_gamma_b2f;

        // Nonlinear Observer
        double current_NDO_p_kai_b2f = 0;
        double current_NDO_p_gamma_b2f = 0;
        double current_NDO_p_Vk = 0;
        double NDO_d_kai_b2f_output;
        double NDO_d_gamma_b2f_output;
        double NDO_d_Vk_output;

        double NDO_l_kai_b2f = 5;
        double NDO_l_gamma_b2f = 10;
        double NDO_l_Vk = 10;

        double epsilon_gamma = 0.000001;
        double epsilon_kai = 0.000001;

        // 自动油门相关参数
        double alphaI_sum = 0;
        double VkI_sum = 0;
        double current_err_alpha = 0;
        double current_err_Vk = 0;
        double current_delta_e_T = 0.10; // 升降舵惯性环节参数
        double accel_z_T = 0.10; // 法向过载惯性环节参数
        double filter_accel_z = 0;
        double filter_current_delta_e = 0;

        // 中间变量
        Vector<double> e2;
        Vector<double> filter_u1;
        Vector<double> previous_u2;
        Vector<double> derive_X2 = vb.Dense(2, 0); //[kai,gamma]'
        double current_delta_tef_desired;
        double previous_delta_tef_desired;
        double current_delta_tef_filtered;
        double previous_T;

        //event RecordFlightPathLoopEvent;
        //event RecordFlightPathLoopVarEvent;

        public FlightPathLoop(Plane jet, Ship ac)
        {
            plane = jet;
            ship = ac;
            current_T_index = vb.Dense(sample_num_T, 0);
            current_X2 = vb.Dense(new[]
                { plane.current_kai, plane.current_gamma });
            current_u2 = vb.Dense(new[] { plane.desired_alpha, 0, 0 });
            filter_u1 = current_u1;
            previous_u2 = current_u2;
            previous_T = plane.current_T;
            current_delta_tef_desired = plane.current_delta_tef;
            current_delta_tef_filtered = current_delta_tef_desired;
            //addlistener(plane, 'X2ChangedEvent', @updateState);
        }

        public void calculateFilter(double dt)
        {
            current_T_index[current_T_index_count] = plane.current_T;
            current_T_index_count++;

            if (current_T_index_count >= sample_num_T)
            {
                current_T_index_count = 0;
            }

            plane.current_T = current_T_index.Sum() / sample_num_T;

            // // dynamic performance. new added in mk4 .1
            // thrust_tau_e = 0.650;
            // previous_filter_delta_p = current_filter_delta_p;
            // current_filter_delta_p = (thrust_tau_e) / (thrust_tau_e + dt * sample_num_delta_p) * previous_filter_delta_p + dt * sample_num_delta_p / (thrust_tau_e + dt * sample_num_delta_p) * current_delta_p;
            // current_delta_p = current_filter_delta_p;
        }

        public void calculateLimiter(double dt)
        {
            //delta_tef_range = plane.delta_tef_range;
            //delta_tef_rate_range = plane.delta_tef_rate_range;
            //theta_range = plane.theta_range;
            //miu_range = plane.miu_range;
            //theta_rate_range = plane.theta_rate_range;
            //miu_rate_range = plane.miu_rate_range;
            //thrust_range = plane.thrust_range;
            //thrust_rate_range = plane.thrust_rate_range;

            // Description    : 后缘襟翼变化幅度限制
            if (current_delta_tef_desired < plane.delta_tef_range[0])
            {
                current_delta_tef_desired = plane.delta_tef_range[0];
            }
            else if (current_delta_tef_desired > plane.delta_tef_range[1])
            {
                current_delta_tef_desired = plane.delta_tef_range[1];
            }
            // Description    : 后缘襟翼变化速率限制
            double derive_delta_tef = (current_delta_tef_desired - previous_delta_tef_desired) / dt;
            if (derive_delta_tef < plane.delta_tef_rate_range[0])
            {
                current_delta_tef_desired = previous_delta_tef_desired + plane.delta_tef_rate_range[0] * dt;
            }
            else if (derive_delta_tef > plane.delta_tef_rate_range[1])
            {
                current_delta_tef_desired = previous_delta_tef_desired + plane.delta_tef_rate_range[1] * dt;
            }
            double previous_delta_tef_filtered = current_delta_tef_filtered;
            current_delta_tef_filtered = 1 / 30.0 / (1 / 30.0 + dt * 1) * previous_delta_tef_filtered
                + dt * 1 / (1 / 30.0 + dt * 1) * current_delta_tef_desired;
            plane.current_delta_tef = current_delta_tef_filtered;

            // Description    : 角度变化幅度限制
            // 角度变化幅度限制
            if (current_u2[0] < plane.theta_range[0])
            {
                current_u2[0] = plane.theta_range[0];
            }
            if (current_u2[0] > plane.theta_range[1])
            {
                current_u2[0] = plane.theta_range[1];
            }

            if (current_u2[2] < plane.miu_range[0])
            {
                current_u2[2] = plane.miu_range[0];
            }
            if (current_u2[2] > plane.miu_range[1])
            {
                current_u2[2] = plane.miu_range[1];
            }

            // Description    : 角度变化速率限制
            // 角度变化速率限制
            var derive_u2 = (current_u2 - previous_u2) / dt;

            if (derive_u2[0] < plane.theta_rate_range[0])
            {
                current_u2[0] = previous_u2[0] + plane.theta_rate_range[0] * dt;
            }
            if (derive_u2[0] > plane.theta_rate_range[1])
            {
                current_u2[0] = previous_u2[0] + plane.theta_rate_range[1] * dt;
            }

            if (derive_u2[1] < plane.miu_rate_range[0])
            {
                current_u2[1] = previous_u2[0] + plane.miu_rate_range[0] * dt;
            }
            if (derive_u2[1] > plane.miu_rate_range[1])
            {
                current_u2[1] = previous_u2[0] + plane.miu_rate_range[1] * dt;
            }


            // Description    : 油门变化幅度限制
            // 油门变化幅度限制
            if (plane.current_T < plane.thrust_range[0])
            {
                plane.current_T = plane.thrust_range[0];
            }
            if (plane.current_T > plane.thrust_range[1])
            {
                plane.current_T = plane.thrust_range[1];
            }

            // Description    : 油门变化速度限制
            // 油门变化速度限制
            double derive_T = (plane.current_T - previous_T) / dt;

            if (derive_T < plane.thrust_rate_range[0])
            {
                plane.current_T = previous_T + plane.thrust_rate_range[0] * dt;
            }
            if (derive_T > plane.thrust_rate_range[1])
            {
                plane.current_T = previous_T + plane.thrust_rate_range[1] * dt;
            }
        }

        public void calculateNonlinearObserver(double dt, Disturbance disturbance)
        {
            //kai_b2f = plane.kai_b2f;
            //gamma_b2f = plane.gamma_b2f;
            //current_gamma = plane.current_gamma;
            //current_miu = plane.current_miu;
            //current_Vk = plane.current_Vk;
            //current_Q = plane.current_Q;
            //current_D = plane.current_D;
            //current_delta_tef = plane.current_delta_tef;
            //current_T = plane.current_T;
            //m = plane.m;
            //wing_S = plane.wing_S;
            //CY_alpha = plane.CY_alpha;
            //CC_beta = plane.CC_beta;

            double previous_NDO_p_kai_b2f = current_NDO_p_kai_b2f;
            double derive_NDO_p_kai_b2f = -NDO_l_kai_b2f
                * (NDO_l_kai_b2f * plane.kai_b2f + current_NDO_p_kai_b2f
                + F_kai_b2f + B_kai_b2f * plane.current_miu);
            current_NDO_p_kai_b2f = previous_NDO_p_kai_b2f + derive_NDO_p_kai_b2f * dt;
            double NDO_d_kai_b2f = current_NDO_p_kai_b2f + NDO_l_kai_b2f * plane.kai_b2f;
            NDO_d_kai_b2f_output = disturbance.wind_disturbance_start ? NDO_d_kai_b2f : 0;
            //if (disturbance.wind_disturbance_start)
            //    NDO_d_kai_b2f_output = NDO_d_kai_b2f;
            //else
            //    NDO_d_kai_b2f_output = 0;
            //end

            double previous_NDO_p_gamma_b2f = current_NDO_p_gamma_b2f;
            double derive_NDO_p_gamma_b2f = -NDO_l_gamma_b2f
                * (NDO_l_gamma_b2f * plane.gamma_b2f + current_NDO_p_gamma_b2f
                + F_gamma_b2f + B_gamma_b2f * plane.current_delta_tef);
            current_NDO_p_gamma_b2f = previous_NDO_p_gamma_b2f + derive_NDO_p_gamma_b2f * dt;
            double NDO_d_gamma_b2f = current_NDO_p_gamma_b2f + NDO_l_gamma_b2f * plane.gamma_b2f;
            NDO_d_gamma_b2f_output = disturbance.wind_disturbance_start ? NDO_d_gamma_b2f : 0;
            //if (disturbance.wind_disturbance_start > 0)
            //    NDO_d_gamma_b2f_output = NDO_d_gamma_b2f;
            //else
            //    NDO_d_gamma_b2f_output = 0;
            //end

            double previous_NDO_p_Vk = current_NDO_p_Vk;
            double derive_NDO_p_Vk = -NDO_l_Vk
                * (NDO_l_Vk * plane.current_Vk + current_NDO_p_Vk
                + F_Vk + B_Vk * plane.current_T);
            current_NDO_p_Vk = previous_NDO_p_Vk + derive_NDO_p_Vk * dt;
            double NDO_d_Vk = current_NDO_p_Vk + NDO_l_Vk * plane.current_Vk;
            NDO_d_Vk_output = disturbance.wind_disturbance_start ? NDO_d_Vk : 0;
            //if (disturbance.wind_disturbance_start > 0)
            //    NDO_d_Vk_output = NDO_d_Vk;
            //else
            //    NDO_d_Vk_output = 0;
            //end

            double a_temp = (plane.current_D + plane.current_Q * Plane.wing_S * plane.CY_alpha)
                * Sin(plane.current_miu)
                / (Cos(plane.current_gamma) * Plane.m * plane.current_Vk);
            double b_temp = (plane.current_Q * Plane.wing_S * plane.CC_beta * Cos(plane.current_miu)
                - plane.current_D * Cos(plane.current_miu)) / (Cos(plane.current_gamma)
                * Plane.m * plane.current_Vk);
            double c_temp = (plane.current_D + plane.current_Q * Plane.wing_S * plane.CY_alpha)
                * Cos(plane.current_miu)
                / (Plane.m * plane.current_Vk);
            double d_temp = (-plane.current_Q * Plane.wing_S * plane.CC_beta * Sin(plane.current_miu)
                + plane.current_D * Sin(plane.current_miu))
                / (Plane.m * plane.current_Vk);

            double wind_estimation_NDO = plane.current_Vk
                * ((Cos(plane.gamma_b2f) / Cos(plane.current_gamma) * NDO_d_kai_b2f) * d_temp
                - NDO_d_gamma_b2f * b_temp) / (a_temp * d_temp - b_temp * c_temp); // 通过干扰观测器观测disturbance_gamma反推出的风速
            double wind_estimation_NDO_lat = -plane.current_Vk * (NDO_d_gamma_b2f * a_temp
                - (Cos(plane.gamma_b2f) / Cos(plane.current_gamma) * NDO_d_kai_b2f) * c_temp)
                / (a_temp * d_temp - b_temp * c_temp);

            //ev = XChangedEventArgs(NDO_d_kai_b2f, NDO_d_gamma_b2f, NDO_d_Vk, wind_estimation_NDO, wind_estimation_NDO_lat);
            //notify(obj, "RecordFlightPathLoopVarEvent", ev);
        }

        public void calculateObservation()
        {
            //kai_b2f = plane.kai_b2f;
            //gamma_b2f = plane.gamma_b2f;

            //engine_delta = plane.engine_delta;
            //current_alpha = plane.current_alpha;
            //current_beta = plane.current_beta;
            //current_delta_tef = plane.current_delta_tef;
            //current_gamma = plane.current_gamma;
            //current_miu = plane.current_miu;
            //current_Vk = plane.current_Vk;
            //current_Q = plane.current_Q;
            //current_C = plane.current_C;
            //current_D = plane.current_D;
            //current_Y = plane.current_Y;
            //current_T = plane.current_T;
            //current_q = plane.current_q;
            //g = plane.g;
            //m = plane.m;
            //omega_fy_2f = plane.omega_fy_2f;
            //omega_fz_2f = plane.omega_fz_2f;
            //wing_S = plane.wing_S;
            //CY_alpha = plane.CY_alpha;
            //CY_delta_tef = plane.CY_delta_tef;

            // 航迹控制环
            // 保持迎角
            double F2_1 = (plane.current_T * (
                -Cos(plane.current_alpha + plane.engine_delta) * Sin(plane.current_beta)
                * Cos(plane.current_miu)) + plane.current_C * Cos(plane.current_miu))
                / (Plane.m * plane.current_Vk * Cos(plane.current_gamma));
            double F2_2 = (plane.current_T * (-Sin(plane.current_alpha + plane.engine_delta) * Cos(plane.current_miu)
                - Cos(plane.current_alpha + plane.engine_delta) * Sin(plane.current_beta)
                * Sin(plane.current_miu)) + plane.current_C * Sin(plane.current_miu)
                - (plane.current_Y - plane.current_Q * Plane.wing_S * (plane.CY_alpha * plane.current_alpha)) * Cos(plane.current_miu)
                + Plane.m * Plane.g * Cos(plane.current_gamma))
                / (-Plane.m * plane.current_Vk);
            F2 = vb.Dense(new[] { F2_1, F2_2 });
            B2 = 1 / (Plane.m * plane.current_Vk) * mb.DenseOfArray(new[,] {
                { (plane.current_T * Sin(plane.current_alpha + plane.engine_delta) + plane.current_Y)
                    / Cos(plane.current_gamma), 0},
                { 0, plane.current_Q* Plane.wing_S *plane.CY_alpha }});

            // 迎角控制环
            F_alpha = plane.current_q - plane.current_Y / (Plane.m * plane.current_Vk)
                + Plane.g * Cos(plane.current_gamma) / (plane.current_Vk);
            // B_alpha = -Sin(current_alpha + engine_delta) / (m * current_Vk) * T_max;
            B_alpha = -Sin(plane.current_alpha + plane.engine_delta) / (Plane.m * plane.current_Vk);

            // 速度控制环
            F_Vk = -(1 / Plane.m) * (plane.current_D + Plane.m * Plane.g * Sin(plane.current_gamma));
            // B_Vk = (1 / m) * (Cos(current_alpha + engine_delta) * Cos(current_beta) * T_max);
            B_Vk = (1 / Plane.m) * (Cos(plane.current_alpha + plane.engine_delta) * Cos(plane.current_beta));

            // 直接升力控制
            F_kai = (1 / (Plane.m * plane.current_Vk * Cos(plane.current_gamma)))
                * ((plane.current_T * Sin(plane.current_alpha) + plane.current_Y)
                * (Sin(plane.current_miu) - plane.current_miu)
                + (-plane.current_T * Cos(plane.current_alpha) * Sin(plane.current_beta) + plane.current_C)
                * Cos(plane.current_miu));
            B_kai = (plane.current_T * Sin(plane.current_alpha) + plane.current_Y)
                / (Plane.m * plane.current_Vk * Cos(plane.current_gamma));

            F_gamma = -(1 / (Plane.m * plane.current_Vk))
                * (plane.current_T * (-Sin(plane.current_alpha) * Cos(plane.current_miu)
                - Cos(plane.current_alpha) * Sin(plane.current_beta) * Sin(plane.current_miu))
                + plane.current_C * Sin(plane.current_miu)
                + Plane.m * Plane.g * Cos(plane.current_gamma)
                - (plane.current_Y - plane.current_Q * Plane.wing_S * plane.CY_delta_tef * plane.current_delta_tef)
                * Cos(plane.current_miu));
            B_gamma = (plane.current_Q * Plane.wing_S * Cos(plane.current_miu) * plane.CY_delta_tef)
                / (Plane.m * plane.current_Vk);

            // 3D移动路径跟踪
            F_kai_b2f = (Cos(plane.current_gamma) / Cos(plane.gamma_b2f)) * F_kai
                - Tan(plane.gamma_b2f) * Cos(plane.kai_b2f) * plane.omega_fy_2f
                - plane.omega_fz_2f;
            B_kai_b2f = (Cos(plane.current_gamma) / Cos(plane.gamma_b2f)) * B_kai;

            F_gamma_b2f = F_gamma - Cos(plane.kai_b2f) * plane.omega_fy_2f;
            B_gamma_b2f = B_gamma;
        }

        public void calculateOutput()
        {
            //current_Vk = plane.current_Vk;

            // 直接升力控制
            double derive_kai_desired = derive_X2[0]; // derive_kai 在 observer.m 文件中出现，不要重复使用
            double derive_gamma_desired = derive_X2[1]; // derive_gamma 在 observer.m 文件中出现，不要重复使用
            double e_kai = e2[0];
            double e_gamma = e2[1];
            previous_delta_tef_desired = current_delta_tef_desired;
            double current_desired_miu = 0;
            if (Configuration.AttitudeController == AttitudeConfig.IDLC)
            {
                if (Configuration.GuidanceController == GuidanceConfig.G3dMPF)
                {
                    // Description    : 3D移动路径跟踪
                    if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NDO)// 判断使用何种干扰观测器
                    {
                        current_desired_miu = 1 / B_kai_b2f * (-F_kai_b2f + k_kai_mpf * e_kai + derive_kai_desired
                            - NDO_d_kai_b2f_output - epsilon_kai * plane.current_Vk * plane.y_b_2f) ; // 使用NDO
                        current_delta_tef_desired = 1 / B_gamma_b2f * (-F_gamma_b2f + k_gamma_mpf * e_gamma + derive_gamma_desired
                            - NDO_d_gamma_b2f_output + epsilon_gamma * plane.current_Vk * plane.z_b_2f) ;
                    }
                    else if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NONE)
                    {
                        current_desired_miu = (-F_kai_b2f + k_kai_mpf * e_kai + derive_kai_desired)
                            / B_kai_b2f; // 不使用干扰观测器
                        current_delta_tef_desired = (-F_gamma_b2f + k_gamma_mpf * e_gamma + derive_gamma_desired)
                            / B_gamma_b2f;
                    }
                    else
                    {
                        Console.WriteLine("请指定干扰观测器种类 id 23");
                        //pause();
                    }
                }
                else
                {
                    // Description    : 不使用3D移动路径跟踪
                    if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NONE)
                    {
                        current_desired_miu =
                            (-F_kai + k_kai * e_kai + derive_kai_desired) / B_kai; // 不使用干扰观测器
                        current_delta_tef_desired =
                            (-F_gamma + k_gamma * e_gamma + derive_gamma_desired) / B_gamma;
                    }
                    else
                    {
                        Console.WriteLine("请指定干扰观测器种类 id 23");
                        //pause();
                    }
                }

            }
            else
            {

                Console.WriteLine("请指定控制器种类 id 22");
                //pause();
            }

            previous_u2 = current_u2;
            current_u2 = vb.Dense(new double[] { plane.desired_alpha, 0, current_desired_miu });

            if (Configuration.AttitudeController == AttitudeConfig.IDLC)
            {
                // 综合直接升力控制，油门控制速度
                if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NDO)// 判断使用何种干扰观测器
                {
                    plane.current_T =
                        (-F_Vk + k_Vk_backstepping * current_err_Vk - NDO_d_Vk_output) / B_Vk; // 保持速度
                }
                else if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NONE)
                {
                    plane.current_T =
                        (-F_Vk + k_Vk_backstepping * current_err_Vk) / B_Vk; // backstepping control without disturbance observer 保持速度
                }
                else
                {
                    Console.WriteLine("请指定油门控制器所用干扰观测器 id 26");
                    //pause();
                }
            }
            else
            {
                Console.WriteLine("请指定油门控制器 id 24");
                //pause();
            }
        }

        public void calculateState(double dt, Vector<double> input)
        {
            current_u1 = input;

            if (Configuration.attitude_command_filter_flag)// 判断使用何种滤波器
            {
                // 使用指令滤波器
                if (Configuration.GuidanceController == GuidanceConfig.G3dMPF)
                {
                    // Description    : 3D移动路径跟踪
                    //kai_b2f = plane.kai_b2f;
                    //gamma_b2f = plane.gamma_b2f;

                    var derive2_X2 = -2 * epsilon_X2 * omega_X2 * derive_X2 - omega_X2.Power(2) * (filter_u1 - current_u1);
                    derive_X2 += derive2_X2 * dt;
                    filter_u1 += derive_X2 * dt;
                    var current_path_angle_b2f = vb.Dense(new[] { plane.kai_b2f, plane.gamma_b2f });
                    e2 = filter_u1 - current_path_angle_b2f; // 特别注意，3D - MPF中，e2为kai_b2f gamma_b2f的跟踪误差
                }
                else
                {
                    // Description    : 不使用3D移动路径跟踪
                    var derive2_X2 = -2 * epsilon_X2 * omega_X2 * derive_X2 - omega_X2.Power(2) * (filter_u1 - current_u1);
                    derive_X2 += derive2_X2 * dt;
                    filter_u1 += derive_X2 * dt;
                    e2 = filter_u1 - current_X2;
                }
            }
            else
            {
                Console.WriteLine("请指定滤波器种类 id 21");
            }
            // Description    : 自动油门控制器
            // 自动油门
            var previous_T = plane.current_T;

            // Description    : 保持迎角自动油门参量
            var previous_err_alpha = current_err_alpha;
            current_err_alpha = plane.current_alpha - plane.desired_alpha; // 保持迎角自动油门P项
                                                                           // err_alpha = plane.desired_alpha - plane.current_alpha;
            alphaI_sum = alphaI_sum + current_err_alpha * dt; // 保持迎角自动油门I项
            var derive_err_alpha = (current_err_alpha - previous_err_alpha) / dt; // 保持迎角 自动油门D项

            // Description    : 保持速度自动油门参量
            var previous_err_Vk = current_err_Vk;
            current_err_Vk = plane.desired_Vk - plane.current_Vk;
            VkI_sum = VkI_sum + current_err_Vk * dt; // 保持速度自动油门I项
            var derive_err_Vk = (current_err_Vk - previous_err_Vk) / dt; // 保持速度自动油门D项

            var accel_z = (plane.current_Y - Plane.m * Plane.g * Cos(plane.current_theta)) / Plane.m;
            filter_accel_z = accel_z_T * filter_accel_z / (accel_z_T + dt) + dt * accel_z / (accel_z_T + dt);
            filter_current_delta_e = current_delta_e_T * filter_current_delta_e / (current_delta_e_T + dt) + dt * plane.current_delta_e / (current_delta_e_T + dt);
            // current_delta_p = 95 * current_err_alpha + 40 * alphaI_sum + 0 * derive_err_alpha + 0 * filter_accel_z + 0 * filter_Uact(2) + 0 * current_q;   // 使用PID控制油门 迎角稳定 "linearized model of carrier-based aircraft dynamics in final-approach air condition"
            // current_delta_p = 6.532 * current_err_Vk + 4.7286 * VkI_sum + 1.42 * derive_err_Vk + 0 * filter_accel_z + 0 * filter_current_delta_e;   // 使用PID控制油门 速度稳定
            // current_delta_p = B_alpha \ (-F_alpha + k_alpha_backstepping * err_alpha - NDO_d_alpha); // 使用非线性方法控制油门，保持迎角
        }

        public void record(double dt)
        {
            //ev = XChangedEventArgs(dt, e2, current_err_alpha, derive_X2, previous_u2);
            //notify(obj, "RecordFlightPathLoopEvent", ev);
        }

        public void reset()
        {
            current_u1 = vb.Dense(2, 0);
            current_T_index = vb.Dense(sample_num_T, 0);

            current_T_index = vb.Dense(sample_num_T, 0);
            current_X2 = vb.Dense(new[]
                { plane.current_kai, plane.current_gamma });
            current_u2 = vb.Dense(new[] { plane.desired_alpha, 0, 0 });
            filter_u1 = current_u1;
            previous_u2 = current_u2;
            previous_T = plane.current_T;
            current_delta_tef_desired = plane.current_delta_tef;
            current_delta_tef_filtered = current_delta_tef_desired;

            // 有DMC有omega
            k_kai_mpf = 0.6; // 用于移动路径跟踪的控制参数
            k_gamma_mpf = 3;

            // 无3D移动路径跟踪 直接升力控制参数
            k_kai = 1.0;
            k_gamma = 30;

            alphaI_sum = 0;
            VkI_sum = 0;
            current_err_alpha = 0;
            current_err_Vk = 0;
            current_delta_e_T = 0.10; // 升降舵惯性环节参数
            accel_z_T = 0.10; // 法向过载惯性环节参数
            filter_accel_z = 0;
            filter_current_delta_e = 0;
            derive_X2 = vb.Dense(2, 0); //[kai,gamma]'

            current_NDO_p_kai_b2f = 0;
            current_NDO_p_gamma_b2f = 0;
            current_NDO_p_Vk = 0;
        }

        public void updateState(double dt, Disturbance disturbance)
        {
            //dt = e.data{ 1};
            //current_X2_dot = e.data{ 2};
            //current_X2 = current_X2 + current_X2_dot * dt;
        }

        public void OnUpdateState(object sender, XChangedEventArgs e)
        {
            current_X2 += e.Data * e.Dt;
        }
    }
}
