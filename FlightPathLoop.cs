﻿// 要不让每个控制模块维护一个plane和ship的引用吧，用得很多
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
        Vector<double> U1 = vb.Dense(2, 0);

        // State Variable
        public Vector<double> X2;

        // Output Variable
        public Vector<double> U2;

        // Interior Variable
        Matrix<double> epsilonX2 = mb.DenseDiagonal(2, 0.707);
        Matrix<double> omegaX2 = mb.DenseDiagonal(2, 50);

        // 反步法参数
        Matrix<double> k2_backstepping = mb.DenseDiagonal(2, 0.5); // 0.5
        double k_alpha_backstepping = 2.8; // 自动油门系数 5
        double k_Vk_backstepping = 3.0;

        // 有DMC有omega
        double k_kai_mpf = 0.6; // 用于移动路径跟踪的控制参数
        double k_gamma_mpf = 3;

        // 无3D移动路径跟踪 直接升力控制参数
        double k_kai = 1.0;
        double k_gamma = 30;

        // 滤波器参数
        int sampleNumber = 1;
        int TFilterBufferIndex = 0; // 推力
        Vector<double> TFilterBuffer;

        // 观测器输出变量
        Vector<double> F2;
        Matrix<double> B2;
        double F_alpha;
        double B_alpha;
        double F_Vk;
        double B_Vk;
        double F_chi;
        double B_chi;
        double F_gamma;
        double B_gamma;
        double F_chi_b2f;
        double B_chi_b2f;
        double F_gamma_b2f;
        double B_gamma_b2f;

        // Nonlinear Observer
        double current_NDO_p_chi_b2f = 0;
        double current_NDO_p_gamma_b2f = 0;
        double current_NDO_p_Vk = 0;
        double NDO_d_chi_b2f_output;
        double NDO_d_gamma_b2f_output;
        double NDO_d_Vk_output;

        double NDO_l_chi_b2f = 5;
        double NDO_l_gamma_b2f = 10;
        double NDO_l_Vk = 10;

        double epsilonGamma = 0.000001;
        double epsilonChi = 0.000001;

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
        Vector<double> filterdU1;
        Vector<double> previousU2;
        Vector<double> deriveX2 = vb.Dense(2, 0); //[kai,gamma]'
        double desiredDeltaTEF;
        double previousDesiredDeltaTEF;
        double filteredDeltaTEF;
        double previousT;

        //event RecordFlightPathLoopEvent;
        //event RecordFlightPathLoopVarEvent;

        public FlightPathLoop(Plane jet, Ship ac)
        {
            plane = jet;
            ship = ac;
            TFilterBuffer = vb.Dense(sampleNumber, 0);
            X2 = vb.Dense(new[]
                { plane.Chi, plane.Gamma });
            U2 = vb.Dense(new[] { plane.DesiredParameter.Alpha, 0, 0 });
            filterdU1 = U1;
            previousU2 = U2;
            previousT = plane.T;
            desiredDeltaTEF = plane.DeltaTEF;
            filteredDeltaTEF = desiredDeltaTEF;
            //addlistener(plane, 'X2ChangedEvent', @updateState);
        }

        public void CalculateFilter(double dt)
        {
            TFilterBuffer[TFilterBufferIndex] = plane.T;
            TFilterBufferIndex++;

            if (TFilterBufferIndex >= sampleNumber)
            {
                TFilterBufferIndex = 0;
            }

            plane.T = TFilterBuffer.Sum() / sampleNumber;

            // // dynamic performance. new added in mk4 .1
            // thrust_tau_e = 0.650;
            // previous_filter_delta_p = current_filter_delta_p;
            // current_filter_delta_p = (thrust_tau_e) / (thrust_tau_e + dt * sample_num_delta_p) * previous_filter_delta_p + dt * sample_num_delta_p / (thrust_tau_e + dt * sample_num_delta_p) * current_delta_p;
            // current_delta_p = current_filter_delta_p;
        }

        public void CalculateLimiter(double dt)
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
            if (desiredDeltaTEF < plane.DeltaTEFRange[0])
            {
                desiredDeltaTEF = plane.DeltaTEFRange[0];
            }
            else if (desiredDeltaTEF > plane.DeltaTEFRange[1])
            {
                desiredDeltaTEF = plane.DeltaTEFRange[1];
            }
            // Description    : 后缘襟翼变化速率限制
            double derive_delta_tef = (desiredDeltaTEF - previousDesiredDeltaTEF) / dt;
            if (derive_delta_tef < plane.DeltaTEFRateRange[0])
            {
                desiredDeltaTEF = previousDesiredDeltaTEF + plane.DeltaTEFRateRange[0] * dt;
            }
            else if (derive_delta_tef > plane.DeltaTEFRateRange[1])
            {
                desiredDeltaTEF = previousDesiredDeltaTEF + plane.DeltaTEFRateRange[1] * dt;
            }
            double previous_delta_tef_filtered = filteredDeltaTEF;
            filteredDeltaTEF = 1 / 30.0 / (1 / 30.0 + dt * 1) * previous_delta_tef_filtered
                + dt * 1 / (1 / 30.0 + dt * 1) * desiredDeltaTEF;
            plane.DeltaTEF = filteredDeltaTEF;

            // Description    : 角度变化幅度限制
            // 角度变化幅度限制
            if (U2[0] < plane.ThetaRange[0])
            {
                U2[0] = plane.ThetaRange[0];
            }
            if (U2[0] > plane.ThetaRange[1])
            {
                U2[0] = plane.ThetaRange[1];
            }

            if (U2[2] < plane.MiuRange[0])
            {
                U2[2] = plane.MiuRange[0];
            }
            if (U2[2] > plane.MiuRange[1])
            {
                U2[2] = plane.MiuRange[1];
            }

            // Description    : 角度变化速率限制
            // 角度变化速率限制
            var derive_u2 = (U2 - previousU2) / dt;

            if (derive_u2[0] < plane.ThetaRateRange[0])
            {
                U2[0] = previousU2[0] + plane.ThetaRateRange[0] * dt;
            }
            if (derive_u2[0] > plane.ThetaRateRange[1])
            {
                U2[0] = previousU2[0] + plane.ThetaRateRange[1] * dt;
            }

            if (derive_u2[1] < plane.MiuRateRange[0])
            {
                U2[1] = previousU2[0] + plane.MiuRateRange[0] * dt;
            }
            if (derive_u2[1] > plane.MiuRateRange[1])
            {
                U2[1] = previousU2[0] + plane.MiuRateRange[1] * dt;
            }


            // Description    : 油门变化幅度限制
            // 油门变化幅度限制
            if (plane.T < plane.ThrustRange[0])
            {
                plane.T = plane.ThrustRange[0];
            }
            if (plane.T > plane.ThrustRange[1])
            {
                plane.T = plane.ThrustRange[1];
            }

            // Description    : 油门变化速度限制
            // 油门变化速度限制
            double derive_T = (plane.T - previousT) / dt;

            if (derive_T < plane.ThrustRateRange[0])
            {
                plane.T = previousT + plane.ThrustRateRange[0] * dt;
            }
            if (derive_T > plane.ThrustRateRange[1])
            {
                plane.T = previousT + plane.ThrustRateRange[1] * dt;
            }
        }

        public void CalculateNonlinearObserver(double dt, Disturbance disturbance)
        {
            double WingS = plane.PlaneInertia.WingS;
            double Mass = plane.PlaneInertia.Mass;

            double previous_NDO_p_kai_b2f = current_NDO_p_chi_b2f;
            double derive_NDO_p_kai_b2f = -NDO_l_chi_b2f
                * (NDO_l_chi_b2f * plane.kai_b2f + current_NDO_p_chi_b2f
                + F_chi_b2f + B_chi_b2f * plane.Miu);
            current_NDO_p_chi_b2f = previous_NDO_p_kai_b2f + derive_NDO_p_kai_b2f * dt;
            double NDO_d_kai_b2f = current_NDO_p_chi_b2f + NDO_l_chi_b2f * plane.kai_b2f;
            NDO_d_chi_b2f_output = disturbance.wind_disturbance_start ? NDO_d_kai_b2f : 0;
            //if (disturbance.wind_disturbance_start)
            //    NDO_d_kai_b2f_output = NDO_d_kai_b2f;
            //else
            //    NDO_d_kai_b2f_output = 0;
            //end

            double previous_NDO_p_gamma_b2f = current_NDO_p_gamma_b2f;
            double derive_NDO_p_gamma_b2f = -NDO_l_gamma_b2f
                * (NDO_l_gamma_b2f * plane.gamma_b2f + current_NDO_p_gamma_b2f
                + F_gamma_b2f + B_gamma_b2f * plane.DeltaTEF);
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
                * (NDO_l_Vk * plane.Vk + current_NDO_p_Vk
                + F_Vk + B_Vk * plane.T);
            current_NDO_p_Vk = previous_NDO_p_Vk + derive_NDO_p_Vk * dt;
            double NDO_d_Vk = current_NDO_p_Vk + NDO_l_Vk * plane.Vk;
            NDO_d_Vk_output = disturbance.wind_disturbance_start ? NDO_d_Vk : 0;
            //if (disturbance.wind_disturbance_start > 0)
            //    NDO_d_Vk_output = NDO_d_Vk;
            //else
            //    NDO_d_Vk_output = 0;
            //end

            double a_temp = (plane.D + plane.Flow * WingS * plane.CY_alpha)
                * Sin(plane.Miu)
                / (Cos(plane.Gamma) * Mass * plane.Vk);
            double b_temp = (plane.Flow * WingS * plane.CC_beta * Cos(plane.Miu)
                - plane.D * Cos(plane.Miu)) / (Cos(plane.Gamma)
                * Mass * plane.Vk);
            double c_temp = (plane.D + plane.Flow * WingS * plane.CY_alpha)
                * Cos(plane.Miu)
                / (Mass * plane.Vk);
            double d_temp = (-plane.Flow * WingS * plane.CC_beta * Sin(plane.Miu)
                + plane.D * Sin(plane.Miu))
                / (Mass * plane.Vk);

            double wind_estimation_NDO = plane.Vk
                * ((Cos(plane.gamma_b2f) / Cos(plane.Gamma) * NDO_d_kai_b2f) * d_temp
                - NDO_d_gamma_b2f * b_temp) / (a_temp * d_temp - b_temp * c_temp); // 通过干扰观测器观测disturbance_gamma反推出的风速
            double wind_estimation_NDO_lat = -plane.Vk * (NDO_d_gamma_b2f * a_temp
                - (Cos(plane.gamma_b2f) / Cos(plane.Gamma) * NDO_d_kai_b2f) * c_temp)
                / (a_temp * d_temp - b_temp * c_temp);

            //ev = XChangedEventArgs(NDO_d_kai_b2f, NDO_d_gamma_b2f, NDO_d_Vk, wind_estimation_NDO, wind_estimation_NDO_lat);
            //notify(obj, "RecordFlightPathLoopVarEvent", ev);
        }

        public void CalculateObservation()
        {
            double WingS = plane.PlaneInertia.WingS;
            double Mass = plane.PlaneInertia.Mass;
            double G = plane.PlaneInertia.G;

            double EngineDelta = plane.DesiredParameter.EngineDelta; // 发动机安装角

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
            double F2_1 = (plane.T * (
                -Cos(plane.Alpha + EngineDelta) * Sin(plane.Beta)
                * Cos(plane.Miu)) + plane.C * Cos(plane.Miu))
                / (Mass * plane.Vk * Cos(plane.Gamma));
            double F2_2 = (plane.T * (-Sin(plane.Alpha + EngineDelta) * Cos(plane.Miu)
                - Cos(plane.Alpha + EngineDelta) * Sin(plane.Beta)
                * Sin(plane.Miu)) + plane.C * Sin(plane.Miu)
                - (plane.Y - plane.Flow * WingS * (plane.CY_alpha * plane.Alpha)) * Cos(plane.Miu)
                + Mass * G * Cos(plane.Gamma))
                / (-Mass * plane.Vk);
            F2 = vb.Dense(new[] { F2_1, F2_2 });
            B2 = 1 / (Mass * plane.Vk) * mb.DenseOfArray(new[,] {
                { (plane.T * Sin(plane.Alpha + EngineDelta) + plane.Y)
                    / Cos(plane.Gamma), 0},
                { 0, plane.Flow* WingS *plane.CY_alpha }});

            // 迎角控制环
            F_alpha = plane.Q - plane.Y / (Mass * plane.Vk)
                + G * Cos(plane.Gamma) / (plane.Vk);
            // B_alpha = -Sin(current_alpha + engine_delta) / (m * current_Vk) * T_max;
            B_alpha = -Sin(plane.Alpha + EngineDelta) / (Mass * plane.Vk);

            // 速度控制环
            F_Vk = -(1 / Mass) * (plane.D + Mass * G * Sin(plane.Gamma));
            // B_Vk = (1 / m) * (Cos(current_alpha + engine_delta) * Cos(current_beta) * T_max);
            B_Vk = (1 / Mass) * (Cos(plane.Alpha + EngineDelta) * Cos(plane.Beta));

            // 直接升力控制
            F_chi = (1 / (Mass * plane.Vk * Cos(plane.Gamma)))
                * ((plane.T * Sin(plane.Alpha) + plane.Y)
                * (Sin(plane.Miu) - plane.Miu)
                + (-plane.T * Cos(plane.Alpha) * Sin(plane.Beta) + plane.C)
                * Cos(plane.Miu));
            B_chi = (plane.T * Sin(plane.Alpha) + plane.Y)
                / (Mass * plane.Vk * Cos(plane.Gamma));

            F_gamma = -(1 / (Mass * plane.Vk))
                * (plane.T * (-Sin(plane.Alpha) * Cos(plane.Miu)
                - Cos(plane.Alpha) * Sin(plane.Beta) * Sin(plane.Miu))
                + plane.C * Sin(plane.Miu)
                + Mass * G * Cos(plane.Gamma)
                - (plane.Y - plane.Flow * WingS * plane.CY_delta_tef * plane.DeltaTEF)
                * Cos(plane.Miu));
            B_gamma = (plane.Flow * WingS * Cos(plane.Miu) * plane.CY_delta_tef)
                / (Mass * plane.Vk);

            // 3D移动路径跟踪
            F_chi_b2f = (Cos(plane.Gamma) / Cos(plane.gamma_b2f)) * F_chi
                - Tan(plane.gamma_b2f) * Cos(plane.kai_b2f) * plane.omega_fy_2f
                - plane.omega_fz_2f;
            B_chi_b2f = (Cos(plane.Gamma) / Cos(plane.gamma_b2f)) * B_chi;

            F_gamma_b2f = F_gamma - Cos(plane.kai_b2f) * plane.omega_fy_2f;
            B_gamma_b2f = B_gamma;
        }

        public void CalculateOutput()
        {
            // 直接升力控制
            double deriveChiDesired = deriveX2[0]; // derive_kai 在 observer.m 文件中出现，不要重复使用
            double deriveGammaDesired = deriveX2[1]; // derive_gamma 在 observer.m 文件中出现，不要重复使用
            double eChi = e2[0];
            double eGamma = e2[1];
            previousDesiredDeltaTEF = desiredDeltaTEF;
            double current_desired_miu = 0;
            if (Configuration.AttitudeController == AttitudeConfig.IDLC)
            {
                if (Configuration.GuidanceController == GuidanceConfig.G3dMPF)
                {
                    // Description    : 3D移动路径跟踪
                    if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NDO)// 判断使用何种干扰观测器
                    {
                        current_desired_miu = 1 / B_chi_b2f * (-F_chi_b2f + k_kai_mpf * eChi + deriveChiDesired
                            - NDO_d_chi_b2f_output - epsilonChi * plane.Vk * plane.y_b_2f); // 使用NDO
                        desiredDeltaTEF = 1 / B_gamma_b2f * (-F_gamma_b2f + k_gamma_mpf * eGamma + deriveGammaDesired
                            - NDO_d_gamma_b2f_output + epsilonGamma * plane.Vk * plane.z_b_2f);
                    }
                    else if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NONE)
                    {
                        current_desired_miu = (-F_chi_b2f + k_kai_mpf * eChi + deriveChiDesired)
                            / B_chi_b2f; // 不使用干扰观测器
                        //desiredDeltaTEF = (-F_gamma_b2f + k_gamma_mpf * eGamma + deriveGammaDesired)
                            /// B_gamma_b2f;
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
                            (-F_chi + k_kai * eChi + deriveChiDesired) / B_chi; // 不使用干扰观测器
                        desiredDeltaTEF =
                            (-F_gamma + k_gamma * eGamma + deriveGammaDesired) / B_gamma;
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

            previousU2 = U2;
            double delta_alpha = plane.DesiredParameter.Alpha - plane.Alpha;
            if (delta_alpha > 0.2 * Math.PI / 180)
            {
                delta_alpha = 0.2 * Math.PI / 180;
            }
            else if (delta_alpha < -0.2 * Math.PI / 180)
            {
                delta_alpha = -0.2 * Math.PI / 180;
            }
            U2 = vb.Dense(new double[] {
                U1[1] + plane.gamma_b2f - plane.Gamma + plane.DesiredParameter.Alpha - delta_alpha,
                0,
                current_desired_miu
            });

            if (Configuration.AttitudeController == AttitudeConfig.IDLC)
            {
                // 综合直接升力控制，油门控制速度
                if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NDO)// 判断使用何种干扰观测器
                {
                    plane.T =
                        (-F_Vk + k_Vk_backstepping * current_err_Vk - NDO_d_Vk_output) / B_Vk; // 保持速度
                }
                else if (Configuration.DisturbanceObserver == DisturbanceObserverConfig.NONE)
                {
                    plane.T =
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

        public void CalculateState(double dt, Vector<double> input)
        {
            double Mass = plane.PlaneInertia.Mass;
            double G = plane.PlaneInertia.G;

            U1 = input;

            if (Configuration.attitude_command_filter_flag)// 判断使用何种滤波器
            {
                // 使用指令滤波器
                if (Configuration.GuidanceController == GuidanceConfig.G3dMPF)
                {
                    // Description    : 3D移动路径跟踪
                    //kai_b2f = plane.kai_b2f;
                    //gamma_b2f = plane.gamma_b2f;

                    var derive2_X2 = -2 * epsilonX2 * omegaX2 * deriveX2 - omegaX2.Power(2) * (filterdU1 - U1);
                    deriveX2 += derive2_X2 * dt;
                    filterdU1 += deriveX2 * dt;
                    var current_path_angle_b2f = vb.Dense(new[] { plane.kai_b2f, plane.gamma_b2f });
                    e2 = filterdU1 - current_path_angle_b2f; // 特别注意，3D - MPF中，e2为kai_b2f gamma_b2f的跟踪误差
                }
                else
                {
                    // Description    : 不使用3D移动路径跟踪
                    var derive2_X2 = -2 * epsilonX2 * omegaX2 * deriveX2 - omegaX2.Power(2) * (filterdU1 - U1);
                    deriveX2 += derive2_X2 * dt;
                    filterdU1 += deriveX2 * dt;
                    e2 = filterdU1 - X2;
                }
            }
            else
            {
                Console.WriteLine("请指定滤波器种类 id 21");
            }
            // Description    : 自动油门控制器
            // 自动油门
            var previous_T = plane.T;

            // Description    : 保持迎角自动油门参量
            var previous_err_alpha = current_err_alpha;
            current_err_alpha = plane.Alpha - plane.DesiredParameter.Alpha; // 保持迎角自动油门P项
                                                                                   // err_alpha = plane.desired_alpha - plane.current_alpha;
            alphaI_sum = alphaI_sum + current_err_alpha * dt; // 保持迎角自动油门I项
            var derive_err_alpha = (current_err_alpha - previous_err_alpha) / dt; // 保持迎角 自动油门D项

            // Description    : 保持速度自动油门参量
            var previous_err_Vk = current_err_Vk;
            current_err_Vk = plane.DesiredParameter.Vk - plane.Vk;
            VkI_sum = VkI_sum + current_err_Vk * dt; // 保持速度自动油门I项
            var derive_err_Vk = (current_err_Vk - previous_err_Vk) / dt; // 保持速度自动油门D项

            var accel_z = (plane.Y - Mass * G * Cos(plane.Theta)) / Mass;
            filter_accel_z = accel_z_T * filter_accel_z / (accel_z_T + dt) + dt * accel_z / (accel_z_T + dt);
            filter_current_delta_e = current_delta_e_T * filter_current_delta_e / (current_delta_e_T + dt) + dt * plane.DeltaE / (current_delta_e_T + dt);
            // current_delta_p = 95 * current_err_alpha + 40 * alphaI_sum + 0 * derive_err_alpha + 0 * filter_accel_z + 0 * filter_Uact(2) + 0 * current_q;   // 使用PID控制油门 迎角稳定 "linearized model of carrier-based aircraft dynamics in final-approach air condition"
            // current_delta_p = 6.532 * current_err_Vk + 4.7286 * VkI_sum + 1.42 * derive_err_Vk + 0 * filter_accel_z + 0 * filter_current_delta_e;   // 使用PID控制油门 速度稳定
            // current_delta_p = B_alpha \ (-F_alpha + k_alpha_backstepping * err_alpha - NDO_d_alpha); // 使用非线性方法控制油门，保持迎角
        }

        public void Record(double dt)
        {
            //ev = XChangedEventArgs(dt, e2, current_err_alpha, derive_X2, previous_u2);
            //notify(obj, "RecordFlightPathLoopEvent", ev);
        }

        public void Reset()
        {
            U1 = vb.Dense(2, 0);
            TFilterBuffer = vb.Dense(sampleNumber, 0);

            TFilterBuffer = vb.Dense(sampleNumber, 0);
            X2 = vb.Dense(new[]
                { plane.Chi, plane.Gamma });
            U2 = vb.Dense(new[] { plane.DesiredParameter.Alpha, 0, 0 });
            filterdU1 = U1;
            previousU2 = U2;
            previousT = plane.T;
            desiredDeltaTEF = plane.DeltaTEF;
            filteredDeltaTEF = desiredDeltaTEF;

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
            deriveX2 = vb.Dense(2, 0); //[kai,gamma]'

            current_NDO_p_chi_b2f = 0;
            current_NDO_p_gamma_b2f = 0;
            current_NDO_p_Vk = 0;
        }

        public void UpdateState(double dt, Disturbance disturbance)
        {
            //dt = e.data{ 1};
            //current_X2_dot = e.data{ 2};
            //current_X2 = current_X2 + current_X2_dot * dt;
        }

        public void OnUpdateState(object sender, XChangedEventArgs e)
        {
            X2 += e.Data * e.Dt;
        }
    }
}
