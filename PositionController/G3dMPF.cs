using MathNet.Numerics.LinearAlgebra;
using static MathNet.Numerics.Trig;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion.PositionController
{
    class G3dMPF : IPositionController
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;

        Plane plane;
        Ship ship;

        // 3D路径跟踪
        double l_path_0 = 1620; // 期望路径参数，初始路径长度
        double k_x = 1.0;
        double k_y = 1.5;
        double k_z = 1.0;
        double l_path = 0; // 期望路径参数，路径长度参数->特别注意，l_path初始值必须为0
        double l_path_dot;
        double psi_dmc_p2i_y;
        double psi_dmc_p2i_z;

        public G3dMPF(Plane plane, Ship ship)
        {
            this.plane = plane;
            this.ship = ship;
        }

        public Plane Plane { get => plane; set => plane = value; }
        public Ship Ship { get => ship; set => ship = value; }

        public Vector<double> CalculateOutput(double dt, double current_time, int step_count)
        {
            double theta_s = ship.theta_s;
            double gamma_s = ship.gamma_s;
            double psi_s = ship.psi_s;
            double current_Vk = plane.current_Vk;

            // 更新角度参数
            double velocity_deck_total_x = ship.current_velocity_ship * Cos(psi_s);
            double velocity_deck_total_y = ship.current_velocity_ship * Sin(psi_s) + ship.derive_deck_control_lat;
            double velocity_deck_total_z = ship.derive_deck_control;
            // 由于甲板纵向运动与侧向运动引起的P系航向角与爬升角,但由于P系始终保持水平，故这两个角度不需要增加至kai_f与gamma_f中
            psi_dmc_p2i_y = Atan(ship.derive_deck_control_lat
                / Math.Sqrt(Math.Pow(velocity_deck_total_x, 2) + Math.Pow(velocity_deck_total_y, 2) + Math.Pow(velocity_deck_total_z, 2)));
            psi_dmc_p2i_z = Atan(ship.derive_deck_control
                / Math.Sqrt(Math.Pow(velocity_deck_total_x, 2) + Math.Pow(velocity_deck_total_y, 2) + Math.Pow(velocity_deck_total_z, 2)));

            double kai_f = -theta_s + psi_s;
            double gamma_f = gamma_s;
            double kai_b2f = plane.current_kai - kai_f;
            double gamma_b2f = plane.current_gamma - gamma_f;
            plane.kai_b2f = kai_b2f;
            plane.gamma_b2f = gamma_b2f;

            // 期望跟踪点坐标计算
            double x_d_2p = -(l_path_0 - l_path) * Cos(theta_s) * Cos(gamma_s); // 期望点坐标 P系下表示
            double y_d_2p = (l_path_0 - l_path) * Sin(theta_s) * Cos(gamma_s);
            double z_d_2p = (l_path_0 - l_path) * Sin(gamma_s);
            Vector<double> p_d_2p = vb.Dense(new double[] { x_d_2p, y_d_2p, z_d_2p }); // 期望点坐标 P系下表示

            var current_deck_position_ship = ship.current_position_ship; // 甲板在I系下坐标，考虑甲板起伏与侧向偏移影响
                                                                         // if (deck_compensation_start_flag > 0)
            if (ship.deck_compensation_start_flag > (ship.deck_compensation_start_count - 1))
            {
                current_deck_position_ship[1] = current_deck_position_ship[1] + ship.current_deck_control_lat[step_count];
            }
            // if (deck_compensation_start_flag_lat > 0)
            if (ship.deck_compensation_start_flag_lat > (ship.deck_compensation_start_count_lat - 1))
            {
                current_deck_position_ship[2] -= ship.current_deck_control[step_count];
            }

            Matrix<double> R_i2p = mb.DenseOfArray(new double[,]{
                { Cos(psi_s), Sin(psi_s), 0 },
                { -Sin(psi_s), Cos(psi_s), 0 },
                { 0, 0, 1 } });
            Matrix<double> R_p2i = R_i2p.Transpose();
            Vector<double> p_d_2i = R_p2i * p_d_2p + current_deck_position_ship; // 期望点坐标 I系下表示
            Matrix<double> R_i2f = mb.DenseOfArray(new double[,] {
                { Cos(gamma_f) * Cos(kai_f), Cos(gamma_f) * Sin(kai_f), -Sin(gamma_f) },
                { -Sin(kai_f), Cos(kai_f), 0 },
                { Sin(gamma_f) * Cos(kai_f), Sin(gamma_f) * Sin(kai_f), Cos(gamma_f) } });
            Vector<double> p_b_2f = R_i2f * (plane.current_position - p_d_2i); // 飞机在F坐标系中坐标
            plane.x_b_2f = p_b_2f[0];
            plane.y_b_2f = p_b_2f[1];
            plane.z_b_2f = p_b_2f[2];

            // 期望坐标点与P系原点坐标差
            Vector<double> delta_d20_xyz = p_d_2i - current_deck_position_ship;
            double delta_d20_x = delta_d20_xyz[0];
            double delta_d20_y = delta_d20_xyz[1];
            double delta_d20_z = delta_d20_xyz[2];

            // 航母运动速度
            double velocity_ship_x = ship.current_velocity_ship * Cos(psi_s); // x_ship_dot参数在dof_update.m文件中有定义
            double velocity_ship_y = ship.current_velocity_ship * Sin(psi_s); // 有y_ship_dot参数在dof_update.m文件中有定义

            // 制导律
            double omega_dx_2i = ship.omega_dx_2i;
            double omega_dy_2i = ship.omega_dy_2i;
            double omega_dz_2i = ship.omega_dz_2i; // 航母转动角速度，惯性系下表示 - 0.2

            l_path_dot = current_Vk * Cos(gamma_b2f) * Cos(kai_b2f)
                - Cos(gamma_f) * Cos(kai_f) * (velocity_ship_x + omega_dy_2i * delta_d20_z - omega_dz_2i * delta_d20_y)
                - Cos(gamma_f) * Sin(kai_f) * (velocity_ship_y + ship.derive_deck_control_lat + omega_dz_2i * delta_d20_x - omega_dx_2i * delta_d20_z)
                + Sin(gamma_f) * (ship.derive_deck_control + omega_dx_2i * delta_d20_y - omega_dy_2i * delta_d20_x)
                + k_x * plane.x_b_2f;
            double kai_b2f_desired = (1 / current_Vk)
                * (-current_Vk * (Cos(gamma_b2f) * Sin(kai_b2f) - kai_b2f)
                - Sin(kai_f) * (velocity_ship_x + omega_dy_2i * delta_d20_z - omega_dz_2i * delta_d20_y)
                + Cos(kai_f) * (velocity_ship_y + ship.derive_deck_control_lat + omega_dz_2i * delta_d20_x - omega_dx_2i * delta_d20_z)
                - k_y * plane.y_b_2f);
            double gamma_b2f_desired = (1 / current_Vk) * (-current_Vk * (Sin(gamma_b2f) - gamma_b2f) +
                            -Sin(gamma_f) * Cos(kai_f) * (velocity_ship_x + omega_dy_2i * delta_d20_z - omega_dz_2i * delta_d20_y)
                - Sin(gamma_f) * Sin(kai_f) * (velocity_ship_y + ship.derive_deck_control_lat + omega_dz_2i * delta_d20_x - omega_dx_2i * delta_d20_z)
                - Cos(gamma_f) * (ship.derive_deck_control + omega_dx_2i * delta_d20_y - omega_dy_2i * delta_d20_x)
                + k_z * plane.z_b_2f);

            // 更新l_path
            l_path += l_path_dot * dt;

            // 更新角速度参数
            Vector<double> omega_d_2f = R_i2f * ship.omega_d_2i;
            plane.omega_fx_2f = omega_d_2f[0];
            plane.omega_fy_2f = omega_d_2f[1];
            plane.omega_fz_2f = omega_d_2f[2];
            return vb.Dense(new[] { kai_b2f_desired, gamma_b2f_desired });
        }

        public void InvokeRecordEvent()
        {
            //ev = XChangedEventArgs(l_path, l_path_dot, psi_dmc_p2i_y, psi_dmc_p2i_z);
            //notify(obj, "RecordPositionLoopVarEvent", ev);
        }

        public void Reset()
        {
            l_path = 0; // 期望路径参数，路径长度参数->特别注意，l_path初始值必须为0
        }
    }
}

