using MathNet.Numerics.LinearAlgebra;
using ModelEntities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion.Controllers
{
    class L1Adaptive : IController
    {
        public Plane Plane { get; set; }
        public Ship Ship { get; set; }
        public IControlModule ControlModule { get; set; }
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;

        readonly double k_p = 2;
        readonly double k_q = 1;
        readonly double k_r = 1;
        readonly Matrix<double> k_pr;

        double q_ndi_dot = 0;
        Vector<double> pr_ndi_dot = vb.Dense(2, 0);

        // 俯仰轴
        readonly double T_pitch = 0.5;
        readonly double Amq;
        readonly double Bmq = 1;
        double sigma_q_est = 0;
        double q_est = 0;
        readonly double T_cq = 1.0 / 5;
        readonly double w_cq;
        double q_ad_dot = 0;
        readonly double ksq = 0;
        readonly double Asq;
        // lockheed 增强参数
        readonly double k_ad_q = 1.0;

        // 横侧向
        readonly double T_roll = 2;
        readonly double T_yaw = 2;

        // 滚转轴
        readonly double Amp;
        readonly double Bmp = 1;
        double sigma_p_est = 0;
        double p_est = 0;
        readonly double T_cp = 1.0 / 5;
        readonly double w_cp;
        double p_ad_dot = 0;

        // 偏航轴
        readonly double Amr;
        readonly double Bmr = 1;
        double sigma_r_est = 0;
        double r_est = 0;
        readonly double T_cr = 1.0 / 5;
        readonly double w_cr;
        double r_ad_dot = 0;

        // 横侧向双通道
        Vector<double> pr_ad_dot { get; set; }

        public Configuration Configuration { get; }

        //Matrix<double> Ampr => mb.DenseOfDiagonalArray(new[] { -1 / T_roll, -1 / T_yaw });
        //Matrix<double> Bmpr => mb.DenseOfDiagonalArray(new[] { Bmp, Bmr });
        //Vector<double> sigma_pr_est => vb.Dense(new[] { sigma_p_est, sigma_r_est });
        //Vector<double> pr_est => vb.Dense(new[] { p_est, r_est });
        //Vector<double> pr_est_dot => vb.Dense(new[] { p_est_dot, r_est_dot });
        //Vector<double> pr_est_err => vb.Dense(new[] { p_est_err, r_est_err });
        //Matrix<double> w_cpr => mb.DenseOfDiagonalArray(new[] { w_cp, w_cr });
        //Vector<double> pr_ad_dot2 => vb.Dense(new[] { p_ad_dot2, r_ad_dot2 });
        //Vector<double> pr_mear => vb.Dense(new[] { p_mear, r_mear });


        readonly double adap_constraint = 0.5;

        public L1Adaptive(Ship ship, Plane plane, IControlModule module, Configuration config)
        {
            Ship = ship;
            Plane = plane;
            ControlModule = module;
            Configuration = config;
            k_pr = mb.DenseOfDiagonalArray(new[] { k_p, k_r });
            Amq = -1 / T_pitch;
            w_cq = 1 / T_cq;
            Asq = Amq + ksq;
            Amp = -1 / T_roll;
            w_cp = 1 / T_cp;
            Amr = -1 / T_yaw;
            w_cr = 1 / T_cr;
            pr_ad_dot = vb.Dense(new[] { p_ad_dot, r_ad_dot });
        }

        public Vector<double> CalculateOutput(double dt, double current_time, int step_count)
        {
            // 角速度指令使用二阶滤波器
            var controlModule = (AngularRateLoop)ControlModule;
            double p_ref = controlModule.FilteredU3[0];
            double q_ref = controlModule.FilteredU3[1];
            double r_ref = controlModule.FilteredU3[2];
            Vector<double> pr_ref = vb.Dense(new[] { p_ref, r_ref });

            double p_ref_dot = controlModule.DeriveX4[0];
            double q_ref_dot = controlModule.DeriveX4[1];
            double r_ref_dot = controlModule.DeriveX4[2];
            Vector<double> pr_ref_dot = vb.Dense(new[] { p_ref_dot, r_ref_dot });

            double p_mear = controlModule.X4[0];
            double q_mear = controlModule.X4[1];
            double r_mear = controlModule.X4[2];
            Vector<double> pr_mear = vb.Dense(new[] { p_mear, r_mear });


            q_ndi_dot = k_q * (q_ref - q_mear) + q_ref_dot;
            pr_ndi_dot = k_pr * (pr_ref - pr_mear) + pr_ref_dot;

            if (Configuration.UseL1Adaptive)
            {
                double r_q = k_ad_q * q_ref / T_pitch;
                double q_est_dot = Amq * q_est + Bmq * (r_q + q_ref_dot + q_ad_dot + sigma_q_est);
                q_est += q_est_dot * dt;
                double q_est_err = q_est - q_mear;
                double phi_q = (1 / Amq) * (Math.Exp(Amq * dt) - 1);
                double mu_q = Math.Exp(Amq * dt) * q_est_err;
                sigma_q_est = -(1 / Bmq) * (1 / phi_q) * mu_q;
                double q_ad_dot2 = -w_cq * q_ad_dot - w_cq * sigma_q_est;
                q_ad_dot += q_ad_dot2 * dt;

                if (q_ad_dot > adap_constraint)
                {
                    q_ad_dot = adap_constraint;
                }
                else if (q_ad_dot < -adap_constraint)
                {
                    q_ad_dot = -adap_constraint;
                }

                // 单通道设计 roll
                double r_p = p_ref / T_roll;
                double p_est_dot = Amp * p_est + Bmp * (r_p + p_ad_dot + p_ref_dot + sigma_p_est);
                p_est += p_est_dot * dt;
                double p_est_err = p_est - p_mear;
                double phi_p = (1 / Amp) * (Math.Exp(Amp * dt) - 1);
                double mu_p = Math.Exp(Amp * dt) * p_est_err;
                sigma_p_est = -(1 / Bmp) * (1 / phi_p) * mu_p;
                double p_ad_dot2 = -w_cp * p_ad_dot - w_cp * sigma_p_est;
                p_ad_dot += p_ad_dot2 * dt;

                if (p_ad_dot > adap_constraint)
                {
                    p_ad_dot = adap_constraint;
                }
                else if (p_ad_dot < -adap_constraint)
                {
                    p_ad_dot = -adap_constraint;
                }

                // 单通道设计 yaw
                double r_r = r_ref / T_yaw;
                double r_est_dot = Amr * r_est + Bmr * (r_r + r_ad_dot + r_ref_dot + sigma_r_est);
                r_est += r_est_dot * dt;
                double r_est_err = r_est - r_mear;
                double phi_r = (1 / Amr) * (Math.Exp(Amr * dt) - 1);
                double mu_r = Math.Exp(Amr * dt) * r_est_err;
                sigma_r_est = -(1 / Bmr) * (1 / phi_r) * mu_r;
                double r_ad_dot2 = -w_cr * r_ad_dot - w_cr * sigma_r_est;
                r_ad_dot += r_ad_dot2 * dt;

                if (r_ad_dot > adap_constraint)
                {
                    r_ad_dot = adap_constraint;
                }
                else if (r_ad_dot < -adap_constraint)
                {
                    r_ad_dot = -adap_constraint;
                }
                pr_ad_dot = vb.Dense(new[] { p_ad_dot, r_ad_dot });
            }
            else
            {
                q_ad_dot = 0;
                pr_ad_dot = vb.Dense(2, 0);
            }
            double q_d_dot = q_ndi_dot + q_ad_dot;

            var q_uact = (-controlModule.Fq + q_d_dot) / controlModule.Gq;
            var pr_d_dot = pr_ndi_dot + pr_ad_dot;
            var pr_uact = controlModule.Gpr.Inverse() * (-controlModule.Fpr + pr_d_dot);
            return vb.Dense(new[] { pr_uact[0], q_uact, pr_uact[1] });
        }

        public void InvokeRecordEvent()
        {
            //throw new NotImplementedException();
        }

        public void Reset()
        {
            q_ndi_dot = 0;
            pr_ndi_dot = vb.Dense(2, 0);

            sigma_q_est = 0;
            q_est = 0;
            q_ad_dot = 0;

            sigma_p_est = 0;
            p_est = 0;
            p_ad_dot = 0;

            // 偏航轴
            sigma_r_est = 0;
            r_est = 0;
            r_ad_dot = 0;

            // 横侧向双通道
            pr_ad_dot = vb.Dense(new[] { p_ad_dot, r_ad_dot });
        }
    }
}
