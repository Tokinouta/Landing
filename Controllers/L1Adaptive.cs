using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion.Controllers
{
    class L1Adaptive : IController
    {
        public Plane Plane { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }
        public Ship Ship { get => throw new NotImplementedException(); set => throw new NotImplementedException(); }
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;

        double k_p = 2;
        double k_q = 1;
        double k_r = 1;
        Matrix<double> k_pr;

        double q_ndi_dot = 0;
        Vector<double> pr_ndi_dot = vb.Dense(2, 0);

        // 俯仰轴
        double T_pitch = 0.5;
        double Amq;
        double Bmq = 1;
        double sigma_q_est = 0;
        double q_est = 0;
        double q_est_dot = 0;
        double q_est_err = 0;
        double T_cq = 1 / 5;
        double w_cq;
        double q_ad_dot = 0;
        double q_ad_dot2 = 0;
        double ksq = 0;
        double Asq;
        // lockheed 增强参数
        double hq = 0;
        double k_ad_q = 1.0;

        double q_mear = 0;

        // 横侧向
        double T_roll = 2;
        double T_yaw = 2;

        // 滚转轴
        double Amp;
        double Bmp = 1;
        double sigma_p_est = 0;
        double p_est = 0;
        double p_est_dot = 0;
        double p_est_err = 0;
        double T_cp = 1 / 5;
        double w_cp;
        double p_ad_dot = 0;
        double p_ad_dot2 = 0;

        double p_mear = 0;

        // 偏航轴
        double Amr;
        double Bmr = 1;
        double sigma_r_est = 0;
        double r_est = 0;
        double r_est_dot = 0;
        double r_est_err = 0;
        double T_cr = 1 / 5;
        double w_cr;
        double r_ad_dot = 0;
        double r_ad_dot2 = 0;

        double r_mear = 0;

        // 横侧向双通道
        Matrix<double> Ampr => mb.DenseOfDiagonalArray(new[] { -1 / T_roll, -1 / T_yaw });
        Matrix<double> Bmpr => mb.DenseOfDiagonalArray(new[] { Bmp, Bmr });
        Vector<double> sigma_pr_est => vb.Dense(new[] { sigma_p_est, sigma_r_est });
        Vector<double> pr_est => vb.Dense(new[] { p_est, r_est });
        Vector<double> pr_est_dot => vb.Dense(new[] { p_est_dot, r_est_dot });
        Vector<double> pr_est_err => vb.Dense(new[] { p_est_err, r_est_err });
        Matrix<double> w_cpr => mb.DenseOfDiagonalArray(new[] { w_cp, w_cr });
        Vector<double> pr_ad_dot
        {
            // 这里可能需要考虑改成一个能存储这个向量的字段或者属性，
            // 要不然每次用到的时候都需要新建一个向量
            // 这样可能会很耗时间
            // 到时候再看吧，毕竟这种方式其实还是挺直观的
            get => vb.Dense(new[] { p_ad_dot, r_ad_dot });
            set
            {
                p_ad_dot = value[0];
                r_ad_dot = value[1];
            }
        }
        Vector<double> pr_ad_dot2 => vb.Dense(new[] { p_ad_dot2, r_ad_dot2 });
        Vector<double> pr_mear => vb.Dense(new[] { p_mear, r_mear });

        double adap_constraint = 0.5;

        public L1Adaptive()
        {
            k_pr = mb.DenseOfDiagonalArray(new[] { k_p, k_r });
            Amq = -1 / T_pitch;
            w_cq = 1 / T_cq;
            Asq = Amq + ksq;
            Amp = -1 / T_roll;
            w_cp = 1 / T_cp;
            Amr = -1 / T_yaw;
            w_cr = 1 / T_cr;
        }

        public Vector<double> CalculateOutput(double dt, double current_time, int step_count)
        {
            throw new NotImplementedException();
        }

        public void InvokeRecordEvent()
        {
            throw new NotImplementedException();
        }

        public void Reset()
        {
            throw new NotImplementedException();
        }
    }
}
