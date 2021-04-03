using MathNet.Numerics.LinearAlgebra;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    public class SimulationRecord
    {
        public List<double> time_record;

        // Data record

        List<Vector<double>> position_ship_record;
        List<double> psi_s_record; // 航母偏航角记录
                                   // Data record

        List<double> delta_tef_record;
        List<Vector<double>> position_record;
        List<double> alpha_record;
        List<double> delta_p_record;
        List<double> Vk_record;
        List<double> current_Q_record;
        List<double> current_T_record;
        // 欧拉角记录
        List<double> phi_record;
        List<double> psi_record;
        List<double> theta_record;

        List<double> x_b_2f_record;
        List<double> y_b_2f_record;
        List<double> z_b_2f_record;

        List<Vector<double>> epc_record;
        List<Vector<double>> u1_record;
        List<Vector<double>> X1_record;
        List<Vector<double>> derive_X1_record;
        List<Vector<double>> x1_dot_record;
        List<Vector<double>> x2_dot_record;

        //List<Vector<double>> vector_trac_err_record;
        //List<Vector<double>> pp_xi_lon_record;
        //List<Vector<double>> pp_xi_lat_record;

        List<double> l_path_record;
        List<double> l_path_dot_record;
        List<double> psi_dmc_p2i_y_record;
        List<double> psi_dmc_p2i_z_record;
        List<double> kai_b2f_record;
        List<double> gamma_b2f_record;
        // Data record
        List<Vector<double>> e2_record;
        List<Vector<double>> u2_record;
        List<Vector<double>> X2_record;
        List<double> current_err_alpha_record;
        List<Vector<double>> derive_X2_record;
        List<Vector<double>> x3_dot_record;

        List<double> NDO_d_kai_b2f_record;
        List<double> NDO_d_gamma_b2f_record;
        List<double> NDO_d_Vk_record;

        List<double> wind_estimation_NDO;
        List<double> wind_estimation_NDO_lat;

        // Data record
        List<Vector<double>> e3_record;
        List<Vector<double>> u3_record;
        List<Vector<double>> X3_record;
        List<Vector<double>> derive_X3_record;
        List<Vector<double>> x4_dot_record;

        // Data record
        List<Vector<double>> e4_record;
        List<Vector<double>> uact_record;
        List<Vector<double>> X4_record;
        List<Vector<double>> derive_X4_record;

        List<double> NDO_d_omega_record;

        public SimulationRecord()
        {
            time_record = new List<double>();

            // Data record

            position_ship_record = new List<Vector<double>>();
            psi_s_record = new List<double>(); // 航母偏航角记录
                                       // Data record

            delta_tef_record = new List<double>();
            position_record = new List<Vector<double>>();
            alpha_record = new List<double>();
            delta_p_record = new List<double>();
            Vk_record = new List<double>();
            current_Q_record = new List<double>();
            current_T_record = new List<double>();
            // 欧拉角记录
            phi_record = new List<double>();
            psi_record = new List<double>();
            theta_record = new List<double>();

            x_b_2f_record = new List<double>();
            y_b_2f_record = new List<double>();
            z_b_2f_record = new List<double>();
            //addlistener(varargin{ 1}, "RecordShipStateEvent", @recordShipStateEventHandler);
            //addlistener(varargin{ 2}, "RecordPlaneStateEvent", @recordPlaneStateEventHandler);
            //addlistener(varargin{ 3}, "RecordPositionLoopEvent", @recordPositionLoopEventHandler);
            //addlistener(varargin{ 3}, "RecordPositionLoopVarEvent", @recordPositionLoopVarEventHandler);
            //addlistener(varargin{ 4}, "RecordFlightPathLoopEvent", @recordFlightPathLoopEventHandler);
            //addlistener(varargin{ 4}, "RecordFlightPathLoopVarEvent", @recordFlightPathLoopVarEventHandler);
            //addlistener(varargin{ 5}, "RecordAttitudeLoopEvent", @recordAttitudeLoopEventHandler);
            //addlistener(varargin{ 6}, "RecordAngularRateLoopEvent", @recordAngularRateLoopEventHandler);
            //addlistener(varargin{ 6}, "RecordAngularRateLoopVarEvent", @recordAngularRateLoopVarEventHandler);
            //addlistener(sim, "PlotEvent", @plotDotEventHandler);
        }

        void saveToFile()
        {
            // add simulation date and time
            // add simulation configuration
            // add add initial state of plane

            //sobj.time_record = time_record;
            //sobj.position_ship_record = position_ship_record;
            //sobj.psi_s_record = psi_s_record;

            //sobj.delta_tef_record = delta_tef_record;
            //sobj.position_record = position_record;
            //sobj.alpha_record = alpha_record;
            //sobj.delta_p_record = delta_p_record;
            //sobj.Vk_record = Vk_record;
            //sobj.current_Q_record = current_Q_record;
            //sobj.current_T_record = current_T_record;

            //sobj.phi_record = phi_record;
            //sobj.psi_record = psi_record;
            //sobj.theta_record = theta_record;

            //save("ra.mat", 'sobj');
        }

        public void OnRecordPlaneState(object sender, EventArgs e)
        {
            Plane plane = (Plane)sender;
            delta_tef_record.Add(plane.DeltaTEF);
            position_record.Add(plane.Position.Clone());
            alpha_record.Add(plane.Alpha);
            delta_p_record.Add(plane.DeltaP);
            Vk_record.Add(plane.Vk);
            current_Q_record.Add(plane.Flow);
            current_T_record.Add(plane.T);
            // 欧拉角记录
            phi_record.Add(plane.Phi);
            psi_record.Add(plane.Psi);
            theta_record.Add(plane.Theta);

            x_b_2f_record.Add(plane.x_b_2f);
            y_b_2f_record.Add(plane.y_b_2f);
            z_b_2f_record.Add(plane.z_b_2f);
            //kai_b2f_record.Add(plane.kai_b2f);
            //gamma_b2f_record.Add(plane.gamma_b2f);
        }
    }
}
