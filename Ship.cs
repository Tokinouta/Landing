using static MathNet.Numerics.Constants;
using static MathNet.Numerics.Trig;
using MathNet.Numerics.LinearAlgebra;


namespace CsharpVersion
{
    class Ship
    {
        static readonly VectorBuilder<double> v = Vector<double>.Build;

        double current_velocity_ship = 27 * 1.852 * 5 / 18; // 航母速度，27knot
        double theta_s = 9 * Pi / 180; // 斜角甲板角度 9度
        double gamma_s = -3.5 * Pi / 180; // 期望下滑角 3.5度
        double psi_s = 0 * Pi / 180; // 航母初始偏转角度
        double omega_dx_2i = 0;
        double omega_dy_2i = 0;
        double omega_dz_2i = -0.2 * Pi / 180; // 航母转动角速度，惯性系下表示 -0.2
        Vector<double> omega_d_2i;
        Vector<double> current_position_ship = v.Dense(3, 0); // 航母初始位置  特别注意，current_position_ship仅代表航母直线前进位置，未考虑甲板起伏与侧向偏移
        Vector<double> current_deck_position_ship; // current_deck_position在top文件中有定义

        // 甲板运动补偿参数
        bool deck_enable = false; // 是否使能甲板运动补偿 0:不使能 1:使能
        int deck_motion_count = 1;
        int deck_motion_count_lat = 1; // new added in mk4.1
        double deck_compensation_position = 1600; // 甲板运动补偿开始作用距离
        double deck_compensation_start_range = 0.01;
        int deck_compensation_start_flag = 0; // 1开启DMC 0关闭DMC，等待补偿指令足够小
        int deck_compensation_start_flag_lat = 0; // 1开启DMC 0关闭DMC，等待补偿指令足够小 用于横侧向补偿 new added in mk4.1

        int cal_count;
        Vector<double> forward_filter_state;
        Vector<double> current_deck_predict;
        Vector<double> current_deck_control;
        Vector<double> current_deck_control_lat;
        double derive_deck_control = 0;
        double derive_deck_control_lat = 0;

        int deck_compensation_start_count = 5;
        int deck_compensation_start_count_lat = 5;

        // event RecordShipStateEvent;

        public Ship()
        {
            deck_enable = Configuration.deck_enable;
            omega_d_2i = v.Dense(new double[] { omega_dx_2i, omega_dy_2i, omega_dz_2i });
            current_deck_position_ship = current_position_ship;
            if (deck_enable)
            {
                // Code For Import Forward filter model
                //open_system('forward_filter');
                //[obj.cal_count, obj.forward_filter_state, obj.current_deck_predict, obj.current_deck_control, obj.current_deck_control_lat] = sim('forward_filter');
            }
        }

        public void updateState(double dt)
        {
            double x_ship_dot = current_velocity_ship * Cos(psi_s);
            double y_ship_dot = current_velocity_ship * Sin(psi_s);
            current_position_ship[0] += x_ship_dot * dt; // 更新航母位置
            current_position_ship[1] += y_ship_dot * dt;
            current_position_ship[2] = current_position_ship[2];
            psi_s += omega_dz_2i * dt;
        }

        void calculateCompensation(double dt, plane, positionLoop, config, step_count)
        {
            if (deck_enable)
            {
                if (deck_compensation_start_flag < deck_compensation_start_count)
                {
                    deck_compensation_start_flag++;
                }
                else
                {
                    deck_motion_count++;
                    positionLoop.current_desired_X1(2) = positionLoop.current_desired_X1(2) - obj.current_deck_control(step_count);
                }

                // lateral deck motion, new added in mk4.1
                if (deck_compensation_start_flag_lat < deck_compensation_start_count_lat)
                {
                    deck_compensation_start_flag_lat++;
                }
                else
                {
                    deck_motion_count_lat++;
                    positionLoop.current_desired_X1(1) = positionLoop.current_desired_X1(1) + obj.current_deck_control_lat(step_count);

                }
            }
        }

        void record()
        {
            notify(obj, "RecordShipStateEvent");
        }


        void reset()
        {
            theta_s = 9 * Pi / 180; // 斜角甲板角度 9度
            gamma_s = -3.5 * Pi / 180; // 期望下滑角 3.5度
            psi_s = 0 * Pi / 180; // 航母初始偏转角度
            omega_dx_2i = 0;
            omega_dy_2i = 0;
            omega_dz_2i = -0.2 * Pi / 180; // 航母转动角速度，惯性系下表示 - 0.2
            // 这里需要考虑避免这种方式，没有必要每次都重新分配一块内存，
            // 比如可以考虑用备份变量的形式
            omega_d_2i = v.Dense(new double[] { omega_dx_2i, omega_dy_2i, omega_dz_2i });
            current_position_ship = v.Dense(3, 0); // 航母初始位置  特别注意，current_position_ship仅代表航母直线前进位置，未考虑甲板起伏与侧向偏移

            deck_motion_count = 1;
            deck_motion_count_lat = 1; // new added in mk4.1
            deck_compensation_position = 1600; // 甲板运动补偿开始作用距离
            deck_compensation_start_range = 0.01;
            deck_compensation_start_flag = 0; // 1开启DMC 0关闭DMC，等待补偿指令足够小
            deck_compensation_start_flag_lat = 0; // 1开启DMC 0关闭DMC，等待补偿指令足够小 用于横侧向补偿 new added in mk4.1
            derive_deck_control = 0;
            derive_deck_control_lat = 0;

            deck_compensation_start_count = 5;
            deck_compensation_start_count_lat = 5;
        }
    }
}
