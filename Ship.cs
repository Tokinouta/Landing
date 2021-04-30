using static MathNet.Numerics.Constants;
using static MathNet.Numerics.Trig;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Data.Matlab;
using System;

namespace CsharpVersion
{
    public class Ship
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;

        public double Velocity = 27 * 1.852 * 5 / 18; // 航母速度，27knot
        public double Theta = 9 * Pi / 180; // 斜角甲板角度 9度
        public double Gamma = -3.5 * Pi / 180; // 期望下滑角 3.5度
        public double Psi = 0 * Pi / 180; // 航母初始偏转角度
        public double omega_dx_2i = 0;
        public double omega_dy_2i = 0;
        public double omega_dz_2i = 0; // 航母转动角速度，惯性系下表示 -0.2
        public Vector<double> omega_d_2i;
        public Vector<double> Position = vb.Dense(3, 0); // 航母初始位置  特别注意，current_position_ship仅代表航母直线前进位置，未考虑甲板起伏与侧向偏移
        public Vector<double> DeckPosition; // current_deck_position在top文件中有定义

        // 甲板运动补偿参数
        public bool DeckEnable = false; // 是否使能甲板运动补偿 0:不使能 1:使能
        public int DeckMotionCount = 1;
        public int DeckMotionLateralCount = 1; // new added in mk4.1
        public double DeckCompensationPosition = 1600; // 甲板运动补偿开始作用距离
        public double DeckCompensationStartRange = 0.01;
        public int DeckCompensationStartCount = 0; // 1开启DMC 0关闭DMC，等待补偿指令足够小
        public int DeckCompensationLateralStartCount = 0; // 1开启DMC 0关闭DMC，等待补偿指令足够小 用于横侧向补偿 new added in mk4.1

        public int CalCount;
        public Matrix<double> ForwardFilterState;
        public Vector<double> CurrentDeckPredict;
        public Vector<double> CurrentDeckControl;
        public Vector<double> CurrentDeckLateralControl;
        public double DeriveDeckControl = 0;
        public double DeriveDeckLateralControl = 0;

        public int DeckCompensationStartThreshold = 5;
        public int DeckCompensationLateralStartThreshold = 5;
        public Vector<double> vector_trac_err;

        // event RecordShipStateEvent;

        public Ship()
        {
            DeckEnable = Configuration.deck_enable;
            omega_d_2i = vb.Dense(new double[] { omega_dx_2i, omega_dy_2i, omega_dz_2i });
            DeckPosition = Position;
            if (DeckEnable)
            {
                ForwardFilterState = MatlabReader.Read<double>("./ForwardFilter.mat", "forward_filter_state");
                CurrentDeckPredict = MatlabReader.Read<double>("./ForwardFilter.mat", "current_deck_predict").Column(0);
                CurrentDeckControl = MatlabReader.Read<double>("./ForwardFilter.mat", "current_deck_control").Column(0);
                CurrentDeckLateralControl = MatlabReader.Read<double>("./ForwardFilter.mat", "current_deck_control_lat").Column(0);
            }

        }

        public void updateState(double dt)
        {
            double x_ship_dot = Velocity * Cos(Psi);
            double y_ship_dot = Velocity * Sin(Psi);
            if (y_ship_dot != 0)
            {
                Console.WriteLine("ra");
            }
            Position[0] += x_ship_dot * dt; // 更新航母位置
            Position[1] += y_ship_dot * dt;
            // Position[2] = Position[2];
            Psi += omega_dz_2i * dt;
        }

        public void calculateCompensation(double dt, Plane plane, PositionLoop positionLoop, int step_count)
        {
            if (DeckEnable && (plane.l_path > plane.l_path_0 - 1620))
            {
                if (DeckCompensationStartCount < DeckCompensationStartThreshold)
                {
                    DeckCompensationStartCount++;
                }
                else
                {
                    DeckMotionCount++;
                    positionLoop.X1Desired[1] = positionLoop.X1Desired[1] - CurrentDeckControl[step_count];
                }

                // lateral deck motion, new added in mk4.1
                if (DeckCompensationLateralStartCount < DeckCompensationLateralStartThreshold)
                {
                    DeckCompensationLateralStartCount++;
                }
                else
                {
                    DeckMotionLateralCount++;
                    positionLoop.X1Desired[0] = positionLoop.X1Desired[0] + CurrentDeckLateralControl[step_count];

                }
            }

            switch (Configuration.TrajactoryConfig)
            {
                case TrajactoryType.TypeI:
                    vector_trac_err = HelperFunction.vector_filed_trac(plane.Position, Position);
                    break;
                case TrajactoryType.TypeII:
                    vector_trac_err = HelperFunction.vector_filed_trac(plane.Position, Position);
                    if (DeckCompensationStartCount > (DeckCompensationStartThreshold - 1))
                    {
                        vector_trac_err[1] = vector_trac_err[1] - (-CurrentDeckControl[step_count]);
                        DeriveDeckControl = ((-CurrentDeckControl[step_count]) - (-CurrentDeckControl[step_count - 1])) / dt; // current_deck_control向上为正 derive_deck_control向下为正
                    }
                    else
                    {
                        //vector_trac_err(2) = vector_trac_err(2);
                        DeriveDeckControl = 0;
                    }
                    // 横向运动与补偿 new added in mk4.1
                    if (DeckCompensationLateralStartCount > (DeckCompensationLateralStartThreshold - 1))
                    {
                        vector_trac_err[0] = vector_trac_err[0] - (CurrentDeckLateralControl[step_count]);
                        DeriveDeckLateralControl = ((CurrentDeckLateralControl[step_count]) - (CurrentDeckLateralControl[step_count - 1])) / dt; // 向右为正
                    }
                    else
                    {             
                        //vector_trac_err(1) = vector_trac_err(1);
                        DeriveDeckLateralControl = 0;
                    }
                    break;
                default:
                    break;
            }
        }

        public void record()
        {
            //notify(obj, "RecordShipStateEvent");
        }

        public void reset()
        {
            Theta = 9 * Pi / 180; // 斜角甲板角度 9度
            Gamma = -3.5 * Pi / 180; // 期望下滑角 3.5度
            Psi = 0 * Pi / 180; // 航母初始偏转角度
            omega_dx_2i = 0;
            omega_dy_2i = 0;
            omega_dz_2i = -0.2 * Pi / 180; // 航母转动角速度，惯性系下表示 - 0.2
            // 这里需要考虑避免这种方式，没有必要每次都重新分配一块内存，
            // 比如可以考虑用备份变量的形式
            omega_d_2i = vb.Dense(new double[] { omega_dx_2i, omega_dy_2i, omega_dz_2i });
            Position = vb.Dense(3, 0); // 航母初始位置  特别注意，current_position_ship仅代表航母直线前进位置，未考虑甲板起伏与侧向偏移

            DeckMotionCount = 1;
            DeckMotionLateralCount = 1; // new added in mk4.1
            DeckCompensationPosition = 1600; // 甲板运动补偿开始作用距离
            DeckCompensationStartRange = 0.01;
            DeckCompensationStartCount = 0; // 1开启DMC 0关闭DMC，等待补偿指令足够小
            DeckCompensationLateralStartCount = 0; // 1开启DMC 0关闭DMC，等待补偿指令足够小 用于横侧向补偿 new added in mk4.1
            DeriveDeckControl = 0;
            DeriveDeckLateralControl = 0;

            DeckCompensationStartThreshold = 5;
            DeckCompensationLateralStartThreshold = 5;
        }
    }
}
