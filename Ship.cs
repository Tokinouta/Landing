using static MathNet.Numerics.Constants;
using static MathNet.Numerics.Trig;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.Data.Matlab;
using System;
using ModelEntities.Enumerations;
using ModelEntities;

namespace CsharpVersion
{
    /// <summary>
    /// 航母类
    /// </summary>
    public class Ship
    {
        static readonly VectorBuilder<double> vb = Vector<double>.Build;
        static readonly MatrixBuilder<double> mb = Matrix<double>.Build;
        Configuration Configuration { get; }

        /// <summary>
        /// 航母速度
        /// </summary>
        public double Velocity = 27 * 1.852 * 5 / 18;
        /// <summary>
        /// 斜角甲板角度
        /// </summary>
        public double Theta = 9 * Pi / 180;
        /// <summary>
        /// 期望下滑角
        /// </summary>
        /// <remarks>可能会随舰载机机型不同有变化</remarks>
        public double Gamma = -3.5 * Pi / 180;
        /// <summary>
        /// 航母初始偏转角度
        /// </summary>
        public double Psi = 0 * Pi / 180;
        /// <summary>
        /// 航母转动角速度x分量，惯性系下表示
        /// </summary>
        public double omega_dx_2i = 0;
        /// <summary>
        /// 航母转动角速度y分量，惯性系下表示
        /// </summary>
        public double omega_dy_2i = 0;
        /// <summary>
        /// 航母转动角速度z分量，惯性系下表示
        /// </summary>
        public double omega_dz_2i = 0;
        /// <summary>
        /// 航母转动角速度，惯性系下表示
        /// </summary>
        public Vector<double> omega_d_2i;
        /// <summary>
        /// 航母位置
        /// </summary>
        /// <remarks>特别注意，current_position_ship仅代表航母直线前进位置，未考虑甲板起伏与侧向偏移</remarks>
        public Vector<double> Position = vb.Dense(3, 0);
        /// <summary>
        /// 航母甲板位置
        /// </summary>
        public Vector<double> DeckPosition;

        // 甲板运动补偿参数
        /// <summary>
        /// 是否使能甲板运动补偿
        /// </summary>
        public bool DeckEnable = false;
        /// <summary>
        /// 【作用不明】
        /// </summary>
        public int DeckMotionCount = 1;
        /// <summary>
        /// 【作用不明】
        /// </summary>
        public int DeckMotionLateralCount = 1; // new added in mk4.1
        /// <summary>
        /// 甲板运动补偿开始作用距离
        /// </summary>
        public double DeckCompensationPosition = 1600;
        /// <summary>
        /// 甲板运动补偿开始作用范围
        /// </summary>
        public double DeckCompensationStartRange = 0.01;
        /// <summary>
        /// 1开启DMC 0关闭DMC，等待补偿指令足够小
        /// </summary>
        /// <remarks>上面的描述感觉不太对，这个东西实际上是需要加到阈值以上才会起作用</remarks>
        public int DeckCompensationStartCount = 0;
        /// <summary>
        /// 1开启DMC 0关闭DMC，等待补偿指令足够小，用于横侧向补偿
        /// </summary>
        /// <remarks>上面的描述感觉不太对，这个东西实际上是需要加到阈值以上才会起作用</remarks>
        public int DeckCompensationLateralStartCount = 0; // new added in mk4.1

        int CalCount;
        Matrix<double> ForwardFilterState;
        Vector<double> CurrentDeckPredict;
        /// <summary>
        /// 由补偿模型计算出的当前甲板控制量，用于位置计算
        /// </summary>
        public Vector<double> CurrentDeckControl;
        /// <summary>
        /// 由补偿模型计算出的当前甲板侧向控制量，用于位置计算
        /// </summary>
        public Vector<double> CurrentDeckLateralControl;
        /// <summary>
        /// 甲板控制导数
        /// </summary>
        public double DeriveDeckControl = 0;
        /// <summary>
        /// 甲板侧向控制导数
        /// </summary>
        public double DeriveDeckLateralControl = 0;

        /// <summary>
        /// 甲板补偿开始作用的阈值
        /// </summary>
        public int DeckCompensationStartThreshold = 5;
        /// <summary>
        /// 甲板侧向补偿开始作用的阈值
        /// </summary>
        public int DeckCompensationLateralStartThreshold = 5;
        Vector<double> vector_trac_err;

        // event RecordShipStateEvent;

        /// <summary>
        /// 构造航母类型的实例
        /// </summary>
        /// <param name="config">仿真配置对象</param>
        public Ship(Configuration config)
        {
            Configuration = config;
            DeckEnable = Configuration.IsDeckCompensationEnabled;
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


        /// <summary>
        /// 更新扰动变量
        /// </summary>
        /// <param name="dt">舰载机对象</param>
        public void UpdateState(double dt)
        {
            double x_ship_dot = Velocity * Cos(Psi);
            double y_ship_dot = Velocity * Sin(Psi);
            Position[0] += x_ship_dot * dt; // 更新航母位置
            Position[1] += y_ship_dot * dt;
            // Position[2] = Position[2];
            Psi += omega_dz_2i * dt;
        }

        /// <summary>
        /// 计算甲板补偿
        /// </summary>
        /// <param name="dt">仿真时间步长</param>
        /// <param name="plane">舰载机对象</param>
        /// <param name="positionLoop">位置环对象</param>
        /// <param name="step_count">当前仿真步数</param>
        public void CalculateCompensation(double dt, Plane plane, PositionLoop positionLoop, int step_count)
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
                        DeriveDeckLateralControl = 0;
                    }
                    break;
                default:
                    break;
            }
        }

        /// <summary>
        /// 记录航母变量
        /// </summary>
        [Obsolete("这个函数本是用于移植旧版本的记录，现在发现航母变量暂时不用记录")]
        public void record()
        {
            //notify(obj, "RecordShipStateEvent");
        }

        /// <summary>
        /// 重置航母，将相关变量恢复至初始状态
        /// </summary>
        public void Reset()
        {
            Theta = 9 * Pi / 180; // 斜角甲板角度 9度
            Gamma = -3.5 * Pi / 180; // 期望下滑角 3.5度
            Psi = 0 * Pi / 180; // 航母初始偏转角度
            omega_dx_2i = 0;
            omega_dy_2i = 0;
            omega_dz_2i = 0 * Pi / 180; // 航母转动角速度，惯性系下表示 - 0.2
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
