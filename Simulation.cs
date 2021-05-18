using System;
using System.Collections.Concurrent;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using MathNet.Numerics;
using MathNet.Numerics.Data.Text;
using ModelEntities;
using ModelEntities.Enumerations;

namespace CsharpVersion
{
    /// <summary>
    /// 仿真的顶层类，用于管理仿真过程中用到的所有资源
    /// </summary>
    public class Simulation
    {
        double frequency = 400; // 计算频率 Hz
        double dt; // 1 / f
        int step_count = -1;
        double current_time = 0;

        bool landing_gear = false; // 1放起落架
        bool tail_hook = false; // 1放尾钩
        bool wing_damage = false; //1机翼损伤
        bool land_flag = false;

        /// <summary>
        /// 航母对象，主要作为仿真中的参考对象，没有大的变动需要特别处理
        /// </summary>
        public Ship Ship { get; set; }

        /// <summary>
        /// 舰载机对象，这是仿真过程中的控制对象，需要为其他对象提供状态信息
        /// </summary>
        public Plane Plane { get; set; }

        /// <summary>
        /// 仿真配置对象，存储仿真配置
        /// </summary>
        /// <remarks>
        /// 现有版本中这个是一个仿真类型的实例，旧版本中用的是静态类，因此有的地方传进去的参数会比以前多一个<c>Configuration</c>
        /// </remarks>
        public Configuration Configuration { get; set; }

        /// <summary>
        /// 扰动对象，提供风场扰动信息
        /// </summary>
        public Disturbance Disturbance { get; set; }

        /// <summary>
        /// 姿态环控制器
        /// </summary>
        public AttitudeLoop AttitudeLoop { get; set; }

        /// <summary>
        /// 航迹环控制器
        /// </summary>
        public FlightPathLoop FlightPathLoop { get; set; }

        /// <summary>
        /// 位置环控制器
        /// </summary>
        public PositionLoop PositionLoop { get; set; }

        /// <summary>
        /// 角速度环控制器
        /// </summary>
        public AngularRateLoop AngularRateLoop { get; set; }

        /// <summary>
        /// 数据记录对象，用于缓存仿真中产生的数据，并提供将数据存入磁盘的功能
        /// </summary>
        public SimulationRecord Record { get; set; }

        /// <summary>
        /// 当前仿真步数，用于外部引用
        /// </summary>
        /// <remarks>
        /// 只读属性，实际操作通过内部私有字段进行
        /// </remarks>
        public int Step_count { get => step_count; }

        /// <summary>
        /// 用于处理数据发送的委托
        /// </summary>
        /// <remarks>
        /// 需要发送的数据包括两部分
        /// <list type="bullet">
        /// <item>通过SignalR发到前端进行实时绘图</item>
        /// <item>通过UDP协议发送至视景软件</item>
        /// </list>
        /// 这里需要控制的是第二类数据
        /// </remarks>
        public Action DataSending;

        /// <summary>
        /// 提供向视景软件发送的字节流
        /// </summary>
        public byte[] DataToSend
        {
            get
            {
                var t = new DataForUdpDatagram()
                {
                    current_deck_position_shipx = (float)Ship.DeckPosition[0],
                    current_deck_position_shipy = (float)Ship.DeckPosition[1],
                    current_deck_position_shipz = (float)Ship.DeckPosition[2],
                    current_positionx = (float)Plane.Position[0],
                    current_positiony = (float)PositionLoop.X1[0],
                    current_positionz = (float)PositionLoop.X1[1],
                    current_phi = (float)Plane.Phi,
                    current_theta = (float)Plane.Theta,
                    current_psi = (float)Plane.Psi,
                    filter_Uact1l = (float)AngularRateLoop.filteredUact[0],
                    filter_Uact1r = (float)-AngularRateLoop.filteredUact[0],
                    filter_Uact2l = (float)AngularRateLoop.filteredUact[1],
                    filter_Uact2r = (float)AngularRateLoop.filteredUact[1],
                    filter_Uact3l = (float)AngularRateLoop.filteredUact[2],
                    filter_Uact3r = (float)AngularRateLoop.filteredUact[2],
                    current_delta_tefl = (float)Plane.DeltaTEF,
                    current_delta_tefr = (float)Plane.DeltaTEF,
                    current_Tl = (float)Plane.T / 2,
                    current_Tr = (float)Plane.T / 2,
                    landing_gear = landing_gear,
                    tail_hook = tail_hook,
                    wing_damage = wing_damage,
                    land_flag = land_flag
                };
                //BinaryFormatter bf = new();
                //using var ms = new MemoryStream();
                //bf.Serialize(ms, t);
                return t.ToBytes();
            }
        }

        /// <summary>
        /// 获取仿真的初始状态
        /// </summary>
        public Initialization SimulationInitialization
        {
            get
            {
                return new Initialization()
                {
                    X = Plane.Position[0],
                    Y = Plane.Position[1],
                    Z = Plane.Position[2],
                    Phi = Plane.Phi,
                    Psi = Plane.Psi,
                    Theta = Plane.Theta,
                    P = Plane.P,
                    Q = Plane.Q,
                    R = Plane.R,
                    Alpha = Plane.Alpha,
                    Vk = Plane.Vk,
                    XShip = Ship.Position[0],
                    YShip = Ship.Position[1],
                    ZShip = Ship.Position[2],
                    PsiShip = Ship.Psi
                };
            }
        }


        NeuronNetwork neuronNetwork;
        /// <summary>
        /// 创建仿真类型的实例
        /// </summary>
        public Simulation()
        {
            var opts = new Options()
            {
                BatchSize = 5,
                NumberEpochs = 500,
            };
            neuronNetwork = new(new int[] { 50, 512, 256, 128, 64, 19 }, opts);
            neuronNetwork.W[0] = DelimitedReader.Read<double>("W1,n=5.csv", delimiter: ",");
            neuronNetwork.W[1] = DelimitedReader.Read<double>("W2,n=5.csv", delimiter: ",");
            neuronNetwork.W[2] = DelimitedReader.Read<double>("W3,n=5.csv", delimiter: ",");
            neuronNetwork.W[3] = DelimitedReader.Read<double>("W4,n=5.csv", delimiter: ",");
            neuronNetwork.W[4] = DelimitedReader.Read<double>("W5,n=5.csv", delimiter: ",");

            InitializeInitialParameters();

            Configuration = new()
            {
                GuidanceController = GuidanceConfig.G3dMPF,
                AttitudeController = AttitudeConfig.IDLC,
                AngularRateController = AngularRateConfig.NDI,
                DisturbanceObserver = DisturbanceObserverConfig.NONE,

                // 导数滤波器配置参数
                GuidanceFilter = GuidanceFilters.Command,
                AttitudeFilter = AttitudeFilters.Command,
                UseAttitudeTrackingDifferentiator = false,

                // 轨迹配置
                TrajactoryConfig = TrajactoryType.TypeII,

                // 扰动类型配置
                UseDisturbanceTypeI = true,
                IsWindEnabled = true,
                IsDeckCompensationEnabled = true,
                UseL1Adaptive = true,
            };
            Ship = new Ship(Configuration);
            Plane = new Plane(Ship);
            Disturbance = new Disturbance(Configuration);
            PositionLoop = new PositionLoop(Plane, Ship, Configuration);
            FlightPathLoop = new FlightPathLoop(Plane, Ship, Configuration);
            AttitudeLoop = new AttitudeLoop(Plane, Ship, Configuration);
            AngularRateLoop = new AngularRateLoop(Plane, Ship, Configuration);
            Record = new SimulationRecord();
            HelperFunction.Configuration = Configuration;
            Plane.X1ChangedEvent += PositionLoop.OnUpdateState;
            Plane.X2ChangedEvent += FlightPathLoop.OnUpdateState;
            Plane.X3ChangedEvent += AttitudeLoop.OnUpdateState;
            Plane.X4ChangedEvent += AngularRateLoop.OnUpdateState;
            Plane.RecordPlaneStateEvent += Record.OnRecordPlaneState;

            Console.WriteLine("text");
        }

        /// <summary>
        /// 创建仿真类型的实例
        /// </summary>
        /// <param name="configuration">创建本实例需要使用仿真配置</param>
        public Simulation(Configuration configuration)
        {
            var opts = new Options()
            {
                BatchSize = 5,
                NumberEpochs = 500,
            };
            neuronNetwork = new(new int[] { 50, 512, 256, 128, 64, 19 }, opts);
            neuronNetwork.W[0] = DelimitedReader.Read<double>("W1,n=5.csv", delimiter: ",");
            neuronNetwork.W[1] = DelimitedReader.Read<double>("W2,n=5.csv", delimiter: ",");
            neuronNetwork.W[2] = DelimitedReader.Read<double>("W3,n=5.csv", delimiter: ",");
            neuronNetwork.W[3] = DelimitedReader.Read<double>("W4,n=5.csv", delimiter: ",");
            neuronNetwork.W[4] = DelimitedReader.Read<double>("W5,n=5.csv", delimiter: ",");

            InitializeInitialParameters();

            Configuration = configuration;
            Ship = new Ship(Configuration);
            Plane = new Plane(Ship);
            Disturbance = new Disturbance(Configuration);
            PositionLoop = new PositionLoop(Plane, Ship, Configuration);
            FlightPathLoop = new FlightPathLoop(Plane, Ship, Configuration);
            AttitudeLoop = new AttitudeLoop(Plane, Ship, Configuration);
            AngularRateLoop = new AngularRateLoop(Plane, Ship, Configuration);
            Record = new SimulationRecord();
            HelperFunction.Configuration = Configuration;
            Plane.X1ChangedEvent += PositionLoop.OnUpdateState;
            Plane.X2ChangedEvent += FlightPathLoop.OnUpdateState;
            Plane.X3ChangedEvent += AttitudeLoop.OnUpdateState;
            Plane.X4ChangedEvent += AngularRateLoop.OnUpdateState;
            Plane.RecordPlaneStateEvent += Record.OnRecordPlaneState;

            Console.WriteLine("text");
        }

        /// <summary>
        /// 进行仿真
        /// </summary>
        /// <param name="DataQueue">数据队列，用于存储仿真过程中产生的数据并用于实时数据发送</param>
        public void Simulate(ConcurrentQueue<DataToSend> DataQueue)
        {
            Control.UseNativeMKL();
            while ((Plane.Position[2] - Ship.Position[2]) < 0)
            {
                SingleStep();
                if (step_count % 50 == 0)
                {
                    DataSending?.Invoke();
                    DataQueue.Enqueue(new DataToSend()
                    {
                        Time = current_time,
                        X = Plane.Position[0],
                        Y = Plane.Position[1],
                        Z = Plane.Position[2],
                        Phi = Plane.Phi,
                        Psi = Plane.Psi,
                        Theta = Plane.Theta,
                        Alpha = Plane.Alpha,
                        P = Plane.P,
                        Q = Plane.Q,
                        R = Plane.R,
                        Chi = Plane.Chi,
                        Gamma = Plane.Gamma,
                        Vk = Plane.Vk,
                        Miu = Plane.Miu
                    });
                }
            }
            Console.WriteLine(step_count);
            land_flag = true;
            DataSending?.Invoke();
        }

        public bool Simulate50(ConcurrentQueue<DataToSend> DataQueue)
        {
            int oldStep = step_count;
            while ((Plane.Position[2] - Ship.Position[2]) < 0 && step_count - oldStep < 50)
            {
                SingleStep();
            }
            DataSending?.Invoke();
            DataQueue.Enqueue(new DataToSend()
            {
                Time = current_time,
                X = Plane.Position[0],
                Y = Plane.Position[1],
                Z = Plane.Position[2],
                Phi = Plane.Phi,
                Psi = Plane.Psi,
                Theta = Plane.Theta,
                Alpha = Plane.Alpha,
                P = Plane.P,
                Q = Plane.Q,
                R = Plane.R,
                Chi = Plane.Chi,
                Gamma = Plane.Gamma,
                Vk = Plane.Vk,
                Miu = Plane.Miu
            });
            return (Plane.Position[2] - Ship.Position[2]) < 0;
        }

        public int Detect()
        {
            var data = Record.DataForDetection();
            //Console.WriteLine(data);
            return (int)neuronNetwork.Predict(data)[0];
        }

        void SingleStep()
        {
            step_count++;
            current_time = dt * step_count;

            PositionLoop.CalculateObservation();
            FlightPathLoop.CalculateObservation();
            AttitudeLoop.CalculateObservation();
            AngularRateLoop.CalculateObservation();

            FlightPathLoop.CalculateNonlinearObserver(dt, Disturbance);
            AngularRateLoop.CalculateNonlinearObserver(dt, Disturbance);

            PositionLoop.CalculatePrescribedParameter();
            Ship.CalculateCompensation(dt, Plane, PositionLoop, step_count);
            PositionLoop.CalculateState(dt, null);
            PositionLoop.CalculateOutput(dt, current_time, step_count);
            PositionLoop.CalculateLimiter(dt);

            FlightPathLoop.CalculateState(dt, PositionLoop.U1);
            FlightPathLoop.CalculateOutput();
            FlightPathLoop.CalculateLimiter(dt);
            FlightPathLoop.CalculateFilter(dt);

            AttitudeLoop.CalculateState(dt, FlightPathLoop.U2, FlightPathLoop.DeriveX2[1]);
            AttitudeLoop.CalculateOutput();
            AttitudeLoop.CalculateLimiter(dt);

            AngularRateLoop.CalculateState(dt, AttitudeLoop.U3);
            AngularRateLoop.CalculateOutput(dt, current_time, step_count);
            AngularRateLoop.CalculateLimiter(dt);
            AngularRateLoop.CalculateFilter(dt);

            Plane.UpdateState(dt, Disturbance);
            Ship.UpdateState(dt);
            Disturbance.updateWind(Plane, Ship, step_count);

            Plane.Record();
            Ship.record();
            PositionLoop.Record(dt);
            FlightPathLoop.Record(dt);
            AttitudeLoop.Record(dt);
            AngularRateLoop.Record(dt);
            Record.time_record.Add(current_time);
            if (Plane.l_path > 1000)
            {
                landing_gear = true;
                tail_hook = true;
            }
        }

        /// <summary>
        /// 重置仿真状态，以便进行下一次仿真
        /// </summary>
        public void Reset()
        {
            step_count = -1;
            current_time = 0;
            Ship.Reset();
            Plane.Reset(Ship);
            PositionLoop.Reset();
            FlightPathLoop.Reset();
            AttitudeLoop.Reset();
            AngularRateLoop.Reset();
            Disturbance.reset();
        }

        void PlotData()
        {
            //notify(obj, "PlotEvent")
        }

        void InitializeInitialParameters()
        {
            dt = 1 / frequency; // 1 / f
        }

        void ModifyFrequency()
        {
            //dt = e.AffectedObject.simulationStep;
            //frequency = 1 / dt;
            //disp(dt);
        }
    }
}
