using System;
using System.Collections.Concurrent;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using MathNet.Numerics;
using ModelEntities;
using ModelEntities.Enumerations;

namespace CsharpVersion
{
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

        public Ship Ship { get; set; }
        public Plane Plane { get; set; }
        public Configuration Configuration { get; set; }
        public Disturbance Disturbance { get; set; }
        public AttitudeLoop AttitudeLoop { get; set; }
        public FlightPathLoop FlightPathLoop { get; set; }
        public PositionLoop PositionLoop { get; set; }
        public AngularRateLoop AngularRateLoop { get; set; }
        public SimulationRecord Record { get; set; }
        public int Step_count { get => step_count; }
        public Action DataSending;
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

        public Simulation()
        {
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

        public Simulation(Configuration configuration)
        {
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
