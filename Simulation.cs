using System;
using System.Collections.Concurrent;
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

        public Ship Ship { get; set; }
        public Plane Plane { get; set; }
        public Configuration Configuration { get; set; }
        public Disturbance Disturbance { get; set; }
        public AttitudeLoop AttitudeLoop { get; set; }
        public FlightPathLoop FlightPathLoop { get; set; }
        public PositionLoop PositionLoop { get; set; }
        public AngularRateLoop AngularRateLoop { get; set; }
        public SimulationRecord Record { get; set; }

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

        public void Simulate(ConcurrentQueue<double> DataQueue)
        {
            Control.UseNativeMKL();
            while ((Plane.Position[2] - Ship.Position[2]) < 0)
            {
                SingleStep();
                if (step_count % 50 == 0)
                {
                    DataQueue.Enqueue(Plane.Alpha);
                }
            }
            Console.WriteLine(step_count);
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

        void Reset()
        {
            step_count = 0;
            current_time = 0;
            Ship.reset();
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
