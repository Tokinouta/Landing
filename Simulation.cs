using MathNet.Numerics.Data.Matlab;
using MathNet.Numerics.LinearAlgebra;
using Microsoft.AspNetCore.SignalR.Client;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Timers;
using System.Threading.Tasks;
using HistoryDemo.Entities;
using HistoryDemo;

namespace CsharpVersion
{
    public class Simulation
    {
        HubConnection connection;
        double frequency = 400; // 计算频率 Hz
        double dt; // 1 / f
        int step_count = 0;
        double current_time = 0;

        bool landing_gear = false; // 1放起落架
        bool tail_hook = false; // 1放尾钩
        bool wing_damage = false; //1机翼损伤

        public Ship Ship { get; set; }
        public Plane Plane { get; set; }
        public Disturbance Disturbance { get; set; }
        public AttitudeLoop AttitudeLoop { get; set; }
        public FlightPathLoop FlightPathLoop { get; set; }
        public PositionLoop PositionLoop { get; set; }
        public AngularRateLoop AngularRateLoop { get; set; }
        public SimulationRecord Record { get; set; }
        public ConcurrentQueue<double> DataQueue { get; set; }
        public Timer DataSendTimer { get; set; }
        public Simulation()
        {
            InitializeInitialParameters();

            Ship = new Ship();
            Plane = new Plane(Ship);
            Disturbance = new Disturbance();
            PositionLoop = new PositionLoop(Plane, Ship);
            FlightPathLoop = new FlightPathLoop(Plane, Ship);
            AttitudeLoop = new AttitudeLoop(Plane, Ship);
            AngularRateLoop = new AngularRateLoop(Plane, Ship);
            // plane.addListeners(positionLoop, flightPathLoop, attitudeLoop, angularRateLoop);
            Record = new SimulationRecord();

            Plane.X1ChangedEvent += PositionLoop.OnUpdateState;
            Plane.X2ChangedEvent += FlightPathLoop.OnUpdateState;
            Plane.X3ChangedEvent += AttitudeLoop.OnUpdateState;
            Plane.X4ChangedEvent += AngularRateLoop.OnUpdateState;
            Plane.RecordPlaneStateEvent += Record.OnRecordPlaneState;
            //disturbance.AddListener(flightPathLoop);
            // addlistener(app, "simulationStep", "PostSet", @modifyFrequency);
            DataSendTimer = new Timer(100);
            DataSendTimer.Elapsed += (sender, e) =>
            {
                if (DataQueue.TryDequeue(out double data))
                {
                    connection.InvokeAsync("SendData", "user", data);
                }
                if (DataQueue.IsEmpty)
                {
                    DataSendTimer.Stop();
                    Console.WriteLine("Timer Stoped");
                }
            };
            DataQueue = new ConcurrentQueue<double>();
            Console.WriteLine("text");
        }

        async public void StartHubConnection()
        {
            connection = new HubConnectionBuilder()
                .WithUrl("https://localhost:5001/simulationHub")
                .Build();
            await connection.StartAsync();
            Console.WriteLine(connection.State);
            if (connection.State == HubConnectionState.Connected)
            {
                Console.WriteLine("connection started");
            }
        }

        public Task SendData()
        {
            return new Task(() =>
            {
                if (DataQueue.TryDequeue(out double data))
                {
                    connection.InvokeAsync("SendData", "user", data);
                }
            });
        }

        public void Simulate()
        {
            var ini = new Initialization()
            {
                InitialPositionX = Plane.Position[0],
                InitialPositionY = Plane.Position[1],
                InitialPositionZ = Plane.Position[2],
                InitialAttitudePhi = Plane.Phi,
                InitialAttitudePsi = Plane.Psi,
                InitialAttitudeTheta = Plane.Theta
            };
            var conf = new HistoryDemo.Entities.Configuration()
            {
                GuidanceConfig = (HistoryDemo.Entities.GuidanceConfig)Configuration.GuidanceController
            };

            DataSendTimer.Start();
            while ((Plane.Position[2] - Ship.Position[2]) < 0)
            {
                SingleStep();
                if (step_count % 50 == 0)
                {
                    DataQueue.Enqueue(Plane.Alpha);
                }
            }

            Console.WriteLine(step_count);
            //Record.SaveToDatabase(ini, conf);
        }

        ~Simulation()
        {
            DataSendTimer.Dispose();
            Console.WriteLine("Timer Disposed");
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

            PositionLoop.calculatePrescribedParameter();
            Ship.calculateCompensation(dt,Plane, PositionLoop, step_count);
            PositionLoop.CalculateState(dt, null);
            PositionLoop.calculateOutput(dt, current_time, step_count);
            PositionLoop.CalculateLimiter(dt);

            FlightPathLoop.CalculateState(dt, PositionLoop.U1);
            FlightPathLoop.CalculateOutput();
            FlightPathLoop.CalculateLimiter(dt);
            FlightPathLoop.CalculateFilter(dt);

            AttitudeLoop.CalculateState(dt, FlightPathLoop.U2);
            AttitudeLoop.CalculateOutput();
            AttitudeLoop.CalculateLimiter(dt);

            AngularRateLoop.CalculateState(dt, AttitudeLoop.U3);
            AngularRateLoop.CalculateOutput(dt, current_time, step_count);
            AngularRateLoop.CalculateLimiter(dt);
            AngularRateLoop.CalculateFilter(dt);

            Plane.UpdateState(dt, Disturbance);
            Ship.updateState(dt);
            Disturbance.updateWind(Plane, Ship, step_count);

            Plane.Record();
            Ship.record();
            PositionLoop.Record(dt);
            FlightPathLoop.Record(dt);
            AttitudeLoop.Record(dt);
            AngularRateLoop.Record(dt);
            //Task.Run(() => Console.WriteLine(step_count));
            //record.time_record.Add(current_time);
            if (Plane.l_path>1000)
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
