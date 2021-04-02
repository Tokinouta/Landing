using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CsharpVersion
{
    class Simulation
    {
        public Plane plane;
        Ship ship;
        Disturbance disturbance;
        AttitudeLoop attitudeLoop;
        FlightPathLoop flightPathLoop;
        PositionLoop positionLoop;
        AngularRateLoop angularRateLoop;
        SimulationRecord record;

        double frequency = 400; // 计算频率 Hz
        double dt; // 1 / f
        int step_count = 0;
        double current_time = 0;

        public Simulation()
        {
            initializeInitialParameters();

            ship = new Ship();
            plane = new Plane(ship);
            disturbance = new Disturbance();
            positionLoop = new PositionLoop(plane, ship);
            flightPathLoop = new FlightPathLoop(plane, ship);
            attitudeLoop = new AttitudeLoop(plane, ship);
            angularRateLoop = new AngularRateLoop(plane, ship);
            Console.WriteLine("text");
            // plane.addListeners(positionLoop, flightPathLoop, attitudeLoop, angularRateLoop);
            record = new SimulationRecord();

            plane.X1ChangedEvent += positionLoop.OnUpdateState;
            plane.X2ChangedEvent += flightPathLoop.OnUpdateState;
            plane.X3ChangedEvent += attitudeLoop.OnUpdateState;
            plane.X4ChangedEvent += angularRateLoop.OnUpdateState;

            //disturbance.AddListener(flightPathLoop);
            // addlistener(app, "simulationStep", "PostSet", @modifyFrequency);
        }

        public void simulate()
        {
            //fid = fopen('out.txt', 'a');
            // load('position.mat');
            // while (step_count < 2000)
            while ((plane.current_position[0] - ship.current_position_ship[0]) < 0)
            {
                // t1 = position_record(step_count,:);
                // con = sum(t1.* t1) ~= sum(plane.current_position.* plane.current_position);
            singleStep();//fclose(fid);

            }
            Console.WriteLine(step_count);
        }

        void singleStep()
        {
            step_count++;
            current_time = dt * step_count;

            positionLoop.calculateObservation();
            flightPathLoop.calculateObservation();
            attitudeLoop.calculateObservation();
            angularRateLoop.calculateObservation();

            flightPathLoop.calculateNonlinearObserver(dt, disturbance);
            angularRateLoop.calculateNonlinearObserver(dt, disturbance);

            positionLoop.calculatePrescribedParameter();
            ship.calculateCompensation(dt, positionLoop, step_count);
            positionLoop.calculateState(dt, null);
            positionLoop.calculateOutput(dt, current_time, step_count);
            positionLoop.calculateLimiter(dt);

            flightPathLoop.calculateState(dt, positionLoop.current_u1);
            flightPathLoop.calculateOutput();
            flightPathLoop.calculateLimiter(dt);
            flightPathLoop.calculateFilter(dt);

            attitudeLoop.calculateState(dt, flightPathLoop.current_u2);
            attitudeLoop.calculateOutput();
            attitudeLoop.calculateLimiter(dt);

            angularRateLoop.calculateState(dt, attitudeLoop.current_u3);
            angularRateLoop.calculateOutput();
            angularRateLoop.calculateLimiter(dt, step_count);
            angularRateLoop.calculateFilter(dt);

            plane.updateState(dt, disturbance);
            ship.updateState(dt);
            disturbance.updateWind(plane, ship, step_count);

            plane.record();
            ship.record();
            positionLoop.record(dt);
            flightPathLoop.record(dt);
            attitudeLoop.record(dt);
            angularRateLoop.record(dt);
            //record.time_record.Add(current_time);
        }

        void reset()
        {
            step_count = 0;
            current_time = 0;
            ship.reset();
            plane.reset(ship);
            positionLoop.reset();
            flightPathLoop.reset();
            attitudeLoop.reset();
            angularRateLoop.reset();
            disturbance.reset();
        }

        void plotData()
        {
            //notify(obj, "PlotEvent")
        }

        void initializeInitialParameters()
        {
            dt = 1 / frequency; // 1 / f
        }

        void modifyFrequency()
        {
            //dt = e.AffectedObject.simulationStep;
            //frequency = 1 / dt;
            //disp(dt);
        }
    }
}
