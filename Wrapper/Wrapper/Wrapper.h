#pragma once
#include <ruckig/ruckig.hpp>

using namespace System;
using namespace ruckig;
using namespace System::Collections::Generic;

namespace ruckig {
    namespace Wrapper {

        public value struct ResultValues {
            double CalculationTime;
            double Duration;
            int CalculationResult;
        };

        public value struct Parameter {
            double CurrentPosition;
            double CurrentVelocity;
            double CurrentAcceleration;
            double TargetPosition;
            double TargetVelocity;
            double TargetAcceleration;
            double MaxVelocity;
            double MaxAcceleration;
            double MaxJerk;
        };
        public value struct Positions {
            double Tro;
            double Gnt;
            double Hst;
            double Slg;

            Positions(double tro, double gnt, double hst, double slg) {
                Tro = tro;
                Gnt = gnt;
                Hst = hst;
                Slg = slg;
            }
        };

        public value struct JerkStates {
            int Step;
            double Jerk;
            double Accelereration;
            double Velocity;
            double Position;
            JerkStates(int step, double jerk, double acceleration, double velocity, double position)
            {
                Step = step;
                Jerk = jerk;
                Accelereration = acceleration;
                Velocity = velocity;
                Position = position;
            }
        };

        public ref class RuckigWrapper
        {
        public:
            static ValueTuple< List<JerkStates>^, ResultValues>  GetValues(double td, Parameter parameter);

            RuckigWrapper(Parameter parameter);
            ~RuckigWrapper();
            JerkStates GetStep(double td);
            static ValueTuple< List<Positions>^, ResultValues> GetPositions(double td, Parameter tro, Parameter gnt, Parameter hst, Parameter slg);

        private:
            Trajectory<1>* _trajectory;
        };
    }
}
