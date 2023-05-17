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
        public value struct CurrentState {
            double TroPos;
            double GntPos;
            double HstPos;
            double SlgPos;

            double TroVel;
            double GntVel;
            double HstVel;
            double SlgVel;

            CurrentState(double troPos, double gntPos, double hstPos, double slgPos, double troVel, double gntVel, double hstVel, double slgVel) {
                TroPos = troPos;
                GntPos = gntPos;
                HstPos = hstPos;
                SlgPos = slgPos;

                TroVel = troVel;
                GntVel = gntVel;
                HstVel = hstVel;
                SlgVel = slgVel;
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
            static ValueTuple< List<CurrentState>^, ResultValues> GetPositions(double td, Parameter tro, Parameter gnt, Parameter hst, Parameter slg, int stepLimit);

        private:
            Trajectory<1>* _trajectory;
        };
    }
}
