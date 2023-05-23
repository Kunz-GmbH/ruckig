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
            double Pos;
            double Vel;
            double Acc;

            CurrentState(double pos, double vel, double acc) {
                Pos = pos;
                Vel = vel;
                Acc = acc;
            }
        };
        public value struct StepState {
            int CalculationResult;
            double Jerk;
            double Accelereration;
            double Velocity;
            double Position;
            StepState(int calculationResult, double acceleration, double velocity, double position)
            {
                CalculationResult = calculationResult;
                Accelereration = acceleration;
                Velocity = velocity;
                Position = position;
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

          
            static StepState GetStep(Parameter parameter, double td);
            static ValueTuple< List<CurrentState>^, ResultValues> GetPositions(double td, Parameter para, int stepLimit, bool useVelocityInterface);

        private:
            Trajectory<1>* _trajectory;
        };
    }
}
