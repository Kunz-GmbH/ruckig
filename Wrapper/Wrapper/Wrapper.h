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
            bool CalculationSuccessful;
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

        public value struct JerkStates {
            int Step;
            double Jerk;
        };

        public ref class RuckigWrapper
        {
        public:
            static ValueTuple< List<JerkStates>^, ResultValues>  GetValues(double td, Parameter parameter);
        };
    }

}
