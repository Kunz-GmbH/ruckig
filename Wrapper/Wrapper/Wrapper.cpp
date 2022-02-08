#include "pch.h"

#include "Wrapper.h"

using namespace System::Collections::Generic;

namespace ruckig {
    namespace Wrapper {

        ValueTuple< List<JerkStates>^, ResultValues> RuckigWrapper::GetValues(double td, Parameter parameter)
        {
            Ruckig<1> otg{ td };
            InputParameter<1> input;
            OutputParameter<1> output;

            // Set input parameters
            input.current_position = { parameter.CurrentPosition };
            input.current_velocity = { parameter.CurrentVelocity };
            input.current_acceleration = { parameter.CurrentAcceleration };

            input.target_position = { parameter.TargetPosition };
            input.target_velocity = { parameter.TargetVelocity };
            input.target_acceleration = { parameter.TargetAcceleration };

            input.max_velocity = { parameter.MaxVelocity };
            input.max_acceleration = { parameter.MaxAcceleration };
            input.max_jerk = { parameter.MaxJerk };

            auto aOld = 0.0;
            auto jOld = 0.0;
            auto counter = 0;
            ResultValues resultValues{ };
            List<JerkStates>^ results = gcnew List<JerkStates>();

            while (otg.update(input, output) == Result::Working) {
                if (counter == 0) {
                    resultValues.CalculationTime = output.calculation_duration;
                    resultValues.Duration = output.trajectory.get_duration();
                }

                auto a = output.new_acceleration[0];
                auto j = (a - aOld) / td;

                if (Math::Abs(j - jOld) > 0.5) {
                    JerkStates jerkState{ counter, j };
                    results->Add(jerkState);
                }

                aOld = a;
                jOld = j;
                ++counter;
                output.pass_to_input(input);
            }
            JerkStates finalJerkState{ counter, 0 };
            results->Add(finalJerkState);
            resultValues.CalculationSuccessful = otg.update(input, output) == Result::Finished;
            ValueTuple< List<JerkStates>^, ResultValues> resultTuple{ results, resultValues };
            return  resultTuple;
        }
    }
}
