#include "Wrapper.h"

using namespace System::Collections::Generic;

namespace ruckig {
	namespace Wrapper {


		StepState RuckigWrapper::GetStep(Parameter parameter, double td) {
			double new_time{ td };
			Ruckig<1> otg{ };
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

			Trajectory<1> trajectory;
			Result res = otg.calculate(input, trajectory);
			// Then, we can calculate the kinematic state at a given time
			std::array<double, 1> new_position, new_velocity, new_acceleration;
			trajectory.at_time(new_time, new_position, new_velocity, new_acceleration);

			StepState state{ res, new_acceleration[0], new_velocity[0], new_position[0] };
			return state;
		}

		// 4 axis for cps
		ValueTuple< List<CurrentState>^, ResultValues> RuckigWrapper::GetPositions(double td, Parameter tro, Parameter gnt, Parameter hst, Parameter slg, int stepLimit) {
			Ruckig<4> otg{ td }; // tro, gnt, hst, slg
			InputParameter<4> input;
			OutputParameter<4> output;

			input.current_position = { tro.CurrentPosition, gnt.CurrentPosition, hst.CurrentPosition, slg.CurrentPosition };
			input.current_velocity = { tro.CurrentVelocity, gnt.CurrentVelocity, hst.CurrentVelocity, slg.CurrentVelocity };
			input.current_acceleration = { tro.CurrentAcceleration, gnt.CurrentAcceleration, hst.CurrentAcceleration, slg.CurrentAcceleration };

			input.target_position = { tro.TargetPosition, gnt.TargetPosition, hst.TargetPosition, slg.TargetPosition };
			input.target_velocity = { tro.TargetVelocity, gnt.TargetVelocity, hst.TargetVelocity, slg.TargetVelocity };
			input.target_acceleration = { tro.TargetAcceleration, gnt.TargetAcceleration, hst.TargetAcceleration, slg.TargetAcceleration };

			input.max_velocity = { tro.MaxVelocity, gnt.MaxVelocity, hst.MaxVelocity, slg.MaxVelocity };
			input.max_acceleration = { tro.MaxAcceleration, gnt.MaxAcceleration, hst.MaxAcceleration, slg.MaxAcceleration };
			input.max_jerk = { tro.MaxJerk, gnt.MaxJerk, hst.MaxJerk, slg.MaxJerk };

			input.synchronization = Synchronization::None;


			ResultValues resultValues{ };
			List<CurrentState>^ results = gcnew List<CurrentState>();
			auto counter = 0;
			while (otg.update(input, output) == Result::Working && (stepLimit < 0 || counter < stepLimit)) {
				CurrentState state{
					output.new_position[0],output.new_position[1] ,output.new_position[2] ,output.new_position[3],
					output.new_velocity[0],output.new_velocity[1] ,output.new_velocity[2] ,output.new_velocity[3]
				};
				results->Add(state);
				++counter;
				output.pass_to_input(input);
			}

			resultValues.CalculationResult = otg.update(input, output);
			CurrentState lastState{
					output.new_position[0],output.new_position[1] ,output.new_position[2] ,output.new_position[3],
					output.new_velocity[0],output.new_velocity[1] ,output.new_velocity[2] ,output.new_velocity[3]
			};
			results->Add(lastState);
			ValueTuple< List<CurrentState>^, ResultValues> resultTuple{ results, resultValues };
			return  resultTuple;
		}

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

			auto vOld = 0.0;
			auto pOld = 0.0;

			while (otg.update(input, output) == Result::Working) {
				if (counter == 0) {
					resultValues.CalculationTime = output.calculation_duration;
					resultValues.Duration = output.trajectory.get_duration();
				}

				auto j = output.new_jerk[0];

				if (Math::Abs(j - jOld) > 0.5) {
					JerkStates jerkState{ counter, j, aOld, vOld, pOld };
					results->Add(jerkState);
				}

				pOld = output.new_position[0];
				vOld = output.new_velocity[0];
				aOld = output.new_acceleration[0];
				jOld = j;
				++counter;
				output.pass_to_input(input);
			}
			JerkStates finalJerkState{ counter, output.new_jerk[0], output.new_acceleration[0], output.new_velocity[0], output.new_position[0] };
			results->Add(finalJerkState);
			resultValues.CalculationResult = otg.update(input, output);
			ValueTuple< List<JerkStates>^, ResultValues> resultTuple{ results, resultValues };
			return  resultTuple;
		}
	}
}
