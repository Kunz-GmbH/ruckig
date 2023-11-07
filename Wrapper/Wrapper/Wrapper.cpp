#include "Wrapper.h"

using namespace System::Collections::Generic;

namespace ruckig {
	namespace Wrapper {

		RuckigWrapper::RuckigWrapper(double td, Parameter para, bool useVelocityInterface) {
			_otg = new Ruckig<1>(td);
			_input = new InputParameter<1>();
			_output = new OutputParameter<1>();

			_input->current_position = { para.CurrentPosition };

			_input->current_position = { para.CurrentPosition };
			_input->current_velocity = { para.CurrentVelocity };
			_input->current_acceleration = { para.CurrentAcceleration };

			_input->target_position = { para.TargetPosition };
			_input->target_velocity = { para.TargetVelocity };
			_input->target_acceleration = { para.TargetAcceleration };

			_input->max_velocity = { para.MaxVelocity };
			_input->max_acceleration = { para.MaxAcceleration };
			_input->max_jerk = { para.MaxJerk };

			if (useVelocityInterface)
				_input->control_interface = ControlInterface::Velocity;

			_input->synchronization = Synchronization::None;
		}


		ValueTuple<CurrentState, ResultValues> RuckigWrapper::GetNextState(Parameter para, bool useVelocityInterface) {
			_input->current_position = { para.CurrentPosition };
			_input->current_velocity = { para.CurrentVelocity };
			_input->current_acceleration = { para.CurrentAcceleration };

			if (useVelocityInterface)
				_input->control_interface = ControlInterface::Velocity;
			else
				_input->control_interface = ControlInterface::Position;


			_output->pass_to_input(*_input);

			auto res = _otg->update(*_input, *_output);

			CurrentState lastState{ _output->new_position[0], _output->new_velocity[0], _output->new_acceleration[0] };
			ResultValues resultValues{   };
			resultValues.CalculationResult = res;
			ValueTuple< CurrentState, ResultValues> resultTuple{ lastState, resultValues };
			return  resultTuple;
		}

		StepState RuckigWrapper::GetStep(Parameter parameter, double td) {
			// todo also update? use velocity interface?
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

			input.control_interface = ControlInterface::Velocity;
			Trajectory<1> trajectory;
			Result res = otg.calculate(input, trajectory);
			// Then, we can calculate the kinematic state at a given time
			std::array<double, 1> new_position, new_velocity, new_acceleration;
			trajectory.at_time(new_time, new_position, new_velocity, new_acceleration);

			StepState state{ res, new_acceleration[0], new_velocity[0], new_position[0] };
			return state;
		}

		ValueTuple< array<List<CurrentState>^>^, ResultValues> RuckigWrapper::GetPositionsIncludingBrakePosition(double td, Parameter para, int stepLimit, bool useVelocityInterface) {

			ResultValues resultValues{ };
			array<List<CurrentState>^>^ results = gcnew array< List<CurrentState>^>(stepLimit);

			Ruckig<1> otg{ td };

			InputParameter<1> input;
			OutputParameter<1> output;
			Trajectory<1> phase1Trajectory;
			Trajectory<1> trajectory;
			double brakeTime1;

			auto use2PhaseChange = false;

			input.max_acceleration = { para.MaxAcceleration };
			input.max_jerk = { para.MaxJerk };
			input.duration_discretization = DurationDiscretization::Discrete;
			input.control_interface = ControlInterface::Velocity;
			input.synchronization = Synchronization::None;

			// workaround for hard kinematic constraints (ruckig tries getting into the constraints without being time optimality)
			if (para.CurrentVelocity > 0 && para.CurrentVelocity > para.MaxVelocity || para.CurrentVelocity < 0 && para.CurrentVelocity < -para.MaxVelocity) {
				input.current_position = { para.CurrentPosition };
				input.current_velocity = { para.CurrentVelocity };
				input.current_acceleration = { para.CurrentAcceleration };
				input.target_velocity = { para.MaxVelocity };
				input.target_acceleration = { 0 };
				input.max_velocity = { para.CurrentVelocity };

				auto res1 = otg.calculate(input, phase1Trajectory);
				Result res2;
				double brakeTime2;

				brakeTime1 = phase1Trajectory.get_duration();

				if (res1 == Result::Finished) {
					StandardVector<double, 1> newPosition;
					trajectory.at_time(brakeTime1, newPosition);

					input.current_position = { newPosition[0] };
					input.current_velocity = { para.MaxVelocity };
					input.current_acceleration = { 0 };
					input.target_velocity = { 0 };
					input.target_acceleration = { 0 };

					input.max_velocity = { para.CurrentVelocity };

					res2 = otg.calculate(input, trajectory);
					brakeTime2 = trajectory.get_duration();
				}

				if (res1 == Result::Finished && res2 == Result::Finished) {
					StandardVector<double, 1> endPosition;
					trajectory.at_time(brakeTime2, endPosition);

					// check for overshoot
					if (para.CurrentPosition > para.TargetPosition && endPosition[0] < para.TargetPosition ||
						para.CurrentPosition < para.TargetPosition && endPosition[0] > para.TargetPosition) {
						// with 2 phase we would overshoot
					}
					else 
						use2PhaseChange = true;					
				}
				else {
					resultValues.CalculationResult = res1 != Result::Finished ? res1 : res2;
					ValueTuple< array<List<CurrentState>^>^, ResultValues> resultTuple{ results, resultValues };
					return  resultTuple;
				}
			}

			for (auto i = 0; i < stepLimit; ++i) {
				auto counter = 0;

				input.current_position = { para.CurrentPosition };
				input.current_velocity = { para.CurrentVelocity };
				input.current_acceleration = { para.CurrentAcceleration };

				input.target_position = { para.TargetPosition };
				input.target_velocity = { para.TargetVelocity };
				input.target_acceleration = { para.TargetAcceleration };

				input.max_velocity = { para.MaxVelocity };

				if (use2PhaseChange || useVelocityInterface)
					input.control_interface = ControlInterface::Velocity;

				auto braking = false;
				auto needToSwitchBack = use2PhaseChange;
				while (otg.update(input, output) == Result::Working && (stepLimit < 0 || counter < stepLimit)) {

					CurrentState lastState{ output.new_position[0], output.new_velocity[0], output.new_acceleration[0] };
					results[i]->Add(lastState);

					if (!braking && !useVelocityInterface && needToSwitchBack && brakeTime1 <= output.time) {
						input.control_interface = ControlInterface::Position;
						needToSwitchBack = false;
					}

					if (counter == i) {
						braking = true;
						input.control_interface = ControlInterface::Velocity;
						input.target_velocity = { 0 };
						input.target_acceleration = { 0 };
					}

					++counter;
					output.pass_to_input(input);
				}

				auto res = otg.update(input, output);
				if (res < 0 || i == 0)
					resultValues.CalculationResult = res;
				CurrentState lastState{ output.new_position[0], output.new_velocity[0], output.new_acceleration[0] };
				results[i]->Add(lastState);
			}

			ValueTuple< array<List<CurrentState>^>^, ResultValues> resultTuple{ results, resultValues };
			return  resultTuple;
		}

		// 1 axis for cps
		ValueTuple< List<CurrentState>^, ResultValues> RuckigWrapper::GetPositions(double td, Parameter para, int stepLimit, bool useVelocityInterface) {
			Ruckig<1> otg{ td };
			InputParameter<1> input;
			OutputParameter<1> output;

			input.current_position = { para.CurrentPosition };
			input.current_velocity = { para.CurrentVelocity };
			input.current_acceleration = { para.CurrentAcceleration };

			input.target_position = { para.TargetPosition };
			input.target_velocity = { para.TargetVelocity };
			input.target_acceleration = { para.TargetAcceleration };

			input.max_velocity = { para.MaxVelocity };
			input.max_acceleration = { para.MaxAcceleration };
			input.max_jerk = { para.MaxJerk };

			if (useVelocityInterface)
				input.control_interface = ControlInterface::Velocity;

			input.synchronization = Synchronization::None;


			ResultValues resultValues{ };
			List<CurrentState>^ results = gcnew List<CurrentState>(stepLimit);
			auto counter = 0;
			while (otg.update(input, output) == Result::Working && (stepLimit < 0 || counter < stepLimit)) {
				CurrentState state{ output.new_position[0], output.new_velocity[0], output.new_acceleration[0] };
				results->Add(state);
				++counter;
				output.pass_to_input(input);
			}

			resultValues.CalculationResult = otg.update(input, output);
			CurrentState lastState{ output.new_position[0], output.new_velocity[0], output.new_acceleration[0] };
			results->Add(lastState);
			ValueTuple< List<CurrentState>^, ResultValues> resultTuple{ results, resultValues };
			return  resultTuple;
		}

		// PCP Trajectory generator
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
			List<JerkStates>^ results = gcnew List<JerkStates>(200);

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
