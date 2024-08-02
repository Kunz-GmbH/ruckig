#include "Wrapper.h"

using namespace System::Collections::Generic;

namespace ruckig {
	namespace Wrapper {

#pragma region Cps
		// Cps Trajectory generation without brake trajectories
		RuckigWrapper::RuckigWrapper(double td) {
			_otg = new Ruckig<1>(td);
		}

		// 1 axis for cps
		ResultValues RuckigWrapper::GetPositions(Parameter para, int stepLimit, bool useVelocityInterface, array<CurrentState>^ currentStates) {
			ResultValues resultValues{ };
			double brakeTime1;

			InputParameter<1> input;
			OutputParameter<1> output;

			auto use2PhaseChange = false;

			input.max_acceleration = { para.MaxAcceleration };
			input.max_jerk = { para.MaxJerk };
			// TODO ? input.duration_discretization = DurationDiscretization::Discrete;
			input.control_interface = ControlInterface::Velocity;
			input.synchronization = Synchronization::None;

			auto signedMaxVelocity = para.CurrentVelocity > 0 ? para.MaxVelocity : -para.MaxVelocity;
			// 

			if (!useVelocityInterface &&
				(para.CurrentVelocity > 0 && para.TargetPosition > para.CurrentPosition && para.CurrentVelocity > para.MaxVelocity
					|| para.CurrentVelocity < 0 && para.TargetPosition < para.CurrentPosition && para.CurrentVelocity < -para.MaxVelocity))
			{

				auto [use2PhaseChangeTmp, brakeTime1Tmp, res1, res2] = WorkaroundCurrentVelocityToHigh(input, para, signedMaxVelocity, *_otg);
				brakeTime1 = brakeTime1Tmp;
				use2PhaseChange = use2PhaseChangeTmp;

				if (res1 < 0 || res2 < 0)
				{
					resultValues.CalculationResult = res1 < 0 ? res1 : res2;
					resultValues.Count = 0;
					return resultValues;
				}
			}

			auto targetVelocityReachable = true;
			if (!useVelocityInterface && para.TargetVelocity != 0) {
				auto [resTargetVelocity, targetVelocityReachableTmp] = WorkaroundTargetVelocity(input, para, *_otg);
				if (resTargetVelocity < 0) {
					resultValues.CalculationResult = resTargetVelocity;
					resultValues.Count = 0;
					return resultValues;
				}
				targetVelocityReachable = targetVelocityReachableTmp;
			}

			auto brakingIsPossible = true;
			if (!useVelocityInterface) {
				auto [resBrakeTrajectory, brakingIsPossibleTmp] = CheckBrakeTrajectory(input, para, *_otg);
				if (resBrakeTrajectory < 0) {
					resultValues.CalculationResult = resBrakeTrajectory;
					resultValues.Count = 0;
					return resultValues;
				}
				brakingIsPossible = brakingIsPossibleTmp;
			}

			input.current_position = { para.CurrentPosition };
			input.current_velocity = { para.CurrentVelocity };
			input.current_acceleration = { para.CurrentAcceleration };

			input.target_position = { para.TargetPosition };
			input.target_velocity = { para.TargetVelocity };
			input.target_acceleration = { para.TargetAcceleration };

			input.max_velocity = { para.MaxVelocity };

			input.control_interface = ControlInterface::Position;

			if (use2PhaseChange) {
				input.target_velocity = { signedMaxVelocity };
				input.max_velocity = { para.CurrentVelocity };
				input.control_interface = ControlInterface::Velocity;
			}

			if (!targetVelocityReachable)
				input.control_interface = ControlInterface::Velocity;

			if (useVelocityInterface) {
				input.control_interface = ControlInterface::Velocity;
				use2PhaseChange = false;
			}

			auto needToSwitchBack = use2PhaseChange;
			auto oneMoreRound = false;
			auto counter = 0;
			while ((_otg->update(input, output) == Result::Working || needToSwitchBack || para.TargetVelocity != 0 || !targetVelocityReachable) && (stepLimit < 0 || counter < stepLimit - 1) && currentStates->Length - 1 > counter) {

				currentStates[counter].Pos = output.new_position[0];
				currentStates[counter].Vel = output.new_velocity[0];
				currentStates[counter].Acc = output.new_acceleration[0];

				if (!useVelocityInterface && needToSwitchBack && brakeTime1 <= output.time) {
					input.control_interface = ControlInterface::Position;
					input.target_velocity = { para.TargetVelocity };
					input.max_velocity = { para.MaxVelocity };
					needToSwitchBack = false;
					oneMoreRound = true;
				}

				if (para.TargetVelocity > 0 && (output.new_position[0] >= para.TargetPosition || !targetVelocityReachable)) {
					input.control_interface = ControlInterface::Velocity;
					input.target_position = { para.TargetPosition + 1000 };
				}
				if (para.TargetVelocity < 0 && (output.new_position[0] <= para.TargetPosition || !targetVelocityReachable)) {
					input.control_interface = ControlInterface::Velocity;
					input.target_position = { para.TargetPosition - 1000 };
				}

				++counter;
				output.pass_to_input(input);
			}

			resultValues.CalculationResult = _otg->update(input, output);
			resultValues.Count = counter; // TODO +1?

			currentStates[counter].Pos = output.new_position[0];
			currentStates[counter].Vel = output.new_velocity[0];
			currentStates[counter].Acc = output.new_acceleration[0];

			return  resultValues;
		}


		ResultValues RuckigWrapper::GetPositionsIncludingBrakePosition(Parameter para, int stepLimit, int brakeTrajectoriesLimit, bool useVelocityInterface, array<array<CurrentState>^>^ currentStates, array<int>^ lengths) {

			ResultValues resultValues{ };

			InputParameter<1> input;
			OutputParameter<1> output;
			double brakeTime1;

			for (auto i = 0; i < lengths->Length; ++i)
				lengths[i] = 0;

			auto use2PhaseChange = false;

			input.max_acceleration = { para.MaxAcceleration };
			input.max_jerk = { para.MaxJerk };
			// TODO ? input.duration_discretization = DurationDiscretization::Discrete;
			input.control_interface = ControlInterface::Velocity;
			input.synchronization = Synchronization::None;
			auto signedMaxVelocity = para.CurrentVelocity > 0 ? para.MaxVelocity : -para.MaxVelocity;

			if (!useVelocityInterface &&
				(para.CurrentVelocity > 0 && para.TargetPosition > para.CurrentPosition && para.CurrentVelocity > para.MaxVelocity
					|| para.CurrentVelocity < 0 && para.TargetPosition < para.CurrentPosition && para.CurrentVelocity < -para.MaxVelocity)) {

				auto [use2PhaseChangeTmp, brakeTime1Tmp, res1, res2] = WorkaroundCurrentVelocityToHigh(input, para, signedMaxVelocity, *_otg);
				brakeTime1 = brakeTime1Tmp;
				use2PhaseChange = use2PhaseChangeTmp;

				if (res1 < 0 || res2 < 0)
				{
					resultValues.CalculationResult = res1 < 0 ? res1 : res2;
					return resultValues;
				}
			}

			auto targetVelocityReachable = true;
			if (!useVelocityInterface && para.TargetVelocity != 0) {
				auto [resTargetVelocity, targetVelocityReachableTmp] = WorkaroundTargetVelocity(input, para, *_otg);
				if (resTargetVelocity < 0) {
					resultValues.CalculationResult = resTargetVelocity;
					return resultValues;
				}
				targetVelocityReachable = targetVelocityReachableTmp;
			}

			auto brakingIsPossible = true;
			if (!useVelocityInterface) {
				auto [resBrakeTrajectory, brakingIsPossibleTmp] = CheckBrakeTrajectory(input, para, *_otg);
				if (resBrakeTrajectory < 0) {
					resultValues.CalculationResult = resBrakeTrajectory;
					return resultValues;
				}
				brakingIsPossible = brakingIsPossibleTmp;
			}

			for (auto i = 0; i < brakeTrajectoriesLimit + 1; ++i) {

				input.current_position = { para.CurrentPosition };
				input.current_velocity = { para.CurrentVelocity };
				input.current_acceleration = { para.CurrentAcceleration };

				input.target_position = { para.TargetPosition };
				input.target_velocity = { para.TargetVelocity };
				input.target_acceleration = { para.TargetAcceleration };

				input.max_velocity = { para.MaxVelocity };
				input.control_interface = ControlInterface::Position;

				if (use2PhaseChange) {
					input.target_velocity = { signedMaxVelocity };
					input.max_velocity = { para.CurrentVelocity };
					input.control_interface = ControlInterface::Velocity;
				}

				if (!targetVelocityReachable)
					input.control_interface = ControlInterface::Velocity;

				if (useVelocityInterface) {
					input.control_interface = ControlInterface::Velocity;
					use2PhaseChange = false;
				}

				auto counter = 1;
				auto braking = false;
				auto needToSwitchBack = use2PhaseChange;
				while ((_otg->update(input, output) == Result::Working || needToSwitchBack || (i == 0 && para.TargetVelocity != 0) || (i == 0 && !targetVelocityReachable)) && (stepLimit < 0 || counter - 1 <= stepLimit) && currentStates[i]->Length - 1 >= counter) { //  -1 from counter start with 1

					if (counter > i) {
						// ignore the values we already get from the main trajectory -> reduced collision check costs
						currentStates[i][counter - 1].Pos = output.new_position[0];
						currentStates[i][counter - 1].Vel = output.new_velocity[0];
						currentStates[i][counter - 1].Acc = output.new_acceleration[0];
					}
					else {
						currentStates[i][counter - 1].Pos = -42'000'000;
						currentStates[i][counter - 1].Vel = -42'000'000;
						currentStates[i][counter - 1].Acc = -42'000'000;
					}

					if (needToSwitchBack && brakeTime1 <= output.time) {
						input.control_interface = ControlInterface::Position;
						input.target_velocity = { para.TargetVelocity };
						input.max_velocity = { para.MaxVelocity };
						needToSwitchBack = false;
					}

					if (IsItTimeToBrake(counter, i)) {
						braking = true;
						needToSwitchBack = false; // doesnt matter any more
						input.control_interface = ControlInterface::Velocity;
						input.target_velocity = { 0 };
						input.target_acceleration = { 0 };
					}

					if (para.TargetVelocity > 0 && (output.new_position[0] >= para.TargetPosition || !targetVelocityReachable)) {
						input.control_interface = ControlInterface::Velocity;
						input.target_position = { para.TargetPosition + 1000 };
					}
					if (para.TargetVelocity < 0 && (output.new_position[0] <= para.TargetPosition || !targetVelocityReachable)) {
						input.control_interface = ControlInterface::Velocity;
						input.target_position = { para.TargetPosition - 1000 };
					}

					++counter;
					output.pass_to_input(input);
				}

				auto res = _otg->update(input, output);
				if (res < 0 || i == 0)
					resultValues.CalculationResult = res;

				if (res >= 0) {
					if (counter == 1 && Math::Abs(output.new_position[0] - para.CurrentPosition) < 0.0001f) // no pos change so dont report back
						lengths[i] = 0;
					else
						lengths[i] = counter;
				}

				currentStates[i][counter - 1].Pos = output.new_position[0];
				currentStates[i][counter - 1].Vel = output.new_velocity[0];
				currentStates[i][counter - 1].Acc = output.new_acceleration[0];
			}

			return  resultValues;
		}

		bool IsItTimeToBrake(int counter, int i)
		{
			return counter == i && i != 0;
		}


		// 1 axis for cps
		ValueTuple< List<CurrentState>^, ResultValues> RuckigWrapper::GetPositions(double td, Parameter para, int stepLimit, bool useVelocityInterface) {
			ResultValues resultValues{ };
			List<CurrentState>^ results = gcnew List<CurrentState>(stepLimit);

			Ruckig<1> otg{ td };
			InputParameter<1> input;
			OutputParameter<1> output;

			double brakeTime1;

			auto use2PhaseChange = false;

			input.max_acceleration = { para.MaxAcceleration };
			input.max_jerk = { para.MaxJerk };
			// TODO ? input.duration_discretization = DurationDiscretization::Discrete;
			input.control_interface = ControlInterface::Velocity;
			input.synchronization = Synchronization::None;

			auto signedMaxVelocity = para.CurrentVelocity > 0 ? para.MaxVelocity : -para.MaxVelocity;
			// 

			if (!useVelocityInterface &&
				(para.CurrentVelocity > 0 && para.TargetPosition > para.CurrentPosition && para.CurrentVelocity > para.MaxVelocity
					|| para.CurrentVelocity < 0 && para.TargetPosition < para.CurrentPosition && para.CurrentVelocity < -para.MaxVelocity))
			{

				auto [use2PhaseChangeTmp, brakeTime1Tmp, res1, res2] = WorkaroundCurrentVelocityToHigh(input, para, signedMaxVelocity, otg);
				brakeTime1 = brakeTime1Tmp;
				use2PhaseChange = use2PhaseChangeTmp;

				if (res1 < 0 || res2 < 0)
				{
					resultValues.CalculationResult = res1 < 0 ? res1 : res2;
					ValueTuple< List<CurrentState>^, ResultValues> resultTuple{ results, resultValues };
					return  resultTuple;
				}
			}

			auto targetVelocityReachable = true;
			if (!useVelocityInterface && para.TargetVelocity != 0) {
				auto [resTargetVelocity, targetVelocityReachableTmp] = WorkaroundTargetVelocity(input, para, otg);
				if (resTargetVelocity < 0) {
					resultValues.CalculationResult = resTargetVelocity;
					ValueTuple< List<CurrentState>^, ResultValues> resultTuple{ results, resultValues };
					return  resultTuple;
				}
				targetVelocityReachable = targetVelocityReachableTmp;
			}


			auto brakingIsPossible = true;
			if (!useVelocityInterface) {
				auto [resBrakeTrajectory, brakingIsPossibleTmp] = CheckBrakeTrajectory(input, para, otg);
				if (resBrakeTrajectory < 0) {
					resultValues.CalculationResult = resBrakeTrajectory;
					ValueTuple< List<CurrentState>^, ResultValues> resultTuple{ results, resultValues };
					return  resultTuple;
				}
				brakingIsPossible = brakingIsPossibleTmp;
			}

			input.current_position = { para.CurrentPosition };
			input.current_velocity = { para.CurrentVelocity };
			input.current_acceleration = { para.CurrentAcceleration };

			input.target_position = { para.TargetPosition };
			input.target_velocity = { para.TargetVelocity };
			input.target_acceleration = { para.TargetAcceleration };

			input.max_velocity = { para.MaxVelocity };

			input.control_interface = ControlInterface::Position;

			if (use2PhaseChange) {
				input.target_velocity = { signedMaxVelocity };
				input.max_velocity = { para.CurrentVelocity };
				input.control_interface = ControlInterface::Velocity;
			}

			if (!targetVelocityReachable)
				input.control_interface = ControlInterface::Velocity;

			if (useVelocityInterface) {
				input.control_interface = ControlInterface::Velocity;
				use2PhaseChange = false;
			}

			auto needToSwitchBack = use2PhaseChange;
			auto oneMoreRound = false;
			auto counter = 0;
			while ((otg.update(input, output) == Result::Working || needToSwitchBack || para.TargetVelocity != 0 || !targetVelocityReachable) && (stepLimit < 0 || counter < stepLimit - 1)) {

				CurrentState lastState{ output.new_position[0], output.new_velocity[0], output.new_acceleration[0] };
				results->Add(lastState);

				if (!useVelocityInterface && needToSwitchBack && brakeTime1 <= output.time) {
					input.control_interface = ControlInterface::Position;
					input.target_velocity = { para.TargetVelocity };
					input.max_velocity = { para.MaxVelocity };
					needToSwitchBack = false;
					oneMoreRound = true;
				}

				if (para.TargetVelocity > 0 && (output.new_position[0] >= para.TargetPosition || !targetVelocityReachable)) {
					input.control_interface = ControlInterface::Velocity;
					input.target_position = { para.TargetPosition + 1000 };
				}

				if (para.TargetVelocity < 0 && (output.new_position[0] <= para.TargetPosition || !targetVelocityReachable)) {
					input.control_interface = ControlInterface::Velocity;
					input.target_position = { para.TargetPosition - 1000 };
				}

				++counter;
				output.pass_to_input(input);
			}

			resultValues.CalculationResult = otg.update(input, output);
			CurrentState lastState{ output.new_position[0], output.new_velocity[0], output.new_acceleration[0] };
			results->Add(lastState);
			ValueTuple< List<CurrentState>^, ResultValues> resultTuple{ results, resultValues };
			return  resultTuple;
		}


		ValueTuple< array<List<CurrentState>^>^, ResultValues> RuckigWrapper::GetPositionsIncludingBrakePosition(double td, Parameter para, int stepLimit, int brakeTrajectoriesLimit, bool useVelocityInterface) {

			ResultValues resultValues{ };
			array<List<CurrentState>^>^ results = gcnew array< List<CurrentState>^>(brakeTrajectoriesLimit + 1);

			Ruckig<1> otg{ td };

			InputParameter<1> input;
			OutputParameter<1> output;
			double brakeTime1;

			auto use2PhaseChange = false;

			input.max_acceleration = { para.MaxAcceleration };
			input.max_jerk = { para.MaxJerk };
			// TODO ? input.duration_discretization = DurationDiscretization::Discrete;
			input.control_interface = ControlInterface::Velocity;
			input.synchronization = Synchronization::None;
			auto signedMaxVelocity = para.CurrentVelocity > 0 ? para.MaxVelocity : -para.MaxVelocity;

			if (!useVelocityInterface &&
				(para.CurrentVelocity > 0 && para.TargetPosition > para.CurrentPosition && para.CurrentVelocity > para.MaxVelocity
					|| para.CurrentVelocity < 0 && para.TargetPosition < para.CurrentPosition && para.CurrentVelocity < -para.MaxVelocity)) {

				auto [use2PhaseChangeTmp, brakeTime1Tmp, res1, res2] = WorkaroundCurrentVelocityToHigh(input, para, signedMaxVelocity, otg);
				brakeTime1 = brakeTime1Tmp;
				use2PhaseChange = use2PhaseChangeTmp;

				if (res1 < 0 || res2 < 0)
				{
					resultValues.CalculationResult = res1 < 0 ? res1 : res2;
					ValueTuple< array<List<CurrentState>^>^, ResultValues> resultTuple{ results, resultValues };
					return  resultTuple;
				}
			}

			auto targetVelocityReachable = true;
			if (!useVelocityInterface && para.TargetVelocity != 0) {
				auto [resTargetVelocity, targetVelocityReachableTmp] = WorkaroundTargetVelocity(input, para, otg);
				if (resTargetVelocity < 0) {
					resultValues.CalculationResult = resTargetVelocity;
					ValueTuple< array<List<CurrentState>^>^, ResultValues> resultTuple{ results, resultValues };
					return  resultTuple;
				}
				targetVelocityReachable = targetVelocityReachableTmp;
			}

			auto brakingIsPossible = true;
			if (!useVelocityInterface) {
				auto [resBrakeTrajectory, brakingIsPossibleTmp] = CheckBrakeTrajectory(input, para, otg);
				if (resBrakeTrajectory < 0) {
					resultValues.CalculationResult = resBrakeTrajectory;
					ValueTuple< array<List<CurrentState>^>^, ResultValues> resultTuple{ results, resultValues };
					return  resultTuple;
				}
				brakingIsPossible = brakingIsPossibleTmp;
			}

			for (auto i = 0; i < brakeTrajectoriesLimit + 1; ++i) {
				results[i] = gcnew List<CurrentState>(stepLimit);

				input.current_position = { para.CurrentPosition };
				input.current_velocity = { para.CurrentVelocity };
				input.current_acceleration = { para.CurrentAcceleration };

				input.target_position = { para.TargetPosition };
				input.target_velocity = { para.TargetVelocity };
				input.target_acceleration = { para.TargetAcceleration };

				input.max_velocity = { para.MaxVelocity };
				input.control_interface = ControlInterface::Position;

				if (use2PhaseChange) {
					input.target_velocity = { signedMaxVelocity };
					input.max_velocity = { para.CurrentVelocity };
					input.control_interface = ControlInterface::Velocity;
				}

				if (!targetVelocityReachable)
					input.control_interface = ControlInterface::Velocity;

				if (useVelocityInterface || i != 0) {
					input.control_interface = ControlInterface::Velocity;
					use2PhaseChange = false;
				}

				auto counter = 1;
				auto braking = false;
				auto needToSwitchBack = use2PhaseChange;
				while ((otg.update(input, output) == Result::Working || needToSwitchBack || (i == 0 && para.TargetVelocity != 0) || (i == 0 && !targetVelocityReachable)) && (stepLimit < 0 || counter < stepLimit + i)) {

					if (counter > i) {
						CurrentState lastState{ output.new_position[0], output.new_velocity[0], output.new_acceleration[0] };
						results[i]->Add(lastState);
					}

					if (needToSwitchBack && brakeTime1 <= output.time) {
						input.control_interface = ControlInterface::Position;
						input.target_velocity = { para.TargetVelocity };
						input.max_velocity = { para.MaxVelocity };
						needToSwitchBack = false;
					}

					if (counter == i && i != 0) {
						braking = true;
						input.control_interface = ControlInterface::Velocity;
						input.target_velocity = { 0 };
						input.target_acceleration = { 0 };
					}

					if (para.TargetVelocity > 0 && (output.new_position[0] >= para.TargetPosition || !targetVelocityReachable)) {
						input.control_interface = ControlInterface::Velocity;
						input.target_position = { para.TargetPosition + 1000 };
					}
					if (para.TargetVelocity < 0 && (output.new_position[0] <= para.TargetPosition || !targetVelocityReachable)) {
						input.control_interface = ControlInterface::Velocity;
						input.target_position = { para.TargetPosition - 1000 };
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

		ValueTuple<int, bool> RuckigWrapper::CheckBrakeTrajectory(ruckig::InputParameter<1Ui64>& input, ruckig::Wrapper::Parameter& para, ruckig::Ruckig<1Ui64>& otg)
		{
			Trajectory<1> brakeTrajectory;

			auto brakingIsPossible = true;
			input.current_position = { para.CurrentPosition };
			input.current_velocity = { para.CurrentVelocity };
			input.current_acceleration = { para.CurrentAcceleration };

			input.target_velocity = { 0 };
			input.target_acceleration = { 0 };
			input.max_velocity = { para.MaxVelocity };

			input.control_interface = ControlInterface::Velocity;

			auto res = otg.calculate(input, brakeTrajectory);

			if (res >= 0) {
				// Dont use extrema (min value is incorrect: eg -6E-66)
				ruckig::StandardVector<double, 1> pos;
				auto t = brakeTrajectory.get_duration();
				brakeTrajectory.at_time(t, pos);

				if (para.CurrentPosition < para.TargetPosition) {
					if (para.TargetPosition < pos[0])
						brakingIsPossible = false;
				}
				else if (para.CurrentPosition > para.TargetPosition) {
					if (pos[0] < para.TargetPosition)
						brakingIsPossible = false;
				}

				return ValueTuple<int, bool> {res, brakingIsPossible};
			}
			return ValueTuple<int, bool> {res, false};
		}

		ValueTuple<int, bool> RuckigWrapper::WorkaroundTargetVelocity(ruckig::InputParameter<1Ui64>& input, ruckig::Wrapper::Parameter& para, ruckig::Ruckig<1Ui64>& otg)
		{
			// 2nd workaround for hard kinematic limits
			// check if transition from current velocity to target velocity can be done in time -> otherwise use velocity interface
			Trajectory<1> targetSpeedCheckTrajectory;

			auto targetVelocityReachable = true;
			input.current_position = { para.CurrentPosition };
			input.current_velocity = { para.CurrentVelocity };
			input.current_acceleration = { para.CurrentAcceleration };

			input.target_velocity = { para.TargetVelocity };
			input.target_acceleration = { 0 };
			input.max_velocity = { para.MaxVelocity };

			input.control_interface = ControlInterface::Velocity;

			auto res = otg.calculate(input, targetSpeedCheckTrajectory);
			if (res >= 0) {
				ruckig::StandardVector<double, 1> pos;
				auto t = targetSpeedCheckTrajectory.get_duration();
				targetSpeedCheckTrajectory.at_time(t, pos);

				if (para.TargetVelocity > 0) {
					if (pos[0] > para.TargetPosition)
						targetVelocityReachable = false;
				}
				else if (para.TargetVelocity < 0) {
					if (pos[0] < para.TargetPosition)
						targetVelocityReachable = false;
				}

				return ValueTuple<int, bool> {res, targetVelocityReachable};
			}

			return ValueTuple<int, bool> {res, false};
		}


		// workaround for hard kinematic constraints (ruckig tries getting into the constraints without being time optimale)
		ValueTuple<bool, double, int, int> RuckigWrapper::WorkaroundCurrentVelocityToHigh(ruckig::InputParameter<1Ui64>& input, ruckig::Wrapper::Parameter& para, double signedMaxVelocity, ruckig::Ruckig<1Ui64>& otg)
		{
			Trajectory<1> phase1Trajectory;
			Trajectory<1> trajectory;

			input.current_position = { para.CurrentPosition };
			input.current_velocity = { para.CurrentVelocity };
			input.current_acceleration = { para.CurrentAcceleration };

			input.target_velocity = { signedMaxVelocity };
			input.target_acceleration = { 0 };
			input.max_velocity = { para.CurrentVelocity };

			auto res1 = otg.calculate(input, phase1Trajectory);
			Result res2{};
			auto use2PhaseChange = false;
			double brakeTime2;

			auto brakeTime1 = phase1Trajectory.get_duration();

			if (res1 >= 0) {
				StandardVector<double, 1> newPosition;
				trajectory.at_time(brakeTime1, newPosition);

				input.current_position = { newPosition[0] };
				input.current_velocity = { signedMaxVelocity };
				input.current_acceleration = { 0 };
				input.target_velocity = { 0 };
				input.target_acceleration = { 0 };

				input.max_velocity = { para.CurrentVelocity };

				res2 = otg.calculate(input, trajectory);
				brakeTime2 = trajectory.get_duration();
			}

			if (res1 >= 0 && res2 >= 0) {
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

			return ValueTuple<bool, double, int, int>{ use2PhaseChange, brakeTime1, res1, res2 };
		}

#pragma endregion

#pragma region DT

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

#pragma endregion

#pragma region Pcp


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
#pragma endregion

	}
}