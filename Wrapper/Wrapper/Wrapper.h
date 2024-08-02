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
			int Count;
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
			// CPS
			RuckigWrapper(double td);
			ResultValues GetPositions(Parameter para, int stepLimit, bool useVelocityInterface, array<CurrentState>^ currentStates);
			ResultValues GetPositionsIncludingBrakePosition(Parameter para, int stepLimit, int brakeTrajectoriesLimit, bool useVelocityInterface, array<array<CurrentState>^>^ currentStates, array<int>^ lengths);

			static ValueTuple< List<CurrentState>^, ResultValues> GetPositions(double td, Parameter para, int stepLimit, bool useVelocityInterface);
			static ValueTuple< array<List<CurrentState>^>^, ResultValues> GetPositionsIncludingBrakePosition(double td, Parameter para, int stepLimit, int brakeTrajectoriesLimit, bool useVelocityInterface);

			// PCP
			static ValueTuple< List<JerkStates>^, ResultValues>  GetValues(double td, Parameter parameter);

			// DT
			static StepState GetStep(Parameter parameter, double td);


		private:
			Trajectory<1>* _trajectory;
			Ruckig<1>* _otg;
			
			static ValueTuple<bool, double, int, int> WorkaroundCurrentVelocityToHigh(ruckig::InputParameter<1Ui64>& input, ruckig::Wrapper::Parameter& para, double signedMaxVelocity, ruckig::Ruckig<1Ui64>& otg);
			static ValueTuple<int, bool> WorkaroundTargetVelocity(ruckig::InputParameter<1Ui64>& input, ruckig::Wrapper::Parameter& para, ruckig::Ruckig<1Ui64>& otg);
			static ValueTuple<int, bool> CheckBrakeTrajectory(ruckig::InputParameter<1Ui64>& input, ruckig::Wrapper::Parameter& para, ruckig::Ruckig<1Ui64>& otg);
		};
		bool IsItTimeToBrake(int counter, int i);
	}
}
