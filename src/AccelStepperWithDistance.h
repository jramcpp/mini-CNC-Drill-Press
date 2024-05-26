#ifndef AccelStepperWithDistance_h
#define AccelStepperWithDistance_h

#include <AccelStepper.h>

class AccelStepperWithDistance : public AccelStepper {
    private:
    // Set-up values for travel with distances in mm
		int microSteps = 4 ;           // Set microsteps from Driver
		int stepsPerRotation = 800;    // Set steps per Rotation from Driver
		float distancePerRotation = 4; // Set distance traveled per rotation (same as "thread" lead from catalog of lead screw)
		float anglePerRotation = 360;

    public:
		AccelStepperWithDistance(
			uint8_t interface = AccelStepper::DRIVER, 
			uint8_t pin1 = 2, 
			uint8_t pin2 = 3, 
			uint8_t pin3 = 4, 
			uint8_t pin4 = 5, 
			bool enable = true);

		void setMicroStep(int value);

		void setStepsPerRotation(int value);

		void setDistancePerRotation(float value);

		float getCurrentPositionDistance();

		void moveToDistance(float value);

    void moveRelative(float value);
    
    void runRelative(float value);

		void runToNewDistance(float value);

		void setAnglePerRotation(float value);

		void moveToAngle(float value);

		void runToNewAngle(float value);
};

#endif
