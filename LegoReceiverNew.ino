#include "Arduino.h"

//#include "CustomIRReceiver.h"
#include "MotorDriverTB6612FNG.h"
#include "Servo.h"
#include "IrRemote.h"
#include "LegoPowerFunctionsProtocol.h"

/** define section
 */


// DEFINE DEBUG STUFF
	#define IS_DEBUG_ALL 0
	#define IS_DEBUG_STEERING 1
	#define IS_DEBUG_MOTOR_DRIVE 0
	#define IS_DEBUG_MOTOR_DUMP 0
	#define IS_DEBUG_IR 0
	#define IS_DEBUG_DRIVING_LIGHTS 0
	#define IS_DEBUG_BLINKING_LIGHTS 0

	#define IS_DEBUG_DELAY 0
	#define DEBUG_DELAY 100	// define the delay for debug, -> we have not so fast terminal output with it. BUT REMOVE IT for normal operation


	// overwrite debug settings above if all IS_DEBUG_ALL is set
	#if IS_DEBUG_ALL
		#define IS_DEBUG_STEERING 1
		#define IS_DEBUG_MOTOR_DRIVE 1
		#define IS_DEBUG_MOTOR_DUMP 1
		#define IS_DEBUG_IR 1
	#define IS_DEBUG_DRIVING_LIGHTS 1
	#endif

// DEFINE STATIC PIN OUTPUT MOTOR STUFF

	#define MOTOR_1_IN_1 4
	#define MOTOR_1_IN_2 7
	#define MOTOR_1_PWM 3
	#define MOTOR_1_STDBY 8

	#define MOTOR_2_IN_1 12
	#define MOTOR_2_IN_2 13
	#define MOTOR_2_PWM 5
	#define MOTOR_2_STDBY  8


// DEFINE MOTOR_DRIVER_MAPPING
	#define MOTOR_DRIVER_DRIVE 0

//	DEFINE STATIC PIN OUTPUT SERVO STUFF (USE PWM PINS)
	#define STEERING_SERVO_PIN 6
	#define STEERING_SERVO_LEFT_MAX 20
	#define STEERING_SERVO_RIGHT_MAX 160
//	#define DUMP_SERVO_PIN 6



// DEFINE LIGHT PINS
	#define FRONT_DRIVING_LIGHT_LEFT_PIN 10
	#define FRONT_DRIVING_LIGHT_RIGHT_PIN 2
	#define REAR_DRIVING_LIGHT_LEFT_PIN A2
	#define REAR_DRIVING_LIGHT_RIGHT_PIN A1


// DEFINE BLINKING_LIGHTS

	#define BLINKING_START_AFTER_STEERING 350
	#define BLINKING_STOP_AFTER_STEERING 450
	#define BLINK_LEFT -1
	#define BLINK_CENTER 0
	#define BLINK_RIGHT 1
	#define BLINKING_SPEED_ON 500
	#define BLINKING_SPEED_OFF 500


// DEFINE CHANNEL MAPPING

	#define CHANNEL_1ST 0
	#define CHANNEL_2ND 1
	#define CHANNEL_3RD 2
	#define CHANNEL_4TH 3

//	#define RED 0
//	#define BLUE 1

	#define MOTOR_DRIVE_CHANNEL CHANNEL_3RD
	#define MOTOR_DRIVE_SUBCHANNEL RED

	#define STEERING_SERVO_CHANNEL CHANNEL_3RD
	#define STEERING_SERVO_SUBCHANNEL BLUE

	#define PWM_LIMIT_LOW -7
	#define PWM_LIMIT_HIGH 7

	#define MOTOR_STOP 0 // pwm center for motor stop (used for timeout)
// DEFINE STATIC PIN IR STUFF
	#define IR_RECEIVE_PIN 11

	#define TIMEOUT_CHECK_INTERVALL 100	// interval in ms in which timeout should be checked



/**
 * object / var definitions
 */

// outputs
	// two lego motors
	//MotorDriverTB6612FNG motorA1(MOTOR_1_IN_1, MOTOR_1_IN_2, MOTOR_1_PWM, MOTOR_1_STDBY);
	MotorDriverTB6612FNG motorDriver[] = {MotorDriverTB6612FNG(MOTOR_1_IN_1, MOTOR_1_IN_2, MOTOR_1_PWM, MOTOR_1_STDBY)};

	// servo output
	Servo steeringServo;


// inputs
	IRrecv irrecv(IR_RECEIVE_PIN);
	decode_results results;


	// LegoPowerFunctionsProtocol legoReceiver[2] = {LegoPowerFunctionsProtocol(CHANNEL_1ST), LegoPowerFunctionsProtocol(CHANNEL_2ND)};
	LegoPowerFunctionsProtocol legoReceiver[4] = {LegoPowerFunctionsProtocol(CHANNEL_1ST),LegoPowerFunctionsProtocol(CHANNEL_2ND),LegoPowerFunctionsProtocol(CHANNEL_3RD),LegoPowerFunctionsProtocol(CHANNEL_4TH)};

	char steeringPosition = 0;
	char steeringTrim = 0;
	char newSteeringPositionWithoutTrim=0;

	char driveMotorSpeed = 0;
	char driveMotorTrim = 0;

//	char dumpMotorSpeed = 0;
//	char dumpMotorTrim = 0;

	// lights
	bool isDrivingLightEnabled = true;	// is light to be enabled or not
	bool isOverrideDrivingLights = false; // if set to true, current light setting will be ignored.

	// blinking lights
	char oldSteeringBlinkPosition = BLINK_CENTER;
	long steeringDirectionChangeTime = 0;
	// define which pins are used for left / right blinking lights
//	int blinkLeftPins[] = {FRONT_DRIVING_LIGHT_LEFT_PIN, REAR_DRIVING_LIGHT_LEFT_PIN };
//	int blinkRightPins[] = {FRONT_DRIVING_LIGHT_RIGHT_PIN, REAR_DRIVING_LIGHT_RIGHT_PIN};

	long blinkLastChange = 0; // time of last change of blinking on / off
	bool blinkLastStatus = false; // was the blinkingLight on or off on last loop iteration
	bool isBlinkingLightSharedWithDrivingLight = true; // sets if blinking light is shared with driving light

	// timeout
	long lastTimeoutCheck = 0; // time of last timeout check




//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(115200); // start the terminal output

	delay(300);
	Serial.println("starting main setup...");


	#if IS_DEBUG_DELAY
		Serial.print("DELAY FOR DEBUGGING IS ENABLED WITH");
		Serial.print((int) DEBUG_DELAY);
		Serial.println("ms. YOU WILL HAVE BAD REACTION TIMES!");

	#endif

//	Serial.print("blinkLeftPins: ");
//	for (int i=0; i < sizeof(blinkLeftPins); i++)
//	{
//		Serial.print(blinkLeftPins[i]);
//		Serial.print(" ");
//	}
//
//	Serial.print("blinkRightPins: ");
//	for (int i=0; i < sizeof(blinkRightPins); i++)
//	{
//		Serial.print(blinkRightPins[i]);
//		Serial.print(" ");
//	}

	irrecv.enableIRIn();


	// setup lights
	pinMode(FRONT_DRIVING_LIGHT_LEFT_PIN, OUTPUT);
	pinMode(FRONT_DRIVING_LIGHT_RIGHT_PIN, OUTPUT);
	pinMode(REAR_DRIVING_LIGHT_LEFT_PIN, OUTPUT);
	pinMode(REAR_DRIVING_LIGHT_RIGHT_PIN, OUTPUT);

	// attach servo and move to center position
	steeringServo.attach(STEERING_SERVO_PIN);
	char steeringCenter = calculateNewTargetWithTrim(steeringTrim, 0, PWM_LIMIT_LOW, PWM_LIMIT_HIGH);
	steeringServo.write(map(steeringCenter,PWM_LIMIT_LOW, PWM_LIMIT_HIGH, STEERING_SERVO_LEFT_MAX, STEERING_SERVO_RIGHT_MAX));

}

// The loop function is called in an endless loop
void loop()
{


	// get Data from IR
	if (irrecv.decode(&results))
	{
		#if IS_DEBUG_IR
			Serial.print("DEBUG_IR: decode_type=");
			Serial.println(results.decode_type);
		#endif

		// check if results type is lego
		if (results.decode_type == 9)
		{
			// from here on we have a valid lego protocol sample

			unsigned int receivedData = (unsigned int) results.value;

			#if IS_DEBUG_IR
				Serial.print(" results.value=");
				Serial.print(results.value,BIN);
				Serial.print(" receivedData=");
				Serial.println(receivedData);
			#endif


			// update current / correct receiver
			char currentReceiver = legoReceiver[0].getChannel(receivedData);
			legoReceiver[(int)currentReceiver].updateDataAndProcess(receivedData);


			// process drive motor
			char newDriveSpeed = legoReceiver[MOTOR_DRIVE_CHANNEL].getSpeedBySubChannelID(MOTOR_DRIVE_SUBCHANNEL);
			#if IS_DEBUG_MOTOR_DRIVE
				Serial.print("DEBUG_MOTOR_DRIVE: newDriveSpeed=");
				Serial.println((int) newDriveSpeed);
			#endif
			newDriveSpeed = calculateNewTargetWithTrim(driveMotorTrim, newDriveSpeed, PWM_LIMIT_LOW, PWM_LIMIT_HIGH);
			#if IS_DEBUG_MOTOR_DRIVE
				Serial.print("DEBUG_MOTOR_DRIVE: newDriveSpeed after Trim=");
				Serial.print((int) newDriveSpeed);
				Serial.print(" driveMotorTrim=");
				Serial.println((int) driveMotorTrim);
			#endif
			motorDriver[MOTOR_DRIVER_DRIVE].movePWMTwoWay(newDriveSpeed, PWM_LIMIT_LOW, PWM_LIMIT_HIGH);


			// process the steering
			newSteeringPositionWithoutTrim = legoReceiver[STEERING_SERVO_CHANNEL].getSpeedBySubChannelID(STEERING_SERVO_SUBCHANNEL);
			#if IS_DEBUG_STEERING
				Serial.print("DEBUG_STEERING: newSteeringPosition=");
				Serial.println((int) newSteeringPositionWithoutTrim);
			#endif
			char newSteeringPositionWithTrim = calculateNewTargetWithTrim(steeringTrim, newSteeringPositionWithoutTrim, PWM_LIMIT_LOW, PWM_LIMIT_HIGH);
			#if IS_DEBUG_STEERING
				Serial.print("DEBUG_STEERING: newSteeringPosition after Trim=");
				Serial.print((int) newSteeringPositionWithTrim);
				Serial.print(" steeringTrim=");
				Serial.println((int) steeringTrim);
				Serial.println(map(newSteeringPositionWithTrim,PWM_LIMIT_LOW, PWM_LIMIT_HIGH, 20, 160));
			#endif
			steeringServo.write(map(newSteeringPositionWithTrim,PWM_LIMIT_LOW, PWM_LIMIT_HIGH, 20, 160));




//			// process dump motor
//			char newDumpSpeed = legoReceiver[MOTOR_DUMP_CHANNEL].getSpeedBySubChannelID(MOTOR_DUMP_SUBCHANNEL);
//			#if IS_DEBUG_MOTOR_DUMP
//				Serial.print("DEBUG_MOTOR_DUMP: newDumpSpeed=");
//				Serial.println(newDumpSpeed);
//			#endif
//			newDumpSpeed = calculateNewTargetWithTrim(dumpMotorTrim, newDumpSpeed, PWM_LIMIT_LOW, PWM_LIMIT_HIGH);
//			#if IS_DEBUG_STEERING
//				Serial.print("DEBUG_MOTOR_DUMP: newDumpSpeed after Trim=");
//				Serial.print(newDumpSpeed);
//				Serial.print(" dumpMotorTrim=");
//				Serial.println(dumpMotorTrim);
//			#endif
//			dumpServoMotor.write(map(newDumpSpeed,PWM_LIMIT_LOW, PWM_LIMIT_HIGH, 0, 180));

		}
		irrecv.resume();
	}
	// process lights
	processBlinkingLights(newSteeringPositionWithoutTrim); // process is first, because it may override the driving lights
	processDrivingLights();

	// in any case process the timeouts. not in else tree, because we may receive on channel 1 and channel 4 may run into timeout.
	processTimeouts();
	#if IS_DEBUG_DELAY
		delay (100);
	#endif
}

/**
 * checks if the driving light needs to be enabled or disabled.
 * after check it sets the light.
 */
void processDrivingLights()
{
	#if IS_DEBUG_DRIVING_LIGHTS
		Serial.print("processDrivingLights: override=");
		Serial.println(isOverrideDrivingLights);
	#endif

	if (!isOverrideDrivingLights)
	{
		#if IS_DEBUG_DRIVING_LIGHTS
			Serial.print("processDrivingLights: setting lights to ");
			Serial.println(isDrivingLightEnabled);
		#endif
		digitalWrite(FRONT_DRIVING_LIGHT_LEFT_PIN, isDrivingLightEnabled);
		digitalWrite(FRONT_DRIVING_LIGHT_RIGHT_PIN, isDrivingLightEnabled);
		digitalWrite(REAR_DRIVING_LIGHT_LEFT_PIN, isDrivingLightEnabled);
		digitalWrite(REAR_DRIVING_LIGHT_RIGHT_PIN, isDrivingLightEnabled);
	}
}





/**
 * process timeouts
 */
void processTimeouts()
{
	if (millis() - lastTimeoutCheck - TIMEOUT_CHECK_INTERVALL > 0)
	{
		// interval is through, need to check again
		// process timeout for steering servo
		if (legoReceiver[STEERING_SERVO_CHANNEL].isTimeoutReached(STEERING_SERVO_SUBCHANNEL))
		{
			#if IS_DEBUG_STEERING
				Serial.println("processTimeouts: Timeout for STEERING is reached. Set to center.");
			#endif
			char steeringCenter = calculateNewTargetWithTrim(steeringTrim, 0, PWM_LIMIT_LOW, PWM_LIMIT_HIGH);
			steeringServo.write(map(steeringCenter,PWM_LIMIT_LOW, PWM_LIMIT_HIGH, STEERING_SERVO_LEFT_MAX, STEERING_SERVO_RIGHT_MAX));
		}

		// process timeout for drive motor
		if (legoReceiver[MOTOR_DRIVE_CHANNEL].isTimeoutReached(MOTOR_DRIVE_SUBCHANNEL))
		{
			#if IS_DEBUG_MOTOR_DRIVE
				Serial.println("processTimeouts: Timeout for MOTOR_DRIVE is reached. Set to center.");
			#endif
			motorDriver[MOTOR_DRIVER_DRIVE].movePWMTwoWay(MOTOR_STOP, PWM_LIMIT_LOW, PWM_LIMIT_HIGH);
		}
	}

}


/**
 * applies trim to the new value
 */
char calculateNewTargetWithTrim(char trimValue, char newValue, char limitLow, char limitUp )
{
	char out=0;
	out = newValue + trimValue;
	return out;
}


/**
 * does the blinking on the current steering position send by sender. needs to be without trim.
 */
void processBlinkingLights(char steeringPosition)
{
	// check if we need to blink
	char blinkingPosition = needToBlink(steeringPosition);
	bool blinkTargetStatus = blinkLastStatus;

	#if IS_DEBUG_BLINKING_LIGHTS
		Serial.print("processBlinkingLights: blinkingPos=");
		Serial.print((int) blinkingPosition);
	#endif

	if ( blinkingPosition == BLINK_CENTER)
	{

		// process blinking off after steering time

		long diff = (millis() - steeringDirectionChangeTime - BLINKING_STOP_AFTER_STEERING);
		if ( diff > 0)
		{

			#if IS_DEBUG_BLINKING_LIGHTS
				Serial.print("processBlinkingLights: diff=");
				Serial.print((int) diff);
			#endif

			// switch of blinking. Restore old driving lights, if they have been overridden by blinking.
			// just stop the override
			if (isBlinkingLightSharedWithDrivingLight)
			{
				isOverrideDrivingLights = false;
			}
		}
		else
		{
			// do not change anything, because timer is not through
		}

		#if IS_DEBUG_BLINKING_LIGHTS
			Serial.print(", switch off blinking. Set blinkingPos=");
			Serial.print((int) blinkingPosition);
		#endif
	}
	else
	{
		if (isBlinkingLightSharedWithDrivingLight)
		{
			isOverrideDrivingLights = true;
		}
		// we need to blink.
		if (blinkLastStatus)
		{
	 		// blinking light was on last time

			if (millis() - blinkLastChange > BLINKING_SPEED_ON )
			{
				// disable blinking
				blinkTargetStatus = false;
			}
			else
			{
				// do not change BlinkTargetStatus
			}

			#if IS_DEBUG_BLINKING_LIGHTS
				Serial.print("processBlinkingLights: blinkingPos=");
				Serial.print((int) blinkingPosition);
			#endif
		}
		else
		{
			// blinking light was off
			if (millis() - blinkLastChange > BLINKING_SPEED_OFF )
			{
				// enable blinking
				blinkTargetStatus = true;
			}
		}


		#if IS_DEBUG_BLINKING_LIGHTS
			Serial.print(", blinkTargetStatus=");
			Serial.print((int) blinkTargetStatus);
		#endif

		if (blinkLastStatus != blinkTargetStatus)
			{
			if (blinkingPosition == BLINK_LEFT)
			{
				// blink left
//				for (int i=0; i < sizeof(blinkLeftPins); i++)
//				{
//					digitalWrite(blinkLeftPins[i], blinkTargetStatus);
//					blinkLastChange=millis();
//					#if IS_DEBUG_BLINKING_LIGHTS
//						Serial.print(", set ");
//						Serial.print((int) blinkLeftPins[i]);
//						Serial.print(blinkTargetStatus);
//					#endif
//				}
				digitalWrite(FRONT_DRIVING_LIGHT_LEFT_PIN, blinkTargetStatus);
				digitalWrite(REAR_DRIVING_LIGHT_LEFT_PIN, blinkTargetStatus);
				blinkLastChange=millis();

			}
			else
			{
				// blink right
//				for (int i=0; i < sizeof(blinkRightPins); i++)
//				{
//					digitalWrite(blinkRightPins[i], blinkTargetStatus);
//					blinkLastChange=millis();
//					#if IS_DEBUG_BLINKING_LIGHTS
//						Serial.print(", set ");
//						Serial.print((int) blinkRightPins[i]);
//						Serial.print(blinkTargetStatus);
//					#endif
//				}
				digitalWrite(FRONT_DRIVING_LIGHT_RIGHT_PIN, blinkTargetStatus);
				digitalWrite(REAR_DRIVING_LIGHT_RIGHT_PIN, blinkTargetStatus);
				blinkLastChange=millis();
			}
		}
	}

	// remember the target status in any case
	blinkLastStatus = blinkTargetStatus;

	#if IS_DEBUG_BLINKING_LIGHTS
		Serial.print(", blinkLastChange=");
		Serial.print((long) blinkLastChange);
		Serial.print(", final isOverrideDrivingLights=");
		Serial.println(isOverrideDrivingLights);
	#endif
}

/**
 *  this function calculates, if we need to simulate blinking
 *  following assumptions:
 *  steering center before trim is 0
 *  steering left is < 0
 *  steering right is > 0
 *
 *
 *  definition:
 *  - if steering by transmitter is set for longer than BLINKING_START_AFTER_STEERING, then blink
 *  - if steering is in opposite direction or neutral after blinking for BLINKING_STOP_AFTER_STEERING, then stop blinking
 *  returns the state, if we need to blink and in which direction. Will return BLINK_LEFT = -1, BLINK_CENTER = 0 = stop blinking, BLINK_RIGHT = 1
 *
 *
 */
char needToBlink(char steeringPosition)
{
	char needToBlink=BLINK_CENTER; // (set to no blinking as default)
	char currentSteerPosition = BLINK_CENTER;

	// parse the current steering position
	if (steeringPosition == 0)
	{
		// center
		currentSteerPosition = BLINK_CENTER;
	}
	else
	{
		if (steeringPosition <0)
		{
			// left
			currentSteerPosition = BLINK_LEFT;
		}
		else
		{
			// right
			currentSteerPosition = BLINK_RIGHT;
		}
	}

	#if IS_DEBUG_BLINKING_LIGHTS
		Serial.print("needToBlink: SteeringInput=");
		Serial.print((int)steeringPosition);
		Serial.print(", currentSteerPosition=");
		Serial.print((int)currentSteerPosition);
		Serial.print(", oldSteeringBlinkPosition=");
		Serial.println((int)oldSteeringBlinkPosition);
	#endif

	// check if the steering position is still the same
	if (currentSteerPosition == oldSteeringBlinkPosition)
	{
		// still the same direction, now process center and left / right differently
		// check if position is center
		if (currentSteerPosition == BLINK_CENTER)
		{
			// check if we should turn of blinking
			if (millis() - steeringDirectionChangeTime - BLINKING_STOP_AFTER_STEERING >=0)
			{
				// long enough in center position to stop blinking
//				needToBlink=false;
				needToBlink=BLINK_CENTER;
			}
			else
			{
				// not enough delay yet after steering stop to stop blinking -> keep the last blinking position
				needToBlink=oldSteeringBlinkPosition;
			}

			#if IS_DEBUG_BLINKING_LIGHTS
				Serial.print("needToBlink: currentSteerPosition=");
				Serial.print((int)currentSteerPosition);
				Serial.print(", timer stop after steering=");
				Serial.print((long) (millis() - steeringDirectionChangeTime - BLINKING_STOP_AFTER_STEERING >=0));
				Serial.print(", set needToBlink=");
				Serial.println((int)needToBlink);
			#endif

		}
		else
		{
		// still the same direction and left or right -> check the millis
			if (millis() - steeringDirectionChangeTime >= BLINKING_START_AFTER_STEERING )
			{
				needToBlink = currentSteerPosition;
			}
			else
			{
				// not enough delay yet after steering start to start blinking -> set to no blinking
				needToBlink = BLINK_CENTER;
			}
		}
	}
	else
	{
		// the steering direction has been changed
		// -> remember that the steering state has been changed yet. It does not matter, how it changed.
		steeringDirectionChangeTime = millis();
		Serial.print("needToBlink: setting steeringDirectionChangeTime=");
		Serial.println((long)steeringDirectionChangeTime);
	}
	oldSteeringBlinkPosition=currentSteerPosition;
	#if IS_DEBUG_BLINKING_LIGHTS
		Serial.print("needToBlink: return=");
		Serial.println((int)needToBlink);
	#endif
	return needToBlink;
}
