/* Copyright (c) 2014 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */


/* Copyright (c) 2015 MISSION CONTROL ROBOTICS

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are not permitted (subject to the exceptions in the disclaimer below)
Exceptions are provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Mission Control Robotics nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Credits: Rishi Shridharan, Yen Shei. */


//Begin Code

//THIS CODE DOES NOT USE MOTOR ENCODERS

/*


CAUTION, UPON INIT BUTTON PRESS, GYRO WILL BE CALIBRATED TO THE POSITION IT IS IN!


 */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


public class TeleTest5Dec extends OpMode {

    public double tophatpower = 0.25; //Use for tophat drive
    public boolean sweepcount = false;

    //Component declarations
    DcMotorController steerControl;
    DcMotor motorR;
    DcMotor motorL;
    DcMotor motorX, motorY, motorZ;
    TouchSensor touch;
    ColorSensor color;
    GyroSensor Gyro;
    ServoController scontrol;
    Servo continuous1;

    /*
     * Code to run when the op mode is first enabled goes here.
     *
     * Code Credits: Rishi Shridharan, Yen Shei
     */
    @Override
    public void init() {


		/*
         * Use the hardwareMap to get the dc motors and servos by name. Note
		 * that the names of the devices must match the names used when you
		 * configured your robot and created the configuration file.
		 */

		/*
         * For our robot, we use the following:
		 *   motorR is right two wheels
		 *   motorL is lett two wheels
		 *   Both of these are controlled by HiTechnic "steerControl"
		 *   motorX/motorY/motorZ are for basket and extension controls
		 *
		 *   This code is in standard format.
		 *      • Init will run initialization code
		 *      • Loop will continuously run until 2:00 of driver controlled are up
		 *      • Stop occurs when the stop button is pressed... Keep nothing here for safety sake
		 */

        telemetry.addData("INIT IS NOW BEGINNING", "ROBOT SHOULD BE FULLY POSITIONED");

        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        motorX = hardwareMap.dcMotor.get("motorXY");
        motorY = hardwareMap.dcMotor.get("motorXZ");
        motorY.setDirection(DcMotor.Direction.REVERSE);
        motorZ = hardwareMap.dcMotor.get("motorYZ");
        steerControl = hardwareMap.dcMotorController.get("steerControl");
        motorL.setDirection(DcMotor.Direction.REVERSE);
        touch = hardwareMap.touchSensor.get("touch");
        color = hardwareMap.colorSensor.get("Color");
        Gyro = hardwareMap.gyroSensor.get("Gyro");
        Gyro.calibrate();
        scontrol = hardwareMap.servoController.get("Servo Controller 1");
        continuous1 = hardwareMap.servo.get("continuous1");
        continuous1.setDirection(Servo.Direction.FORWARD);

        telemetry.addData("INIT IS NOW ENDING", "AUTONOMOUS IS STARTING");
    }


    @Override
    public void loop() {



        int xVal, yVal, zVal = 0;
        int heading = 0;
        /*
         * Gamepad 1
		 * 
		 * Gamepad 1 controls the motors via the left stick, and it controls the
		 * servos via the a,b, x, y buttons
		 */

    /*
    *
    * Gamepad 2
    *
    * Gamepad 2 controls the basket X/ Y/ Z axis of movement which includes extension of the crossbar scoring mechanism
    * via left stick X/Y and right stick X
     */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
        float right = throttle - direction;
        float left = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // write the values to the motors
        motorR.setPower(right);
        motorL.setPower(left);

        motorX.setPower((gamepad2.left_stick_x)); //Extension of the arm must be as fast as possible
        //Encoder will need to be used if possible to stop motor from running past stopping point

        if (gamepad2.left_stick_y == 0) {
            motorY.setPower(0.06); //Hold position of basket
        } else if (gamepad2.left_stick_y <= 0) {
            motorY.setPower(0.02); //Decrease speed when basket mechanism is lowering
        } else if (gamepad2.left_stick_y >= 0) {
            motorY.setPower((gamepad2.left_stick_y) / 4); //Let joystick dictate power of motor when lifting
        } else {
            //Nothing
            motorY.setPower(0);
        }

        motorZ.setPower((gamepad2.right_stick_x) / 3); //Turning Left or Right must be a little slower than joystick command


        if (gamepad1.back || gamepad2.back) {
            System.exit(0);
            //Something has gone incorrect in testing: Emergency Stop
        }

        //Update tophat values
        if (gamepad1.left_stick_button && tophatpower > 0.05) {
            tophatpower = tophatpower - 0.05;
        } else if (gamepad1.left_stick_button && tophatpower == 0.05) {
            //Nothing
        } else if (gamepad1.left_bumper && tophatpower < 0.95) {
            tophatpower = tophatpower + 0.05;
        } else if (gamepad1.left_bumper && tophatpower == 0.95) {
            tophatpower = 0.90;
        }

        //Just to initialize tophat commands for future use
        boolean topup = gamepad1.dpad_up;
        boolean topright = gamepad1.dpad_right;
        boolean topdown = gamepad1.dpad_down;
        boolean topleft = gamepad1.dpad_left;

        //use of tophat values
        if (topup) {
            motorR.setPower(tophatpower);
            motorL.setPower(tophatpower);
        } else if (topdown) {
            motorR.setPower(-tophatpower);
            motorL.setPower(-tophatpower);
        } else if (topright) {
            motorL.setPower(tophatpower / 2);
            motorR.setPower(-tophatpower / 2);
        } else if (topleft) {
            motorR.setPower(tophatpower / 2);
            motorL.setPower(-tophatpower / 2);
        } else {
            //Nothing
        }

        // update the position of the servos
        if (gamepad1.a) {
            // code

        }

        if (gamepad1.y) {
            // code for button y press here

        }

        // update the position of the servo
        if (gamepad1.x) {
            // code
        }

        if (gamepad1.b || gamepad2.b) {

            if (continuous1.getPosition() != 0.5) {
                /// code to toggle continuous servo on
                continuous1.setPosition(0.5);
            }
            else {
                continuous1.setPosition(1);
                sweepcount = false;
            }
        }

        if (touch.isPressed()) {
            //action for pressed
        } else {
            //action for unpressed
        }

        if (color.red() > color.blue()) {
            //I found the red LED!
        } else if (color.blue() > color.red()) {
            //I found the blue LED!
        } else {
            //Don't know... let's guess or get closer!
        }

        xVal = Gyro.rawX();
        yVal = Gyro.rawY();
        zVal = Gyro.rawZ();

        heading = Gyro.getHeading();

        telemetry.addData("1. x", String.format("%03d", xVal));
        telemetry.addData("2. y", String.format("%03d", yVal));
        telemetry.addData("3. z", String.format("%03d", zVal));
        telemetry.addData("4. HEADING (resets)", String.format("%03d", heading));


    }

    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {
        //Immediately stop robot
        killmotors();
    }

    public void killmotors() {
        //Write power 0 to all motors
        motorL.setPower(0);
        motorR.setPower(0);
        motorX.setPower(0);
        motorY.setPower(0);
        motorZ.setPower(0);
    }
//Not used method for regulating values to only fall into certain value positions
    /*double scaleInput(double dVal)  {
        double[] scaleArray = {  -1.00,-0.85,-0.72,-0.60,-0.50,-0.43,-0.36,-0.30,-0.24,-0.18,-0.15,-0.12,-0.10,-0.09,-0.05,
                0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00 };

		// get the corresponding index for the scaleInput array.
		int index = (int) ((dVal * 16.0)+16);

		// index should be positive.
		if (index < 0) {
            index = -index;
        }

		// index cannot exceed size of array minus 1.
		if (index > 16) {
			index = 16;
		}

		// get value from the array.
		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		// return scaled value.
		return dScale;
	}*/

}