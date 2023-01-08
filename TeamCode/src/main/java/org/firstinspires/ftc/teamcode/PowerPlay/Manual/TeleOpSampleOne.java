/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.PowerPlay.Manual;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.closePosition;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.openPosition;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class TeleOpSampleOne extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        initializeNvyusRobotHardware(this);

        double currentPosition = LSMotor1.getCurrentPosition();
        LSMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

/*
            leftFrontPower = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower = gamepad1.b ? 1.0 : 0.0;  // B gamepad*/


            // Send calculated power to wheels
            FrontLeftMotor.setPower(leftFrontPower * .5);
            FrontRightMotor.setPower(rightFrontPower * .5);
            BackLeftMotor.setPower(leftBackPower * .5);
            BackRightMotor.setPower(rightBackPower * .5);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();


            if (gamepad2.left_stick_y < -0.3) { //left go up
                setSlidesVelocity(LSMotor1, 0.8);
                setSlidesVelocity(LSMotor2, 0.8);
            } else if (gamepad2.left_stick_y > 0.3) { // down retract
                setSlidesVelocity(LSMotor1, -0.4);
                setSlidesVelocity(LSMotor2, -0.4);
            } else {
                LSMotor1.setZeroPowerBehavior(FLOAT);
                setSlidesVelocity(LSMotor1, 0);
                setSlidesVelocity(LSMotor2, 0);
            }

            if (gamepad2.a) { //close grabber
                Grabber.setPosition(closePosition);
            } else if (gamepad2.b) { //open grabber
                Grabber.setPosition(openPosition);
            }

            if (gamepad2.x) { //break motor
                LSMotor1.setZeroPowerBehavior(BRAKE);
                LSMotor2.setZeroPowerBehavior(BRAKE);
                setSlidesVelocity(LSMotor1, 0);
                setSlidesVelocity(LSMotor2, 0);
            }

            if (currentPosition < 0) {
                setSlidesVelocity(LSMotor1, 0.1);
                setSlidesVelocity(LSMotor2, 0.1);
            }

            telemetry.addLine("pos: " + currentPosition);
            telemetry.update();
            currentPosition = LSMotor1.getCurrentPosition();
        }
    }
}
