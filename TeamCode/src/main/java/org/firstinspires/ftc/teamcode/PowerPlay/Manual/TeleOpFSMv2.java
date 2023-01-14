/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice,  list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, 
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from  software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY 
 * LICENSE.  SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.PowerPlay.Manual;

import  static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.closePosition;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.groundEncoderCount;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.lowEncoderCount;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.medEncoderCount;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.openPosition;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.topEncoderCount;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.PIDControl.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;

import org.firstinspires.ftc.teamcode.PowerPlay.Helpers.PIDControl.*;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class TeleOpFSMv2 extends LinearOpMode {

    private PIDController controller;

    public static double p = 0.005, i = 0, d = 0.000;
    public static double f = 0.05;

    double ticks_in_degrees = 360 / 145.1;

    public static int target1 = lowEncoderCount;
    
    int DEPOSIT_STATES = 0;
    // 0 = ground
    // 1 = low
    // 2 = med
    // 3 = high



    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    double prevTime = runtime.milliseconds();
    @Override
    public void runOpMode() {

        initializeNvyusRobotHardware(this);
        double currentPosition = LSMotor1.getCurrentPosition();



        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   =  gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_y;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            //  ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            double slow_multiplier = Math.max(1-gamepad1.right_trigger,0.3);
            // Send calculated power to wheels
            FrontLeftMotor.setPower(leftFrontPower * .5*slow_multiplier);
            FrontRightMotor.setPower(rightFrontPower * .5*slow_multiplier);
            BackLeftMotor.setPower(leftBackPower * .5*slow_multiplier);
            BackRightMotor.setPower(rightBackPower * .5*slow_multiplier);

            currentPosition = -LSMotor1.getCurrentPosition();
//            telemetry.addLine("position: " + LSMotor1.getCurrentPosition());
//            telemetry.update();

            telemetry.addData("pos: ", currentPosition); // added this line and cut from PID function
            telemetry.addData("target ", target1);
            telemetry.update();



            total(currentPosition);

            if (gamepad1.left_bumper) { //close grabber
                Grabber.setPosition(closePosition);
            } else if (gamepad1.left_trigger > 0) { //open grabber
                Grabber.setPosition(openPosition);
            }

        }

        LSMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void controller2Buttons () {
        if (gamepad2.a){
            DEPOSIT_STATES = 0;
        }
        if (gamepad2.x && DEPOSIT_STATES < 1){
            DEPOSIT_STATES = 1;
        }
        if (gamepad2.b && DEPOSIT_STATES < 2){
            DEPOSIT_STATES = 2;
        }
        if (gamepad2.y && DEPOSIT_STATES < 3){
            DEPOSIT_STATES = 3;
        }
    }

    public void moveToState(double pos){
        switch (DEPOSIT_STATES){
            case 0:
                if (gamepad2.left_stick_y < -0.3) { //left go up
                    if (pos < 400) {
                        setSlidesVelocity(LSMotor1, 0.3);
                        setSlidesVelocity(LSMotor2, 0.3);
                    }
                    else if (pos < 800) {
                        setSlidesVelocity(LSMotor1, 0.6);
                        setSlidesVelocity(LSMotor2, 0.6);
                    }
                    else {
                        setSlidesVelocity(LSMotor1, 0.8);
                        setSlidesVelocity(LSMotor2, 0.8 );
                    }
                } else if (gamepad1.left_stick_y > 0.3) { //tap to make linear slide stop during retract
                    setSlidesVelocity(LSMotor1, -0.1);
                    setSlidesVelocity(LSMotor2, -0.1);
                }
                else {
                    LSMotor1.setZeroPowerBehavior(FLOAT);
                    setSlidesVelocity(LSMotor1, 0);
                    setSlidesVelocity(LSMotor2, 0);
                }

                break;
            case 1: // when a is pressed
                if (pos <= lowEncoderCount + 50) { //50 is the tolerance
                        PIDTarget2(lowEncoderCount);

                    break;
                }
                break;
            case 2:
                if (pos <= medEncoderCount + 50) {
                        PIDTarget2(medEncoderCount);

                    break;
                }
                break;
            case 3:
                if (pos <= topEncoderCount + 50) {
                     PIDTarget2(topEncoderCount);
                    break;
                }
                break;
        }

    }

    public void total(double pos1){
        controller2Buttons();
        moveToState(pos1);
    }
    
    public void PIDTarget2(int target) {
        controller.setPID(p, i, d);
        int armPos = -LSMotor1.getCurrentPosition();
        double output = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = output;

        LSMotor1.setPower(-power/6);
        LSMotor2.setPower(-power/6);

    }

}
