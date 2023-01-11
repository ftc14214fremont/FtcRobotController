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

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.closePosition;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.groundEncoderCount;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.lowEncoderCount;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.medEncoderCount;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.openPosition;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.topEncoderCount;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.PIDControl.*;

import org.firstinspires.ftc.teamcode.PowerPlay.Helpers.PIDControl.*;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class TeleOpFSM extends LinearOpMode {

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
            // This ensures that the robot maintains the desired motion.
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

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "Prev Time: " + String.valueOf(prevTime));
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

            total();

            if (gamepad1.a) { //close grabber
                Grabber.setPosition(closePosition);
            } else if (gamepad1.b) { //open grabber
                Grabber.setPosition(openPosition);
            }

        }
    }
    public void controller2Buttons () {
        if (gamepad2.a){
            DEPOSIT_STATES = 0;
        }
        if (gamepad2.x){
            DEPOSIT_STATES = 1;
        }
        if (gamepad2.b){
            DEPOSIT_STATES = 2;
        }
        if (gamepad2.y){
            DEPOSIT_STATES = 3;
        }
    }

    public void moveToState(){
        switch (DEPOSIT_STATES){
            case 0:
                PIDTarget(groundEncoderCount, this);
                break;
            case 1:
                PIDTarget(lowEncoderCount, this);
                break;
            case 2:
                PIDTarget(medEncoderCount, this);
                break;
            case 3:
                PIDTarget(topEncoderCount, this);
                break;
        }
    }

    public void total(){
        controller2Buttons();
        moveToState();
    }


}
