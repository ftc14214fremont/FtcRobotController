package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.Grabber;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor1;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.initializeNvyusRobotHardware;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class LinearSlidesTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        LSMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LSMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y < -0.3) { //stick up go up
                setSlidesVelocity(LSMotor1, 0.5);
            } else if (gamepad1.left_stick_y > 0.3) { //stick down then do nothing except prevent
                // motor from overturning
                    setSlidesVelocity(LSMotor1, 0.1);
            }
            else {
                LSMotor1.setZeroPowerBehavior(FLOAT);
                setSlidesVelocity(LSMotor1, 0);
            }

            if (gamepad1.a) { //open
                Grabber.setPosition(0.25);
            }
            else if (gamepad1.b) { //open
                Grabber.setPosition(0.6);
            }
            telemetry.addLine("position: " + LSMotor1.getCurrentPosition());
            telemetry.update();
        }
    }

}

