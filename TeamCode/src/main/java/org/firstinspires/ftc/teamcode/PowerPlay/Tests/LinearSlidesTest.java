package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmUp;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmDown;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.Grabber;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LinearSlideMotor;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.initializeNvyusRobotHardware;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class LinearSlidesTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        LinearSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y < -0.3) { //stick up go up
                setSlidesVelocity(LinearSlideMotor, 0.8);
            } else if (gamepad1.left_stick_y > 0.3) { //stick down then do nothing except prevent
                // motor from overturning
                    setSlidesVelocity(LinearSlideMotor, 0.1);
            }
            else {
                LinearSlideMotor.setZeroPowerBehavior(FLOAT);
                setSlidesVelocity(LinearSlideMotor, 0);
            }

            if (gamepad1.a) { //open
                Grabber.setPosition(0.25);
            }
            else if (gamepad1.b) { //open
                Grabber.setPosition(0.6);
            }
            telemetry.addLine("position: " + LinearSlideMotor.getCurrentPosition());
            telemetry.update();
        }
    }

}

