package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class EncoderLiftArm extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //LinearSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER); // - makes motor unable to run in auto

        waitForStart();

        double currentPosition = LinearSlideMotor.getCurrentPosition();

        while (opModeIsActive() && currentPosition < 800) {

            LinearSlideMotor.setPower(.8);
            telemetry.addLine("pos: " + currentPosition);
            telemetry.update();
            sleep(500);
            currentPosition = LinearSlideMotor.getCurrentPosition();
        }
        LinearSlideMotor.setPower(0);
        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sleep(500);

        while (opModeIsActive() && currentPosition > 50) {

            LinearSlideMotor.setPower(-.8);
            telemetry.addLine("pos: " + currentPosition);
            telemetry.update();
            currentPosition = LinearSlideMotor.getCurrentPosition();
        }

        LinearSlideMotor.setPower(0);
        LinearSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

}
