package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.MovementFunctions.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class DropConeTest extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        LinearSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LinearSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        double currentPosition = LinearSlideMotor.getCurrentPosition();

        Grabber.setPosition(0.25);
        sleep(1000);
        while (opModeIsActive() && currentPosition < 770) {

            setSlidesVelocity(LinearSlideMotor, .8);
            telemetry.addLine("pos: " + currentPosition);
            telemetry.update();
            currentPosition = LinearSlideMotor.getCurrentPosition();
        }
        moveForward(0.3,500,this);
        Grabber.setPosition(0.6);
        while (opModeIsActive() && currentPosition > 450) {
            setSlidesVelocity(LinearSlideMotor, 0);
            telemetry.addLine("pos: " + currentPosition);
            telemetry.update();
            currentPosition = LinearSlideMotor.getCurrentPosition();
        }
        setSlidesVelocity(LinearSlideMotor,.1);
        sleep(500);
        setSlidesVelocity(LinearSlideMotor,0);

    }

}
