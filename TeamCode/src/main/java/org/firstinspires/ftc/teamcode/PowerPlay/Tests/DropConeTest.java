package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.MovementFunctions.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class DropConeTest extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        LSMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        LSMotor1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        double currentPosition = LSMotor1.getCurrentPosition();

        Grabber.setPosition(0.25);
        sleep(1000);
        while (opModeIsActive() && currentPosition < 770) {

            setSlidesVelocity(LSMotor1, .8);
            telemetry.addLine("pos: " + currentPosition);
            telemetry.update();
            currentPosition = LSMotor1.getCurrentPosition();
        }
        moveForward(0.3,500,this);
        Grabber.setPosition(0.6);
        while (opModeIsActive() && currentPosition > 450) {
            setSlidesVelocity(LSMotor1, 0);
            telemetry.addLine("pos: " + currentPosition);
            telemetry.update();
            currentPosition = LSMotor1.getCurrentPosition();
        }
        setSlidesVelocity(LSMotor1,.1);
        sleep(500);
        setSlidesVelocity(LSMotor1,0);

    }

}
