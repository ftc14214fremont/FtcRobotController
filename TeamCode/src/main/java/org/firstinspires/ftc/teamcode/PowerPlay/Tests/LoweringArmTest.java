package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class LoweringArmTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        // code to lift motor up to right position while moving
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        LSMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LSMotor1.setTargetPosition(700);
        setSlidesVelocity(LSMotor1,0.8);

        LSMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(2000);

        LSMotor1.setTargetPosition(0);
        setSlidesVelocity(LSMotor1,-0.2);

        LSMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
