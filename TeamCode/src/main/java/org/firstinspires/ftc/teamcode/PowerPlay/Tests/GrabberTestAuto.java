package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmUp;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmDown;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.initializeNvyusRobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmUp;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmDown;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.initializeNvyusRobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class GrabberTestAuto extends LinearOpMode {
    double position =0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        position=1;
        Grabber.setPosition(position);
        sleep(500);
        position = 0;
        Grabber.setPosition(position);


    }
}
