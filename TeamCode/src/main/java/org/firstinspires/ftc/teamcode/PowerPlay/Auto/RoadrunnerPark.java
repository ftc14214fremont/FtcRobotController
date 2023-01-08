package org.firstinspires.ftc.teamcode.PowerPlay.Auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.*;
//import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusAprilTagPipeline.getSleevePosition;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.Grabber;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor1;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor2;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.initializeNvyusRobotCamera;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous
public class RoadrunnerPark extends LinearOpMode {

    public static double TopConeTime = 0.9;
    public static double incrementPerConeLevel = 0.1;

    // coordinates
    public static double junctionX = -8;
    public static double junctionY = -27;
    public static double coneX = -4;
    public static double coneY = -63;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeNvyusRobotCamera(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //call a new Mecanum drive
        // and get the hardware Map

        // arm HW map
        LSMotor1 = hardwareMap.get(DcMotorEx.class, "20");
        LSMotor2 = hardwareMap.get(DcMotorEx.class, "33");
        Grabber = hardwareMap.get(Servo.class, "servo35");

        LSMotor1.setDirection(REVERSE);


        // define starting position (Pose)
        Pose2d startingPose = new Pose2d(-60, -36, 0);

        // necessary to make sure encoder localization matches with starting Pose
        drive.setPoseEstimate(startingPose);


        Trajectory Park0 = drive.trajectoryBuilder(startingPose)
                .splineToConstantHeading(new Vector2d(-54, -14), 0)
                .forward(31.5)
                .build();

        Trajectory Park1 = drive.trajectoryBuilder(startingPose)
                .splineToConstantHeading(new Vector2d(-54, -40), 0)
                .forward(31.5)
                .build();

//        Trajectory Park2 = drive.trajectoryBuilder(startingPose)




        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();

        if (getSleevePosition() == 0) {
            telemetry.addData("Sleeve Position", "0");
            telemetry.update();
            drive.followTrajectory(Park0);

        }
        if (getSleevePosition() == 1) {
            telemetry.addData("Sleeve Position", "1");
            telemetry.update();
            drive.followTrajectory(Park1);

        }
        if (getSleevePosition() == 2) {
            telemetry.addData("Sleeve Position", "2");
            telemetry.update();
            drive.followTrajectory(Park1);
//            drive.followTrajectory(Park2);

        }
    }
}
