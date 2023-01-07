package org.firstinspires.ftc.teamcode.PowerPlay.Auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.*;
//import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.Grabber;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor1;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor2;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class RoadrunnerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

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


        Trajectory traj1 = drive.trajectoryBuilder(startingPose) //creating trajectory
                .splineToConstantHeading(new Vector2d(-54, -16), 0)
                .forward(31.5)
                .splineToConstantHeading(new Vector2d(-9, -30), 0)
                .addDisplacementMarker(() -> { //run after spline
                    Grabber.setPosition(openPosition);
                })
                .build(); // makes the trajectory

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end()) //second trajectory with starting
                // pose as traj1's ending pose
                .lineToLinearHeading(new Pose2d(-6, -74, -Math.toRadians(90)))
//                .addTemporalMarker(1, () -> {
//                    setSlidesVelocity(LSMotor1,0);
//                    setSlidesVelocity(LSMotor2, 0);
//                })

                .build();


        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();

        Grabber.setPosition(closePosition);
        sleep(250);
        setSlidesVelocity(LSMotor1, 0.8);
        setSlidesVelocity(LSMotor2, 0.8);
        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);


    }
}
