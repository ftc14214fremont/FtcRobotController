package org.firstinspires.ftc.teamcode.PowerPlay.Auto;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.*;
//import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusAprilTagPipeline.getSleevePosition;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.Grabber;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor1;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor2;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;

import org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusAprilTagPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

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
public class FinalAutoRedF2andBlueA5 extends LinearOpMode {

    public static double TopConeTime = .95;
    public static double incrementPerConeLevel = 0.1;
    public static double DropConeTime = .15;

    // coordinates
    public static double junctionX = 0;
    public static double junctionY = -37;
    public static double coneX = -8;
    public static double coneY = -64.0;
    public static double backDistance = 5;

    // new junction coords are (-13.1, -30)
    public static double Park0x = 11;
    public static double Park0y = -15;
    public static double Park1x = 11;
    public static double Park1y = -42;
    public static double Park2x = 9;
    public static double Park2y = -65;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap); //call a new Mecanum drive
        // and get the hardware Map

        // arm HW map
        LSMotor1 = hardwareMap.get(DcMotorEx.class, "20");
        LSMotor2 = hardwareMap.get(DcMotorEx.class, "33");
        Grabber = hardwareMap.get(Servo.class, "servo34");

        LSMotor1.setDirection(REVERSE);


        // define starting position (Pose)
        Pose2d startingPose = new Pose2d(60, -36, Math.toRadians(180));

        // necessary to make sure encoder localization matches with starting Pose
        drive.setPoseEstimate(startingPose);

        // robot drives to junction with cone and drops
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(startingPose)
                .lineToConstantHeading(new Vector2d(58, -16))
                .forward(35.5)
                .splineToConstantHeading(new Vector2d(junctionX, junctionY), 0)
                .addTemporalMarker(() -> { //run after spline
                    setSlidesVelocity(LSMotor1, -0.4);
                    setSlidesVelocity(LSMotor2, 0);
                })
                .waitSeconds(DropConeTime)
                .addTemporalMarker(() -> {
                    Grabber.setPosition(openPosition);
                    setSlidesVelocity(LSMotor1, 0.8);
                    setSlidesVelocity(LSMotor2, 0.8);
                })
                .back(backDistance)
                .build(); // makes the trajectory

        Trajectory Park0 = drive.trajectoryBuilder(trajectory.end())
                .lineToConstantHeading(new Vector2d(Park0x, Park0y))
//                .addDisplacementMarker( () -> {
//                    setSlidesVelocity(LSMotor1, 0);
//                    setSlidesVelocity(LSMotor2, 0);
//                })
                .build();

        Trajectory Park1 = drive.trajectoryBuilder(trajectory.end())
                .lineToConstantHeading(new Vector2d(Park1x, Park1y))
//                .addDisplacementMarker( () -> {
//                    setSlidesVelocity(LSMotor1, 0);
//                    setSlidesVelocity(LSMotor2, 0);
//                })
                .build();

        Trajectory Park2 = drive.trajectoryBuilder(trajectory.end())
                .lineToConstantHeading(new Vector2d(Park2x, Park2y))
//                .addDisplacementMarker( () -> {
//                    setSlidesVelocity(LSMotor1, 0);
//                    setSlidesVelocity(LSMotor2, 0);
//                })
                .build();

        OpenCvCamera camera;
        NvyusAprilTagPipeline aprilTagDetectionPipeline;

        double fx = 578.272 * 1.6;
        double fy = 578.272 * 1.6;
        double cx = 402.145 * 1.6;
        double cy = 221.506 * 1.6;

        // UNITS ARE METERS
        double tagsize = 0.166;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // creating an internal camera instance, set direction, and enter ID for live view
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        aprilTagDetectionPipeline = new NvyusAprilTagPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });


        telemetry.addData("Status", "Initialized");

        telemetry.update();

        waitForStart();

        camera.closeCameraDevice();

        Grabber.setPosition(closePosition);
        sleep(250);
        setSlidesVelocity(LSMotor1, 0.8);
        setSlidesVelocity(LSMotor2, 0.8);
        drive.followTrajectorySequence(trajectory);
//        drive.followTrajectory(DropCone1);
//        drive.followTrajectory(GrabCone2);
//        drive.followTrajectory(DropCone2);
//        drive.followTrajectory(GrabCone3);
//        drive.followTrajectory(DropCone3);
//
        if (getSleevePosition() == 0) {
            telemetry.addData("Sleeve Position", "0");
            telemetry.update();
            drive.followTrajectory(Park2);

        }
        if (getSleevePosition() == 1) {
            telemetry.addData("Sleeve Position", "1");
            telemetry.update();
            drive.followTrajectory(Park1);

        }
        if (getSleevePosition() == 2) {
            telemetry.addData("Sleeve Position", "2");
            telemetry.update();
            drive.followTrajectory(Park0);

        }
    }
}
