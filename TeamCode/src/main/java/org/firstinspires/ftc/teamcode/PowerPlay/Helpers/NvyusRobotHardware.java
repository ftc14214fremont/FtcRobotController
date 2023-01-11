package org.firstinspires.ftc.teamcode.PowerPlay.Helpers;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.List;
import com.qualcomm.robotcore.hardware.Servo;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;


public class NvyusRobotHardware {
    public static DcMotorEx FrontLeftMotor;
    public static DcMotorEx FrontRightMotor;
    public static DcMotorEx BackLeftMotor;
    public static DcMotorEx BackRightMotor;
    public static DcMotorEx LSMotor1;
    public static DcMotorEx LSMotor2;
    public static Servo Grabber;


    public static void initializeNvyusRobotHardware(LinearOpMode opMode) {
        //altering some settings on rev hub, supposedly makes encoders update quicker
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //initialize motor hardware map
        FrontLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, "22");
        FrontRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "31");
        BackLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, "23");
        BackRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "30");

        LSMotor1 = opMode.hardwareMap.get(DcMotorEx.class, "20");
        LSMotor2 = opMode.hardwareMap.get(DcMotorEx.class, "33");
        //GrabberServo = opMode.hardwareMap.get(Servo.class, "grabber");
        Grabber = opMode.hardwareMap.get(Servo.class, "servo33");


        //set zero power behavior
        FrontLeftMotor.setZeroPowerBehavior(BRAKE);
        FrontRightMotor.setZeroPowerBehavior(BRAKE);
        BackLeftMotor.setZeroPowerBehavior(BRAKE);
        BackRightMotor.setZeroPowerBehavior(BRAKE);


        //set motor direction to move forward
        FrontLeftMotor.setDirection(REVERSE);
        FrontRightMotor.setDirection(FORWARD);
        BackLeftMotor.setDirection(REVERSE);
        BackRightMotor.setDirection(FORWARD);

        //set linear slide settings
        LSMotor1.setDirection(REVERSE); //for reverse, positive power is up
//        LSMotor1.setZeroPowerBehavior(BRAKE);
//        LSMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    }

    public static void initializeNvyusRobotCamera(LinearOpMode opMode) {

        OpenCvCamera camera;
        NvyusAprilTagPipeline aprilTagDetectionPipeline;

        double fx = 578.272*1.6;
        double fy = 578.272*1.6;
        double cx = 402.145*1.6;
        double cy = 221.506*1.6;

        // UNITS ARE METERS
        double tagsize = 0.166;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        // creating an internal camera instance, set direction, and enter ID for live view
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        aprilTagDetectionPipeline = new NvyusAprilTagPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }
}
