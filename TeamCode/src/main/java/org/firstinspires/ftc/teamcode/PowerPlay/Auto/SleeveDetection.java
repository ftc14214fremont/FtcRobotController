package org.firstinspires.ftc.teamcode.PowerPlay.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.camera.WebcamExample;
import org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusAprilTagPipeline;
import org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

// this is the basic code to initialize a camera
// the actual image processing is in NvyusPipeline.java in Helpers

@TeleOp
public class SleeveDetection extends LinearOpMode {

    // variables
    OpenCvInternalCamera phoneCam;
    NvyusAprilTagPipeline pipeline;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    @Override
        public void runOpMode() {

        // For the live camera view on the Robot Controller phone
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // creating an internal camera instance, set direction, and enter ID for live view
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        // attaching a pipeline (the OpenCV processing)
        pipeline = new NvyusAprilTagPipeline(tagsize, fx, fy, cx, cy);
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        // Opening connection to camera device Asynchronously
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // start streaming
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }

        });
        waitForStart();

    }
}