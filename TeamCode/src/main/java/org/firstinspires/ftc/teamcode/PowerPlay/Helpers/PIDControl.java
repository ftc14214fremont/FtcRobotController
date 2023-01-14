package org.firstinspires.ftc.teamcode.PowerPlay.Helpers;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDControl extends LinearOpMode {
    private PIDController controller;

    public static double p = 0.03, i = 0, d = 0.00015;
//    public static double f = 0.05;

    public static int target = 200;

//    private final double ticks_in_degrees = 360 / 145.1;


    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LSMotor1 = hardwareMap.get(DcMotorEx.class, "20");
        LSMotor2 = hardwareMap.get(DcMotorEx.class, "33");

        LSMotor1.setDirection(FORWARD);
//        LSMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LSMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            controller.setPID(p, i, d);
            int armPos = -LSMotor1.getCurrentPosition();
            double output = controller.calculate(armPos, target);
//            double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

            double power = output;

            LSMotor1.setPower(power/2);

            telemetry.addData("pos: ", armPos);
            telemetry.addData("target ", target);
            telemetry.update();
        }

    }

    public static void PIDTarget(int target, LinearOpMode opMode) {

        PIDController controller = new PIDController(p, i, d);
        MultipleTelemetry telemetry = new MultipleTelemetry(opMode.telemetry, FtcDashboard.getInstance().getTelemetry());

//        double ticks_in_degrees = 360 / 145.1;

        controller.setPID(p, i, d);
        int armPos = LSMotor1.getCurrentPosition();
        double output = controller.calculate(armPos, target);
//        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = output;

        LSMotor1.setPower(power/2);

        telemetry.addData("pos: ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();

    }

}
