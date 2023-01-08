package org.firstinspires.ftc.teamcode.PowerPlay.Helpers;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class PIDControl extends OpMode {
    private PIDController controller;

    public static double p = 0.03, i = 0, d = 0.00015;
    public static double f = 0.05;

    public static int target = 200;

    private final double ticks_in_degrees = 360/145.1;


    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        LSMotor1 = hardwareMap.get(DcMotorEx.class, "20");
        LSMotor2 = hardwareMap.get(DcMotorEx.class, "33");

        LSMotor1.setDirection(REVERSE);

    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = LSMotor1.getCurrentPosition();
        double output = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degrees)) * f;

        double power = output;

        LSMotor1.setPower(power);

        telemetry.addData("pos: ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();

    }
}
