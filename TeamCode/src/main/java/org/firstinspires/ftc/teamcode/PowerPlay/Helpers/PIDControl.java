package org.firstinspires.ftc.teamcode.PowerPlay.Helpers;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class PIDControl extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;


    @Override
    public void init() {
        //initializeNvyusRobotHardware(this);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = LSMotor1.getCurrentPosition();
        double output = controller.calculate(armPos, target);

        double power = output;

        LSMotor1.setPower(power);

        telemetry.addData("pos: ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();

    }
}
