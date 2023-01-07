package org.firstinspires.ftc.teamcode.PowerPlay.Helpers;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmFunctions {

    public static void liftArmUp(double power, int time, LinearOpMode opMode) {
        LSMotor1.setDirection(REVERSE);

        //set power to motor
        LSMotor1.setPower(power);

        opMode.sleep(time);

        LSMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSMotor1.setPower(0);

        opMode.sleep(100);

    }

    public static void liftArmDown(double power, int time, LinearOpMode opMode) {
        LSMotor1.setDirection(FORWARD);

        //set power to motor
        LSMotor1.setPower(power);

        opMode.sleep(time);

        LSMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LSMotor1.setPower(0);

        opMode.sleep(100);

    }
}
