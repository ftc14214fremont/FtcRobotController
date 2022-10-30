package org.firstinspires.ftc.teamcode.PowerPlay.Helpers;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LinearSlideMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ArmFunctions {

    public static void liftArmUp(double power, int time, LinearOpMode opMode) {
        LinearSlideMotor.setDirection(REVERSE);

        //set power to motor
        LinearSlideMotor.setPower(power);

        opMode.sleep(time);

        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlideMotor.setPower(0);

        opMode.sleep(100);

    }

    public static void liftArmDown(double power, int time, LinearOpMode opMode) {
        LinearSlideMotor.setDirection(FORWARD);

        //set power to motor
        LinearSlideMotor.setPower(power);

        opMode.sleep(time);

        LinearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearSlideMotor.setPower(0);

        opMode.sleep(100);

    }
}
