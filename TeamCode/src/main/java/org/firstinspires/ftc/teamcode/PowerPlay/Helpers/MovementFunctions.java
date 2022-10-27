package org.firstinspires.ftc.teamcode.PowerPlay.Helpers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.BackLeftMotor;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.BackRightMotor;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.FrontLeftMotor;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.FrontRightMotor;



import org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;

public class MovementFunctions {

    public static void moveForward(double power, int time, LinearOpMode opMode) {
        //set motor direction
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //set power to motor
        FrontLeftMotor.setPower(power);
        FrontRightMotor.setPower(power);
        BackLeftMotor.setPower(power);
        BackRightMotor.setPower(power);

        //time
        opMode.sleep(time);

        //stop motor
        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);

        //don't want to overload code
        opMode.sleep(100);
    }

    public static void moveBackward(double power, int time, LinearOpMode opMode) {
        //set motor direction
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //set power to motor
        FrontLeftMotor.setPower(power);
        FrontRightMotor.setPower(power);
        BackLeftMotor.setPower(power);
        BackRightMotor.setPower(power);

        //time
        opMode.sleep(time);

        //stop motor
        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);

        //don't want to overload code
        opMode.sleep(100);
    }

    public static void strafeRight(double power, int time, LinearOpMode opMode) {
        //set motor direction
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //set power to motor
        FrontLeftMotor.setPower(power);
        FrontRightMotor.setPower(power);
        BackLeftMotor.setPower(power);
        BackRightMotor.setPower(power);

        //time
        opMode.sleep(time);

        //stop motor
        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);

        //don't want to overload code
        opMode.sleep(100);
    }

    public static void strafeLeft(double power, int time, LinearOpMode opMode) {
        //set motor direction
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //set power to motor
        FrontLeftMotor.setPower(power);
        FrontRightMotor.setPower(power);
        BackLeftMotor.setPower(power);
        BackRightMotor.setPower(power);

        //time
        opMode.sleep(time);

        //stop motor
        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);

        //don't want to overload code
        opMode.sleep(100);
    }
}
