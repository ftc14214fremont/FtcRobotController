package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Autonomous
public class MoveMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor FrontLeftMotor = null;
        DcMotor FrontRightMotor = null;
        DcMotor BackLeftMotor = null;
        DcMotor BackRightMotor = null;
        //test
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "30");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "22");
        BackLeftMotor = hardwareMap.get(DcMotor.class, "23");
        BackRightMotor = hardwareMap.get(DcMotor.class, "31");




    }
    /*public void moveForward(double power, double time){
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft    Motor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        FrontLeftMotor.setPower(power);
        FrontRightMotor.setPower(power);
        BackLeftMotor.setPower(power);
        BackRightMotor.setPower(power);
        sleep(time);
        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        BackLeftMotor.setPower(0);
        BackRightMotor.setPower(0);

     */

}
