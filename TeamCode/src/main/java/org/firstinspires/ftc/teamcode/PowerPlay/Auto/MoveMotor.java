package org.firstinspires.ftc.teamcode.PowerPlay.Auto;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.MovementFunctions.moveBackward;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.MovementFunctions.moveForward;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.MovementFunctions.strafeLeft;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.MovementFunctions.strafeRight;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.PowerPlay.Helpers.MovementFunctions.*;



@Autonomous
public class MoveMotor extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        moveForward(0.5,1000, this);
        strafeLeft(0.5,1000,this);
        strafeRight(0.5,1000,this);
        moveBackward(0.5,1000,this);






    }
    /*
    public void moveForward(double power, int time) {

        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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
    }
    */

}