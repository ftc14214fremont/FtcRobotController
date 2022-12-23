package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmUp;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmDown;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LinearSlideMotor;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.initializeNvyusRobotHardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class LinearSlidesTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        LinearSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y < -0.3) { //stick up go up
                LinearSlideMotor.setPower(1);
            }
            else if (gamepad1.left_stick_y > 0.3) { //stick down go down
                LinearSlideMotor.setPower(-1);
            }
            else {
                // do nothing
                LinearSlideMotor.setZeroPowerBehavior(BRAKE);
                LinearSlideMotor.setVelocity(0);
            }

            telemetry.addLine("position: " + LinearSlideMotor.getCurrentPosition());
            telemetry.update();
            }
        }

    }

