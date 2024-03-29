package org.firstinspires.ftc.teamcode.PowerPlay.Manual;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.openPosition;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.BackLeftMotor;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.BackRightMotor;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.FrontLeftMotor;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.FrontRightMotor;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.Grabber;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor1;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.LSMotor2;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.initializeNvyusRobotHardware;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class TeleOpTheOldOne extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() {
        initializeNvyusRobotHardware(this);
        telemetry.addLine("ready");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   =  gamepad1.left_stick_x;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_y;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            FrontLeftMotor.setPower(leftFrontPower);
            FrontRightMotor.setPower(rightFrontPower);
            BackLeftMotor.setPower(leftBackPower);
            BackRightMotor.setPower(rightBackPower);


            //linear slide + servo code

            if (gamepad2.left_bumper) { //left go up
                setSlidesVelocity(LSMotor1, 0.8);
                setSlidesVelocity(LSMotor2, 0.8);
            } else if (gamepad2.right_bumper) { //tap to make linear slide stop during retract
                setSlidesVelocity(LSMotor1, -0.4);
                setSlidesVelocity(LSMotor2, -0.4);
            }
            else {
                LSMotor1.setZeroPowerBehavior(FLOAT);
                setSlidesVelocity(LSMotor1, 0);
                setSlidesVelocity(LSMotor2, 0);
            }

            if (gamepad2.a) { //close grabber
                setSlidesVelocity(LSMotor1, 0.1);
                setSlidesVelocity(LSMotor2, 0.1);
            }
            else if (gamepad2.b) { //open grabber
                Grabber.setPosition(openPosition);
            }
            telemetry.addLine("position: " + LSMotor1.getCurrentPosition());
            telemetry.update();


        }
    }
}
