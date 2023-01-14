package org.firstinspires.ftc.teamcode.PowerPlay.Manual;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.Constants.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.SetVelocity.setSlidesVelocity;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class TeleOpTheOldOnev2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {
        initializeNvyusRobotHardware(this);

        LSMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double currentPosition = LSMotor1.getCurrentPosition();

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
            telemetry.update();
            FrontLeftMotor.setPower(leftFrontPower);
            FrontRightMotor.setPower(rightFrontPower);
            BackLeftMotor.setPower(leftBackPower);
            BackRightMotor.setPower(rightBackPower);


            //linear slide + servo code

            if (gamepad2.left_bumper) { //left go up
                if (currentPosition < 300) {
                    setSlidesVelocity(LSMotor1, 0.3);
                    setSlidesVelocity(LSMotor2, 0.3);
                }
                else if (currentPosition < 700) {
                    setSlidesVelocity(LSMotor1, 0.7);
                    setSlidesVelocity(LSMotor2, 0.7);
                }
                else {
                    setSlidesVelocity(LSMotor1, 0.9);
                    setSlidesVelocity(LSMotor2, 0.9 );
                }
            } else if (gamepad2.right_bumper) { //tap to make linear slide stop during retract
                setSlidesVelocity(LSMotor1, -0.1);
                setSlidesVelocity(LSMotor2, -0.1);
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

            currentPosition = LSMotor1.getCurrentPosition();
            telemetry.addLine("position: " + LSMotor1.getCurrentPosition());
            telemetry.update();


        }
    }
}
