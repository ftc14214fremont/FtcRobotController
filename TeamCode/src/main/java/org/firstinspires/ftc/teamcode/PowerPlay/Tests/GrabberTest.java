package org.firstinspires.ftc.teamcode.PowerPlay.Tests;

import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmUp;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.ArmFunctions.liftArmDown;
import static org.firstinspires.ftc.teamcode.PowerPlay.Helpers.NvyusRobotHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class GrabberTest extends LinearOpMode {

    public static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    public static final int CYCLE_MS = 50;     // period of each cycle
    public static final double MAX_POS = 1.0;     // Maximum rotational position
    public static final double MIN_POS = 0.0;     // Minimum rotational position

    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;

    @Override

    public void runOpMode() throws InterruptedException {
        initializeNvyusRobotHardware(this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        while (opModeIsActive()) {

            // slew the servo, according to the rampUp (direction) variable.
            if (rampUp && gamepad1.left_stick_y < -0.3) {
                // Keep stepping up until we hit the max value.
                position += INCREMENT;
                if (position >= MAX_POS) {
                    position = MAX_POS;
                    rampUp = !rampUp;   // Switch ramp direction
                }
            } else if (gamepad1.left_stick_y > 0.3) {
                // Keep stepping down until we hit the min value.
                position -= INCREMENT;
                if (position <= MIN_POS) {
                    position = MIN_POS;
                    rampUp = !rampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("Servo Position", "%5.2f", position);
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;
            Grabber.setPosition(position);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();




    }
}
