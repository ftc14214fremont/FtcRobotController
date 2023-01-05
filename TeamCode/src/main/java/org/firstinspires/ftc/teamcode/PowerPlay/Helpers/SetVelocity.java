package org.firstinspires.ftc.teamcode.PowerPlay.Helpers;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class SetVelocity {
    public static void setSlidesVelocity(DcMotorEx motor, double velocity) {
        motor.setVelocity(velocity * 2781.08333);
    }
}
