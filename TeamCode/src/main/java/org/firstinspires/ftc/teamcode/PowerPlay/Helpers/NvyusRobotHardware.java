package org.firstinspires.ftc.teamcode.PowerPlay.Helpers;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;





import java.util.List;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

public class NvyusRobotHardware {
    public static DcMotorEx FrontLeftMotor;
    public static DcMotorEx FrontRightMotor;
    public static DcMotorEx BackLeftMotor;
    public static DcMotorEx BackRightMotor;

    public static void initializeNvyusRobotHardware(LinearOpMode opMode) {
        //altering some settings on rev hub, supposedly makes encoders update quicker
        List<LynxModule> allHubs = opMode.hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //initialize motor hardware map
        FrontLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, "30");
        FrontRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "22");
        BackLeftMotor = opMode.hardwareMap.get(DcMotorEx.class, "23");
        BackRightMotor = opMode.hardwareMap.get(DcMotorEx.class, "31");


        //set zero power behavior
        FrontLeftMotor.setZeroPowerBehavior(BRAKE);
        FrontRightMotor.setZeroPowerBehavior(BRAKE);
        BackLeftMotor.setZeroPowerBehavior(BRAKE);
        BackRightMotor.setZeroPowerBehavior(BRAKE);


        //set motor direction to move forward
        FrontLeftMotor.setDirection(REVERSE);
        FrontRightMotor.setDirection(REVERSE);
        BackLeftMotor.setDirection(FORWARD);
        BackRightMotor.setDirection(FORWARD);



    }

}
