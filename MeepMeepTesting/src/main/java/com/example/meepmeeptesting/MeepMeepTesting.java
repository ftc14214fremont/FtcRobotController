package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double DropConeTime = .15;

    // coordinates
    public static double junctionX = 6.1;
    public static double junctionY = -30;
    public static double coneX = -8;
    public static double coneY = -64.0;
    public static double backDistance = 5;

    // new junction coords are (-13.1, -30)
    public static double Park0x = -11;
    public static double Park0y = -15;
    public static double Park1x = -11;
    public static double Park1y = -45;
    public static double Park2x = -9;
    public static double Park2y = -65;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -36, 0))
                                .splineToConstantHeading(new Vector2d(-54, -56), 0)
                                .forward(31.5)
                                .splineToConstantHeading(new Vector2d(junctionX, junctionY), 0)
                                .waitSeconds(DropConeTime)
                                .back(backDistance)
                                .build() // makes the trajectory
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}