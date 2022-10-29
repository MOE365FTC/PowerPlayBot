package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class LeftAutonMM {
    public static void main(String[] args) {
        int a = 22;
        int strafeDistance = 29;
        com.noahbres.meepmeep.MeepMeep meepMeep = new com.noahbres.meepmeep.MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep).setDimensions(12, 12.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 60, Math.toRadians(270)))
                        .forward(47)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeLeft(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeRight(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeLeft(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeRight(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeLeft(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeRight(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeLeft(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeRight(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeLeft(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .strafeRight(strafeDistance)
                        .addDisplacementMarker(() -> {
                        })
                        .build());

        meepMeep.setBackground(com.noahbres.meepmeep.MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}