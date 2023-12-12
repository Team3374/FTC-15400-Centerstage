package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.52, 63.50, Math.toRadians(-90.00)))
                                .splineToSplineHeading(new Pose2d(-29.72, 38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .lineTo(new Vector2d(22.36, 38.41))
                                .lineToSplineHeading(new Pose2d(48.79, 37.84, Math.toRadians(180.00)))
                                .build()

                );

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.98, 63.50, Math.toRadians(270.00)))
                                .splineTo(new Vector2d(29.54, 57.66), Math.toRadians(370.79))
                                .splineTo(new Vector2d(59.17, 60.49), Math.toRadians(360.00))
                                .build()

                );

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.52, -63.50, Math.toRadians(90.00)))
                                .splineToSplineHeading(new Pose2d(-29.72, -38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                                .lineTo(new Vector2d(22.36, -38.41))
                                .lineToSplineHeading(new Pose2d(48.79, -37.84, Math.toRadians(180.00)))
                                .build()

                );

        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(11.98, -63.50, Math.toRadians(90.00)))
                                .splineTo(new Vector2d(29.54, -57.66), Math.toRadians(-10.79))
                                .splineTo(new Vector2d(59.17, -60.49), Math.toRadians(0.00))
                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .addEntity(myBot4)
                .start();
    }
}