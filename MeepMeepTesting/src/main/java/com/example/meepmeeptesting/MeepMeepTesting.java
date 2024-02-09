package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //* middle paths
        RoadRunnerBotEntity bcMiddle = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.00, 63.50, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(26.00, 48.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(26.00, 24.00))
                .lineTo(new Vector2d(51.00, 36.00))
                .build());


        RoadRunnerBotEntity bfMiddle = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(-49.00, 48.00), Math.toRadians(180.00))
                .lineTo(new Vector2d(-49.00, 24.00))
                .lineTo(new Vector2d(-49.00, 36.00))
                .lineTo(new Vector2d(0.00, 36.00))
                .lineToSplineHeading(new Pose2d(36.00, 36.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(51.00, 36.00))
                .build());

        RoadRunnerBotEntity rcMiddle = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.50, Math.toRadians(450.00)))
                .splineTo(new Vector2d(26.00, -48.00), Math.toRadians(360.00))
                .lineTo(new Vector2d(26.00, -24.00))
                .lineTo(new Vector2d(51.00, -36.00))
                .build());

        RoadRunnerBotEntity rfMiddle = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.50, Math.toRadians(450.00)))
                .splineTo(new Vector2d(-49.00, -48.00), Math.toRadians(180.00))
                .lineTo(new Vector2d(-49.00, -24.00))
                .lineTo(new Vector2d(-49.00, -36.00))
                .lineTo(new Vector2d(0.00, -36.00))
                .lineToSplineHeading(new Pose2d(36.00, -36.00, Math.toRadians(360.00)))
                .lineTo(new Vector2d(51.00, -36.00))
                .build());



        //* far paths
        RoadRunnerBotEntity bcFar = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                                .lineToSplineHeading(new Pose2d(24.00, 43.00, Math.toRadians(90.00)))
                                .lineToSplineHeading(new Pose2d(40.00, 43.00, Math.toRadians(0.00)))
                                .splineTo(new Vector2d(51.00, 42.00), Math.toRadians(0.00))
                                .build());


        RoadRunnerBotEntity bfFar = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(-48.00, 43.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-48.00, 46.00))
                .lineTo(new Vector2d(-34.00, 46.00))
                .lineTo(new Vector2d(-36.00, 24.00))
                .lineToSplineHeading(new Pose2d(-36.00, 12.00, Math.toRadians(0)))
                .lineTo(new Vector2d(12.00, 12.00))
                .splineTo(new Vector2d(51.00, 30.00), Math.toRadians(0.00))
                .build());

        RoadRunnerBotEntity rcFar = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(450.00)))
                                .lineToSplineHeading(new Pose2d(24.00, -43.00, Math.toRadians(270.00)))
                                .lineToSplineHeading(new Pose2d(40.00, -43.00, Math.toRadians(360.00)))
                                .splineTo(new Vector2d(51.00, -42.00), Math.toRadians(0.00))
                                .build());

        RoadRunnerBotEntity rfFar = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(450.00)))
                .lineToSplineHeading(new Pose2d(-48.00, -43.00, Math.toRadians(270.00)))
                .lineTo(new Vector2d(-48.00, -46.00))
                .lineTo(new Vector2d(-34.00, -46.00))
                .lineTo(new Vector2d(-36.00, -24.00))
                .lineToSplineHeading(new Pose2d(-36.00, -12.00, Math.toRadians(360.00)))
                .lineTo(new Vector2d(12.00, -12.00))
                .splineTo(new Vector2d(51.00, -30.00), Math.toRadians(360.00))
                .build());


        //* close paths
        RoadRunnerBotEntity bcClose = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                        .lineToSplineHeading(new Pose2d(9.00, 42.00, Math.toRadians(45.00)))
                        .lineToSplineHeading(new Pose2d(40.00, 30.00, Math.toRadians(0.00)))
                        .lineTo(new Vector2d(51.00, 30.00))
                        .build());




        RoadRunnerBotEntity bfClose = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(-33.00, 42.00, Math.toRadians(135.00)))
                .lineTo(new Vector2d(-48.00, 42.00))
                .lineToSplineHeading(new Pose2d(-36.00, 12.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(12.00, 12.00))
                .splineTo(new Vector2d(51.00, 42.00), Math.toRadians(0.00))
                .build());

        RoadRunnerBotEntity rcClose = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(450.00)))
                        .lineToSplineHeading(new Pose2d(9.00, -42.00, Math.toRadians(315.00)))
                        .lineToSplineHeading(new Pose2d(40.00, -30.00, Math.toRadians(360.00)))
                        .lineTo(new Vector2d(51.00, -30.00))
                        .build());



        RoadRunnerBotEntity rfClose = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(450.00)))
                .lineToSplineHeading(new Pose2d(-33.00, -42.00, Math.toRadians(225.00)))
                .lineTo(new Vector2d(-48.00, -42.00))
                .lineToSplineHeading(new Pose2d(-36.00, -12.00, Math.toRadians(360.00)))
                .lineTo(new Vector2d(12.00, -12.00))
                .splineTo(new Vector2d(51.00, -42.00), Math.toRadians(360.00))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bcMiddle)
                .addEntity(bfMiddle)
                .addEntity(rcMiddle)
                .addEntity(rfMiddle)
                .addEntity(bcClose)
                .addEntity(bfClose)
                .addEntity(rcClose)
                .addEntity(rfClose)
                .addEntity(bcFar)
                .addEntity(bfFar)
                .addEntity(rcFar)
                .addEntity(rfFar)
//                .addEntity(bcStrafe)
//                .addEntity(bfStrafe)
//                .addEntity(rcStrafe)
//                .addEntity(rfStrafe)
//                .addEntity(bluePixel)
//                .addEntity(redPixel)
//                .addEntity(blueStrafeLeft)
//                .addEntity(blueStrafeRight)
//                .addEntity(redStrafeRight)
//                .addEntity(redStrafeLeft)
                .start();
    }
}