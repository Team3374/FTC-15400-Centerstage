package com.example.meepmeeptesting;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //* middle paths
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(12.00, 34.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(12.00, 42.00))
                .splineTo(new Vector2d(60.00, 60.00), Math.toRadians(0.00))
                .build());

        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(-36.00, 34.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-36.00, 44.00))
                .splineTo(new Vector2d(3.5, 60.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(60.00, 60.00))
                .build());

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.50, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(12.00, -34.00, Math.toRadians(-90)))
                .lineTo(new Vector2d(12.00, -42.00))
                .splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .build());

        RoadRunnerBotEntity myBot4 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-36.00, -34.00, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(-36.00, -44.00))
                .splineTo(new Vector2d(3.50, -60.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(60.00, -60.00))
                .build());

        //* strafe paths
        RoadRunnerBotEntity myBot5 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.00, 63.50, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(24.00, 63.50))
                .build());

        RoadRunnerBotEntity myBot6 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(-48.00, 63.50))
                .build());

        RoadRunnerBotEntity myBot7 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.50, Math.toRadians(90.00)))
                .lineTo(new Vector2d(24.00, -63.50))
                .build());

        RoadRunnerBotEntity myBot8 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.50, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-48.00, -63.50))
                .build());

        //* far paths
        RoadRunnerBotEntity myBot9 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(14.00, 30.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .splineTo(new Vector2d(24.00, 48.00), Math.toRadians(45.00))
                .splineTo(new Vector2d(60.00, 60.00), Math.toRadians(0.00))
                .build());

        RoadRunnerBotEntity myBot10 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(-38.00, 30.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .lineToLinearHeading(new Pose2d(-33.00, 60.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(60.00, 60.00))
                .build());

        RoadRunnerBotEntity myBot11 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(14.00, -30.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .splineTo(new Vector2d(24.00, -48.00), Math.toRadians(-45.00))
                .splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .build());

        RoadRunnerBotEntity myBot12 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-38.00, -30.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .lineToLinearHeading(new Pose2d(-33.00, -60.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(60.00, -60.00))
                .build());


        //* close paths
        RoadRunnerBotEntity myBot13 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(12.00, 30.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .splineTo(new Vector2d(60.00, 60.00), Math.toRadians(0.00))
                .build());

        RoadRunnerBotEntity myBot14 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(-36.00, 30.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .splineTo(new Vector2d(-30.00, 60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(3.50, 60.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(60.00, 60.00))
                .build());

        RoadRunnerBotEntity myBot15 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(12.00, -30.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .splineTo(new Vector2d(60.00, -60.00), Math.toRadians(0.00))
                .build());

        RoadRunnerBotEntity myBot16 = new DefaultBotBuilder(meepMeep)
                .setDimensions(15.5,17)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-36.00, -30.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .splineTo(new Vector2d(-30.00, -60.00), Math.toRadians(0.00))
                .splineTo(new Vector2d(3.50, -60.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(60.00, -60.00))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .addEntity(myBot4)
                .addEntity(myBot5)
                .addEntity(myBot6)
                .addEntity(myBot7)
                .addEntity(myBot8)
                .addEntity(myBot9)
                .addEntity(myBot10)
                .addEntity(myBot11)
                .addEntity(myBot12)
                .addEntity(myBot13)
                .addEntity(myBot14)
                .addEntity(myBot15)
                .addEntity(myBot16)
                .start();
    }
}