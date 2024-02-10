package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.Storage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Park")
public class Park extends LinearOpMode {

    private String selectedAuto = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot drive = new Robot(hardwareMap);

        TrajectorySequence bfPark = drive.trajectorySequenceBuilder(new Pose2d(-36, 63.50, Math.toRadians(-90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, 38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, 38.41))
                .lineToSplineHeading(new Pose2d(48.79, 37.84, Math.toRadians(180.00)))
                .build();

        TrajectorySequence bcPark = drive.trajectorySequenceBuilder(new Pose2d(12, 63.50, Math.toRadians(270.00)))
                .splineTo(new Vector2d(29.54, 57.66), Math.toRadians(370.79))
                .splineTo(new Vector2d(59.17, 60.49), Math.toRadians(360.00))
                .build();


        TrajectorySequence rfPark = drive.trajectorySequenceBuilder(new Pose2d(-36, -63.50, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, -38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, -38.41))
                .lineToSplineHeading(new Pose2d(48.79, -37.84, Math.toRadians(180.00)))
                .build();

        TrajectorySequence rcPark = drive.trajectorySequenceBuilder(new Pose2d(12, -63.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(29.54, -57.66), Math.toRadians(-10.79))
                .splineTo(new Vector2d(59.17, -60.49), Math.toRadians(0.00))
                .build();

        telemetry.addLine("Please choose from the following Autonomous programs:");
        telemetry.addData("A", "Blue Park (Close)");
        telemetry.addData("B", "Blue Park (Far)");
        telemetry.addData("X", "Red Park (Close)");
        telemetry.addData("Y", "Red Park (Far)");
        telemetry.update();

        while(opModeInInit()) {
            if (gamepad1.a) {
                selectedAuto = "bcp";

                drive.setPoseEstimate(bcPark.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Blue Park (Close)");
                telemetry.update();
            } else if (gamepad1.b) {
                selectedAuto = "bfp";

                drive.setPoseEstimate(bfPark.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Blue Park (Far)");
                telemetry.update();
            } else if (gamepad1.x) {
                selectedAuto = "rcp";

                drive.setPoseEstimate(rcPark.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Red Park (Close)");
                telemetry.update();
            } else if (gamepad1.y) {
                selectedAuto = "rfp";

                drive.setPoseEstimate(rfPark.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Red Park (Far)");
                telemetry.update();
            }
        }

        waitForStart();

        switch (selectedAuto) {
            case "bcp":
                drive.followTrajectorySequence(bcPark);
                Storage.currentColor = "blue";
                break;
            case "bfp":
                drive.followTrajectorySequence(bfPark);
                Storage.currentColor = "blue";
                break;
            case "rcp":
                drive.followTrajectorySequence(rcPark);
                Storage.currentColor = "red";
                break;
            case "rfp":
                drive.followTrajectorySequence(rfPark);
                Storage.currentColor = "blue";
                break;
        }

        Storage.currentPose = drive.getPoseEstimate();
        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.clearAll();
        telemetry.addData("Current Auto", selectedAuto);
        telemetry.addLine();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

    }
}