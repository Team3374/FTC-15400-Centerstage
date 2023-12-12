package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Park")
public class Park extends LinearOpMode {

    private String selectedAuto = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence bfpPath = drive.trajectorySequenceBuilder(new Pose2d(-36.52, 63.50, Math.toRadians(-90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, 38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, 38.41))
                .lineToSplineHeading(new Pose2d(48.79, 37.84, Math.toRadians(180.00)))
                .build();

        TrajectorySequence bcpPath = drive.trajectorySequenceBuilder(new Pose2d(11.98, 63.50, Math.toRadians(270.00)))
                .splineTo(new Vector2d(29.54, 57.66), Math.toRadians(370.79))
                .splineTo(new Vector2d(59.17, 60.49), Math.toRadians(360.00))
                .build();


        TrajectorySequence rfpPath = drive.trajectorySequenceBuilder(new Pose2d(-36.52, -63.50, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, -38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, -38.41))
                .lineToSplineHeading(new Pose2d(48.79, -37.84, Math.toRadians(180.00)))
                .build();

        TrajectorySequence rcpPath = drive.trajectorySequenceBuilder(new Pose2d(11.98, -63.50, Math.toRadians(90.00)))
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

                drive.setPoseEstimate(bcpPath.start());

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

                drive.setPoseEstimate(bfpPath.start());

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

                drive.setPoseEstimate(rcpPath.start());

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

                drive.setPoseEstimate(rfpPath.start());

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
                drive.followTrajectorySequence(bcpPath);
                break;
            case "bfp":
                drive.followTrajectorySequence(bfpPath);
                break;
            case "rcp":
                drive.followTrajectorySequence(rcpPath);
                break;
            case "rfp":
                drive.followTrajectorySequence(rfpPath);
                break;
        }

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