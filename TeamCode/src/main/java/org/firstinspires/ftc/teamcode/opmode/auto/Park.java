package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveBase;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@SuppressWarnings("unused")
@Autonomous(name="Park")
public class Park extends CommandOpMode {
    private DriveBase drive;
    private GamepadEx gamepadOne;

    private String selectedAuto = null;

    private TrajectorySequence bfPark, bcPark, rfPark, rcPark;

    private Pose2d poseEstimate;

    @Override
    public void initialize() {
        drive = new DriveBase(hardwareMap);
        gamepadOne = new GamepadEx(gamepad1);

        buildPaths(drive);

        telemetry.addLine("Please choose from the following Autonomous programs:");
        telemetry.addData("A", "Blue Park (Close)");
        telemetry.addData("B", "Blue Park (Far)");
        telemetry.addData("X", "Red Park (Close)");
        telemetry.addData("Y", "Red Park (Far)");
        telemetry.update();

        while(opModeInInit()) {
            autoSelector(drive);
        }

        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> runPaths(drive)),
                        new InstantCommand(() -> {
                            Storage.currentPose = drive.getPoseEstimate();
                            poseEstimate = drive.getPoseEstimate();
                        })
                ),
                new RunCommand(() -> {
                    telemetry.clearAll();
                    telemetry.addData("Current Auto", selectedAuto);
                    telemetry.addLine();
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("heading", poseEstimate.getHeading());
                    telemetry.update();
                })
        );
    }

    public void buildPaths(DriveBase drive) {
        bfPark = drive.trajectorySequenceBuilder(new Pose2d(-36, 63.50, Math.toRadians(-90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, 38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, 38.41))
                .lineToSplineHeading(new Pose2d(48.79, 37.84, Math.toRadians(180.00)))
                .build();

        bcPark = drive.trajectorySequenceBuilder(new Pose2d(12, 63.50, Math.toRadians(270.00)))
                .splineTo(new Vector2d(29.54, 57.66), Math.toRadians(370.79))
                .splineTo(new Vector2d(59.17, 60.49), Math.toRadians(360.00))
                .build();


        rfPark = drive.trajectorySequenceBuilder(new Pose2d(-36, -63.50, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, -38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, -38.41))
                .lineToSplineHeading(new Pose2d(48.79, -37.84, Math.toRadians(180.00)))
                .build();

        rcPark = drive.trajectorySequenceBuilder(new Pose2d(12, -63.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(29.54, -57.66), Math.toRadians(-10.79))
                .splineTo(new Vector2d(59.17, -60.49), Math.toRadians(0.00))
                .build();
    }

    public void autoSelector(DriveBase drive) {
        gamepadOne.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
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
        });

        gamepadOne.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
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
        });

        gamepadOne.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
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
        });

        gamepadOne.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
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
        });
    }

    public void runPaths(DriveBase drive) {
        switch (selectedAuto) {
            case "bcp":
                drive.followTrajectorySequence(bcPark);
                Storage.currentColor = Storage.CurrentColor.BLUE;
                break;
            case "bfp":
                drive.followTrajectorySequence(bfPark);
                Storage.currentColor = Storage.CurrentColor.BLUE;
                break;
            case "rcp":
                drive.followTrajectorySequence(rcPark);
                Storage.currentColor = Storage.CurrentColor.RED;
                break;
            case "rfp":
                drive.followTrajectorySequence(rfPark);
                Storage.currentColor = Storage.CurrentColor.RED;
                break;
        }
    }
}