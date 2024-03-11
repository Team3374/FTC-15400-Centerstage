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

    private enum SelectedAuto {
        BLUE_CLOSE,
        BLUE_FAR,
        RED_CLOSE,
        RED_FAR
    }
    private SelectedAuto selectedAuto;

    private TrajectorySequence bfPark, bcPark, rfPark, rcPark;

    private Pose2d poseEstimate;

    @Override
    public void initialize() {
        drive = new DriveBase(hardwareMap);
        gamepadOne = new GamepadEx(gamepad1);

        buildPaths(drive);

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
        bfPark = drive.trajectorySequenceBuilder(new Pose2d(-36, 64.50, Math.toRadians(-90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, 38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, 38.41))
                .lineToSplineHeading(new Pose2d(48.79, 37.84, Math.toRadians(180.00)))
                .build();

        bcPark = drive.trajectorySequenceBuilder(new Pose2d(12, 64.50, Math.toRadians(270.00)))
                .splineTo(new Vector2d(29.54, 57.66), Math.toRadians(370.79))
                .splineTo(new Vector2d(59.17, 60.49), Math.toRadians(360.00))
                .build();


        rfPark = drive.trajectorySequenceBuilder(new Pose2d(-36, -64.50, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, -38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, -38.41))
                .lineToSplineHeading(new Pose2d(48.79, -37.84, Math.toRadians(180.00)))
                .build();

        rcPark = drive.trajectorySequenceBuilder(new Pose2d(12, -64.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(29.54, -57.66), Math.toRadians(-10.79))
                .splineTo(new Vector2d(59.17, -60.49), Math.toRadians(0.00))
                .build();
    }

    public void autoSelector(DriveBase drive) {
        telemetry.addLine("Please choose from the following Autonomous programs:");
        telemetry.addData("A", "Blue Park (Close)");
        telemetry.addData("B", "Blue Park (Far)");
        telemetry.addData("X", "Red Park (Close)");
        telemetry.addData("Y", "Red Park (Far)");
        telemetry.addLine();

        gamepadOne.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            selectedAuto = SelectedAuto.BLUE_CLOSE;
            drive.setPoseEstimate(bcPark.start());

            telemetry.addLine("Selected Auto: Blue Park (Close)");
        });

        gamepadOne.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> {
            selectedAuto = SelectedAuto.BLUE_FAR;
            drive.setPoseEstimate(bfPark.start());

            telemetry.addLine("Selected Auto: Blue Park (Far)");
        });

        gamepadOne.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> {
            selectedAuto = SelectedAuto.RED_CLOSE;
            drive.setPoseEstimate(rcPark.start());

            telemetry.addLine("Selected Auto: Red Park (Close)");
        });

        gamepadOne.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> {
            selectedAuto = SelectedAuto.RED_FAR;
            drive.setPoseEstimate(rfPark.start());

            telemetry.addLine("Selected Auto: Red Park (Far)");
        });

        telemetry.update();
    }

    public void runPaths(DriveBase drive) {
        switch (selectedAuto) {
            case BLUE_CLOSE:
                drive.followTrajectorySequence(bcPark);
                Storage.currentColor = Storage.CurrentColor.BLUE;
                break;
            case BLUE_FAR:
                drive.followTrajectorySequence(bfPark);
                Storage.currentColor = Storage.CurrentColor.BLUE;
                break;
            case RED_CLOSE:
                drive.followTrajectorySequence(rcPark);
                Storage.currentColor = Storage.CurrentColor.RED;
                break;
            case RED_FAR:
                drive.followTrajectorySequence(rfPark);
                Storage.currentColor = Storage.CurrentColor.RED;
                break;
        }
    }
}