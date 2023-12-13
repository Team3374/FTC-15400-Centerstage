package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Place Two Pixels (No Vision)")
public class PlaceTwoPixels extends LinearOpMode {

    private String selectedAuto = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot drive = new Robot(hardwareMap);

        TrajectorySequence bftPath = drive.trajectorySequenceBuilder(new Pose2d(-36, 63.50, Math.toRadians(-90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, 38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, 38.41))
                .lineToSplineHeading(new Pose2d(48.79, 37.84, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> {
                    try {
                        placePixel(drive);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .build();

        TrajectorySequence bctPath = drive.trajectorySequenceBuilder(new Pose2d(12, 63.50, Math.toRadians(270.00)))
                .splineTo(new Vector2d(48.79, 37.84), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    try {
                        placePixel(drive);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .build();


        TrajectorySequence rftPath = drive.trajectorySequenceBuilder(new Pose2d(-36, -63.50, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, -38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, -38.41))
                .lineToSplineHeading(new Pose2d(48.79, -37.84, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> {
                    try {
                        placePixel(drive);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .build();

        TrajectorySequence rctPath = drive.trajectorySequenceBuilder(new Pose2d(12, -63.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(48.79, -37.84), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    try {
                        placePixel(drive);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .build();

        telemetry.addLine("Please choose from the following Autonomous programs:");
        telemetry.addData("A", "Blue Two Pixels (Close)");
        telemetry.addData("B", "Blue Two Pixels (Far)");
        telemetry.addData("X", "Red Two Pixels (Close)");
        telemetry.addData("Y", "Red Two Pixels (Far)");
        telemetry.update();

        while(opModeInInit()) {
            if (gamepad1.a) {
                selectedAuto = "bct";

                drive.setPoseEstimate(bctPath.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Two Pixels (Close)");
                telemetry.addData("B", "Blue Two Pixels (Far)");
                telemetry.addData("X", "Red Two Pixels (Close)");
                telemetry.addData("Y", "Red Two Pixels (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Blue Two Pixels (Close)");
                telemetry.update();
            } else if (gamepad1.b) {
                selectedAuto = "bft";

                drive.setPoseEstimate(bftPath.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Two Pixels (Close)");
                telemetry.addData("B", "Blue Two Pixels (Far)");
                telemetry.addData("X", "Red Two Pixels (Close)");
                telemetry.addData("Y", "Red Two Pixels (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Blue Two Pixels (Far)");
                telemetry.update();
            } else if (gamepad1.x) {
                selectedAuto = "rct";

                drive.setPoseEstimate(rctPath.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Two Pixels (Close)");
                telemetry.addData("B", "Blue Two Pixels (Far)");
                telemetry.addData("X", "Red Two Pixels (Close)");
                telemetry.addData("Y", "Red Two Pixels (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Red Two Pixels (Close)");
                telemetry.update();
            } else if (gamepad1.y) {
                selectedAuto = "rft";

                drive.setPoseEstimate(rftPath.start());

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Two Pixels (Close)");
                telemetry.addData("B", "Blue Two Pixels (Far)");
                telemetry.addData("X", "Red Two Pixels (Close)");
                telemetry.addData("Y", "Red Two Pixels (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Red Two Pixels (Far)");
                telemetry.update();
            }
        }

        waitForStart();

        switch (selectedAuto) {
            case "bct":
                drive.followTrajectorySequence(bctPath);
                break;
            case "bft":
                drive.followTrajectorySequence(bftPath);
                break;
            case "rct":
                drive.followTrajectorySequence(rctPath);
                break;
            case "rft":
                drive.followTrajectorySequence(rftPath);
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

    public void placePixel(Robot drive) throws InterruptedException {
        drive.leftLiftMotor.setTargetPosition(1000);
        drive.rightLiftMotor.setTargetPosition(1000);

        drive.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftLiftMotor.setPower(1);
        drive.rightLiftMotor.setPower(1);

        drive.leftArmServo.setPosition(0.75);
        drive.rightArmServo.setPosition(0.75);

        sleep(500);

        drive.holderServo.setPower(1);

        sleep(500);

        drive.holderServo.setPower(0);
        drive.leftArmServo.setPosition(0);
        drive.rightArmServo.setPosition(0);

        drive.leftLiftMotor.setTargetPosition(0);
        drive.rightLiftMotor.setTargetPosition(0);
    }
}
