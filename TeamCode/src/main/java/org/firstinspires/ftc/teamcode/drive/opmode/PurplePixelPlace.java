package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Place Purple Pixel")
public class PurplePixelPlace extends LinearOpMode {

    private String selectedAuto = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot drive = new Robot(hardwareMap);

        //* middle paths
        TrajectorySequence bcMiddle = drive.trajectorySequenceBuilder(new Pose2d(12.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(12.00, 32.00, Math.toRadians(90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(12.00, 36.00))
                .splineTo(new Vector2d(51.00, 36.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence bfMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(-36.00, 32.00, Math.toRadians(90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-36.00, 36.00))
                .splineTo(new Vector2d(-24.00, 36.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(51.00, 36.00))
                .build();


        TrajectorySequence rcMiddle = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(12.00, -32.00, Math.toRadians(-90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(12.00, -36.00))
                .splineTo(new Vector2d(51.00, -36.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence rfMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-36.00, -32.00, Math.toRadians(-90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-36.00, -36.00))
                .splineTo(new Vector2d(-24.00, -36.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(51.00, -36.00))
                .build();

        //* strafe paths
        TrajectorySequence bcStrafe = drive.trajectorySequenceBuilder(new Pose2d(12.00, 63.50, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(24.00, 63.50))
                .build();

        TrajectorySequence bfStrafe = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(-48.00, 63.50))
                .build();

        TrajectorySequence rcStrafe = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.50, Math.toRadians(90.00)))
                .lineTo(new Vector2d(24.00, -63.50))
                .build();

        TrajectorySequence rfStrafe = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.50, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-48.00, -63.50))
                .build();

        //* far paths
        TrajectorySequence bcFar = drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(15.00, 30.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(24.00, 48.00), Math.toRadians(45.00))
                .splineTo(new Vector2d(51.00, 44.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence bfFar = drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(-41.00, 30.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineToLinearHeading(new Pose2d(-36.00, 30.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(-36.00, 12.00))
                .lineTo(new Vector2d(12.00, 12.00))
                .splineTo(new Vector2d(51.00, 30.00), Math.toRadians(0.00))
                .build();


        TrajectorySequence rcFar = drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(450.00)))
                .splineToLinearHeading(new Pose2d(15.00, -30.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(24.00, -48.00), Math.toRadians(315.00))
                .splineTo(new Vector2d(51.00, -44.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence rfFar = drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-41.00, -30.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineToLinearHeading(new Pose2d(-36.00, -30.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(-36.00, -12.00))
                .lineTo(new Vector2d(12.00, -12.00))
                .splineTo(new Vector2d(51.00, -30.00), Math.toRadians(0.00))
                .build();


        //* close paths
        TrajectorySequence bcClose = drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(8.00, 33.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(51.00, 30.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence bfClose = drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(-33.00, 33.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-36.00, 33.00))
                .lineTo(new Vector2d(-36.00, 12.00))
                .lineTo(new Vector2d(18.00, 12.00))
                .splineToSplineHeading(new Pose2d(51.00, 44.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();



        TrajectorySequence rcClose = drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(8.00, -33.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(51.00, -30.00), Math.toRadians(0.00))
                .build();


        TrajectorySequence rfClose = drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-33.00, -33.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-36.00, -33.00))
                .lineTo(new Vector2d(-36.00, -12.00))
                .lineTo(new Vector2d(18.00, -12.00))
                .splineToSplineHeading(new Pose2d(51.00, -44.00, Math.toRadians(0.00)), Math.toRadians(0.00))
                .build();



        telemetry.addLine("Please choose from the following Autonomous programs:");
        telemetry.addData("A", "Blue Purple Pixel Park (Close)");
        telemetry.addData("B", "Blue Purple Pixel Park (Far)");
        telemetry.addData("X", "Red Purple Pixel Park (Close)");
        telemetry.addData("Y", "Red Purple Pixel Park (Far)");
        telemetry.update();

        while(opModeInInit()) {
            if (gamepad1.a) {
                selectedAuto = "bc";

                drive.setPoseEstimate(new Pose2d(12, 63.50, Math.toRadians(-90.00)));

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Purple Pixel Park (Close)");
                telemetry.addData("B", "Blue Purple Pixel Park (Far)");
                telemetry.addData("X", "Red Purple Pixel Park (Close)");
                telemetry.addData("Y", "Red Purple Pixel Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Blue Purple Pixel Park (Close)");
                telemetry.update();
            } else if (gamepad1.b) {
                selectedAuto = "bf";

                drive.setPoseEstimate(new Pose2d(-36, 63.50, Math.toRadians(-90.00)));

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Purple Pixel Park (Close)");
                telemetry.addData("B", "Blue Purple Pixel Park (Far)");
                telemetry.addData("X", "Red Purple Pixel Park (Close)");
                telemetry.addData("Y", "Red Purple Pixel Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Blue Purple Pixel Park (Far)");
                telemetry.update();
            } else if (gamepad1.x) {
                selectedAuto = "rc";

                drive.setPoseEstimate(new Pose2d(12, -63.50, Math.toRadians(90.00)));

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Purple Pixel Park (Close)");
                telemetry.addData("B", "Blue Purple Pixel Park (Far)");
                telemetry.addData("X", "Red Purple Pixel Park (Close)");
                telemetry.addData("Y", "Red Purple Pixel Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Red Purple Pixel Park (Close)");
                telemetry.update();
            } else if (gamepad1.y) {
                selectedAuto = "rf";

                drive.setPoseEstimate(new Pose2d(-36, -63.50, Math.toRadians(90.00)));

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following Autonomous programs:");
                telemetry.addData("A", "Blue Purple Pixel Park (Close)");
                telemetry.addData("B", "Blue Purple Pixel Park (Far)");
                telemetry.addData("X", "Red Purple Pixel Park (Close)");
                telemetry.addData("Y", "Red Purple Pixel Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Auto: Red Purple Pixel Park (Far)");
                telemetry.update();
            }
        }

        waitForStart();

        telemetry.addData("Distance Sensor", drive.distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
        if (drive.distanceSensor.getDistance(DistanceUnit.INCH) > 15 && drive.distanceSensor.getDistance(DistanceUnit.INCH) < 21) {
            switch (selectedAuto) {
                case "bc":
                    drive.followTrajectorySequence(bcMiddle);
                    break;
                case "bf":
                    drive.followTrajectorySequence(bfMiddle);
                    break;
                case "rc":
                    drive.followTrajectorySequence(rcMiddle);
                    break;
                case "rf":
                    drive.followTrajectorySequence(rfMiddle);
                    break;
            }
        } else {
            switch (selectedAuto) {
                case "bc":
                    drive.followTrajectorySequence(bcStrafe);
                    break;
                case "bf":
                    drive.followTrajectorySequence(bfStrafe);
                    break;
                case "rc":
                    drive.followTrajectorySequence(rcStrafe);
                    break;
                case "rf":
                    drive.followTrajectorySequence(rfStrafe);
                    break;
            }

            telemetry.addData("Distance Sensor", drive.distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
            if (drive.distanceSensor.getDistance(DistanceUnit.INCH) > 15 && drive.distanceSensor.getDistance(DistanceUnit.INCH) < 21) {
                switch (selectedAuto) {
                    case "bc":
                        drive.followTrajectorySequence(bcFar);
                        break;
                    case "bf":
                        drive.followTrajectorySequence(bfFar);
                        break;
                    case "rc":
                        drive.followTrajectorySequence(rcFar);
                        break;
                    case "rf":
                        drive.followTrajectorySequence(rfFar);
                        break;
                }
            } else {
                switch (selectedAuto) {
                    case "bc":
                        drive.followTrajectorySequence(bcClose);
                        break;
                    case "bf":
                        drive.followTrajectorySequence(bfClose);
                        break;
                    case "rc":
                        drive.followTrajectorySequence(rcClose);
                        break;
                    case "rf":
                        drive.followTrajectorySequence(rfClose);
                        break;
                }
            }
        }

        placeYellowPixel(drive);

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.clearAll();
        telemetry.addData("Current Auto", selectedAuto);
        telemetry.addLine();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.addLine();
        telemetry.addData("Intake Encoder", drive.intakeMotor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Distance", drive.distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    public void placeYellowPixel(Robot drive) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        drive.leftLiftMotor.setTargetPosition(1250);
        drive.rightLiftMotor.setTargetPosition(1250);

        drive.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (timer.seconds() < 1) {
            drive.leftLiftMotor.setPower(1);
            drive.rightLiftMotor.setPower(1);
        }

        while (timer.seconds() < 2) {
            drive.leftArmServo.setPosition(0.45);
            drive.rightArmServo.setPosition(0.45);
        }

        while (timer.seconds() < 3) {
            drive.holderServo.setPower(-1);
        }

        while (timer.seconds() < 4) {
            drive.holderServo.setPower(0);
            drive.leftArmServo.setPosition(0.1);
            drive.rightArmServo.setPosition(0.1);
        }

        while (timer.seconds() < 5) {
            drive.leftLiftMotor.setTargetPosition(0);
            drive.rightLiftMotor.setTargetPosition(0);
        }
    }
}
