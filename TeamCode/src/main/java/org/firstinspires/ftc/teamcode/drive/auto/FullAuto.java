package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.Storage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Full Auto")
public class FullAuto extends LinearOpMode {

    //* instance variables
    // create variables for selected auto/end strafe
    private String robotPosition = "";
    private String strafeSelector = "";
    private int autoIndex = 0;

    //dPad-held variables
    private boolean upHeld = false;
    private boolean downHeld = false;

    //create delay timer
    private final ElapsedTime timer = new ElapsedTime();
    int delay = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot drive = new Robot(hardwareMap);

        //* middle paths
        TrajectorySequence bcMiddle = drive.trajectorySequenceBuilder(new Pose2d(12.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(12.00, 34.00, Math.toRadians(90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(12.00, 36.00))
                .splineTo(new Vector2d(49.00, 36.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence bfMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                .lineToLinearHeading(new Pose2d(-42.00, 48.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-36.00, 34.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(-30.00, 38.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(49.00, 36.00))
                .build();


        TrajectorySequence rcMiddle = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(12.00, -34.00, Math.toRadians(-90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(12.00, -36.00))
                .splineTo(new Vector2d(49.00, -36.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence rfMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.50, Math.toRadians(450.00)))
                .lineToLinearHeading(new Pose2d(-42.00, -48.00, Math.toRadians(270.00)))
                .lineTo(new Vector2d(-36.00, -34.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(-30.00, -38.00), Math.toRadians(360.00))
                .lineTo(new Vector2d(49.00, -36.00))
                .build();


        //* far paths
        TrajectorySequence bcFar = drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(15.00, 30.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(24.00, 48.00), Math.toRadians(45.00))
                .splineTo(new Vector2d(49.00, 42.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence bfFar = drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(-41.00, 30.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-.5))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineToLinearHeading(new Pose2d(-36.00, 30.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(-36.00, 12.00))
                .lineTo(new Vector2d(12.00, 12.00))
                .splineTo(new Vector2d(49.00, 31.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence rcFar = drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(450.00)))
                .splineToLinearHeading(new Pose2d(15.00, -30.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(24.00, -48.00), Math.toRadians(315.00))
                .splineTo(new Vector2d(49.00, -42.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence rfFar = drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(-41.00, -30.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-.5))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineToLinearHeading(new Pose2d(-36.00, -30.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(-36.00, -12.00))
                .lineTo(new Vector2d(12.00, -12.00))
                .splineTo(new Vector2d(49.00, -31.00), Math.toRadians(0.00))
                .build();

        //* close paths
        TrajectorySequence bcClose = drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(8.00, 33.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(49.00, 31.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence bfClose = drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .splineToLinearHeading(new Pose2d(-35.00, 33.00, Math.toRadians(180.00)), Math.toRadians(0.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-37.00, 33.00))
                .lineTo(new Vector2d(-37.00, 12.00))
                .lineTo(new Vector2d(18.00, 12.00))
                .lineToLinearHeading(new Pose2d(49.00, 42.00, Math.toRadians(0.00)))
                .build();

        TrajectorySequence rcClose = drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(90.00)))
                .splineToLinearHeading(new Pose2d(8.00, -33.00, Math.toRadians(0.00)), Math.toRadians(180.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .splineTo(new Vector2d(49.00, -31.00), Math.toRadians(0.00))
                .build();

        TrajectorySequence rfClose = drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(450.00)))
                .splineToLinearHeading(new Pose2d(-35.00, -33.00, Math.toRadians(180.00)), Math.toRadians(360.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-37.00, -33.00))
                .lineTo(new Vector2d(-37.00, -12.00))
                .lineTo(new Vector2d(18.00, -12.00))
                .lineToLinearHeading(new Pose2d(49.00, -42.00, Math.toRadians(360.00)))
                .build();


        //* strafe paths (scanning)
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

        //run the auto selector while in init
        while(opModeInInit()) {
            autoSelector(drive);
        }

        waitForStart();

        telemetry.addData("Distance Sensor", drive.distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();

        //* delay robot start
        timer.reset();

        try {
            if (delay > 0) {
                Thread.sleep((long) delay * 1000);
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        //* run middle path if sensor detects custom element
        if (drive.distanceSensor.getDistance(DistanceUnit.INCH) > 12 && drive.distanceSensor.getDistance(DistanceUnit.INCH) < 18) {
            switch (robotPosition) {
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
            //* strafe to check for far orientation
            switch (robotPosition) {
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

            //* run close/far path based on if the sensor detects custom element
            if (drive.distanceSensor.getDistance(DistanceUnit.INCH) > 12 && drive.distanceSensor.getDistance(DistanceUnit.INCH) < 18) {
                switch (robotPosition) {
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
                switch (robotPosition) {
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

        //place one yellow pixel on the backboard
        placePixels(drive);

        Storage.currentPose = drive.getPoseEstimate();

        //* extra pixel paths
        TrajectorySequence bluePixel = drive.trajectorySequenceBuilder(new Pose2d(49.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .setReversed(true)
                .splineTo(new Vector2d(12.00,12.00), Math.toRadians(180))
                .lineTo(new Vector2d(-60.00, 12.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(1))
                .addTemporalMarker(() -> drive.holderServo.setPower(1))
                .waitSeconds(2)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .addTemporalMarker(() -> drive.holderServo.setPower(0))
                .setReversed(false)
                .lineTo(new Vector2d(12.00, 12.00))
                .splineTo(new Vector2d(49.00, Storage.currentPose.getY()), Math.toRadians(0.00))
                .build();

        TrajectorySequence redPixel = drive.trajectorySequenceBuilder(new Pose2d(49.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .setReversed(true)
                .splineTo(new Vector2d(12.00,-12.00), Math.toRadians(180))
                .lineTo(new Vector2d(-60.00, -12.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(1))
                .addTemporalMarker(() -> drive.holderServo.setPower(1))
                .waitSeconds(2)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .addTemporalMarker(() -> drive.holderServo.setPower(0))
                .setReversed(false)
                .lineTo(new Vector2d(12.00, -12.00))
                .splineTo(new Vector2d(49.00, Storage.currentPose.getY()), Math.toRadians(0.00))
                .build();

        //* strafe paths (end)
        TrajectorySequence blueStrafeLeft = drive.trajectorySequenceBuilder(new Pose2d(49.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(49.00, 60.00))
                .build();

        TrajectorySequence blueStrafeRight = drive.trajectorySequenceBuilder(new Pose2d(49.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(49.00, 12.00))
                .build();

        TrajectorySequence redStrafeLeft = drive.trajectorySequenceBuilder(new Pose2d(49.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(49.00, -12.00))
                .build();

        TrajectorySequence redStrafeRight = drive.trajectorySequenceBuilder(new Pose2d(49.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(49.00, -60.00))
                .build();

        if (Storage.currentPose.getY() > 0) {
            for (int count = 0; count < autoIndex; count++) {
                drive.followTrajectorySequence(bluePixel);
                placePixels(drive);
            }

            switch (strafeSelector) {
                case "left":
                    drive.followTrajectorySequence(blueStrafeLeft);
                    break;
                case "right":
                    drive.followTrajectorySequence(blueStrafeRight);
                    break;
                default:
                    break;
            }
        } else if (Storage.currentPose.getY() < 0) {
            for (int count = 0; count < autoIndex; count++) {
                drive.followTrajectorySequence(redPixel);
                placePixels(drive);
            }

            switch (strafeSelector) {
                case "left":
                    drive.followTrajectorySequence(redStrafeLeft);
                    break;
                case "right":
                    drive.followTrajectorySequence(redStrafeRight);
                    break;
                default:
                    break;
            }
        }

        Storage.currentPose = drive.getPoseEstimate();
        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.clearAll();
        telemetry.addData("Robot Starting Position", robotPosition);
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

    /**
     * method for placing held pixels on the backboard
     * @param drive the main robot class
     */
    public void placePixels(Robot drive) {
        timer.reset();

        drive.leftLiftMotor.setTargetPosition(1375);
        drive.rightLiftMotor.setTargetPosition(1375);

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

        while (timer.seconds() < 2.5) {
            drive.holderServo.setPower(-1);
        }

        while (timer.seconds() < 3.75) {
            drive.holderServo.setPower(0);
            drive.leftArmServo.setPosition(0.1);
            drive.rightArmServo.setPosition(0.1);
        }

        tryLiftDown(drive);
    }

    /**
     * Recursive attempt to lower the lift
     * @param drive the main robot class
     */
    public void tryLiftDown (Robot drive) {
        if (drive.holderSensor.isPressed()) {
            drive.leftLiftMotor.setTargetPosition(0);
            drive.rightLiftMotor.setTargetPosition(0);
        } else {
            tryLiftDown(drive);
        }
    }

    /**
     * Operator interface to set the robot's auto routine
     * @param drive the main robot class
     */
    public void autoSelector (Robot drive) {
        telemetry.addLine("Please choose from the following starting positions:");
        telemetry.addData("A", "Blue Close");
        telemetry.addData("B", "Blue Far");
        telemetry.addData("X", "Red Close");
        telemetry.addData("Y", "Red Far");
        telemetry.addLine();
        telemetry.addLine("Use bumpers to cycle through auto routines");
        telemetry.addLine("Use dpad left/right to set end strafe");
        telemetry.addLine("Use dpad up/down to set starting delay");

        //select path
        if (gamepad1.a) {
            robotPosition = "bc";
            drive.setPoseEstimate(new Pose2d(12, 63.50, Math.toRadians(-90.00)));
        } else if (gamepad1.b) {
            robotPosition = "bf";
            drive.setPoseEstimate(new Pose2d(-36, 63.50, Math.toRadians(-90.00)));
        } else if (gamepad1.x) {
            robotPosition = "rc";
            drive.setPoseEstimate(new Pose2d(12, -63.50, Math.toRadians(90.00)));
        } else if (gamepad1.y) {
            robotPosition = "rf";
            drive.setPoseEstimate(new Pose2d(-36, -63.50, Math.toRadians(90.00)));
        }

        autoInputs();
        selectorTelemetry();

        telemetry.addData("Distance Sensor", drive.distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    /**
     * Gamepad inputs for the auto selector
     */
    public void autoInputs() {
        //set delay
        if (gamepad1.dpad_up && !upHeld) {
            upHeld = true;
            delay++;
        } else if (gamepad1.dpad_down && delay > 0 && !downHeld) {
            downHeld = true;
            delay--;
        }
        if (!gamepad1.dpad_up) {
            upHeld = false;
        }
        if (!gamepad1.dpad_down) {
            downHeld = false;
        }

        //set strafe
        if (gamepad1.dpad_left) {
            strafeSelector = "left";
        } else if (gamepad1.dpad_right) {
            strafeSelector = "right";
        }

        //set selected auto
        if (gamepad1.right_bumper && autoIndex < 2) {
            autoIndex++;
        } else if (gamepad1.left_bumper && autoIndex > 0) {
            autoIndex--;
        } else if (gamepad1.right_bumper) {
            autoIndex = 0;
        } else if (gamepad1.left_bumper) {
            autoIndex = 2;
        }
    }

    /**
     * Adds telemetry to the control hub based off of the current selections in the autonomous selector
     */
    public void selectorTelemetry() {
        switch (robotPosition) {
            case "bc":
                telemetry.addLine("Robot Starting Position: Blue Close");
                break;
            case "bf":
                telemetry.addLine("Robot Starting Position: Blue Far");
                break;
            case "rc":
                telemetry.addLine("Robot Starting Position: Red Close");
                break;
            case "rf":
                telemetry.addLine("Robot Starting Position: Red Far");
                break;
            default:

                telemetry.addLine("Robot Starting Position: None (Required)");
                break;
        }

        switch (autoIndex) {
            case 0:
                telemetry.addLine("Auto Routine: 2 Pixels");
                break;
            case 1:
                telemetry.addLine("Auto Routine: 2+2 (Unfinished)");
                break;
            case 2:
                telemetry.addLine("Auto Routine: 2+4 (Unfinished)");
                break;
            default:
                telemetry.addLine("ERROR: AUTO ROUTINE NOT FOUND");
                break;
        }

        switch (strafeSelector) {
            case "left":
                telemetry.addLine("Selected Strafe: Left");
                break;
            case "right":
                telemetry.addLine("Selected Strafe: Right");
                break;
            default:
                telemetry.addLine("Selected Strafe: None (Optional)");
        }

        telemetry.addLine();
        telemetry.addLine("Current Delay is " + delay + " seconds");
    }
}