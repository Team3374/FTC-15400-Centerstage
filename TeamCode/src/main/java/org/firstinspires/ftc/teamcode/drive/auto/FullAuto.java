package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
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

    //controller-held variables
    private boolean upHeld = false;
    private boolean downHeld = false;
    private boolean bumperHeld = false;

    //create delay timer
    private final ElapsedTime timer = new ElapsedTime();
    private int delay = 0;

    //create "robot"
    private Robot drive;
    public Rev2mDistanceSensor distanceSensor;

    //create all trajectory sequences
    private TrajectorySequence
            bcMiddle, bfMiddle, rcMiddle, rfMiddle,
            bcFar, bfFar, rcFar, rfFar,
            bcClose, bfClose, rcClose, rfClose,
            bcStrafe, bfStrafe, rcStrafe, rfStrafe,
            bluePixel, redPixel,
            blueStrafeLeft, blueStrafeRight, redStrafeLeft, redStrafeRight;

    @Override
    public void runOpMode() throws InterruptedException {
        //instantiate robot
        DriveConstants.MAX_VEL = 75;
        DriveConstants.MAX_ACCEL = 75;
        DriveConstants.MAX_ANG_VEL = 4.5;
        drive = new Robot(hardwareMap);
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");

        //build initial auto paths
        buildFirstPaths();

        //*run the auto selector while in init
        while(opModeInInit()) {
            autoSelector(drive);
        }

        waitForStart();

        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.INCH));
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

        //*run paths to place purple pixel/go to backboard, then place a pixel
        runFirstPaths(drive);
        placePixels(drive);

        Storage.currentPose = drive.getPoseEstimate();

        //set robot distance from wall when intaking extra pixels
        final double inchesFromWall = 1.5;

        //* build and run final auto paths
        buildFinalPaths(inchesFromWall);
        runFinalPaths(drive);

        if (robotPosition.equals("bc") || robotPosition.equals("bf")) {
            Storage.currentColor = "blue";
        } else {
            Storage.currentColor = "red";
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
        telemetry.addData("Distance", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.update();
    }

    /**
     * Operator interface to set the robot's auto routine
     * @param drive the main robot class
     * @see #autoInputs()
     * @see #selectorTelemetry()
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
        telemetry.addLine();

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
        telemetry.update();
    }

    /**
     * Gamepad inputs for the {@link #autoSelector}
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
        if (gamepad1.right_bumper && autoIndex < 2 && !bumperHeld) {
            bumperHeld = true;
            autoIndex++;
        } else if (gamepad1.left_bumper && autoIndex > 0 && !bumperHeld) {
            bumperHeld = true;
            autoIndex--;
        } else if (gamepad1.right_bumper && !bumperHeld) {
            autoIndex = 0;
        } else if (gamepad1.left_bumper && !bumperHeld) {
            autoIndex = 2;
        }
        if (!gamepad1.right_bumper && !gamepad1.left_bumper) {
            bumperHeld = false;
        }
    }

    /**
     * Adds telemetry to the control hub based off of the current selections in the {@link #autoSelector}
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
                telemetry.addLine("Auto Routine: 2+2");
                break;
            case 2:
                telemetry.addLine("Auto Routine: 2+4");
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
        telemetry.addLine();
        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.INCH));
    }

    /**
     * Builds initial auto paths to be run with {@link #runFirstPaths}
     */
    public void buildFirstPaths() {
        //* middle paths
        bcMiddle = drive.trajectorySequenceBuilder(new Pose2d(12.00, 63.50, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(26.00, 48.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(26.00, 24.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(51.00, 36.00))
                .build();


        bfMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(-51.00, 48.00), Math.toRadians(180.00))
                .lineTo(new Vector2d(-51.00, 25.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-53.00, 25.00))
                .lineTo(new Vector2d(-53.00, 36.00))
                .lineTo(new Vector2d(0.00, 36.00))
                .lineToSplineHeading(new Pose2d(36.00, 36.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(51.00, 36.00))
                .build();

        rcMiddle = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(26.00, -48.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(26.00, -24.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(51.00, -36.00))
                .build();

        rfMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-49.00, -48.00), Math.toRadians(180.00))
                .lineTo(new Vector2d(-50.00, -24.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-0.5))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-52.00, -24.00))
                .lineTo(new Vector2d(-52.00, -36.00))
                .lineTo(new Vector2d(0.00, -36.00))
                .lineToSplineHeading(new Pose2d(36.00, -36.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(51.00, -36.00))
                .build();



        //* far paths
        bcFar = drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(23.00, 44.00, Math.toRadians(90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-0.5))
                .lineTo(new Vector2d(23.00, 46.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineToSplineHeading(new Pose2d(40.00, 42.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(51.00, 42.00))
                .build();

        bfFar = drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(-46.00, 44.00, Math.toRadians(90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-0.5))
                .lineTo(new Vector2d(-46.00, 46.00))
                //.waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-34.00, 46.00))
                .lineTo(new Vector2d(-36.00, 24.00))
                .lineToSplineHeading(new Pose2d(-36.00, 12.00, Math.toRadians(0)))
                .lineTo(new Vector2d(12.00, 12.00))
//                .splineTo(new Vector2d(39.00, 30.00), Math.toRadians(0.00))
//                .lineTo(new Vector2d(51.00, 30.00))
                .lineToSplineHeading(new Pose2d(51, 34, Math.toRadians(0.00)))
                .build();

        rcFar = drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(23.00, -44.00, Math.toRadians(-90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-0.5))
                .lineTo(new Vector2d(23.00, -46.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineToSplineHeading(new Pose2d(40.00, -42.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(51.00, -42.00))
                .build();

        rfFar = drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-46.00, -44.00, Math.toRadians(-90.00)))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-0.5))
                .lineTo(new Vector2d(-46.00, -46.00))
                //.waitSeconds(0.5)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(-34.00, -46.00))
                .lineTo(new Vector2d(-36.00, -24.00))
                .lineToSplineHeading(new Pose2d(-36.00, -12.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(12.00, -12.00))
//                .splineTo(new Vector2d(39.00, -30.00), Math.toRadians(0.00))
//                .lineTo(new Vector2d(51.00, -30.00))
                .lineToSplineHeading(new Pose2d(51, -34, Math.toRadians(0.00)))
                .build();


        //* close paths
        bcClose = drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(24.00, 30.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(11.00, 30.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .lineTo(new Vector2d(24.00, 30.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(51.00, 32.00))
                .build();

        bfClose = drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(-48.00, 30.00, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-36.00, 30.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .lineTo(new Vector2d(-48.00, 30.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0.00))
                .lineToSplineHeading(new Pose2d(-48.00, 12.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(12.00, 12.00))
                .lineTo(new Vector2d(51.00, 42.00))
                .build();

        rcClose = drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(24.00, -30.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(11.00, -30.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .lineTo(new Vector2d(24.00, -30.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .lineTo(new Vector2d(51.00, -32.00))
                .build();

        rfClose = drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-48.00, -30.00, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-36.00, -30.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(-1))
                .lineTo(new Vector2d(-48.00, -30.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0.00))
                .lineToSplineHeading(new Pose2d(-48.00, -12.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(12.00, -12.00))
                .lineTo(new Vector2d(51.00, -42.00))
                .build();

        //* strafe paths (scanning)
        bcStrafe = drive.trajectorySequenceBuilder(new Pose2d(12.00, 63.50, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(24.00, 63.50))
                .build();

        bfStrafe = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                .lineTo(new Vector2d(-48.00, 63.50))
                .build();

        rcStrafe = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.50, Math.toRadians(90.00)))
                .lineTo(new Vector2d(24.00, -63.50))
                .build();

        rfStrafe = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.50, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-48.00, -63.50))
                .build();
    }

    /**
     * Places purple pixel and goes to backboard based on distance sensor
     * @param drive the main robot class
     * @see #buildFirstPaths()
     */
    public void runFirstPaths(Robot drive) {
        //* run middle path if sensor detects custom element
        if (distanceSensor.getDistance(DistanceUnit.INCH) > 14 && distanceSensor.getDistance(DistanceUnit.INCH) < 20) {
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
            if (distanceSensor.getDistance(DistanceUnit.INCH) > 14 && distanceSensor.getDistance(DistanceUnit.INCH) < 20) {
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
    }

    /**
     * Builds final auto paths to be run with {@link #runFinalPaths}
     * @param inchesFromWall the distance between the robot and the wall when intaking pixels
     */
    public void buildFinalPaths(double inchesFromWall) {
        //* extra pixel paths
        bluePixel = drive.trajectorySequenceBuilder(new Pose2d(51.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .setReversed(true)
                .splineTo(new Vector2d(12.00,12.00), Math.toRadians(180))
                .lineTo(new Vector2d((-63.5 + inchesFromWall), 12.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(1))
                .addTemporalMarker(() -> drive.holderServo.setPower(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .addTemporalMarker(() -> drive.holderServo.setPower(0))
                .setReversed(false)
                .lineTo(new Vector2d(12.00, 12.00))
                .splineTo(new Vector2d(51.00, Storage.currentPose.getY()), Math.toRadians(0.00))
                .build();

        redPixel = drive.trajectorySequenceBuilder(new Pose2d(51.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .setReversed(true)
                .splineTo(new Vector2d(12.00,-12.00), Math.toRadians(180))
                .lineTo(new Vector2d((-63.5 + inchesFromWall), -12.00))
                .addTemporalMarker(() -> drive.intakeMotor.setPower(1))
                .addTemporalMarker(() -> drive.holderServo.setPower(1))
                .waitSeconds(1)
                .addTemporalMarker(() -> drive.intakeMotor.setPower(0))
                .addTemporalMarker(() -> drive.holderServo.setPower(0))
                .setReversed(false)
                .lineTo(new Vector2d(12.00, -12.00))
                .splineTo(new Vector2d(51.00, Storage.currentPose.getY()), Math.toRadians(0.00))
                .build();

        //* strafe paths (end)
        blueStrafeLeft = drive.trajectorySequenceBuilder(new Pose2d(51.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(51.00, 60.00))
                .build();

        blueStrafeRight = drive.trajectorySequenceBuilder(new Pose2d(51.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(51.00, 12.00))
                .build();

        redStrafeLeft = drive.trajectorySequenceBuilder(new Pose2d(51.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(51.00, -12.00))
                .build();

        redStrafeRight = drive.trajectorySequenceBuilder(new Pose2d(51.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(51.00, -60.00))
                .build();
    }

    /**
     * Places additional pixels/strafes based on user selection
     * @param drive the main robot class
     * @see #buildFinalPaths(double)
     */
    public void runFinalPaths(Robot drive) {
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
    }

    /**
     * method for placing held pixels on the backboard
     * @param drive the main robot class
     */
    public void placePixels(Robot drive) {
        this.drive = drive;
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
            drive.leftLiftMotor.setTargetPosition(1400);
            drive.rightLiftMotor.setTargetPosition(1400);
            drive.leftArmServo.setPosition(0.5);
            drive.rightArmServo.setPosition(0.5);
        }

        while (timer.seconds() < 2.5) {
            drive.holderServo.setPower(-1);
        }

        while (timer.seconds() < 3.75) {
            drive.holderServo.setPower(0);
            drive.leftArmServo.setPosition(0.1);
            drive.rightArmServo.setPosition(0.1);
        }

        while (timer.seconds() < 4.25) {
            if (drive.holderSensor.isPressed()) {
                drive.leftLiftMotor.setTargetPosition(0);
                drive.rightLiftMotor.setTargetPosition(0);
            } else {
                drive.leftArmServo.setPosition(0.1);
                drive.rightArmServo.setPosition(0.1);
            }
        }
    }
}