package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.auto.PlacePixelCommand;
import org.firstinspires.ftc.teamcode.commands.auto.RunFinalPaths;
import org.firstinspires.ftc.teamcode.commands.auto.RunFirstPaths;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensor;
import org.firstinspires.ftc.teamcode.subsystems.Holder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveBase;
import org.firstinspires.ftc.teamcode.util.Storage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Full Auto")
public class FullAuto extends LinearOpMode {
    public enum RobotPosition {
        BLUE_CLOSE,
        BLUE_FAR,
        RED_CLOSE,
        RED_FAR
    }
    public RobotPosition robotPosition;

    public enum PropPosition {
        CLOSE,
        MIDDLE,
        FAR
    }
    public PropPosition propPosition;

    public enum StrafeSelection {
        NONE,
        LEFT,
        RIGHT
    }
    public StrafeSelection strafeSelection = StrafeSelection.NONE;

    private int autoIndex = 0;

    //create delay timer
    private final ElapsedTime timer = new ElapsedTime();
    private int delay = 0;

    //create "robot"
    private DriveBase driveBase;
    private Drive drive;
    private Intake intake;
    private Holder holder;
    private Lift lift;
    private Arm arm;

    public DistanceSensor distanceSensor;

    private GamepadEx gamepadOne;

    //create all trajectory sequences
    public TrajectorySequence
            bcMiddle, bfMiddle, rcMiddle, rfMiddle,
            bcFar, bfFar, rcFar, rfFar,
            bcClose, bfClose, rcClose, rfClose,
            bcStrafe, bfStrafe, rcStrafe, rfStrafe,
            bluePixel, redPixel,
            blueStrafeLeft, blueStrafeRight, redStrafeLeft, redStrafeRight;

    @Override
    public void runOpMode() throws InterruptedException {
        driveBase = new DriveBase(hardwareMap);
        drive = new Drive(driveBase, false);
        intake = new Intake(hardwareMap);
        holder = new Holder(hardwareMap);
        lift = new Lift(hardwareMap);
        arm = new Arm(hardwareMap);

        distanceSensor = new DistanceSensor(hardwareMap);

        gamepadOne = new GamepadEx(gamepad1);

        //build initial auto paths
        buildFirstPaths();

        //*run the auto selector while in init
        while(opModeInInit() && !isStopRequested()) {
            autoSelector(drive);
        }

        //set robot distance from wall when intaking extra pixels
        final double inchesFromWall = 1.5;
        boolean middle = false;

        CommandScheduler.getInstance().schedule(
            new SequentialCommandGroup(
                    new InstantCommand(this::buildFirstPaths),
                    new RunFirstPaths(this, drive, robotPosition),
                    new PlacePixelCommand(lift, arm, holder, 1450, 0.5),
                    new InstantCommand(() -> Storage.currentPose = drive.getPoseEstimate()),
                    new InstantCommand(() -> buildFinalPaths(inchesFromWall)),
                    new RunFinalPaths(this, drive, lift, arm, holder, robotPosition, autoIndex),
                    new InstantCommand(() -> {
                        if (robotPosition == RobotPosition.BLUE_CLOSE || robotPosition == RobotPosition.BLUE_FAR) {
                            Storage.currentColor = "blue";
                        } else {
                            Storage.currentColor = "red";
                        }

                        Storage.currentPose = drive.getPoseEstimate();
                    })
            )
        );

        waitForStart();

        //* delay robot start
        timer.reset();

        try {
            if (delay > 0) {
                Thread.sleep((long) delay * 1000);
            }
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        CommandScheduler.getInstance().run();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Distance Sensor", distanceSensor.getDistance());
            telemetry.addData("Starting Position", robotPosition);
            telemetry.addLine();
            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());
            telemetry.addLine();
            telemetry.addData("Distance", distanceSensor.getDistance());
            telemetry.update();
        }

    }

    /**
     * Operator interface to set the robot's auto routine
     * @param drive the main robot class
     * @see #selectorInputs()
     * @see #selectorTelemetry()
     */
    private void autoSelector (Drive drive) {
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

        selectorInputs();

        selectorTelemetry();
        telemetry.update();
    }

    /**
     * Gamepad inputs for the {@link #autoSelector}
     */
    private void selectorInputs() {
        //select path
        gamepadOne.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            robotPosition = RobotPosition.BLUE_CLOSE;
            drive.setPoseEstimate(new Pose2d(12, 63.50, Math.toRadians(-90.00)));
        });

        gamepadOne.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            robotPosition = RobotPosition.BLUE_FAR;
            drive.setPoseEstimate(new Pose2d(-36, 63.50, Math.toRadians(-90.00)));
        });

        gamepadOne.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            robotPosition = RobotPosition.RED_CLOSE;
            drive.setPoseEstimate(new Pose2d(12, -63.50, Math.toRadians(90.00)));
        });

        gamepadOne.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> {
            robotPosition = RobotPosition.RED_FAR;
            drive.setPoseEstimate(new Pose2d(-36, -63.50, Math.toRadians(90.00)));
        });

        //set delay
        gamepadOne.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> delay++);
        gamepadOne.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(() -> delay--);

        //set strafe
        gamepadOne.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> strafeSelection = StrafeSelection.LEFT);
        gamepadOne.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> strafeSelection = StrafeSelection.RIGHT);

        //set selected auto
        gamepadOne.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> autoIndex = (autoIndex < 2) ? autoIndex + 1 : 0);
        gamepadOne.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> autoIndex = (autoIndex > 0) ? autoIndex - 1 : 2);
    }

    /**
     * Adds telemetry to the control hub based off of the current selections in the {@link #autoSelector}
     */
    private void selectorTelemetry() {
        switch (robotPosition) {
            case BLUE_CLOSE:
                telemetry.addLine("Drive Starting Position: Blue Close");
                break;
            case BLUE_FAR:
                telemetry.addLine("Drive Starting Position: Blue Far");
                break;
            case RED_CLOSE:
                telemetry.addLine("Drive Starting Position: Red Close");
                break;
            case RED_FAR:
                telemetry.addLine("Drive Starting Position: Red Far");
                break;
            default:

                telemetry.addLine("Drive Starting Position: None (Required)");
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

        switch (strafeSelection) {
            case LEFT:
                telemetry.addLine("Selected Strafe: Left");
                break;
            case RIGHT:
                telemetry.addLine("Selected Strafe: Right");
                break;
            default:
                telemetry.addLine("Selected Strafe: None (Optional)");
        }

        telemetry.addLine();
        telemetry.addLine("Current Delay is " + delay + " seconds");
        telemetry.addLine();
        telemetry.addData("Distance Sensor", distanceSensor.getDistance());
    }

    /**
     * Builds initial auto paths
     */
    public void buildFirstPaths() {
        //* middle paths
        bcMiddle = drive.trajectorySequenceBuilder(new Pose2d(12.00, 63.50, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(26.00, 48.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(26.00, 24.00))
                .addTemporalMarker(() -> intake.setPower(-0.25))
                .waitSeconds(0.5)
                .addTemporalMarker(intake::stop)
                .lineTo(new Vector2d(50.00, 36.00))
                .build();


        bfMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 63.50, Math.toRadians(-90.00)))
                .splineTo(new Vector2d(-51.00, 48.00), Math.toRadians(180.00))
                .lineTo(new Vector2d(-51.00, 24.00))
                .addTemporalMarker(() -> intake.setPower(-0.25))
                .waitSeconds(0.5)
                .addTemporalMarker(intake::stop)
                .lineTo(new Vector2d(-53.00, 24.00))
                .lineTo(new Vector2d(-53.00, 36.00))
                .lineTo(new Vector2d(0.00, 36.00))
                .lineToSplineHeading(new Pose2d(36.00, 36.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(50.00, 36.00))
                .build();

        rcMiddle = drive.trajectorySequenceBuilder(new Pose2d(12.00, -63.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(26.00, -48.00), Math.toRadians(0.00))
                .lineTo(new Vector2d(26.00, -24.00))
                .addTemporalMarker(() -> intake.setPower(-0.25))
                .waitSeconds(0.5)
                .addTemporalMarker(intake::stop)
                .lineTo(new Vector2d(50.00, -36.00))
                .build();

        rfMiddle = drive.trajectorySequenceBuilder(new Pose2d(-36.00, -63.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(-49.00, -48.00), Math.toRadians(180.00))
                .lineTo(new Vector2d(-50.00, -24.00))
                .addTemporalMarker(() -> intake.setPower(-0.25))
                .waitSeconds(0.5)
                .addTemporalMarker(intake::stop)
                .lineTo(new Vector2d(-52.00, -24.00))
                .lineTo(new Vector2d(-52.00, -36.00))
                .lineTo(new Vector2d(0.00, -36.00))
                .lineToSplineHeading(new Pose2d(36.00, -36.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(50.00, -36.00))
                .build();



        //* far paths
        bcFar = drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(23.00, 44.00, Math.toRadians(90.00)))
                .addTemporalMarker(() -> intake.setPower(-0.25))
                .lineTo(new Vector2d(23.00, 46.00))
                .addTemporalMarker(intake::stop)
                .lineToSplineHeading(new Pose2d(40.00, 42.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(50.00, 42.00))
                .build();

        bfFar = drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(-46.00, 44.00, Math.toRadians(90.00)))
                .addTemporalMarker(() -> intake.setPower(-0.25))
                .lineTo(new Vector2d(-46.00, 46.00))
                .addTemporalMarker(intake::stop)
                .lineTo(new Vector2d(-34.00, 46.00))
                .lineTo(new Vector2d(-36.00, 24.00))
                .lineToSplineHeading(new Pose2d(-36.00, 12.00, Math.toRadians(0)))
                .lineTo(new Vector2d(12.00, 12.00))
                .lineToSplineHeading(new Pose2d(50, 34, Math.toRadians(0.00)))
                .build();

        rcFar = drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(23.00, -44.00, Math.toRadians(-90.00)))
                .addTemporalMarker(() -> intake.setPower(-0.25))
                .lineTo(new Vector2d(23.00, -46.00))
                .addTemporalMarker(intake::stop)
                .lineToSplineHeading(new Pose2d(40.00, -42.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(50.00, -42.00))
                .build();

        rfFar = drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-46.00, -44.00, Math.toRadians(-90.00)))
                .addTemporalMarker(() -> intake.setPower(-0.25))
                .lineTo(new Vector2d(-46.00, -46.00))
                .addTemporalMarker(intake::stop)
                .lineTo(new Vector2d(-34.00, -46.00))
                .lineTo(new Vector2d(-36.00, -24.00))
                .lineToSplineHeading(new Pose2d(-36.00, -12.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(12.00, -12.00))
                .lineToSplineHeading(new Pose2d(50, -34, Math.toRadians(0.00)))
                .build();


        //* close paths
        bcClose = drive.trajectorySequenceBuilder(new Pose2d(24.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(24.00, 30.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(11.00, 30.00))
                .addTemporalMarker(intake::out)
                .lineTo(new Vector2d(24.00, 30.00))
                .addTemporalMarker(intake::stop)
                .lineTo(new Vector2d(50.00, 32.00))
                .build();

        bfClose = drive.trajectorySequenceBuilder(new Pose2d(-48.00, 63.50, Math.toRadians(-90.00)))
                .lineToSplineHeading(new Pose2d(-48.00, 30.00, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-36.00, 30.00))
                .addTemporalMarker(intake::out)
                .lineTo(new Vector2d(-48.00, 30.00))
                .addTemporalMarker(intake::stop)
                .lineToSplineHeading(new Pose2d(-48.00, 12.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(12.00, 12.00))
                .lineTo(new Vector2d(50.00, 42.00))
                .build();

        rcClose = drive.trajectorySequenceBuilder(new Pose2d(24.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(24.00, -30.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(11.00, -30.00))
                .addTemporalMarker(intake::out)
                .lineTo(new Vector2d(24.00, -30.00))
                .addTemporalMarker(intake::stop)
                .lineTo(new Vector2d(50.00, -32.00))
                .build();

        rfClose = drive.trajectorySequenceBuilder(new Pose2d(-48.00, -63.50, Math.toRadians(90.00)))
                .lineToSplineHeading(new Pose2d(-48.00, -30.00, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-36.00, -30.00))
                .addTemporalMarker(intake::out)
                .lineTo(new Vector2d(-48.00, -30.00))
                .addTemporalMarker(intake::stop)
                .lineToSplineHeading(new Pose2d(-48.00, -12.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(12.00, -12.00))
                .lineTo(new Vector2d(50.00, -42.00))
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
     * Builds final auto paths
     * @param inchesFromWall the distance between the robot and the wall when intaking pixels
     */
    public void buildFinalPaths(double inchesFromWall) {
        //* extra pixel paths
        bluePixel = drive.trajectorySequenceBuilder(new Pose2d(50.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .setReversed(true)
                .splineTo(new Vector2d(12.00,-10.00), Math.toRadians(180))
                .lineTo(new Vector2d((-63.5 + inchesFromWall), 12.00))
                .addTemporalMarker(intake::in)
                .addTemporalMarker(holder::in)
                .waitSeconds(1)
                .lineTo(new Vector2d((-63.5 + inchesFromWall + 10), 12.00))
                .lineTo(new Vector2d((-63.5 + inchesFromWall), 12.00))
                .waitSeconds(1)
                .addTemporalMarker(holder::stop)
                .addTemporalMarker(intake::out)
                .setReversed(false)
                .lineTo(new Vector2d(12.00, 12.00))
                .addTemporalMarker(intake::stop)
                .splineTo(new Vector2d(49.00, Storage.currentPose.getY()), Math.toRadians(0.00))
                .build();

        redPixel = drive.trajectorySequenceBuilder(new Pose2d(50.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .setReversed(true)
                .splineTo(new Vector2d(12.00,-10.00), Math.toRadians(180))
                .lineTo(new Vector2d((-63.5 + inchesFromWall), -12.00))
                .addTemporalMarker(intake::in)
                .addTemporalMarker(holder::in)
                .waitSeconds(1)
                .lineTo(new Vector2d((-63.5 + inchesFromWall + 10), -12.00))
                .lineTo(new Vector2d((-63.5 + inchesFromWall), -12.00))
                .waitSeconds(1)
                .addTemporalMarker(holder::stop)
                .addTemporalMarker(intake::out)
                .setReversed(false)
                .lineTo(new Vector2d(12.00, -12.00))
                .addTemporalMarker(intake::stop)
                .splineTo(new Vector2d(49.00, Storage.currentPose.getY()), Math.toRadians(0.00))
                .build();

        //* strafe paths (end)
        blueStrafeLeft = drive.trajectorySequenceBuilder(new Pose2d(50.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(48.00, Storage.currentPose.getY()))
                .lineTo(new Vector2d(48.00, 60.00))
                .build();

        blueStrafeRight = drive.trajectorySequenceBuilder(new Pose2d(50.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(48.00, Storage.currentPose.getY()))
                .lineTo(new Vector2d(48.00, 12.00))
                .lineTo(new Vector2d(50.00, 12.00))
                .build();

        redStrafeLeft = drive.trajectorySequenceBuilder(new Pose2d(50.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(48.00, Storage.currentPose.getY()))
                .lineTo(new Vector2d(48.00, -12.00))
                .lineTo(new Vector2d(50.00, -12.00))
                .build();

        redStrafeRight = drive.trajectorySequenceBuilder(new Pose2d(50.00, Storage.currentPose.getY(), Math.toRadians(0.00)))
                .lineTo(new Vector2d(48.00, Storage.currentPose.getY()))
                .lineTo(new Vector2d(48.00, -60.00))
                .build();
    }

    public void findMiddle() {
        if (distanceSensor.getDistance() > 14 && distanceSensor.getDistance() < 23) {
            propPosition = PropPosition.MIDDLE;
        }
    }

    public void findFar() {
        if (distanceSensor.getDistance() > 14 && distanceSensor.getDistance() < 23) {
            propPosition = PropPosition.FAR;
        } else {
            propPosition = PropPosition.CLOSE;
        }
    }
}