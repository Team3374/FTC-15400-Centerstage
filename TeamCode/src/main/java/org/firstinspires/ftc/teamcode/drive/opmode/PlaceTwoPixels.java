package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Place Two Pixels (No Vision)")
public class PlaceTwoPixels extends LinearOpMode {

    private String selectedAuto = null;

    private DcMotor leftLiftMotor = null;
    private DcMotor rightLiftMotor = null;
    private CRServo holderServo = null;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence bftPath = drive.trajectorySequenceBuilder(new Pose2d(-36.52, 63.50, Math.toRadians(-90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, 38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, 38.41))
                .lineToSplineHeading(new Pose2d(48.79, 37.84, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> {
                    try {
                        placePixel();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .build();

        TrajectorySequence bctPath = drive.trajectorySequenceBuilder(new Pose2d(11.98, 63.50, Math.toRadians(270.00)))
                .splineTo(new Vector2d(48.79, 37.84), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    try {
                        placePixel();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .build();


        TrajectorySequence rftPath = drive.trajectorySequenceBuilder(new Pose2d(-36.52, -63.50, Math.toRadians(90.00)))
                .splineToSplineHeading(new Pose2d(-29.72, -38.03, Math.toRadians(0.00)), Math.toRadians(0.00))
                .lineTo(new Vector2d(22.36, -38.41))
                .lineToSplineHeading(new Pose2d(48.79, -37.84, Math.toRadians(0.00)))
                .addDisplacementMarker(() -> {
                    try {
                        placePixel();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .build();

        TrajectorySequence rctPath = drive.trajectorySequenceBuilder(new Pose2d(11.98, -63.50, Math.toRadians(90.00)))
                .splineTo(new Vector2d(48.79, -37.84), Math.toRadians(0.00))
                .addDisplacementMarker(() -> {
                    try {
                        placePixel();
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .build();

        //* add control/expansion hub hardware map (configuaration) here:

        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");

//        holderServo = hardwareMap.get(CRServo.class, "holderServo");

        //* set motor direction:
        leftLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightLiftMotor.setDirection(DcMotor.Direction.FORWARD);

//        holderServo.setDirection(CRServo.Direction.FORWARD);

        //* reset lift encoders/set to brake mode
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

    public void placePixel() throws InterruptedException {


    }
}
