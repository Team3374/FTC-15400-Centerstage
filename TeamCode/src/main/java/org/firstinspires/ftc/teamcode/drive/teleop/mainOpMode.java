package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Robot;

@TeleOp
public class mainOpMode extends LinearOpMode {

    //* instance variables (physical components):
    private ElapsedTime runtime = new ElapsedTime();
    private boolean climberUp = false;
    private boolean yHeld = false;
    private boolean xHeld = false;
    private String lastPressed = "x";

    @Override
    public void runOpMode() throws InterruptedException {

        //Import Road Runner Drivetrain
        Robot drive = new Robot(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        switch (Storage.currentColor) {
//            case "blue":
//                drive.setPoseEstimate(new Pose2d(Storage.currentPose.getX(), Storage.currentPose.getY(), Math.toRadians(-90)));
//                break;
//            case "red":
//                drive.setPoseEstimate(new Pose2d(Storage.currentPose.getX(), Storage.currentPose.getY(), Math.toRadians(90)));
//                break;
//        }

//        drive.setPoseEstimate(Storage.currentPose);

        //* Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
//            //* assign drive commands (field centric)
//            Pose2d poseEstimate = drive.getPoseEstimate();
//
//            Vector2d input = new Vector2d(
//                    -gamepad1.left_stick_y,
//                    -gamepad1.left_stick_x
//            ).rotated(-poseEstimate.getHeading());
//
//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            input.getX(),
//                            input.getY(),
//                            -gamepad1.right_stick_x
//                    )
//            );
//
//            drive.update();

            //* assign drive commands (robot centric)
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            //* assign intake/holder commands
            if (gamepad1.a && !gamepad1.right_bumper && !gamepad1.left_bumper) {
                drive.intakeMotor.setPower(1);
                drive.holderServo.setPower(1);
            } else if (gamepad1.b) {
                drive.intakeMotor.setPower(-1);
                drive.holderServo.setPower(-1);
            } else {
                drive.intakeMotor.setPower(0);
                drive.holderServo.setPower(0);
            }

            //* automatic lift commands
            if (gamepad1.right_bumper) {
                drive.leftLiftMotor.setTargetPosition(2150);
                drive.rightLiftMotor.setTargetPosition(2150);

                drive.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive.leftLiftMotor.setPower(1);
                drive.rightLiftMotor.setPower(1);
            } else if (gamepad1.left_bumper) {
                drive.leftLiftMotor.setTargetPosition(0);
                drive.rightLiftMotor.setTargetPosition(0);

                drive.leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                drive.leftLiftMotor.setPower(1);
                drive.rightLiftMotor.setPower(1);
            }

            //* manual lift commands/soft-stops
            if (gamepad1.right_trigger > 0.05 && drive.leftLiftMotor.getCurrentPosition() <= 2150 && drive.rightLiftMotor.getCurrentPosition() <= 2150) {
                drive.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                drive.leftLiftMotor.setPower(gamepad1.right_trigger);
                drive.rightLiftMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.05 && drive.leftLiftMotor.getCurrentPosition() >= 0 && drive.rightLiftMotor.getCurrentPosition() >= 0) {
                drive.leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                drive.rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                drive.leftLiftMotor.setPower(-gamepad1.left_trigger);
                drive.rightLiftMotor.setPower(-gamepad1.left_trigger);
            } else if (drive.leftLiftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                drive.leftLiftMotor.setPower(0);
                drive.rightLiftMotor.setPower(0);
            }

            //* set arm position

            if (gamepad1.x && !xHeld) {
                climberUp = !climberUp;
                xHeld = true;
                lastPressed = "x";
            } else if (gamepad1.y && !yHeld) {
                climberUp = !climberUp;
                yHeld = true;
                lastPressed = "y";
            } else if (climberUp && lastPressed.equals("x")) {
                drive.leftArmServo.setPosition(0.45);
                drive.rightArmServo.setPosition(0.45);
            } else if (climberUp && lastPressed.equals("y")) {
                drive.leftArmServo.setPosition(0.55);
                drive.rightArmServo.setPosition(0.55);
            } else if (!climberUp) {
                drive.leftArmServo.setPosition(0.1);
                drive.rightArmServo.setPosition(0.1);
            }

            if (!gamepad1.y) {
                yHeld = false;
            }

            if (!gamepad1.x) {
                xHeld = false;
            }

            //* assign airplane commands
            if (gamepad1.dpad_up) {
                drive.airplaneServo.setPosition(1);
            } else if (gamepad1.dpad_down) {
                drive.airplaneServo.setPosition(0);
            }

            //* telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addLine();
            telemetry.addData("Left Lift Encoder", drive.leftLiftMotor.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", drive.rightLiftMotor.getCurrentPosition());
            telemetry.addData("Intake Encoder", drive.intakeMotor.getCurrentPosition());
            telemetry.addData("Holder Speed", drive.holderServo.getPower());
            telemetry.addData("Left Arm Servo Position", drive.leftArmServo.getPosition());
            telemetry.addData("Right Arm Servo Position", drive.rightArmServo.getPosition());
            telemetry.addLine();
            telemetry.addData("Airplane Servo Position", drive.airplaneServo.getPosition());
            telemetry.addLine();
            telemetry.addData("Distance", drive.distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}