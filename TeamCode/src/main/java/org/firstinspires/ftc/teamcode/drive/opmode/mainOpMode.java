package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robot;

@TeleOp
public class mainOpMode extends LinearOpMode {

    //* instance variables (physical components):
    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {

        //Import Road Runner Drivetrain
        Robot drive = new Robot(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //* Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            //* assign drive commands (field centric)
            Pose2d poseEstimate = drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            gamepad1.right_stick_x
                    )
            );

            drive.update();

            //* assign intake commands
            if (gamepad1.a) {
                drive.intakeMotor.setPower(1);
            } else if (gamepad1.b) {
                drive.intakeMotor.setPower(-1);
            } else {
                drive.intakeMotor.setPower(0);
            }

            //* assign lift commands/soft-stops
            if (gamepad1.right_trigger > 0.05 && drive.leftLiftMotor.getCurrentPosition() <= 2167 && drive.rightLiftMotor.getCurrentPosition() <= 2167) {
                drive.leftLiftMotor.setPower(gamepad1.right_trigger);
                drive.rightLiftMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.05 && drive.leftLiftMotor.getCurrentPosition() >= 0 && drive.rightLiftMotor.getCurrentPosition() >= 0) {
                drive.leftLiftMotor.setPower(-gamepad1.left_trigger);
                drive.rightLiftMotor.setPower(-gamepad1.left_trigger);
            } else {
                drive.leftLiftMotor.setPower(0);
                drive.rightLiftMotor.setPower(0);
            }

            //* set arm position based on lift height
            drive.leftArmServo.setPosition(drive.leftLiftMotor.getCurrentPosition()*(1.0/2167.0));
            drive.rightArmServo.setPosition(drive.rightLiftMotor.getCurrentPosition()*(1.0/2167.0));

            //* assign airplane commands
            if (gamepad1.x) {
                drive.airplaneServo.setPosition(1);
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
            telemetry.update();
        }
    }
}
