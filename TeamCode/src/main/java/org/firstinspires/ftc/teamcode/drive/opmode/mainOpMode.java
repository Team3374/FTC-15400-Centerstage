package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
public class mainOpMode extends LinearOpMode {

    //* instance variables (physical components):
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftLiftMotor = null;
    private DcMotor rightLiftMotor = null;

    private DcMotor intakeMotor = null;
    private DcMotor hopperMotor = null;
    private CRServo holderServo = null;

    private Servo airplaneServo = null;
    //TODO: UNCOMMENT ALL COMMENTED COMPONENTS ONCE CONFIGURED

    @Override
    public void runOpMode() throws InterruptedException {

        //Import Road Runner Drivetrain
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //* add control/expansion hub hardware map (configuaration) here:

        leftLiftMotor = hardwareMap.get(DcMotor.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "rightLiftMotor");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//        hopperMotor = hardwareMap.get(DcMotor.class, "hopperMotor");
//        holderServo = hardwareMap.get(CRServo.class, "holderServo");
//
//        airplaneServo = hardwareMap.get(Servo.class, "airplaneServo");

        //* set motor direction:
        leftLiftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightLiftMotor.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
//        hopperMotor.setDirection(DcMotor.Direction.FORWARD);
//        holderServo.setDirection(CRServo.Direction.FORWARD);
//
//        airplaneServo.setDirection(Servo.Direction.FORWARD);

        //* reset lift encoders/set to brake mode
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //! ONLY WORKS WITH DcMotorEx
//        leftLiftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1, 0, 0, 0));
//        rightLiftMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(1, 0, 0, 0));

        leftLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //* reset airplane servo position
        //airplaneServo.setPosition(0.0);

        //* Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {
            //* assign drive commands
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            //* assign intake commands
            if (gamepad1.a) {
                intakeMotor.setPower(1);
            } else if (gamepad1.b) {
                intakeMotor.setPower(-1);
            } else {
                intakeMotor.setPower(0);
            }

            //* assign lift commands/soft-stops
            if (gamepad1.right_trigger > 0.05 && leftLiftMotor.getCurrentPosition() <= 2167 && rightLiftMotor.getCurrentPosition() <= 2167) {
                leftLiftMotor.setPower(gamepad1.right_trigger);
                rightLiftMotor.setPower(gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.05 && leftLiftMotor.getCurrentPosition() >= 0 && rightLiftMotor.getCurrentPosition() >= 0) {
                leftLiftMotor.setPower(-gamepad1.left_trigger);
                rightLiftMotor.setPower(-gamepad1.left_trigger);
            } else {
                leftLiftMotor.setPower(0);
                rightLiftMotor.setPower(0);
            }

//            //* assign hopper/holder commands
//            if (gamepad1.right_bumper) {
//                hopperMotor.setPower(1);
//                holderServo.setPower(1);
//            } else if (gamepad1.left_bumper) {
//                //TODO: ASK ABOUT REVERSE HOPPER!!!
//                hopperMotor.setPower(-1);
//                holderServo.setPower(-1);
//            } else {
//                hopperMotor.setPower(0);
//                holderServo.setPower(0);
//            }
//
//            //* assign airplane commands
//            if (gamepad1.x) {
//                airplaneServo.setPosition(1);
//            }

            //* telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addLine();
            telemetry.addData("Left Lift Encoder", leftLiftMotor.getCurrentPosition());
            telemetry.addData("Right Lift Encoder", rightLiftMotor.getCurrentPosition());
//            telemetry.addData("Intake Encoder", intakeMotor.getCurrentPosition());
//            telemetry.addData("Hopper Encoder", hopperMotor.getCurrentPosition());
//            telemetry.addData("Holder Speed", holderServo.getPower());
//            telemetry.addLine();
//            telemetry.addData("Airplane Servo Position", airplaneServo.getPosition());
            telemetry.update();
        }
    }
}
