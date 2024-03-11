package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drive.DriveBase;
import org.firstinspires.ftc.teamcode.util.Storage;

@SuppressWarnings("unused")
@Autonomous(name="Do Nothing")
public class DoNothing extends LinearOpMode {

    private enum SelectedPosition {
        BLUE_CLOSE,
        BLUE_FAR,
        RED_CLOSE,
        RED_FAR
    }
    private SelectedPosition selectedPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveBase drive = new DriveBase(hardwareMap);

        while(opModeInInit()) {
            telemetry.addLine("Please choose from the following positions:");
            telemetry.addData("A", "Blue Park (Close)");
            telemetry.addData("B", "Blue Park (Far)");
            telemetry.addData("X", "Red Park (Close)");
            telemetry.addData("Y", "Red Park (Far)");
            telemetry.addLine();

            if (gamepad1.a) {
                selectedPosition = SelectedPosition.BLUE_CLOSE;
                drive.setPoseEstimate(new Pose2d(12, 64.50, Math.toRadians(-90.00)));

                telemetry.addLine("Selected Position: Blue Park (Close)");
            } else if (gamepad1.b) {
                selectedPosition = SelectedPosition.BLUE_FAR;
                drive.setPoseEstimate(new Pose2d(-36, 64.50, Math.toRadians(-90.00)));

                telemetry.addLine("Selected Position: Blue Park (Far)");
            } else if (gamepad1.x) {
                selectedPosition = SelectedPosition.RED_CLOSE;
                drive.setPoseEstimate(new Pose2d(12, -64.50, Math.toRadians(90.00)));

                telemetry.addLine("Selected Position: Red Park (Close)");
            } else if (gamepad1.y) {
                selectedPosition = SelectedPosition.RED_FAR;
                drive.setPoseEstimate(new Pose2d(-36, -64.50, Math.toRadians(90.00)));

                telemetry.addLine("Selected Position: Red Park (Far)");
            }

            telemetry.update();
        }

        waitForStart();

        Storage.currentColor = Storage.CurrentColor.NONE;

        Storage.currentPose = drive.getPoseEstimate();
        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.clearAll();
        telemetry.addData("Current Position", selectedPosition);
        telemetry.addLine();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

    }
}