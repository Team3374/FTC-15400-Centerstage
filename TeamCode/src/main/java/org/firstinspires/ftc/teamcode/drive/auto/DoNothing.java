package org.firstinspires.ftc.teamcode.drive.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.drive.Storage;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="Do Nothing")
public class DoNothing extends LinearOpMode {

    private String selectedPosition = null;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot drive = new Robot(hardwareMap);

        telemetry.addLine("Please choose from the following positions:");
        telemetry.addData("A", "Blue Park (Close)");
        telemetry.addData("B", "Blue Park (Far)");
        telemetry.addData("X", "Red Park (Close)");
        telemetry.addData("Y", "Red Park (Far)");
        telemetry.update();

        while(opModeInInit()) {
            if (gamepad1.a) {
                selectedPosition = "bc";

                drive.setPoseEstimate(new Pose2d(12, 63.50, Math.toRadians(-90.00)));

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following positions:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Position: Blue Park (Close)");
                telemetry.update();
            } else if (gamepad1.b) {
                selectedPosition = "bf";

                drive.setPoseEstimate(new Pose2d(-36, 63.50, Math.toRadians(-90.00)));

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following positions:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Position: Blue Park (Far)");
                telemetry.update();
            } else if (gamepad1.x) {
                selectedPosition = "rc";

                drive.setPoseEstimate(new Pose2d(12, -63.50, Math.toRadians(90.00)));

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following positions:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Position: Red Park (Close)");
                telemetry.update();
            } else if (gamepad1.y) {
                selectedPosition = "rf";

                drive.setPoseEstimate(new Pose2d(-36, -63.50, Math.toRadians(90.00)));

                telemetry.clearAll();
                telemetry.addLine("Please choose from the following positions:");
                telemetry.addData("A", "Blue Park (Close)");
                telemetry.addData("B", "Blue Park (Far)");
                telemetry.addData("X", "Red Park (Close)");
                telemetry.addData("Y", "Red Park (Far)");
                telemetry.addLine();
                telemetry.addLine("Selected Position: Red Park (Far)");
                telemetry.update();
            }
        }

        waitForStart();

        switch (selectedPosition) {
            case "bc":
                Storage.currentColor = "blue";
                break;
            case "bf":
                Storage.currentColor = "blue";
                break;
            case "rc":
                Storage.currentColor = "red";
                break;
            case "rf":
                Storage.currentColor = "blue";
                break;
        }

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