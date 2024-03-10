package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.subsystems.drive.DriveBase;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@SuppressWarnings({"unused", "FieldCanBeLocal"})
@Autonomous(name="Drive to AprilTag", group = "Test")
public class pathToAprilTag extends LinearOpMode {
    final double DISTANCE = 12.0; //camera distance to april tag
    private static final int DESIRED_TAG_ID = 2; //set to -1 for any tag
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //create "robot"
        DriveBase drive = new DriveBase(hardwareMap);

        boolean targetFound;

        boolean followingPath = false;

        Pose2d startingPose = new Pose2d(0, 0, 0);

        initAprilTag();

        setManualExposure();

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            targetFound = false;
            desiredTag = null;


            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null)
                        && ((DESIRED_TAG_ID >= 0) || (detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("X", "%5.1f inches", desiredTag.ftcPose.x);
                telemetry.addData("Y", "%5.1f inches", desiredTag.ftcPose.y);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
            } else {
                telemetry.addData(">", "No target found\n");
            }

            if (!followingPath && targetFound) {
                TrajectorySequence splineToAprilTag = drive.trajectorySequenceBuilder(startingPose)
                        .splineTo(new Vector2d(desiredTag.ftcPose.y, desiredTag.ftcPose.x), Math.toRadians(desiredTag.ftcPose.bearing))
                        .build();

                drive.followTrajectorySequenceAsync(splineToAprilTag);

                followingPath = true;
            } else if (followingPath && !targetFound) {
                drive.breakFollowing();
                followingPath = false;
            }

            sleep(10);
        }
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTag)
                .build();
    }

    private void setManualExposure() {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            sleep(20);
        }
    }
}
