package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Storage;

import java.util.List;

@SuppressWarnings("unused")
public class Drive extends SubsystemBase {
    private final DriveBase drive;
    private final boolean fieldCentric;

    public Drive(DriveBase drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public void setPoseEstimate(Pose2d pose) {
        drive.setPoseEstimate(pose);
    }

    public void update() {
        drive.update();
    }

    public void updatePoseEstimate() {
        drive.updatePoseEstimate();
    }

    public void drive(double leftY, double leftX, double rightX) {
        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(-leftY, -leftX).rotated(
                fieldCentric ? -poseEstimate.getHeading() : 0
        );

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -rightX
                )
        );
    }

    public void driveWithLimits(double leftY, double leftX, double rightX) {
        Pose2d poseEstimate = getPoseEstimate();

        Vector2d input = new Vector2d(-leftY, -leftX).rotated(
                fieldCentric ? -poseEstimate.getHeading() : 0
        );

        if (poseEstimate.getY() < -28 && input.getX() > -0.75 && Storage.currentColor == Storage.CurrentColor.RED) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * 0.3,
                            input.getY() * 0.5,
                            -rightX * 0.3
                    )
            );
        } else if (poseEstimate.getY() > 28 && input.getX() > -0.75 && Storage.currentColor == Storage.CurrentColor.BLUE) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * 0.3,
                            input.getY() * 0.5,
                            -rightX * 0.3
                    )
            );
        } else {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -rightX
                    )
            );
        }
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        drive.followTrajectorySequence(trajectorySequence);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    public void turn(double radians) {
        drive.turnAsync(radians);
    }

    public List<Double> getWheelVelocities() {
        return drive.getWheelVelocities();
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public Pose2d getPoseVelocity() {
        return drive.getPoseVelocity();
    }
}
