package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Storage;

import java.util.List;

public class Drive extends SubsystemBase {

    private final DriveBase drive;
    private final boolean fieldCentric;

    public Drive(DriveBase drive, boolean isFieldCentric) {
        this.drive = drive;
        fieldCentric = isFieldCentric;
    }

    public void setMode(DcMotor.RunMode mode) {
        drive.setMode(mode);
    }

    public void setPIDFCoefficients(DcMotor.RunMode mode, PIDFCoefficients coefficients) {
        drive.setPIDFCoefficients(mode, coefficients);
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

        if (poseEstimate.getY() < -28 && input.getX() > -0.75 && Storage.currentColor.equals("red")) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * 0.3,
                            input.getY() * 0.5,
                            -rightX * 0.3
                    )
            );
        } else if (poseEstimate.getY() > 28 && input.getX() > -0.75 && Storage.currentColor.equals("blue")) {
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

    public void setDrivePower(Pose2d drivePower) {
        drive.setDrivePower(drivePower);
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return drive.trajectoryBuilder(startPose);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return drive.trajectoryBuilder(startPose, reversed);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose);
    }

    public void followTrajectory(Trajectory trajectory) {
        drive.followTrajectoryAsync(trajectory);
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

    public Localizer getLocalizer() {
        return drive.getLocalizer();
    }

}
