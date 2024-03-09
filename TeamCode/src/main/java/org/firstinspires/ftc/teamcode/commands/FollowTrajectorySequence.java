package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

public class FollowTrajectorySequence extends CommandBase {
    //* create subsystems
    private final Drive drive;

    //* create helper vars
    private final TrajectorySequence trajectorySequence;

    public FollowTrajectorySequence(Drive drive, TrajectorySequence trajectorySequence) {
        //* initialize subsystems/helpers and set req.
        this.drive = drive;
        this.trajectorySequence = trajectorySequence;

        addRequirements(drive);
    }

    //* follow trajectory sequence
    @Override
    public void initialize() {
        drive.followTrajectorySequence(trajectorySequence);
    }

    //* end when sequence finished
    @Override
    public boolean isFinished() {
        return !drive.isBusy();
    }
};
