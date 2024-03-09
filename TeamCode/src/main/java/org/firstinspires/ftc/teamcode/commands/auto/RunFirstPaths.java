package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.opmode.auto.FullAuto;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;

public class RunFirstPaths extends SequentialCommandGroup {
    public RunFirstPaths(FullAuto auto, Drive drive, FullAuto.RobotPosition robotPosition) {
        addRequirements(drive);

        addCommands(
                new InstantCommand(auto::findMiddle),
                new InstantCommand(() -> runPaths(auto, drive, robotPosition))
        );
    }

    public void runPaths(FullAuto auto, Drive drive, FullAuto.RobotPosition robotPosition) {
        if (auto.propPosition == FullAuto.PropPosition.MIDDLE) {
            switch (robotPosition) {
                case BLUE_CLOSE:
                    addCommands(new FollowTrajectorySequence(drive, auto.bcMiddle));
                    break;
                case BLUE_FAR:
                    addCommands(new FollowTrajectorySequence(drive, auto.bfMiddle));
                    break;
                case RED_CLOSE:
                    addCommands(new FollowTrajectorySequence(drive, auto.rcMiddle));
                    break;
                case RED_FAR:
                    addCommands(new FollowTrajectorySequence(drive, auto.rfMiddle));
                    break;
            }

            return;
        }

        switch (robotPosition) {
            case BLUE_CLOSE:
                addCommands(new FollowTrajectorySequence(drive, auto.bcStrafe));
                break;
            case BLUE_FAR:
                addCommands(new FollowTrajectorySequence(drive, auto.bfStrafe));
                break;
            case RED_CLOSE:
                addCommands(new FollowTrajectorySequence(drive, auto.rcStrafe));
                break;
            case RED_FAR:
                addCommands(new FollowTrajectorySequence(drive, auto.rfStrafe));
                break;
        }

        addCommands(
                new InstantCommand(auto::findFar),
                new InstantCommand(() -> runSecondary(auto, drive, robotPosition))
        );
    }

    public void runSecondary(FullAuto auto, Drive drive, FullAuto.RobotPosition robotPosition) {
        if (auto.propPosition == FullAuto.PropPosition.FAR) {
            switch (robotPosition) {
                case BLUE_CLOSE:
                    addCommands(new FollowTrajectorySequence(drive, auto.bcFar));
                    break;
                case BLUE_FAR:
                    addCommands(new FollowTrajectorySequence(drive, auto.bfFar));
                    break;
                case RED_CLOSE:
                    addCommands(new FollowTrajectorySequence(drive, auto.rcFar));
                    break;
                case RED_FAR:
                    addCommands(new FollowTrajectorySequence(drive, auto.rfFar));
                    break;
            }

            return;
        }

        switch (robotPosition) {
            case BLUE_CLOSE:
                addCommands(new FollowTrajectorySequence(drive, auto.bcClose));
                break;
            case BLUE_FAR:
                addCommands(new FollowTrajectorySequence(drive, auto.bfClose));
                break;
            case RED_CLOSE:
                addCommands(new FollowTrajectorySequence(drive, auto.rcClose));
                break;
            case RED_FAR:
                addCommands(new FollowTrajectorySequence(drive, auto.rfClose));
                break;
        }
    }
}
