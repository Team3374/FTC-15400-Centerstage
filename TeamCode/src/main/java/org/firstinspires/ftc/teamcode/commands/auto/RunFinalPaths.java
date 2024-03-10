package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.opmode.auto.FullAuto;
import org.firstinspires.ftc.teamcode.commands.FollowTrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Holder;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.util.Storage;

public class RunFinalPaths extends SequentialCommandGroup {
    public RunFinalPaths(FullAuto auto, Drive drive, Lift lift, Arm arm, Holder holder, int autoIndex) {
        addRequirements(drive, lift, arm, holder);

        if (Storage.currentPose.getY() > 0) {
            for (int count = 0; count < autoIndex; count++) {
                addCommands(new FollowTrajectorySequence(drive, auto.bluePixel));
                addCommands(new PlacePixelCommand(lift, arm, holder, 1700, 0.5));
            }

            switch (auto.strafeSelection) {
                case LEFT:
                    addCommands(new FollowTrajectorySequence(drive, auto.blueStrafeLeft));
                    break;
                case RIGHT:
                    addCommands(new FollowTrajectorySequence(drive, auto.blueStrafeRight));
                default:
                    break;
            }
        } else {
            for (int count = 0; count < autoIndex; count++) {
                addCommands(new FollowTrajectorySequence(drive, auto.redPixel));
                addCommands(new PlacePixelCommand(lift, arm, holder, 1200, 0.5));
            }

            switch (auto.strafeSelection) {
                case LEFT:
                    addCommands(new FollowTrajectorySequence(drive, auto.redStrafeLeft));
                    break;
                case RIGHT:
                    addCommands(new FollowTrajectorySequence(drive, auto.redStrafeRight));
                    break;
                default:
                    break;
            }
        }
    }
}
