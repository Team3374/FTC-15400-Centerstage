package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.ArmCommand;
import org.firstinspires.ftc.teamcode.commands.AutoLiftCommand;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Holder;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class PlacePixelCommand extends SequentialCommandGroup {
    public PlacePixelCommand(Lift lift, Arm arm, Holder holder, int height, double angle) {
        addRequirements(lift, arm, holder);

        addCommands(
                new AutoLiftCommand(lift, arm, height),
                new ArmCommand(arm, angle),
                new TimedHolderCommand(holder, 1, TimedHolderCommand.HolderDirection.OUT),
                new ArmCommand(arm, 0.075),
                new AutoLiftCommand(lift, arm, 0)
        );

    }
}
