package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class AutoLiftCommand extends CommandBase {
    //* create subsystems
    private final Lift lift;
    private final Arm arm;

    //* create helper vars
    private final int targetPosition;

    public AutoLiftCommand(Lift lift, Arm arm, int targetPosition) {
        //* initialize subsystems/helpers and set req.
        this.lift = lift;
        this.arm = arm;

        this.targetPosition = targetPosition;

        addRequirements(lift, arm);
    }

    //* set lift position based on target
    @Override
    public void execute() {
        if (targetPosition > 0) {
            lift.setPosition(targetPosition);
        } else if (targetPosition == 0 && (lift.getPosition() > 1000 || arm.isDown())) {
            lift.setPosition(0);
            arm.down();
        }
    }

    //* run once
    @Override
    public boolean isFinished() {
        if (targetPosition > 0) {
            return lift.getPosition() >= targetPosition;
        } else {
            return lift.getPosition() <= targetPosition;
        }
    }
}
