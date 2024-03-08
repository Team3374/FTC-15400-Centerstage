package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ArmCommand extends CommandBase {
    //* create subsystems
    private final Arm arm;

    //* create helper vars
    private final double targetPosition;

    public ArmCommand(Arm arm, double targetPosition) {
        //* initialize subsystems/helpers and set req.
        this.arm = arm;

        this.targetPosition = targetPosition;

        addRequirements(arm);
    }

    //* set arm position based on target
    @Override
    public void execute() {
        if (arm.getTargetPosition() == 0) {
            arm.setPosition(targetPosition);
        } else {
            arm.setPosition(0);
        }
    }

    //* for up, check analog. for down, check sensor
    @Override
    public boolean isFinished() {
        if (targetPosition == 0) {
            return arm.isDown();
        } else {
            return arm.getPosition() >= targetPosition;
        }
    }
}
