package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.function.DoubleSupplier;

public class AutoLiftCommand extends CommandBase {
    //* create subsystems
    private final Lift lift;
    private final Arm arm;

    //* create helper vars
    private final int targetPosition;

    private final DoubleSupplier triggerOneSupplier;
    private final DoubleSupplier triggerTwoSupplier;

    public AutoLiftCommand(Lift lift, Arm arm, int targetPosition, DoubleSupplier triggerOneSupplier, DoubleSupplier triggerTwoSupplier) {
        //* initialize subsystems/helpers and set req.
        this.lift = lift;
        this.arm = arm;

        this.targetPosition = targetPosition;

        this.triggerOneSupplier = triggerOneSupplier;
        this.triggerTwoSupplier = triggerTwoSupplier;

        addRequirements(lift, arm);
    }

    public AutoLiftCommand(Lift lift, Arm arm, int targetPosition) {
        this(lift, arm, targetPosition, () -> 0, () -> 0);
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
        return Math.abs(triggerOneSupplier.getAsDouble() + triggerTwoSupplier.getAsDouble()) > 0.05;
    }
}
