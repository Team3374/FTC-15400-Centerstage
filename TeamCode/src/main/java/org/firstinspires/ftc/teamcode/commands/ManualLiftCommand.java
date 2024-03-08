package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.function.DoubleSupplier;

public class ManualLiftCommand extends CommandBase {
    //* create subsystems
    private final Lift lift;
    private final Arm arm;

    //* create helper vars
    private final boolean hasLimits;
    private final DoubleSupplier input;

    public ManualLiftCommand(Lift lift, Arm arm, DoubleSupplier input, boolean hasLimits) {
        //* initialize subsystems/helpers and set req.
        this.lift = lift;
        this.arm = arm;

        this.hasLimits = hasLimits;
        this.input = input;

        addRequirements(lift);
    }

    //* set lift power based on analog input
    @Override
    public void execute() {
        if (input.getAsDouble() > 0 && lift.getPosition() <= 2400) {
            lift.setPower(input.getAsDouble());
        } else if (input.getAsDouble() < 0 && (lift.getPosition() >= 1500 || (lift.getPosition() > 0 && arm.isDown()))) {
            lift.setPower(input.getAsDouble());
        } else if (!hasLimits && Math.abs(input.getAsDouble()) > 0.05) {
            lift.setPower(input.getAsDouble());
        } else {
            lift.setPower(0);
        }
    }

    //* stop lifts when command ends
    @Override
    public void end(boolean interrupted) {
        lift.setPower(0);
    }
}
