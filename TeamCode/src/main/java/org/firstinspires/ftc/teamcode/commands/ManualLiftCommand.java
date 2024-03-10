package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import java.util.function.DoubleSupplier;

public class ManualLiftCommand extends CommandBase {
    //* create subsystems
    private final Lift lift;
    private final Arm arm;

    //* create helper vars
    private final boolean hasLimits;
    private final GamepadEx gamepad;

    public ManualLiftCommand(Lift lift, Arm arm, GamepadEx gamepad, boolean hasLimits) {
        //* initialize subsystems/helpers and set req.
        this.lift = lift;
        this.arm = arm;

        this.hasLimits = hasLimits;
        this.gamepad = gamepad;

        addRequirements(lift);
    }

    //* set lift power based on analog input
    @Override
    public void execute() {
        double rightTrigger = gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double leftTrigger = gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double input = rightTrigger - leftTrigger;

        if (input > 0 && lift.getPosition() <= 2400) {
            lift.setPower(input);
        } else if (input < 0 && (lift.getPosition() >= 1500 || (lift.getPosition() > 0 && arm.isDown()))) {
            lift.setPower(input);
        } else if (!hasLimits && Math.abs(input) > 0.05) {
            lift.setPower(input);
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
