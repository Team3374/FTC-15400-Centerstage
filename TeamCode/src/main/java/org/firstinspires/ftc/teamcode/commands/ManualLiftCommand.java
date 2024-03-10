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
    private final DoubleSupplier inputOneSupplier;
    private final DoubleSupplier inputTwoSupplier;

    public ManualLiftCommand(Lift lift, Arm arm, DoubleSupplier inputOneSupplier, DoubleSupplier inputTwoSupplier) {
        //* initialize subsystems/helpers and set req.
        this.lift = lift;
        this.arm = arm;

        this.inputOneSupplier = inputOneSupplier;
        this.inputTwoSupplier = inputTwoSupplier;

        addRequirements(lift);
    }

    //* set lift power based on analog input
    @Override
    public void execute() {
        double inputOne = inputOneSupplier.getAsDouble();
        double inputTwo = inputTwoSupplier.getAsDouble();

        if (Math.abs(inputTwo) > 0) {
            lift.setPower(inputTwo);
            return;
        }

        if (inputOne > 0 && lift.getPosition() <= 2400) {
            lift.setPower(inputOne);
        } else if (inputOne < 0 && (lift.getPosition() >= 1000 || (lift.getPosition() > 0 && arm.isDown()))) {
            lift.setPower(inputOne);
        } else {
            lift.setPower(0);
        }

        if (lift.getBatteryVoltage() < 10.5) {
            lift.resetPosition();
        }
    }

    //* stop lifts when command ends
    @Override
    public void end(boolean interrupted) {
        lift.setPower(0);
    }
}
