package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Holder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeCommand extends CommandBase {
    //* create subsystems
    private final Intake intake;
    private final Holder holder;

    //* create helper vars
    private final boolean isIntaking;

    public IntakeCommand(Intake intake, Holder holder, boolean isIntaking) {
        //* initialize subsystems/helpers and set req.
       this.intake = intake;
       this.holder = holder;

       this.isIntaking = isIntaking;

       addRequirements(intake, holder);
    }

    //* intake/outtake while running
    @Override
    public void execute() {
        if (isIntaking) {
            intake.in();
            holder.in();
        } else {
            intake.out();
            holder.out();
        }
    }

    //* stop intake when command ends
    @Override
    public void end(boolean interrupted) {
        intake.stop();
        holder.stop();
    }
}
