package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Holder;

public class TimedHolderCommand extends CommandBase {
    //* create subsystems
    private Holder holder;

    //* create helper vars
    private double targetTime;
    private ElapsedTime timer;

    public enum HolderDirection {
        IN,
        OUT
    }
    private final HolderDirection direction;

    public TimedHolderCommand(Holder holder, double targetTime, HolderDirection direction) {
        //* initialize subsystems/helpers and set req.
        this.holder = holder;
        this.targetTime = targetTime;

        this.direction = direction;
        timer = new ElapsedTime();

        addRequirements(holder);
    }

    @Override
    public void initialize() {
        timer.reset();
    }

    @Override
    public void execute() {
        if (direction == HolderDirection.IN) {
            holder.in();
        } else {
            holder.out();
        }
    }

    @Override
    public void end(boolean interrupted) {
        holder.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= targetTime;
    }
}
