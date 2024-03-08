package org.firstinspires.ftc.teamcode.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Holder;

public class TimedHolderCommand extends CommandBase {
    private Holder holder;

    private double targetTime;
    private ElapsedTime timer;

    public enum Direction {
        IN,
        OUT
    }
    public Direction direction;

    public TimedHolderCommand(Holder holder, double targetTime, Direction direction) {
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
        if (direction == Direction.IN) {
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
