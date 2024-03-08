package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift extends SubsystemBase {
    //* create lift motors
    private final Motor leftLiftMotor;
    private final Motor rightLiftMotor;

    public Lift(HardwareMap hardwareMap) {
        //* initialize shooter motors
        leftLiftMotor = new Motor(hardwareMap, "leftLiftMotor");
        rightLiftMotor = new Motor(hardwareMap, "rightLiftMotor");

        leftLiftMotor.setInverted(true);
        rightLiftMotor.setInverted(false);

        leftLiftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftLiftMotor.stopAndResetEncoder();
        rightLiftMotor.stopAndResetEncoder();
    }

    //* set lift to speified position
    public void setPosition(int targetPosition) {
        leftLiftMotor.setTargetPosition(targetPosition);
        rightLiftMotor.setTargetPosition(targetPosition);

        leftLiftMotor.setRunMode(Motor.RunMode.PositionControl);
        rightLiftMotor.setRunMode(Motor.RunMode.PositionControl);

        leftLiftMotor.set(1);
        rightLiftMotor.set(1);
    }

    //* set lift power (volts)
    public void setPower(double power) {
        leftLiftMotor.setRunMode(Motor.RunMode.RawPower);
        rightLiftMotor.setRunMode(Motor.RunMode.RawPower);

        leftLiftMotor.set(power);
        rightLiftMotor.set(power);
    }

    //* returns the max. position of lifts
    public int getPosition() {
        return  Math.max(leftLiftMotor.getCurrentPosition(), rightLiftMotor.getCurrentPosition());
    }
}
