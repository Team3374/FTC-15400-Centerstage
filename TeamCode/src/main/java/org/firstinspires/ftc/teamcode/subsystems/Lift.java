package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Lift extends SubsystemBase {
    //* create lift motors and voltage sensor
    private final MotorEx leftLiftMotor;
    private final MotorEx rightLiftMotor;

    private final VoltageSensor batteryVoltageSensor;

    public Lift(HardwareMap hardwareMap) {
        //* initialize shooter motors and sensor
        leftLiftMotor = new MotorEx(hardwareMap, "leftLiftMotor");
        rightLiftMotor = new MotorEx(hardwareMap, "rightLiftMotor");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftLiftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        leftLiftMotor.stopAndResetEncoder();
        rightLiftMotor.stopAndResetEncoder();
    }

    //* set lift to specified position
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

    //* resets lift position
    public void resetPosition() {
        leftLiftMotor.stopAndResetEncoder();
        rightLiftMotor.stopAndResetEncoder();
    }

    //* returns the max. position of lifts
    public int getPosition() {
        return  Math.max(leftLiftMotor.getCurrentPosition(), rightLiftMotor.getCurrentPosition());
    }

    //* returns battery voltage
    public double getBatteryVoltage() {
        return batteryVoltageSensor.getVoltage();
    }
}
