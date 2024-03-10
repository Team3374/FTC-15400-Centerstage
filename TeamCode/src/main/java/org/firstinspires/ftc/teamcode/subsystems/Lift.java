package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Lift extends SubsystemBase {
    //* create lift motors and voltage sensor
    private final DcMotorEx leftLiftMotor;
    private final DcMotorEx rightLiftMotor;

    public Lift(HardwareMap hardwareMap) {
        //* initialize shooter motors and sensor
        leftLiftMotor = hardwareMap.get(DcMotorEx.class, "leftLiftMotor");
        rightLiftMotor = hardwareMap.get(DcMotorEx.class, "rightLiftMotor");

        leftLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightLiftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //* set lift to specified position
    public void setPosition(int targetPosition) {
        leftLiftMotor.setTargetPosition(targetPosition);
        rightLiftMotor.setTargetPosition(targetPosition);

        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftLiftMotor.setPower(1);
        rightLiftMotor.setPower(1);
    }

    //* set lift power (volts)
    public void setPower(double power) {
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(power);
    }

    //* resets lift position
    public void resetPosition() {
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //* returns the max. position of lifts
    public int getPosition() {
        return  Math.max(leftLiftMotor.getCurrentPosition(), rightLiftMotor.getCurrentPosition());
    }

    //* returns max motor current
    public double getCurrent() {
        return Math.max(leftLiftMotor.getCurrent(CurrentUnit.AMPS), rightLiftMotor.getCurrent(CurrentUnit.AMPS));
    }
}
