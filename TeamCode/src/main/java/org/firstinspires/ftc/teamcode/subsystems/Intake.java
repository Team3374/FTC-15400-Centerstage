package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    //* create intake motor
    private final DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        //* initialize intake motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        //* set the direction of the intake motor
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    //* set the power of the intake
    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    //* shorter methods for controlling intake power
    public void in() {intakeMotor.setPower(1);}
    public void out() {intakeMotor.setPower(-1);}
    public void stop() {intakeMotor.setPower(0);}
}
