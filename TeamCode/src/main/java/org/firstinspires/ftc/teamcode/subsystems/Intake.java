package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends SubsystemBase {
    //* create intake motor
    private final Motor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        //* initialize intake motor
        intakeMotor = new Motor(hardwareMap, "intakeMotor");
    }

    //* set the power of the intake
    public void setPower(double power) {
        intakeMotor.set(power);
    }

    //* shorter methods for controlling intake power
    public void in() {intakeMotor.set(1);}
    public void out() {intakeMotor.set(-1);}
    public void stop() {intakeMotor.set(0);}
}
