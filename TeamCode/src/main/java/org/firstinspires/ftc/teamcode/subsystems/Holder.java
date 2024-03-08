package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Holder extends SubsystemBase {
    //* create holder servo
    private final CRServo holderServo;

    public Holder(HardwareMap hardwareMap) {
        //* initialize holder servo
        holderServo = new CRServo(hardwareMap, "holderServo");
    }

    //* set holder servo power (volts)
    public void setPower(double power) {
        holderServo.set(power);
    }

    //* shorter methods for controlling holder power
    public void in() {holderServo.set(1);}
    public void out() {holderServo.set(-1);}
    public void stop() {holderServo.set(0);}
}
