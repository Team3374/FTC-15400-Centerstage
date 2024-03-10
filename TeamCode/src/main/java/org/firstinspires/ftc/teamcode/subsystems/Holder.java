package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Holder extends SubsystemBase {
    //* create holder servo
    private final CRServo holderServo;

    public Holder(HardwareMap hardwareMap) {
        //* initialize holder servo
        holderServo = hardwareMap.get(CRServo.class, "holderServo");

        holderServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //* set holder servo power (volts)
    public void setPower(double power) {
        holderServo.setPower(power);
    }

    //* shorter methods for controlling holder power
    public void in() {holderServo.setPower(1);}
    public void out() {holderServo.setPower(-1);}
    public void stop() {holderServo.setPower(0);}
}
