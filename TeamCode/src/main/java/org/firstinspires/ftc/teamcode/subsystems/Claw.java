package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    //* create claws
    private final Servo leftClaw;
    private final Servo rightClaw;

    public Claw(HardwareMap hardwareMap) {
        //* initialize claws
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");
    }

    //* opens both claws
    public void open() {
        leftClaw.setPosition(1);
        rightClaw.setPosition(1);
    }

    //* closes both claws
    public void close() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(0);
    }

    //* position to hold pixels at beginning of auto
    public void holdPixel() { //TODO: TUNE THIS NUMBER
        leftClaw.setPosition(0.4);
        rightClaw.setPosition(0.4);
    }
}
