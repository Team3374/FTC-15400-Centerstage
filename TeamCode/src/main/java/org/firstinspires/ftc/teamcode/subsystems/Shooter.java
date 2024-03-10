package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends SubsystemBase {
    //* create airplane shooter servo
    private final Servo shooterServo;

    public Shooter(HardwareMap hardwareMap) {
        //* initialize shooter
        shooterServo = hardwareMap.get(Servo.class, "airplaneServo");

        shooterServo.setPosition(0);
    }

    //* set shooter servo to specified position
    public void up() {shooterServo.setPosition(1);}
    public void down() {shooterServo.setPosition(0);}
}
