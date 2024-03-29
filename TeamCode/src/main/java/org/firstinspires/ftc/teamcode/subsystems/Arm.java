package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    //* create arm servos and sensor
    private final Servo leftArmServo;
    private final Servo rightArmServo;

//    private final AnalogInput servoInput;
    private final RevTouchSensor armSensor;

    public Arm(HardwareMap hardwareMap) {
        //* initialize arm servos and sensor
        leftArmServo = hardwareMap.get(Servo.class, "leftArmServo");
        rightArmServo = hardwareMap.get(Servo.class, "rightArmServo");

//        servoInput = hardwareMap.get(AnalogInput.class, "servoInput");
        armSensor = hardwareMap.get(RevTouchSensor.class, "holder");

        leftArmServo.setPosition(0.075);
        rightArmServo.setPosition(0.075);
    }

    //* set arm to specified position
    public void setPosition(double position) {
        leftArmServo.setPosition(position);
        rightArmServo.setPosition(position);
    }

    public void down() {
        leftArmServo.setPosition(0.075);
        rightArmServo.setPosition(0.075);
    }

    //* returns target position of servos
    public double getTargetPosition() {
        return leftArmServo.getPosition();
    }

    //* returns true if touch sensor pressed
    public boolean isDown() {
        return armSensor.isPressed();
    }
}
