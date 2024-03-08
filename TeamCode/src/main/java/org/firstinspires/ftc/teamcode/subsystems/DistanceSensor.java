package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensor extends SubsystemBase {
    //* create sensor
    private final Rev2mDistanceSensor distanceSensor;

    public DistanceSensor(HardwareMap hardwareMap) {
        //* initialize distance sensor
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "distanceSensor");
    }

    //* returns measured distance in inches
    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.INCH);
    }
}
