package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Storage {
    public static Pose2d currentPose = new Pose2d();

    public enum CurrentColor {
        BLUE,
        RED,
        NONE
    }
    public static CurrentColor currentColor = CurrentColor.BLUE;
}
