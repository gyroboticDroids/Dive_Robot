package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.Pose;

public class TransferConstants {
    public static boolean isAllianceRed = true;
    public static int horiSlidePos = 0;
    public static double heading = 0;
    public static Pose endPose = new Pose(0, 0, 0);

    public static void resetConstants()
    {
        endPose = new Pose(0, 0, 0);
        isAllianceRed = true;
        horiSlidePos = 0;
        heading = 0;
    }
}
