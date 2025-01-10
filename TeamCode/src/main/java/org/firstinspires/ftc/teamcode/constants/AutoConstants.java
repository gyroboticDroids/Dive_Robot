package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

public class AutoConstants {
    //Poses used in SampleAuto
    public static Pose SAMPLE_START = new Pose(7.125, 113.6, Math.toRadians(0));
    public static Pose SAMPLE_SCORE_READY = new Pose(16, 127, Math.toRadians(-45));
    public static Pose SAMPLE_SCORE = new Pose(13, 130, Math.toRadians(-45));
    public static Pose SAMPLE_RIGHT = new Pose(20, 121, Math.toRadians(0));
    public static Pose SAMPLE_CENTER = new Pose(20, 131, Math.toRadians(0));
    public static Pose SAMPLE_LEFT = new Pose(24, 129.5, Math.toRadians(26));
    public static Pose SAMPLE_PARK = new Pose(60, 92, Math.toRadians(0));

    //Poses in SpecimenAuto
    public static Pose SPECIMEN_START = new Pose(7.125, 66.5, Math.toRadians(0));
    public static Pose SPECIMEN_SCORE = new Pose(42.5, 68, Math.toRadians(0));
    public static Pose SPECIMEN_TRANSFER_LEFT = new Pose(22, 24, Math.toRadians(0));
    public static Pose SPECIMEN_TRANSFER_CENTER = new Pose(22, 15, Math.toRadians(0));
    public static Pose SPECIMEN_INTAKE_RIGHT = new Pose(24, 12, Math.toRadians(-27));
    public static Pose SPECIMEN_GRAB_READY = new Pose(12, 34, Math.toRadians(0));
    public static Pose SPECIMEN_GRAB = new Pose(7.125, 34, Math.toRadians(0));

    public static double Y_INCREMENT = 2;
    public static double X_INCREMENT = 0.1;

    //Pushing constants
    public static Pose SPECIMEN_PUSHING1 = new Pose(52, 34, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING2 = new Pose(50, 23, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING3 = new Pose(16, 23, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING4 = new Pose(50, 13, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING5 = new Pose(16, 13, Math.toRadians(0));

    public static Point SPECIMEN_PUSHING_CONTROL_POINT1 = new Point(16, 48);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT2 = new Point(68, 28);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT4 = new Point(68, 17);

}
