package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

public class AutoConstants {
    //Poses used in SampleAuto
    public static Pose FIVE_SAMPLE_START = new Pose(7.125, 113.6, Math.toRadians(0));
    public static Pose SIX_SAMPLE_START = new Pose(5, 111.6, Math.toRadians(-90));
    public static Pose SAMPLE_SCORE_OBS = new Pose(5.1, 126, Math.toRadians(-90));
    public static Pose SAMPLE_SCORE_RIGHT = new Pose(15.5, 133, Math.toRadians(-23));
    public static Pose SAMPLE_SCORE_CENTER = new Pose(18, 136, Math.toRadians(-13));
    public static Pose SAMPLE_SCORE_LEFT = new Pose(18, 136, Math.toRadians(-8));
    public static Pose SAMPLE_OBS = new Pose(6, 55, Math.toRadians(-90));
    public static Pose SAMPLE_RIGHT = new Pose(19, 131.5, Math.toRadians(-23));
    public static Pose SAMPLE_CENTER = new Pose(20.5, 135.5, Math.toRadians(-13));
    public static Pose SAMPLE_LEFT = new Pose(19, 129, Math.toRadians(24));
    public static Pose SAMPLE_SUB = new Pose(60, 92.5, Math.toRadians(-90));
    public static Pose SAMPLE_PARK = new Pose(60, 92, Math.toRadians(0));

    public static Pose SAMPLE_SCORE_READY = new Pose(16, 127, Math.toRadians(-45));
    public static Pose SAMPLE_SCORE = new Pose(13.5, 129.5, Math.toRadians(-45));

    //Poses in SpecimenAuto
    public static Pose SPECIMEN_START = new Pose(7.125, 66.5, Math.toRadians(0));
    public static Pose SPECIMEN_SCORE = new Pose(42.5, 68, Math.toRadians(0));
    public static Pose SPECIMEN_INTAKE_LEFT = new Pose(20, 39, Math.toRadians(-30));
    public static Pose SPECIMEN_OUTTAKE_LEFT = new Pose(20, SPECIMEN_INTAKE_LEFT.getY(), Math.toRadians(-145));
    public static Pose SPECIMEN_INTAKE_CENTER = new Pose(20, 28, Math.toRadians(-30));
    public static Pose SPECIMEN_OUTTAKE_CENTER = new Pose(20, SPECIMEN_INTAKE_CENTER.getY(), Math.toRadians(-145));
    public static Pose SPECIMEN_INTAKE_RIGHT = new Pose(20, 18, Math.toRadians(-30));
    public static Pose SPECIMEN_OUTTAKE_RIGHT = new Pose(20, SPECIMEN_INTAKE_RIGHT.getY(), Math.toRadians(-145));
    public static Pose SPECIMEN_GRAB_READY = new Pose(12, 34, Math.toRadians(0));
    public static Pose SPECIMEN_GRAB = new Pose(7.125, 34, Math.toRadians(0));

    public static Point SPECIMEN_CONTROL_POINT1 = new Point(6, 72);

    public static double Y_INCREMENT = 2;
    public static double X_INCREMENT = 0.07;

    //Pushing constants
    public static Pose SPECIMEN_PUSHING1 = new Pose(52, 34, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING2 = new Pose(51, 23, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING3 = new Pose(30, 23, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING4 = new Pose(51, 11, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING5 = new Pose(20, 13, Math.toRadians(0));

    public static Point SPECIMEN_PUSHING_CONTROL_POINT1 = new Point(16, 48);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT2 = new Point(62, 28);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT4 = new Point(62, 20);

    //Transferring constants
    public static Pose SPECIMEN_TRANSFER_LEFT = new Pose(22, 24, Math.toRadians(0));
    public static Pose SPECIMEN_TRANSFER_CENTER = new Pose(22, 15, Math.toRadians(0));
    public static Pose SPECIMEN_INTAKE_RIGHT_OLD = new Pose(24, 12, Math.toRadians(-27));

}
