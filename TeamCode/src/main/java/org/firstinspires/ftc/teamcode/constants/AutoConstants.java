package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

public class AutoConstants {
    //Poses used in SampleAuto
    public static Pose FIVE_SAMPLE_START = new Pose(7.125, 113.6, Math.toRadians(0));
    public static Pose SIX_SAMPLE_START = new Pose(4.5, 111.6, Math.toRadians(-90));
    public static Pose SAMPLE_SCORE_OBS = new Pose(4.6, 126, Math.toRadians(-90));
    public static Pose SAMPLE_SCORE_RIGHT = new Pose(15.5, 133, Math.toRadians(-24));
    public static Pose SAMPLE_SCORE_CENTER = new Pose(18, 136, Math.toRadians(-13));
    public static Pose SAMPLE_SCORE_LEFT = new Pose(18, 136, Math.toRadians(-8));
    public static Pose SAMPLE_OBS = new Pose(5.5, 58, Math.toRadians(-90));
    public static Pose SAMPLE_ALLIANCE_PARTNER = new Pose(5.5, 107, Math.toRadians(-90));
    public static Pose SAMPLE_RIGHT = new Pose(19, 131.5, Math.toRadians(-24));
    public static Pose SAMPLE_CENTER = new Pose(20.5, 135.5, Math.toRadians(-13));
    public static Pose SAMPLE_LEFT = new Pose(19, 129, Math.toRadians(25));
    public static Pose SAMPLE_SUB = new Pose(60, 93, Math.toRadians(-90));
    public static Pose SAMPLE_PARK = new Pose(60, 92.5, Math.toRadians(0));

    public static Pose SAMPLE_SCORE_READY = new Pose(16, 127, Math.toRadians(-45));
    public static Pose SAMPLE_SCORE = new Pose(14, 129, Math.toRadians(-45));

    //Poses in SpecimenAuto
    public static Pose SPECIMEN_START = new Pose(7.125, 65.5, Math.toRadians(0));
    public static Pose SPECIMEN_SCORE = new Pose(42.5, 73, Math.toRadians(0));

    public static Pose SPECIMEN_GRAB_READY = new Pose(15, 36, Math.toRadians(0));
    public static Pose SPECIMEN_GRAB_AFTER_PUSHING = new Pose(7.125, 13, Math.toRadians(0));
    public static Pose SPECIMEN_GRAB = new Pose(7.125, 37, Math.toRadians(0));
    public static Pose SPECIMEN_GRAB_1 = new Pose(7.125, 23, Math.toRadians(0));

    public static Pose SPECIMEN_PARK = new Pose(12, 20, Math.toRadians(0));

    public static Point SPECIMEN_CONTROL_POINT = new Point(12, 55);
    public static Point SPECIMEN_UNJAM_POINT = new Point(30, 30);

    public static double Y_INCREMENT = -1.25;
    public static double X_INCREMENT = 0.05;

    //Pushing constants
    public static Pose SPECIMEN_PUSHING1 = new Pose(52, 34, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING2 = new Pose(54, 25, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING3 = new Pose(19, 25, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING4 = new Pose(54, 16, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING5 = new Pose(19, 16, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING6 = new Pose(54, 7.5, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING7 = new Pose(15, 13, Math.toRadians(0));

    public static Pose SPECIMEN_PUSHING4_READY = new Pose(51, 23, Math.toRadians(0));

    public static Point SPECIMEN_PUSHING_CONTROL_POINT1 = new Point(18, 16);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT2 = new Point(54, 48);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT4 = new Point(55, 26);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT6 = new Point(55, 16);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT7 = new Point(14, 7.5);

    public static Point SPECIMEN_SCORING_CONTROL_POINT1 = new Point(30, 65.5);
    public static Point SPECIMEN_SCORING_CONTROL_POINT2 = new Point(30, 37);

    //Transferring constants
    public static Pose SPECIMEN_TRANSFER_LEFT = new Pose(22, 24, Math.toRadians(0));
    public static Pose SPECIMEN_TRANSFER_CENTER = new Pose(22, 15, Math.toRadians(0));
    public static Pose SPECIMEN_INTAKE_RIGHT_OLD = new Pose(24, 12, Math.toRadians(-27));

    //Turning
    public static Pose SPECIMEN_INTAKE_LEFT = new Pose(20, 39, Math.toRadians(-30));
    public static Pose SPECIMEN_OUTTAKE_LEFT = new Pose(20, SPECIMEN_INTAKE_LEFT.getY(), Math.toRadians(-145));
    public static Pose SPECIMEN_INTAKE_CENTER = new Pose(20, 28, Math.toRadians(-30));
    public static Pose SPECIMEN_OUTTAKE_CENTER = new Pose(20, SPECIMEN_INTAKE_CENTER.getY(), Math.toRadians(-145));
    public static Pose SPECIMEN_INTAKE_RIGHT = new Pose(20, 18, Math.toRadians(-30));
    public static Pose SPECIMEN_OUTTAKE_RIGHT = new Pose(20, SPECIMEN_INTAKE_RIGHT.getY(), Math.toRadians(-145));
}
