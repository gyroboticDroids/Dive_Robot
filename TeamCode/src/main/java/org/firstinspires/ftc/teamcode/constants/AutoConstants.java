package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Point;

public class AutoConstants {
    //Poses used in SampleAuto
    public static Pose SAMPLE_START = new Pose(7.125, 113.6, Math.toRadians(0));
    public static Pose SAMPLE_SCORE_RIGHT = new Pose(16, 133, Math.toRadians(-24));
    public static Pose SAMPLE_SCORE_CENTER = new Pose(18.5, 136, Math.toRadians(-13));
    public static Pose SAMPLE_SCORE_LEFT = new Pose(18, 136, Math.toRadians(-8));
    public static Pose SAMPLE_RIGHT = new Pose(19, 131.5, Math.toRadians(-24));
    public static Pose SAMPLE_CENTER = new Pose(20.5, 135.5, Math.toRadians(-13));
    public static Pose SAMPLE_LEFT = new Pose(19, 129.75, Math.toRadians(25));
    public static Pose SAMPLE_SUB = new Pose(60, 93, Math.toRadians(-90));
    public static Pose SAMPLE_PARK = new Pose(60, 92.5, Math.toRadians(0));

    public static Pose SAMPLE_SCORE = new Pose(14, 129, Math.toRadians(-45));

    public static Point SAMPLE_COLLECT_CONTROL = new Point(60, 115);

    //Poses in SpecimenAuto
    public static Pose SPECIMEN_START = new Pose(7.125, 65.5, Math.toRadians(0));
    public static Pose SPECIMEN_START_NO_PRELOAD = new Pose(7.125, 54, Math.toRadians(0));
    public static Pose SPECIMEN_SCORE = new Pose(40.5, 70, Math.toRadians(0));
    public static Pose SPECIMEN_SCORE_PRELOAD = new Pose(41, 65.5, Math.toRadians(0));

    public static Pose SPECIMEN_GRAB_AFTER_PUSHING = new Pose(7.125, 13, Math.toRadians(0));
    public static Pose SPECIMEN_GRAB = new Pose(7.125, 37, Math.toRadians(0));

    public static Pose SPECIMEN_PARK = new Pose(19, 124, Math.toRadians(-45));

    public static Point SPECIMEN_CONTROL_POINT = new Point(12, 55);
    public static Point SPECIMEN_UNJAM_POINT = new Point(30, 35);

    //Pushing constants
    public static Pose SPECIMEN_PUSHING2 = new Pose(54, 25, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING3 = new Pose(19, 25, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING4 = new Pose(54, 16, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING5 = new Pose(19, 16, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING6 = new Pose(54, 7, Math.toRadians(0));
    public static Pose SPECIMEN_PUSHING7 = new Pose(15, 13, Math.toRadians(0));

    public static Point SPECIMEN_PUSHING_CONTROL_POINT0 = new Point(16, 36);

    public static Point SPECIMEN_PUSHING_CONTROL_POINT1 = new Point(54, 41);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT2 = new Point(56, 35);//48
    public static Point SPECIMEN_PUSHING_CONTROL_POINT4 = new Point(56, 26);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT6 = new Point(56, 16);
    public static Point SPECIMEN_PUSHING_CONTROL_POINT7 = new Point(14, 7);

    public static Point SPECIMEN_SCORING_CONTROL_POINT1 = new Point(30, 58);
    public static Point SPECIMEN_SCORING_CONTROL_POINT2 = new Point(30, 37);
    public static Point SPECIMEN_SCORING_CONTROL_POINT3 = new Point(15, 70);
}
