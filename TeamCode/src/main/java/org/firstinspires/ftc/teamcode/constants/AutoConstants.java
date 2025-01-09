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
    public static Pose SAMPLE_LEFT = new Pose(24, 129.5, Math.toRadians(29));
    public static Pose SAMPLE_PARK = new Pose(60, 93, Math.toRadians(0));

    //Poses in SpecimenAuto
    public static Pose SPECIMEN_START = new Pose(7.125, 68, Math.toRadians(0));
    public static Pose SPECIMEN_SCORE = new Pose(42, 68, Math.toRadians(0));
    public static Pose SPECIMEN_TRANSFER_LEFT = new Pose(12, 24, Math.toRadians(0));
    public static Pose SPECIMEN_TRANSFER_CENTER = new Pose(12, 12, Math.toRadians(0));
    public static Pose SPECIMEN_INTAKE_RIGHT = new Pose(24, 12, Math.toRadians(0));
    public static Pose SPECIMEN_GRAB_READY = new Pose(12, 34, Math.toRadians(0));
    public static Pose SPECIMEN_GRAB = new Pose(7.125, 34, Math.toRadians(0));

    public static double Y_INCREMENT = 2;
}
