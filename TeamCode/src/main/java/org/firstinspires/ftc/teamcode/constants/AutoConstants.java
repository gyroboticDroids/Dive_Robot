package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.localization.Pose;

public class AutoConstants {
    //Poses used in SampleAuto
    public static Pose SAMPLE_START = new Pose(7.125, 102.5, Math.toRadians(0));
    public static Pose SAMPLE_SCORE = new Pose(13, 130, Math.toRadians(-45));
    public static Pose SAMPLE_RIGHT = new Pose(16, 121, Math.toRadians(0));
    public static Pose SAMPLE_CENTER = new Pose(20, 131, Math.toRadians(0));
    public static Pose SAMPLE_LEFT = new Pose(19, 128, Math.toRadians(28));
    public static Pose SAMPLE_PARK = new Pose(60, 94, Math.toRadians(-90));

    //Poses in SpecimenAuto
    public static Pose SPECIMEN_START = new Pose(7.125, 68, Math.toRadians(0));
    public static Pose SPECIMEN_SCORE = new Pose(40, 68, Math.toRadians(0));
    public static Pose SPECIMEN_INTAKE_LEFT = new Pose(24, 42, Math.toRadians(-30));
    public static Pose SPECIMEN_OUTTAKE_LEFT = new Pose(36, 42, Math.toRadians(-135));
    public static Pose SPECIMEN_INTAKE_CENTER = new Pose(24, 34, Math.toRadians(-30));
    public static Pose SPECIMEN_OUTTAKE_CENTER = new Pose(36, 34, Math.toRadians(-135));
    public static Pose SPECIMEN_INTAKE_RIGHT = new Pose(30, 24, Math.toRadians(-40));
    public static Pose SPECIMEN_OUTTAKE_RIGHT = new Pose(30, 34, Math.toRadians(-135));
    public static Pose SPECIMEN_GRAB_READY = new Pose(12, 34, Math.toRadians(0));
    public static Pose SPECIMEN_GRAB = new Pose(7.125, 34, Math.toRadians(0));

    public static double Y_INCREMENT = 2;
}
