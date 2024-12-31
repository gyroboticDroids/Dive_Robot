package org.firstinspires.ftc.teamcode.constants;

public class OuttakeConstants {
    //Outtake
    public static int SLIDES_SAMPLE_LOW = 1500;
    public static int SLIDES_SAMPLE_HIGH = 1500;
    public static int SLIDES_SPECIMEN_COLLECT = 700;
    public static int SLIDES_SPECIMEN_LOW_SCORING = 750;
    public static int SLIDES_SPECIMEN_HIGH_SCORING = 1300;
    public static int SLIDES_SPECIMEN_INCREASE = 50;
    public static int SLIDES_START = 0;
    public static int SLIDES_HANG = 1700;
    public static int SLIDES_MAX_LIMIT = 2715;
    public static double SLIDES_P_GAIN = 0.031;
    public static double SLIDES_ACCURACY = 20;

    public static double EXTENSION_START = 0;
    public static double EXTENSION_TRANSFER = 0;
    public static double EXTENSION_SPECIMEN_COLLECT = 1;
    public static double EXTENSION_SPECIMEN_SCORE = 1;
    public static double EXTENSION_SAMPLE_SCORE = 0;

    public static double PIVOT_TRANSFER = 1;
    public static double PIVOT_SPECIMEN = 0.75;
    public static double PIVOT_OFF_WALL = 0;
    public static double PIVOT_SAMPLE = 0.25;
    public static double PIVOT_START = 0.5;
    public static double PIVOT_RAISE = 0.5;

    public static double WRIST_TRANSFER = 0;
    public static double WRIST_SPECIMEN = 0.25;
    public static double WRIST_OFF_WALL = 0.75;
    public static double WRIST_SAMPLE = 1;
    public static double WRIST_START = 0.5;
    public static double WRIST_RAISE = 0.5;

    public static double CLAW_OPEN = 0.1;
    public static double CLAW_CLOSED = 1;

    //States
    public static final String START = "start";
    public static final String TRANSFER_INTAKE_READY = "transfer intake ready";
    public static final String TRANSFER_INTAKE = "transfer intake";
    public static final String SCORE_SAMPLE_READY_HIGH = "score sample ready high";
    public static final String SCORE_SAMPLE_READY_LOW = "score sample ready low";
    public static final String SCORE_SAMPLE = "score sample";
    public static final String GRAB_SPECIMEN_READY = "grab specimen ready";
    public static final String SCORE_SPECIMEN_READY_HIGH = "score specimen ready high";
    public static final String SCORE_SPECIMEN_READY_LOW = "score specimen ready low";
    public static final String SCORE_SPECIMEN = "score specimen";
}
