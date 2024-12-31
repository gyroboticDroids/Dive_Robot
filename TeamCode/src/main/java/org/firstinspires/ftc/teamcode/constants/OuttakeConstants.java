package org.firstinspires.ftc.teamcode.constants;

public class OuttakeConstants {
    //Outtake
    public static int SLIDES_SAMPLE_LOW = 1000;//
    public static int SLIDES_SAMPLE_HIGH = 2500;//
    public static int SLIDES_SPECIMEN_COLLECT = 0;//
    public static int SLIDES_SPECIMEN_LOW_SCORING = 0;//
    public static int SLIDES_SPECIMEN_HIGH_SCORING = 1031;//
    public static int SLIDES_SPECIMEN_INCREASE = 476;//
    public static int SLIDES_START = 0;//
    public static int SLIDES_HANG = 2700;//
    public static int SLIDES_MAX_LIMIT = 2700;//
    public static double SLIDES_P_GAIN = 0.031;
    public static double SLIDES_ACCURACY = 50;//

    public static double EXTENSION_START = 0.6367;//
    public static double EXTENSION_TRANSFER = 0.6367;//
    public static double EXTENSION_SPECIMEN_OFF_WALL = 0.4156;//
    public static double EXTENSION_SPECIMEN_SCORE = 0.6367;//
    public static double EXTENSION_SAMPLE_SCORE = 0.6367;

    public static double PIVOT_TRANSFER = 0.410;//
    public static double PIVOT_SPECIMEN = 0.451;//
    public static double PIVOT_OFF_WALL = 0.614;//
    public static double PIVOT_SAMPLE = 0.537;//
    public static double PIVOT_START = 0.4339;//
    public static double PIVOT_RAISE = 0.509;//

    public static double WRIST_TRANSFER = 0.567;//
    public static double WRIST_SPECIMEN = 0.541;//
    public static double WRIST_OFF_WALL = 0.3706;//
    public static double WRIST_SAMPLE = 0.403;//
    public static double WRIST_START = 0.5556;//
    public static double WRIST_RAISE = 0.464;//

    public static double CLAW_OPEN = 0.2544;//
    public static double CLAW_CLOSED = 0.7950;//

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
