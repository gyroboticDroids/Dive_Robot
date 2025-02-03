package org.firstinspires.ftc.teamcode.constants;

public class OuttakeConstants {
    //Outtake
    public static int SLIDES_SAMPLE_LOW = 1660;//
    public static int SLIDES_SAMPLE_HIGH = 3500;//
    public static int SLIDES_SPECIMEN_COLLECT = 0;//
    public static int SLIDES_SPECIMEN_LOW_SCORING = 750;//
    public static int SLIDES_SPECIMEN_HIGH_SCORING = 2250;//
    public static int SLIDES_TRANSFER_UP = 520;//
    public static int SLIDES_START = 0;//
    public static int SLIDES_HANG = 3700;//
    public static int SLIDES_MAX_LIMIT = 3700;//
    public static double SLIDES_P_GAIN = 0.009;
    public static double SLIDES_ACCURACY = 70;//
    public static double SLIDES_PIVOT_CLEAR = 150;//

    public static double EXTENSION_START = 0.6833;//
    public static double EXTENSION_TRANSFER = 0.6833;//
    public static double EXTENSION_SPECIMEN_OFF_WALL = 0.5394;//
    public static double EXTENSION_SPECIMEN_SCORE = 0.3322;//
    public static double EXTENSION_SAMPLE_SCORE = 0.6833;

    public static double PIVOT_TRANSFER_READY = 0.752;//
    public static double PIVOT_TRANSFER = 0.832;//
    public static double PIVOT_SPECIMEN = 0.869;//
    public static double PIVOT_OFF_WALL = 0;//
    public static double PIVOT_SAMPLE = 0.226;//
    public static double PIVOT_START = 0.778;//
    public static double PIVOT_RAISE = 0.5;//

    public static double WRIST_TRANSFER_READY = 0.104;//
    public static double WRIST_TRANSFER = 0.064;//
    public static double WRIST_SPECIMEN = 0.104;//
    public static double WRIST_OFF_WALL = 0.95;//
    public static double WRIST_SAMPLE = 0.826;//
    public static double WRIST_START = 0.297;//
    public static double WRIST_RAISE = 0.5;//

    public static double CLAW_OPEN = 0.262;//
    public static double CLAW_CLOSED = 0.716;//

    //States
    public static final String START = "start";
    public static final String RESET_POS = "reset pos";
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
