package org.firstinspires.ftc.teamcode.constants;

public class OuttakeConstants {
    //Outtake
    public static int SLIDES_SAMPLE_LOW = 1550;//
    public static int SLIDES_SAMPLE_HIGH = 3600;//
    public static int SLIDES_SPECIMEN_COLLECT = 0;
    public static int SLIDES_SPECIMEN_HIGH_SCORING = 1075;//
    public static int SLIDES_SPECIMEN_SCORE_PRELOAD = 1250;//
    public static int SLIDES_TRANSFER_UP = 520;//
    public static int SLIDES_START = 0;
    public static int SLIDES_HANG = 3750;
    public static int SLIDES_MAX_LIMIT = 3750;
    public static int SLIDES_ACCURACY = 70;
    public static int SLIDES_ACCURACY_DOWN = 10;
    public static int SLIDES_PIVOT_CLEAR = 400;
    public static int SLIDES_CLEAR_WALL = 500;
    public static int SLIDES_CLEAR_INTAKE = 400;
    public static double SLIDES_P_GAIN = 0.005;
    public static double SLIDES_HANGING_P_GAIN = 0.0077;

    public static double PIVOT_TRANSFER_READY = 0.835;//
    public static double PIVOT_TRANSFER = 0.997;//
    public static double PIVOT_SPECIMEN_READY = 0.952;//
    public static double PIVOT_SPECIMEN_SCORE = 0.901;//
    public static double PIVOT_SPECIMEN_SCORE_PRELOAD = 0.903;//
    public static double PIVOT_OFF_WALL = 0.1;//
    public static double PIVOT_SAMPLE = 0.35;//
    public static double PIVOT_START = 0.861;//
    public static double PIVOT_RAISE = 0.52;//

    public static double WRIST_TRANSFER_READY = 0.104;//
    public static double WRIST_TRANSFER = 0;//
    public static double WRIST_SPECIMEN_READY = 0.396;//
    public static double WRIST_SPECIMEN_SCORE = 0.496;//
    public static double WRIST_SPECIMEN_SCORE_PRELOAD = 0.349;//
    public static double WRIST_OFF_WALL = 0.952;//
    public static double WRIST_SAMPLE = 0.858;//
    public static double WRIST_START = 0.268;//
    public static double WRIST_RAISE = 0.599;//

    public static double CLAW_OPEN = 0.224;
    public static double CLAW_CLOSED = 0.698;
    public static double CLAW_CLOSED_PRELOAD = 0.88;

    //States
    public static final String START = "start";
    public static final String SPEC_PRELOAD_START = "spec preload start";
    public static final String RESET_POS = "reset pos";
    public static final String TRANSFER_INTAKE_READY = "transfer intake ready";
    public static final String TRANSFER_INTAKE = "transfer intake";
    public static final String GRAB_SAMPLE_OFF_WALL = "grab sample off wall";
    public static final String SCORE_SAMPLE_READY_HIGH = "score sample ready high";
    public static final String SCORE_SAMPLE_READY_LOW = "score sample ready low";
    public static final String SCORE_SAMPLE = "score sample";
    public static final String GRAB_SPECIMEN_READY = "grab specimen ready";
    public static final String SCORE_SPECIMEN_READY_HIGH = "score specimen ready high";
    public static final String SCORE_SPECIMEN = "score specimen";
    public static final String SCORE_SPECIMEN_PRELOAD_READY = "score specimen preload ready";
    public static final String SCORE_SPECIMEN_PRELOAD = "score specimen preload";
}
