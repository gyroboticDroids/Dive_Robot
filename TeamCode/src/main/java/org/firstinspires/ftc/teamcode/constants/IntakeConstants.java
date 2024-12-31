package org.firstinspires.ftc.teamcode.constants;

public class IntakeConstants {
    //Intake
    public static int SLIDES_START = 0;
    public static int SLIDES_MAX = 745;
    public static int SLIDES_TRANSFER = 50;
    public static int SLIDES_OUT = 100;
    public static int SLIDES_ACCURACY = 10;
    public static int SLIDES_HALFWAY = 700;
    public static double SLIDES_P_GAIN = 0.022;

    public static double PIVOT_START = 0;
    public static double PIVOT_TRANSFER = 0.2;
    public static double PIVOT_INTERMEDIATE = 0.5;
    public static double PIVOT_DOWN = 1;

    public static double INTAKE_FORWARD = 1;
    public static double INTAKE_STOP = 0;
    public static double INTAKE_REVERSE = -1;

    //States
    public static final String START = "start";
    public static final String TRANSFER = "transfer";
    public static final String INTAKE_SUB_READY = "intake sub ready";
    public static final String INTAKE = "intake";
    public static final String REJECT = "reject";
}
