package org.firstinspires.ftc.teamcode.constants;

public class IntakeConstants {
    //Intake
    public static int SLIDES_START = 0;//
    public static int SLIDES_MAX = 720;//
    public static int SLIDES_TRANSFER = 15;//
    public static int SLIDES_OUT = 70;//
    public static int SLIDES_ACCURACY = 5;//
    //public static int SLIDES_HALFWAY = 700;
    public static double SLIDES_P_GAIN = 0.022;

    public static double PIVOT_START = 1;//
    public static double PIVOT_TRANSFER = 1;//
    public static double PIVOT_INTERMEDIATE = 0.719;//
    public static double PIVOT_DOWN = 0.290;//

    public static double INTAKE_FORWARD = 1;//
    public static double INTAKE_STOP = 0;//
    public static double INTAKE_REVERSE = -1;//

    //States
    public static final String START = "start";
    public static final String TRANSFER = "transfer";
    public static final String INTAKE_SUB_READY = "intake sub ready";
    public static final String INTAKE = "intake";
    public static final String REJECT = "reject";
}
