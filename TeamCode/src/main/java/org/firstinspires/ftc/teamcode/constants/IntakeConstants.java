package org.firstinspires.ftc.teamcode.constants;

public class IntakeConstants {
    //Intake
    public static int SLIDES_START = 0;//
    public static int SLIDES_MAX = 1970;//
    public static int SLIDES_TRANSFER = 30;//
    public static int SLIDES_OUT = 225;//
    public static int SLIDES_ACCURACY = 40;//
    public static int SLIDES_HALFWAY = 950;
    public static double SLIDES_P_GAIN = 0.01;

    public static double PIVOT_START = 0.924;//
    public static double PIVOT_TRANSFER = 0.924;//
    public static double PIVOT_INTERMEDIATE = 0.532;//
    public static double PIVOT_DOWN = 0.195;//

    public static double INTAKE_FORWARD = 1;//
    public static double INTAKE_STOP = 0;//
    public static double INTAKE_REVERSE = -1;//

    //States
    public static final String START = "start";
    public static final String TRANSFER = "transfer";
    public static final String INTAKE_SUB_READY = "intake sub ready";
    public static final String INTAKE = "intake";
    public static final String REJECT = "reject";
    public static final String CLEAR_SUB = "clear sub";
    public static final String RESET_POS = "reset pos";
    public static final String HALFWAY = "halfway";
}
