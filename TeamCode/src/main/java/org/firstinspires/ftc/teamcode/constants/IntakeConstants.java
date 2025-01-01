package org.firstinspires.ftc.teamcode.constants;

public class IntakeConstants {
    //Intake
    public static int SLIDES_START = 0;//
    public static int SLIDES_MAX = 720;//
    public static int SLIDES_TRANSFER = 12;//
    public static int SLIDES_OUT = 70;//
    public static int SLIDES_ACCURACY = 10;//
    //public static int SLIDES_HALFWAY = 700;
    public static double SLIDES_P_GAIN = 0.03;

    public static double PIVOT_START = 0.980;//
    public static double PIVOT_TRANSFER = 1;//
    public static double PIVOT_INTERMEDIATE = 0.719;//
    public static double PIVOT_DOWN = 0.240;//

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
}
