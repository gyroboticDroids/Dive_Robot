package org.firstinspires.ftc.teamcode.constants;

public class IntakeConstants {
    //Intake
    public static int SLIDES_START = 0;//
    public static int SLIDES_MAX = 1970;//
    public static int SLIDES_TRANSFER = 30;//
    public static int SLIDES_OUT = 325;//
    public static int SLIDES_ACCURACY = 40;//
    public static int SLIDES_HALFWAY = 950;
    public static double SLIDES_P_GAIN = 0.008;
    public static double SLIDES_TICKS_PER_INCH = 84.8;

    public static double PIVOT_START = 0.924;//
    public static double PIVOT_TRANSFER = 0.924;//
    public static double PIVOT_INTERMEDIATE = 0.532;//
    public static double PIVOT_DOWN = 0.195;//

    public static double INTAKE_FORWARD = 1;//0.61
    public static double INTAKE_SLOW_FORWARD = 0.55;//
    public static double INTAKE_STOP = 0.5;//
    public static double INTAKE_REVERSE = 0;//0.39

    //States
    public static final String START = "start";
    public static final String TRANSFER = "transfer";
    public static final String INTAKE_SUB_READY = "intake sub ready";
    public static final String INTAKE = "intake";
    public static final String REJECT = "reject";
    public static final String CLEAR_SUB = "clear sub";
    public static final String RESET_POS = "reset pos";
    public static final String HALFWAY = "halfway";

    //Methods
    public static double linearPivot(int currentSlidePos)
    {
        return 0.0000395415 * currentSlidePos + 0.190103;
    }
}
