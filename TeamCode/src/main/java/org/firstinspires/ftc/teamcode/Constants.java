package org.firstinspires.ftc.teamcode;

public class Constants {
    //Drive
    public static double DRIVE_SPEED_MULTIPLIER = 1;
    public static double DRIVE_SLOW_SPEED_MULTIPLIER = 0.3;
    public static double DRIVE_BACK_POWER = 0.3;

    public static double DRIVE_TURN_P_GAIN = 0.022;

    public float turnOffset = 0;

    //Outtake
    public static int OUTTAKE_SLIDES_SAMPLE_LOW = 1500;
    public static int OUTTAKE_SLIDES_SAMPLE_HIGH = 1500;
    public static int OUTTAKE_SLIDES_SPECIMEN_COLLECT = 700;
    public static int OUTTAKE_SLIDES_SPECIMEN_LOW_SCORING = 750;
    public static int OUTTAKE_SLIDES_SPECIMEN_HIGH_SCORING = 1300;
    public static int OUTTAKE_SLIDES_SPECIMEN_INCREASE = 50;
    public static int OUTTAKE_SLIDES_START = 0;
    public static int OUTTAKE_SLIDES_HANG = 1700;
    public static int OUTTAKE_SLIDES_MAX_LIMIT = 2715;
    public static double OUTTAKE_SLIDES_P_GAIN = 0.031;

    public static double OUTTAKE_EXTENSION_START = 0;
    public static double OUTTAKE_EXTENSION_TRANSFER = 0;
    public static double OUTTAKE_EXTENSION_SPECIMEN_COLLECT = 1;
    public static double OUTTAKE_EXTENSION_SPECIMEN_SCORE = 1;
    public static double OUTTAKE_EXTENSION_SAMPLE_SCORE = 0;

    public static double OUTTAKE_PIVOT_TRANSFER = 1;
    public static double OUTTAKE_PIVOT_SPECIMEN = 0.75;
    public static double OUTTAKE_PIVOT_OFF_WALL = 0;
    public static double OUTTAKE_PIVOT_SAMPLE = 0.25;
    public static double OUTTAKE_PIVOT_START = 0.5;
    public static double OUTTAKE_PIVOT_RAISE = 0.5;

    public static double OUTTAKE_WRIST_TRANSFER = 0;
    public static double OUTTAKE_WRIST_SPECIMEN = 0.25;
    public static double OUTTAKE_WRIST_OFF_WALL = 0.75;
    public static double OUTTAKE_WRIST_SAMPLE = 1;
    public static double OUTTAKE_WRIST_START = 0.5;
    public static double OUTTAKE_WRIST_RAISE = 0.5;

    public static double OUTTAKE_CLAW_OPEN = 0.1;
    public static double OUTTAKE_CLAW_CLOSED = 1;

    //Intake
    public static int INTAKE_SLIDES_START = 0;
    public static int INTAKE_SLIDES_MAX = 745;
    public static int INTAKE_SLIDES_TRANSFER = 50;
    public static int INTAKE_SLIDES_OUT = 100;
    public static int INTAKE_SLIDES_HALFWAY = 700;
    public static double INTAKE_SLIDES_P_GAIN = 0.022;

    public static double INTAKE_PIVOT_START = 0;
    public static double INTAKE_PIVOT_TRANSFER = 0.2;
    public static double INTAKE_PIVOT_INTERMEDIATE = 0.5;
    public static double INTAKE_PIVOT_DOWN = 1;

    public static double INTAKE_FORWARD = 1;
    public static double INTAKE_STOP = 0;
    public static double INTAKE_REVERSE = -1;

    //Hang
    public static double HANG_RIGHT_DOWN = 0.8418;
    public static double HANG_LEFT_DOWN = 0.2896;
    public static double HANG_RIGHT_UP = 0.4298;
    public static double HANG_LEFT_UP = 0.7148;
    public static double HANGING_RIGHT = 0.5902;
    public static double HANGING_LEFT = 0.5592;
}
