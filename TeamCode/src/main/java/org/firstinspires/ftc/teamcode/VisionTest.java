package org.firstinspires.ftc.teamcode;

import com.pedropathing.pathgen.MathFunctions;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.teleop.Vision;

@TeleOp(name = "Vision Test", group = "Vision")
public class VisionTest extends OpMode {
    private static final double SENSITIVITY = 0.001;
    private Vision vision;
    private boolean sampleColor = true;
    private Servo turret;
    private Servo pivot;
    private Servo wrist;
    private Servo claw;
    private boolean isActive = false;

    private Timer timer;

    private static final double CLAW_OPEN = 0.717;
    private static final double CLAW_CLOSED = 0.265;
    private static final double TURRET_READY = 0.469;
    private static final double TURRET_DROP = 0.749;
    private static final double PIVOT_DOWN = 0.293;
    private static final double PIVOT_UP = 0.549;
    private static final double WRIST_MIDDLE = 0.471;
    private static final double WRIST_LEFT = 0.799;
    private static final double WRIST_RIGHT = 0.134;

    SampleData sample = null;

    double turretPos = 0;

    @Override
    public void init() {
        vision = new Vision(hardwareMap);

        turret = hardwareMap.get(Servo.class, "turret");
        pivot = hardwareMap.get(Servo.class, "pivot");
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");

        timer = new Timer();

        turret.setPosition(TURRET_DROP);
        pivot.setPosition(PIVOT_UP);
        wrist.setPosition(WRIST_MIDDLE);
        claw.setPosition(CLAW_OPEN);
    }

    public void loop() {
        if(gamepad1.b) {
            sampleColor = true;
        }
        if(gamepad1.x) {
            sampleColor = false;
        }

        if(!isActive) {
            turret.setPosition(MathFunctions.clamp(turret.getPosition() + gamepad1.left_stick_x * SENSITIVITY, 0, 1));
            pivot.setPosition(MathFunctions.clamp(pivot.getPosition() + -gamepad1.left_stick_y * SENSITIVITY, 0, 1));
            wrist.setPosition(MathFunctions.clamp(wrist.getPosition() + gamepad1.right_stick_x * SENSITIVITY, 0, 1));
            claw.setPosition(MathFunctions.clamp(claw.getPosition() + gamepad1.right_stick_y * SENSITIVITY, 0, 1));
        }

        for(SampleData sample : vision.getSamples()) {
            telemetry.addData("Sample", sample.name);
            telemetry.addData("Sample ratio", sample.ratio);
        }

        telemetry.addData("Best sample xyh", vision.getBestSample(sampleColor));

        telemetry.addData("Is alliance red", sampleColor);

        telemetry.addData("turret position", turret.getPosition());
        telemetry.addData("pivot position", pivot.getPosition());
        telemetry.addData("wrist position", wrist.getPosition());
        telemetry.addData("claw position", claw.getPosition());

        if(gamepad1.a && !isActive) {
            timer.resetTimer();
            sample = vision.getBestSample(sampleColor);
        }

        if(gamepad1.a || isActive) {
            isActive = true;

            if(sample == null) {
                isActive = false;
            } else {

                if(timer.getElapsedTimeSeconds() > 2) {
                    turret.setPosition(TURRET_DROP);
                } else {
                    turretPos = Math.toDegrees(Math.asin(sample.x / VisionConstants.ARM_LENGTH) / VisionConstants.TURRET_DEGREES_PER_TICK);
                    turret.setPosition(MathFunctions.clamp(TURRET_READY + turretPos, 0, 1));
                }

                if (timer.getElapsedTimeSeconds() > 1.5) {
                    pivot.setPosition(PIVOT_UP);
                } else if (timer.getElapsedTimeSeconds() > 0.5) {
                    pivot.setPosition(PIVOT_DOWN);
                }

                if(timer.getElapsedTimeSeconds() > 2.5) {
                    claw.setPosition(CLAW_OPEN);
                }
                else if (timer.getElapsedTimeSeconds() > 1) {
                    claw.setPosition(CLAW_CLOSED);
                }

                if(timer.getElapsedTimeSeconds() > 2) {
                    wrist.setPosition(WRIST_MIDDLE);
                } else {
                    if(sample.ratio < 1.3) {
                        wrist.setPosition(MathFunctions.clamp(WRIST_MIDDLE + turretPos, WRIST_RIGHT, WRIST_LEFT));
                    } else {
                        if(turretPos > 0) {
                            wrist.setPosition(MathFunctions.clamp(WRIST_RIGHT + turretPos, WRIST_RIGHT, WRIST_LEFT));
                        } else {
                            wrist.setPosition(MathFunctions.clamp(WRIST_LEFT + turretPos, WRIST_RIGHT, WRIST_LEFT));
                        }
                    }
                }

                if (timer.getElapsedTimeSeconds() > 3) {
                    isActive = false;
                    sample = null;
                }
            }
        }

        telemetry.addData("isActive", isActive);
        if(sample != null) {
            telemetry.addData("target", Math.toDegrees(Math.asin(sample.x / VisionConstants.ARM_LENGTH)) / VisionConstants.TURRET_DEGREES_PER_TICK);
        }
        telemetry.update();
    }
}
