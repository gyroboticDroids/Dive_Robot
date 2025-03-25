package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.Vision;

@TeleOp(name = "Vision Test", group = "Vision")
public class VisionTest extends OpMode {
    private Vision vision;
    private boolean sampleColor = true;


    @Override
    public void init() {
        vision = new Vision(hardwareMap);
    }

    public void loop() {
        if(gamepad1.b) {
            sampleColor = true;
        }
        if(gamepad1.x) {
            sampleColor = false;
        }

        for(SampleData sample : vision.getSamples()) {
            telemetry.addData("Sample", sample.name);
        }

        SampleData bestSample = vision.getBestSample(sampleColor);

        if(bestSample != null) {
            telemetry.addData("Best sample color", bestSample.name);
            telemetry.addData("Best sample tx", bestSample.tx);
            telemetry.addData("Best sample ty", bestSample.ty);
        }
        telemetry.addData("Best sample xyh", vision.getBestSamplePosition(sampleColor));

        telemetry.addData("Is alliance red", sampleColor);

        telemetry.update();
    }
}
