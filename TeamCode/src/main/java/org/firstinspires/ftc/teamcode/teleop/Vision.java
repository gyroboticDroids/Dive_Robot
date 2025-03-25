package org.firstinspires.ftc.teamcode.teleop;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.MathFunctions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SampleData;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;

import java.util.ArrayList;
import java.util.List;

public class Vision {
    Limelight3A limelight;

    public Vision(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(8);
    }

    public List<SampleData> getSamples() {
        List<SampleData> samples = new ArrayList<>();
        LLResult result = limelight.getLatestResult();

        if(result == null) {
            return null;
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        for (LLResultTypes.DetectorResult detection : detections) {
            samples.add(new SampleData(detection.getClassName(), detection.getTargetXDegrees(), detection.getTargetYDegrees(), 0));
        }

        return samples;
    }

    public SampleData getBestSample(boolean isAllianceRed) {
        List<SampleData> allSamples = getSamples();

        if(allSamples == null) {
            return null;
        }

        int id = -1;
        double bestNum = 100;

        int count = 0;

        for (SampleData sample : allSamples) {
            boolean correctColor = sample.name.equals(VisionConstants.YELLOW) || ((isAllianceRed) ? sample.name.equals(VisionConstants.RED) : sample.name.equals(VisionConstants.BLUE));
            boolean correctHeading = MathFunctions.roughlyEquals(sample.heading, VisionConstants.IDEAL_HEADING, VisionConstants.ERROR);

            if (correctColor && correctHeading) {
                if (Math.abs(sample.tx) + Math.abs(sample.ty) + Math.abs(sample.heading) < bestNum) {
                    bestNum = Math.abs(sample.tx) + Math.abs(sample.ty) + Math.abs(sample.heading);
                    id = count;
                }
            }

            count++;
        }

        if (id == -1) {
            return null;
        } else {
            return allSamples.get(id);
        }
    }

    public Pose getBestSamplePosition(boolean isAllianceRed) {
        SampleData sample = getBestSample(isAllianceRed);

        if(sample == null) {
            return null;
        }

        double y = VisionConstants.CAMERA_HEIGHT * Math.tan(Math.toRadians(90 - (sample.ty + VisionConstants.CAMERA_ANGLE)));
        double x = y / Math.sin(Math.toRadians(sample.ty)) * Math.tan(Math.toRadians(sample.tx));
        double h = sample.heading;

        return new Pose(x, y, h);
    }
}
