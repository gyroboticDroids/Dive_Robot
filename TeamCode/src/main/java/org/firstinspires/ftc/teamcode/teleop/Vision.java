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
            double y = VisionConstants.CAMERA_HEIGHT * Math.tan(Math.toRadians(90 + detection.getTargetYDegrees() - VisionConstants.CAMERA_ANGLE)) - VisionConstants.Y_OFFSET;
            double x = y / Math.sin(Math.toRadians(90 + detection.getTargetYDegrees() - VisionConstants.CAMERA_ANGLE)) * Math.tan(Math.toRadians(detection.getTargetXDegrees())) - VisionConstants.X_OFFSET;
            double h = 0;
            double ratio = Math.abs(detection.getTargetCorners().get(0).get(0) - detection.getTargetCorners().get(1).get(0)) / Math.abs(detection.getTargetCorners().get(0).get(1) - detection.getTargetCorners().get(3).get(1));
            samples.add(new SampleData(detection.getClassName(), x, y, h, ratio));
        }

        return samples;
    }

    public SampleData getBestSample(boolean isAllianceRed) {
        List<SampleData> allSamples = getSamples();

        if(allSamples == null) {
            return null;
        }

        SampleData bestSample = null;
        double bestNum = 1000;

        for (SampleData sample : allSamples) {
            boolean correctColor = sample.name.equals(VisionConstants.YELLOW) || ((isAllianceRed) ? sample.name.equals(VisionConstants.RED) : sample.name.equals(VisionConstants.BLUE));
            double sum = Math.abs(sample.x) + Math.abs(sample.y);

            if (correctColor && sum < bestNum) {
                bestNum = sum;
                bestSample = sample;
            }
        }

        return bestSample;
    }
}
