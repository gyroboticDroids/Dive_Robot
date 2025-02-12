package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "color test")
public class ColorSensorTest extends OpMode {
    private RevColorSensorV3 colorSensor;


    @Override
    public void init(){
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
    }

    @Override
    public void loop(){
        telemetry.addData("nr", colorSensor.getNormalizedColors().red);
        telemetry.addData("ng", colorSensor.getNormalizedColors().green);
        telemetry.addData("nb", colorSensor.getNormalizedColors().blue);

        telemetry.addData("r", colorSensor.red());
        telemetry.addData("g", colorSensor.green());
        telemetry.addData("b", colorSensor.blue());

        telemetry.addData("dist (cm)", colorSensor.getDistance(DistanceUnit.CM));
        telemetry.update();
    }
}
