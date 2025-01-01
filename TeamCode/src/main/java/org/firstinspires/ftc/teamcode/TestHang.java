package org.firstinspires.ftc.teamcode;

import com.pedropathing.pathgen.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "test hang")
public class TestHang extends OpMode {
    double position1;
    double position2;
    Servo hang1;
    Servo hang2;

    public void init()
    {
        hang1 = hardwareMap.get(Servo.class, "hang1");
        hang2 = hardwareMap.get(Servo.class, "hang2");

        hang2.setDirection(Servo.Direction.FORWARD);

        position1 = 0.8418;
        position2 = 0.2896;
    }

    public void loop()
    {
        position1 += gamepad1.left_stick_y * 0.001;
        position2 += gamepad1.right_stick_y * 0.001;

        position1 = MathFunctions.clamp(position1, 0, 1);
        position2 = MathFunctions.clamp(position2, 0, 1);

        telemetry.addLine("y = down");

        if(gamepad1.y)
        {
            position1 = 0.8418;
            position2 = 0.2896;
        }

        telemetry.addLine("a = up");

        if(gamepad1.a)
        {
            position1 = 0.4298;
            position2 = 0.7148;
        }

        telemetry.addLine("x = hang");

        if(gamepad1.x)
        {
            position1 = 0.5902;
            position2 = 0.5592;
        }

        hang1.setPosition(position1);
        hang2.setPosition(position2);

        telemetry.addData("hang 1", hang1.getPosition());
        telemetry.addData("hang 2", hang2.getPosition());
        telemetry.update();
    }
}
