package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test")
public class Test extends OpMode {
    Servo servo;

    public void init()
    {
        servo = hardwareMap.get(Servo.class, "Servo");
    }

    public void loop()
    {
        if (gamepad1.a){
            servo.setPosition(0);
        }
        if(gamepad1.b){
            servo.setPosition(0.5);
        }
        if(gamepad1.x){
            servo.setPosition(1);
        }
    }
}
