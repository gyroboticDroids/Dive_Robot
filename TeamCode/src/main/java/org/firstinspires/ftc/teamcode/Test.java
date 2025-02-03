package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test")
public class Test extends OpMode {
    Servo servo1;
    Servo servo2;
    Servo servo3;


    public void init()
    {
        servo1 = hardwareMap.get(Servo.class, "ch");
        servo2 = hardwareMap.get(Servo.class, "hub0");
        servo3 = hardwareMap.get(Servo.class, "hub1");
    }

    public void loop()
    {
        if (gamepad1.a){
            servo1.setPosition(0);
        }
        if(gamepad1.b){
            servo1.setPosition(0.5);
        }
        if(gamepad1.x){
            servo1.setPosition(1);
        }

        servo1.setPosition(gamepad1.left_stick_y);
        servo2.setPosition(gamepad1.left_stick_y);
        servo3.setPosition(gamepad1.right_stick_y);
    }
}
