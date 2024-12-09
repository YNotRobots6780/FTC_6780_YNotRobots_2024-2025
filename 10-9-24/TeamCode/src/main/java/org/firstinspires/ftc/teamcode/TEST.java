package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="6780 Test", group="Robot")

public class TEST extends OpMode {
    @Override
    public void init() {
        Servo servo = hardwareMap.get(Servo.class, "Servo1");

        servo.setDirection(Servo.Direction.REVERSE);

        servo.setPosition(0);
    }

    @Override
    public void loop() {

    }
}
