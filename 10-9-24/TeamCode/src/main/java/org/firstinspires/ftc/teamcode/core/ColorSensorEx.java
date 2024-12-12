package org.firstinspires.ftc.teamcode.core;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class ColorSensorEx {


    public ColorSensorEx(ColorSensor colorSensor) {

        this.colorSensor = colorSensor;

        if (colorSensor != null) {
            UpdateColorValues();
        }
    }


    private final ColorSensor colorSensor;

    public byte red;
    public byte green;
    public byte blue;
    public byte alpha;
    public short hue;
    public byte saturation;
    public byte brightness;


    public void UpdateColorValues()
    {
        red = (byte)colorSensor.red();
        green = (byte)colorSensor.red();
        blue = (byte)colorSensor.red();
        alpha = (byte)colorSensor.alpha();

        float[] hueSaturationBrightness = new float[3];

        Color.RGBToHSV(red, green, blue, hueSaturationBrightness);

        hue = (byte) Math.round(hueSaturationBrightness[0] * 360);
        saturation = (byte) Math.round(hueSaturationBrightness[1] * 256);
        brightness = (byte) Math.round(hueSaturationBrightness[2] * 256);
    }


    public void enableLED(boolean enableLED)
    {
        colorSensor.enableLed(enableLED);
    }

}
