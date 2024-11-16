package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.core.ColorSensorEx;

public class IntakeHandler {

    public enum Action
    {
        ShutDown,
        OutTake,
        InTake,
    }

    private final Constants.Team team;
    private short exclusiveHueRangeMin;
    private short exclusiveHueRangeMax;


    private byte sampleCount = 0;

    private Action currentAction;
    private double currentActionTime;

    private boolean sampleAddedLastFrame;

    private ColorSensorEx colorSensor;


    public IntakeHandler(Constants.Team team, ColorSensorEx colorSensor)
    {
        this.team = team;
        this.colorSensor = colorSensor;

        if (this.team == Constants.Team.Red)
        {
            exclusiveHueRangeMin = Constants.BLUE_RANGE_MIN;
            exclusiveHueRangeMax = Constants.BLUE_RANGE_MAX;
        }
        else if (this.team == Constants.Team.Blue)
        {
            exclusiveHueRangeMin = Constants.RED_RANGE_MIN;
            exclusiveHueRangeMax = Constants.RED_RANGE_MAX;
        }
    }

    public Action Update(double deltaTime)
    {
        currentActionTime += deltaTime;

        if (currentAction == Action.OutTake && currentActionTime < Constants.OUTTAKE_TIME)
        {
            return Action.OutTake;
        }

        // ============================================= Track new Samples =============================================
        // Sample Did come in
        if (colorSensor.brightness > 100)
        {
            // Does not matter the color
            if (team == Constants.Team.Undetermined)
            {
                if (!sampleAddedLastFrame)
                {
                    sampleCount++;
                    currentActionTime = 0;
                }
            }
            // The Color Does Matter
            else
            {
                if (!sampleAddedLastFrame)
                {
                    sampleAddedLastFrame = true;
                    // Correct Color
                    boolean isInColorRange = colorSensor.hue < exclusiveHueRangeMin && colorSensor.hue > exclusiveHueRangeMax;
                    if (!isInColorRange)
                    {
                        sampleCount++;
                    }
                    // Wrong Color
                    else
                    {
                        SetAction(Action.OutTake);
                        return Action.OutTake;
                    }
                }
            }
        }
        else
        {
            sampleAddedLastFrame = false;
        }

        // Determine Action
        if (sampleCount == 0)
        {
            SetAction(Action.InTake);
            return Action.InTake;
        }
        else if (sampleCount == 1)
        {
            if (currentActionTime > 0.5)
            {
                SetAction(Action.InTake);
                return Action.InTake;
            }
            else
            {
                SetAction(Action.ShutDown);
                return Action.ShutDown;
            }
        }

        else if (sampleCount == 2)
        {
            SetAction(Action.OutTake);
            return Action.OutTake;
        }
        else
        {
            SetAction(Action.OutTake);
            return Action.ShutDown;
        }
    }


    public void DropSample()
    {
        sampleCount--;
        if (sampleCount < 0)
        {
            sampleCount = 0;
        }
    }

    public void Reset()
    {
        sampleCount = 0;
    }

    private void SetAction(Action action)
    {
        if (currentAction != action)
        {
            currentAction = action;
            currentActionTime = 0;
        }
    }
}
