package org.firstinspires.ftc.teamcode;

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

    public IntakeHandler(Constants.Team team)
    {
        this.team = team;

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

    private Action Update(ColorSensorDetails colorSensorDetails, double deltatime)
    {
        currentActionTime += deltatime;

        if (currentAction == Action.OutTake && currentActionTime < Constants.OUTTAKE_TIME)
        {
            return Action.OutTake;
        }

        // ============================================= Track new Samples =============================================
        // Sample Did come in
        if (colorSensorDetails.brightness > 100)
        {
            // Does not matter the color
            if (team == Constants.Team.Undetermined)
            {
                sampleCount++;
            }
            // The Color Does Matter
            else
            {
                // Correct Color
                boolean isInColorRange = colorSensorDetails.hue < exclusiveHueRangeMin && colorSensorDetails.hue > exclusiveHueRangeMax;
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

        // Determine Action
        if (sampleCount == 0)
        {
            SetAction(Action.InTake);
            return Action.InTake;
        }
        else if (sampleCount == 1)
        {
            SetAction(Action.ShutDown);
            return Action.ShutDown;
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
