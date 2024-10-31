package org.firstinspires.ftc.teamcode;

public class IntakeHandler {

    public enum Team
    {
        Undetermined,
        Red,
        Blue
    }

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

    public IntakeHandler(Constants.Team team)
    {
        this.team = team;

        if (team == Constants.Team.Red)
        {
            exclusiveHueRangeMin = Constants.RED_RANGE_MIN;
            exclusiveHueRangeMax = Constants.RED_RANGE_MAX;
        }
        else if (team == Constants.Team.Blue)
        {
            exclusiveHueRangeMin = Constants.BLUE_RANGE_MIN;
            exclusiveHueRangeMax = Constants.BLUE_RANGE_MAX;
        }
    }

    private Action Update(ColorSensorDetails colorSensorDetails)
    {
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
                boolean isInColorRange = colorSensorDetails.hue >= exclusiveHueRangeMin && colorSensorDetails.hue <= exclusiveHueRangeMax;
                if (!isInColorRange)
                {
                    sampleCount++;
                }
                // Wrong Color
                else
                {
                    return Action.OutTake;
                }
            }
        }

        // Determine Action
        if (sampleCount == 0)
        {
            return Action.InTake;
        }
        else if (sampleCount == 1)
        {
            return Action.ShutDown;
        }
        else if (sampleCount == 2)
        {
            return Action.OutTake;
        }
        else
        {
            return Action.ShutDown;
        }
    }

    public void DropSample() {
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

}
