package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Intake;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.intakeActiveSensorName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.intakeActiveServoName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ActiveIntake.intakeCollectPower;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ActiveIntake.intakeHelpTransferPower;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ActiveIntake.intakeScorePower;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ActiveIntake.intakeWaitPower;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Intake.ActiveIntake.safeDistanceToStopIntake;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CodeBTC.Constants;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Algorithms.LowPassFilter;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighServo;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.SampleSensor;

import java.util.Arrays;


public class ActiveIntake implements HighModuleSimple {

    ElapsedTime timerToStartCollectingAgain = new ElapsedTime();
    LowPassFilter filter = new LowPassFilter(0.8, 0);
    public static States state = States.Wait;
    public HighServo intake;
    public SampleSensor intakeSensor;
    public Constants.Color opposingAllianceColor = Constants.Color.None;
    public static Constants.Color color = Constants.Color.None;
    private double power = intakeWaitPower;
    private boolean shouldStartCollectingAgain = false;
    private boolean haveCollected = false;
    private boolean wantedToSpit = false;
    private boolean useSensor = false;
    private double currentDistanceMM = 0;

    public enum States {
        Collect,
        Wait,
        Spit,
        HelpTransfer
    }

    public ActiveIntake(HardwareMap hardwareMap, Constants.Color allianceColor) {
        intake = new HighServo(hardwareMap.get(CRServo.class,intakeActiveServoName), HighServo.RunMode.ContinuousRotation);
        intakeSensor = new SampleSensor(hardwareMap, intakeActiveSensorName);
        power = intakeWaitPower;
        filter.resetFilter(intakeSensor.getDistance(MM));
        intake.setPower(power);
        if (allianceColor == Constants.Color.Red) {
            opposingAllianceColor = Constants.Color.Blue;
        } else {
            opposingAllianceColor = Constants.Color.Red;
        }
    }

    public void setState(States state) {
        ActiveIntake.state = state;
        useSensor = true;
        switch (state) {
            case Collect:
                power = intakeCollectPower;
                color = Constants.Color.None;
                wantedToSpit = false;
                break;
            case Wait:
                power = intakeWaitPower;
                break;
            case Spit:
                power = intakeScorePower;
                wantedToSpit = true;
                break;
            case HelpTransfer:
                power = intakeHelpTransferPower;
                wantedToSpit = false;
                break;
        }
    }

    public void setState(States state, boolean shouldStopBySensor) {
        ActiveIntake.state = state;
        useSensor = shouldStopBySensor;
        switch (state) {
            case Collect:
                power = intakeCollectPower;
                color = Constants.Color.None;
                wantedToSpit = false;
                break;
            case Wait:
                power = intakeWaitPower;
                break;
            case Spit:
                power = intakeScorePower;
                wantedToSpit = true;
                break;
            case HelpTransfer:
                power = intakeHelpTransferPower;
                break;
        }
    }

    public double getPower() {
        return power;
    }

    public States getState() {
        return state;
    }

    public void updateColorAndDistance() {
        currentDistanceMM = filter.getValue(intakeSensor.getDistance(MM));
        if (currentDistanceMM <= 28) {
            intakeSensor.update();
            color = intakeSensor.getColor();
            if (color == opposingAllianceColor)
                haveCollected = false;
        } else {
            color = Constants.Color.None;
            haveCollected = false;
        }
    }


    @Override
    public void update() {
        if(useSensor){
            if (color == opposingAllianceColor) {
                intakeSensor.update();
                if (intakeSensor.getColor() == opposingAllianceColor) {
                    setState(States.Spit);
                    shouldStartCollectingAgain = true;
                    timerToStartCollectingAgain.reset();
                }
            } else if ((color == Constants.Color.Yellow || color == getAllianceColor()) && currentDistanceMM <= 25) {
                haveCollected = true;
            }
            else if (shouldStartCollectingAgain && timerToStartCollectingAgain.milliseconds() >= 800 && color == Constants.Color.None && intakeSensor.getDistance(MM) >= 30) {
                setState(States.Collect);
                shouldStartCollectingAgain = false;
            }
            if (currentDistanceMM <= safeDistanceToStopIntake && !shouldStartCollectingAgain && !wantedToSpit)
                setState(States.Wait);

        }
        intake.setPower(power);
        intake.update();
    }

    public Constants.Color getColor() {
        return color;
    }

    public double getCurrentDistance() {
        return currentDistanceMM;
    }

    public boolean canCollect() {
        return !shouldStartCollectingAgain;
    }

    public Constants.Color getAllianceColor() {
        if (opposingAllianceColor == Constants.Color.Red) {
            return Constants.Color.Blue;
        } else {
            return Constants.Color.Red;
        }
    }

    public boolean isSpittingBadColor() {
        return shouldStartCollectingAgain;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Color in RGB", Arrays.toString(intakeSensor.RGB()));
        telemetry.addData("Color known", intakeSensor.getColor());
    }

    public boolean haveCollected() {
        return haveCollected;
    }

    public void setHaveCollected(boolean haveCollected) {
        this.haveCollected = haveCollected;
    }
}