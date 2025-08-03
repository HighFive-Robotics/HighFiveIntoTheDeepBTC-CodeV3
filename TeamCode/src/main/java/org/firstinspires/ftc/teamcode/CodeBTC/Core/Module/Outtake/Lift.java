package org.firstinspires.ftc.teamcode.CodeBTC.Core.Module.Outtake;

import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.liftMotorLeftName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.DeviceNames.liftMotorRightName;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.LiftPositions.liftCollect;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.LiftPositions.liftHighBasket;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.LiftPositions.liftHighBasketSpecial;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.LiftPositions.liftHighChamber;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.LiftPositions.liftLowBasket;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.LiftPositions.liftLowChamber;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.gravityGain;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.liftPIDCoefficients_goingDown;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.liftPIDCoefficients_goingUp;
import static org.firstinspires.ftc.teamcode.CodeBTC.Constants.Outtake.Lift.maxCurrentDrawn;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighModuleSimple;
import org.firstinspires.ftc.teamcode.CodeBTC.Core.Hardware.HighMotor;

public class Lift implements HighModuleSimple {

    public HighMotor leftLiftMotor, rightLiftMotor;
    boolean shouldResetEnc = false;
    private double power = 0;
    private boolean newTarget = false;
    ElapsedTime timerToResetEnc = new ElapsedTime();
    public States state = States.Collect;

    public enum States {
        Collect,
        HighBasket,
        HighBasketSpecial,
        LowBasket,
        HighChamber,
        LowChamber,
        Reset
    }

    public Lift(HardwareMap hardwareMap){
        leftLiftMotor = new HighMotor(hardwareMap.get(DcMotorEx.class, liftMotorLeftName), HighMotor.RunMode.Standard, false, true);
        rightLiftMotor = new HighMotor(hardwareMap.get(DcMotorEx.class, liftMotorRightName), HighMotor.RunMode.PID, false, true, true, false);
        rightLiftMotor.setPIDCoefficients(liftPIDCoefficients_goingUp.p, liftPIDCoefficients_goingUp.i, liftPIDCoefficients_goingUp.d, gravityGain, HighMotor.FeedForwardType.Lift);
        rightLiftMotor.setTarget(0);
        rightLiftMotor.setTolerance(20);
    }

    public void setState(States state) {
        this.state = state;
        newTarget = true;
        switch (state) {
            case Collect:
                power = -0.9;
                motorsSetPower(power);
                rightLiftMotor.setRunMode(HighMotor.RunMode.Standard);
                rightLiftMotor.setTarget(liftCollect);
                break;
            case LowBasket:
                rightLiftMotor.setTarget(liftLowBasket);
                break;
            case HighBasket:
                rightLiftMotor.setTarget(liftHighBasket);
                break;
            case HighChamber:
                rightLiftMotor.setTarget(liftHighChamber);
                break;
            case LowChamber:
                rightLiftMotor.setTarget(liftLowChamber);
                break;
            case HighBasketSpecial:
                rightLiftMotor.setTarget(liftHighBasketSpecial);
                break;
            case Reset:
                power = -1;
                rightLiftMotor.setRunMode(HighMotor.RunMode.Standard);
                motorsSetPower(power);
                break;
        }
    }

    public void resetEnc() {
        motorsSetPower(0);
        rightLiftMotor.setTarget(0);
        leftLiftMotor.resetMotor();
        rightLiftMotor.resetMotor();
        rightLiftMotor.resetPID();
        state = States.Collect;
    }

    public void setTarget(double target) {
        rightLiftMotor.setTarget(target);
    }

    public boolean atTarget() {
        return rightLiftMotor.atTarget();
    }

    public double getTarget() {
        return rightLiftMotor.getTarget();
    }

    public double getPower(double currentPosition) {
        return rightLiftMotor.getPower();
    }

    public States getState() {
        return state;
    }

    @Override
    public void update() {
        if (state != States.Reset) {//todo make voltage work
            if (rightLiftMotor.getCurrentDrawn() * (12.0 / 12.0) >= (maxCurrentDrawn) && rightLiftMotor.getCurrentPosition() <= 20 && rightLiftMotor.getRunMode() == HighMotor.RunMode.Standard) {
                power = 0;
                motorsSetPower(power);
                shouldResetEnc = true;
                timerToResetEnc.reset();
            }
            if (timerToResetEnc.milliseconds() >= 100 && shouldResetEnc && rightLiftMotor.getCurrentDrawn() <= 2) {
                shouldResetEnc = false;
                rightLiftMotor.setPIDCoefficients(liftPIDCoefficients_goingUp.p, liftPIDCoefficients_goingUp.i, liftPIDCoefficients_goingUp.d, gravityGain, HighMotor.FeedForwardType.Lift,1);
                resetEnc();
            }
            if (newTarget) {
                if (rightLiftMotor.getCurrentPosition() > getTarget())
                    rightLiftMotor.setPIDCoefficients(liftPIDCoefficients_goingDown.p, liftPIDCoefficients_goingDown.i, liftPIDCoefficients_goingDown.d, gravityGain, HighMotor.FeedForwardType.Lift,1);
                else
                    rightLiftMotor.setPIDCoefficients(liftPIDCoefficients_goingUp.p, liftPIDCoefficients_goingUp.i, liftPIDCoefficients_goingUp.d, gravityGain, HighMotor.FeedForwardType.Lift,1);
            }
            if (rightLiftMotor.getRunMode() == HighMotor.RunMode.PID) {
                power = rightLiftMotor.getPower();
                motorsSetPower(power);
            }
        }
    }

    public void motorsSetPower(double power){
        leftLiftMotor.setPower(power);
        rightLiftMotor.setPower(power);
    }
}
