package org.firstinspires.ftc.teamcode.CodeBTC;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@SuppressWarnings("All")
public class Constants {

    public static class LConstants {
        static {
            ThreeWheelConstants.forwardTicksToInches = .001989436789;
            ThreeWheelConstants.strafeTicksToInches = .001989436789;
            ThreeWheelConstants.turnTicksToInches = .001989436789;
            ThreeWheelConstants.leftY = 1;
            ThreeWheelConstants.rightY = -1;
            ThreeWheelConstants.strafeX = -2.5;
            ThreeWheelConstants.leftEncoder_HardwareMapName = "leftFront";
            ThreeWheelConstants.rightEncoder_HardwareMapName = "rightRear";
            ThreeWheelConstants.strafeEncoder_HardwareMapName = "rightFront";
            ThreeWheelConstants.leftEncoderDirection = Encoder.REVERSE;
            ThreeWheelConstants.rightEncoderDirection = Encoder.REVERSE;
            ThreeWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        }
    }

    public static class FConstants {
        static {
            FollowerConstants.localizers = Localizers.PINPOINT;

            FollowerConstants.leftFrontMotorName = DeviceNames.leftFrontMotorName;
            FollowerConstants.leftRearMotorName = DeviceNames.leftBackMotorName;
            FollowerConstants.rightFrontMotorName = DeviceNames.rightFrontMotorName;
            FollowerConstants.rightRearMotorName = DeviceNames.rightBackMotorName;

            FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
            FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
            FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
            FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

            FollowerConstants.mass = 17.1;

            FollowerConstants.xMovement = 77.59;
            FollowerConstants.yMovement = 58.38;

            FollowerConstants.forwardZeroPowerAcceleration = -23.25;
            FollowerConstants.lateralZeroPowerAcceleration = -67.34;

            FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.17, 0, 0.02, 0);
            FollowerConstants.useSecondaryTranslationalPID = true;
            FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.15, 0, 0.005, 0); // Not being used, @see useSecondaryTranslationalPID

            FollowerConstants.headingPIDFCoefficients.setCoefficients(1.8, 0, 0.08, 0);
            FollowerConstants.useSecondaryHeadingPID = true;
            FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2, 0, 0.08, 0); // Not being used, @see useSecondaryHeadingPID

            FollowerConstants.drivePIDFCoefficients.setCoefficients(0.158, 0.0001, 0.0068, 0.6, 0);
            FollowerConstants.useSecondaryDrivePID = true;
            FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.058, 0.0001, 0.0001, 0.6, 0); // Not being used, @see useSecondaryDrivePID

            FollowerConstants.zeroPowerAccelerationMultiplier = 4;
            FollowerConstants.centripetalScaling = 0.0005;

            FollowerConstants.pathEndTimeoutConstraint = 500;
            FollowerConstants.pathEndTValueConstraint = 0.995;
            FollowerConstants.pathEndVelocityConstraint = 0.1;
            FollowerConstants.pathEndTranslationalConstraint = 0.1;
            FollowerConstants.pathEndHeadingConstraint = 0.007;
        }
    }

    @Config
    public static class DeviceNames {
        public static String leftFrontMotorName = "LFM";
        public static String leftBackMotorName = "LBM";
        public static String rightFrontMotorName = "RFM";
        public static String rightBackMotorName = "RBM";
        public static String clawServoName = "C";
        public static String armOuttakeLeftServoName = "AOL";
        public static String armOuttakeRightServoName = "AOR";
        public static String armOuttakeAnalogInputName = "AAO";
        public static String wristOuttakeServoName = "WO";
        public static String wristOuttakeAnalogInputName = "AWO";
        public static String linkageOuttakeAnalogInputName = "ALO";
        public static String linkageOuttakeServoName = "LO";
        public static String intakeActiveServoName = "I";
        public static String intakeActiveSensorName = "IS";
        public static String wristIntakeServoName = "WI";
        public static String wristIntakeAnalogInputName = "AWI";
        public static String armIntakeServoName = "AI";
        public static String armIntakeAnalogInputName = "AAI";
        public static String liftMotorLeftName = "LML";
        public static String liftMotorRightName = "LMR";
        public static String linearSlidesAnalogInputName = "ALS";
        public static String linkageServoLeftName = "LSL";
        public static String linkageServoRightName = "LSR";
        public static String climbMotorLeftName = "ASCL";
        public static String climbMotorRightName = "ASCR";
        public static String led1Name = "L1";
        public static String led2Name = "L2";
        public static String pinPointName = "odo";
    }

    @Config
    public static class Outtake{

        @Config
        public static class Lift {
            public static PIDCoefficients liftPIDCoefficients_goingUp = new PIDCoefficients(0.01, 0, 0.00001);
            public static PIDCoefficients liftPIDCoefficients_goingDown = new PIDCoefficients(0.0012, 0, 0.00001);
            public static double gravityGain = 0.12;
            public static double maxCurrentDrawn = 5;

            @Config
            public static class LiftPositions {
                public static double liftLowChamber = 0, liftHighChamber = 510, liftHighBasketSpecial = 725;
                public static double liftCollect = 0, liftLowBasket = 0, liftHighBasket = 730;
            }
        }

        @Config
        public static class ArmOuttake {
            public static double armCollectSpecimenPose = 0.87; //0.15
            public static double armCollectSpecimenSpecialPose = 0; //0.15
            public static double armCollectSpecimenSpecialSpecialPose = 0; //0.12
            public static double armTransferPose = 0.06; //0.37
            public static double armScoreSpecimenPose = 0.23; //0.5
            public static double armScoreSamplePose = 0.6; //0.85
            public static double armInitPose = 0; //0.48
            public static double waitScorePose = 0; //0.02
            public static double minPose = 0; //0.21
            public static double maxPose = 0; //0.85

            @Config
            public static class ArmOuttakeAnalogInputVoltage {
                public static double armOuttakeMinVoltage = 2.450;
                public static double armOuttakeMaxVoltage = 0.655;
            }
        }

        @Config
        public static class WristOuttake {
            public static double wristSamplePose = 0.4; //0.6
            public static double wristTransferPose = 0.25; //0.25
            public static double wristCollectSpecimenPose = 0.35; // 0.15
            public static double wristCollectSpecimenSpecialPose = 0.17; // 0.15
            public static double wristCollectSpecimenSpecialSpecialPose = 0; // 0.2
            public static double wristScoreSpecimenPose = 0.2;// 0.25
            public static double waitToComeForTransfer = 0; //0.1
            public static double wristSampleScoreSpecial = 0; //1
            public static double wristWaitCollectSpecimenPose= 0;
            public static double minPose = 0.0;
            public static double maxPose = 1;

            @Config
            public static class WristOuttakeAnalogInputVoltage {
                public static double wristOuttakeMinVoltage = 2.7;
                public static double wristOuttakeMaxVoltage = 0.7;
            }
        }

        @Config
        public static class LinkageOuttake {
            public static double slidesOuttakeRetractedPose = 0; //0.2
            public static double slidesOuttakeExtendedPose = 0.9; //0.95
            public static double slidesOuttakeAuxPose = 0; //0.2
            public static double slidesOuttakeSpecPose = 0;
            public static double minPose = 0; //0.15
            public static double maxPose = 0.9; //0.95

            @Config
            public static class LinkageOuttakeVoltages {
                public static double slidesOuttakeMinVoltage = 2.04;
                public static double slidesOuttakeMaxVoltage = 1.71;
            }
        }

        @Config
        public static class Claw {
            public static double clawClosePose = 0.0;
            public static double clawSemiClosePose = 0.05;
            public static double clawOpenPose = 0.6;
        }
    }

    @Config
    public static class Intake {

        @Config
        public static class LinearSlides {
            public static double slidesExtendedPose = 0.3;
            public static double slidesAuxPose = 0.15;
            public static double slidesAuxAutoPose = 0.2;
            public static double slidesRetractedPose = 0.0;
            public static double minPose = 0.0;
            public static double maxPose = 0.3;

            @Config
            public static class LinearSlidesAnalogInputVoltages {
                public static double linearSlideMinVoltage = 0.6;
                public static double linearSlideMaxVoltage = 1.2;
            }
        }

        @Config
        public static class ActiveIntake {
            public static double intakeCollectPower = -1;
            public static double intakeHelpTransferPower = 0.3;
            public static double intakeScorePower = 1;
            public static double intakeWaitPower = 0;
            public static double safeDistanceToStopIntake = 15;

            @Config
            public static class ColorValues {
                public static float TreshHold[] = {15F, 0.2F, 5F};
                public static float RedValues[] = {25F, 0.7F, 12.2F};
                public static float YellowValues[] = {85F, 0.7F, 25F};
                public static float BlueValues[] = {220.5F, 0.75F, 15F};
                public static float GreenValues[] = {};
            }
        }

        @Config
        public static class ArmIntake {
            public static double collectPose = 0.95;
            public static double collectSpecificOnePose = 0.8;
            public static double armWaitSpecificPose = 0.7;
            public static double armWaitPose = 0.73;
            public static double armSpecimenWaitPose = 0.4;
            public static double armSpitPose = 0.2;
            public static double transferPose = 0;
            public static double minPose = 0;
            public static double maxPose = 1;

            @Config
            public static class ArmIntakeAnalogInputVoltage {
                public static double armIntakeMinVoltage = 2.35;
                public static double armIntakeMaxVoltage = 0.9;
            }
        }

        @Config
        public static class WristIntake {
            public static double wristCollectSpecificOnePose = 0.75;
            public static double wristWaitCollectSpecificOnePose = 0.73;
            public static double wristSpecimenWait = 0.65;
            public static double wristCollectPose = 0.67;
            public static double wristTransferPose = 0.5;
            public static double wristPushBadColor = 0.54;
            public static double wristSpitPose = 0;
            public static double minPose = 0.5;
            public static double maxPose = 0.75;

            @Config
            public static class WristIntakeAnalogInputVoltage {
                public static double wristIntakeMinVoltage = 0.52;
                public static double wristIntakeMaxVoltage = 1.056;
            }
        }
    }

    public enum Color {
        Blue,
        Yellow,
        Red,
        None
    }


}




