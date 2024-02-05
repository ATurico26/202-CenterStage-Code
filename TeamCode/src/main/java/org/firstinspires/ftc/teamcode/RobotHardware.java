package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;


    public final DcMotor RF, RB, LF, LB;


    public final DcMotor Intake;


    public final DcMotorEx VFBRight, VFBLeft;


    public final Servo Claw, DroneLauncher;


    public final HuskyLens Camera;


    public double ClawOffset = 0;


    // VFB Position PID variables
    double PosIntegralSum = 0;
    double PosKp = 0;
    double PosKi = 0;
    double PosKd = 0;
    private double PosLastError = 0;

    ElapsedTime PIDtimer = new ElapsedTime();


    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;


        RF = hardwareMap.get(DcMotor.class, "RF"); // Right Encoder
        RB = hardwareMap.get(DcMotor.class, "RB");
        LF = hardwareMap.get(DcMotor.class, "LF"); // Left Encoder
        LB = hardwareMap.get(DcMotor.class, "LB"); // Back Encoder
        Intake = hardwareMap.get(DcMotor.class, "IN");
        VFBRight = hardwareMap.get(DcMotorEx.class, "VFBRight");
        VFBLeft = hardwareMap.get(DcMotorEx.class, "VFBLeft");
        DroneLauncher = hardwareMap.get(Servo.class, "Launcher");
        Claw = hardwareMap.get(Servo.class, "Claw");


        // HuskyLens
        Camera = hardwareMap.get(HuskyLens.class, "HuskyLens");
        telemetry.addData("HuskyLens active:", Camera.knock());
        Camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);


        Intake.setDirection(DcMotor.Direction.REVERSE);
        VFBRight.setDirection(DcMotor.Direction.REVERSE);
        VFBLeft.setDirection(DcMotor.Direction.REVERSE);

        Claw.setDirection(Servo.Direction.FORWARD);
        DroneLauncher.setDirection(Servo.Direction.FORWARD);


        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VFBLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VFBRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VFBLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        VFBRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        VFBLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        telemetry.addData("Status", "Robot Hardware Initialized");
        telemetry.update();
        this.map = hardwareMap;

    } // initializes everything


    public double angleDifference(double CurrentAngle, double TargetAngle) {
        double result1 = Math.floorMod(Math.round((TargetAngle - CurrentAngle) * 100), 360 * 100) * 0.01;
        double result2 = Math.floorMod(Math.round((TargetAngle - CurrentAngle) * 100), -360 * 100) * 0.01;
        if (Math.abs(result1) <= Math.abs(result2)) return result1;
        else return result2;
    }


    public void methodSleep(long time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            // Wait the set amount of time in milliseconds while in a method
        }
    }


    public void driveWithControllers(double strafe, double forward, double turn, double throttle) {
        double max_power = Math.max(1, Math.max(Math.max(
                Math.abs(-forward + strafe - turn), // LF
                Math.abs(-forward - strafe - turn) // LB
        ), Math.max(
                Math.abs(forward + strafe - turn), // RF
                Math.abs(forward - strafe - turn) // RB
        )));
        strafe /= max_power;
        forward /= max_power;
        turn /= max_power;
        LF.setPower(throttle * (-forward + strafe - turn));
        LB.setPower(throttle * (-forward - strafe - turn));
        RF.setPower(throttle * (forward + strafe - turn));
        RB.setPower(throttle * (forward - strafe - turn));
    }


    public double PosPID(double PosReference, double PosState) {
        double PosError = PosReference - PosState;
        PosIntegralSum += PosError * PIDtimer.seconds();
        double PosDerivative = (PosError - PosLastError) / PIDtimer.seconds();
        PosLastError = PosError;

        PIDtimer.reset();

        return (PosError * PosKp) + (PosDerivative * PosKd) + (PosIntegralSum * PosKi);
    }


    public void VFBSetPosition(double position) {
        double power = PosPID(position, VFBLeft.getCurrentPosition());
        VFBRight.setPower(power);
        VFBLeft.setPower(power);
    }


    public boolean contains(final int[] array, final int key) {
        for (final int i : array) if (i == key) return true;
        return false;
    }


    public double[] findTeamObject(int[] idCheck) {
        int interations = 50;
        double averageXTotal = 0;
        double averageYTotal = 0;
        double averageHeightTotal = 0;
        double averageWidthTotal = 0;
        int totalObjects = 0;


        for (int i = 0; i < interations; i++) {
            HuskyLens.Block[] block = Camera.blocks();

            for (HuskyLens.Block value : block) {
                if (contains(idCheck, value.id)) {
                    if ((value.x > 30 && value.x < 250 && value.y > 130 && value.y < 180) && ((double) Math.abs(value.height - value.width) / Math.max(value.height, value.width) < 0.5)) {
                        totalObjects++;
                        averageXTotal = averageXTotal + value.x;
                        averageYTotal = averageYTotal + value.y;
                        averageHeightTotal = averageHeightTotal + value.height;
                        averageWidthTotal = averageWidthTotal + value.width;
                    }
                }
            }
        }

        averageXTotal = averageXTotal / totalObjects;
        averageYTotal = averageYTotal / totalObjects;
        averageHeightTotal = averageHeightTotal / totalObjects;
        averageWidthTotal = averageWidthTotal / totalObjects;


        if ((averageXTotal > 50 && averageXTotal < 75 && averageYTotal > 150 && averageYTotal < 175) && (averageHeightTotal > 45 && averageHeightTotal < 80 && averageWidthTotal > 45 && averageWidthTotal < 80)) {
            return new double[]{0, totalObjects}; // left
        } else if ((averageXTotal > 205 && averageXTotal < 230 && averageYTotal > 135 && averageYTotal < 160) && (averageHeightTotal > 30 && averageHeightTotal < 80 && averageWidthTotal > 30 && averageWidthTotal < 80)) {
            return new double[]{1, totalObjects}; // middle
        } else {
            return new double[]{2, totalObjects}; // right
        }
    }


    public double coincidingArea(double[] Tar, double[] Det) {
        double areaX = Math.min(Math.min(Tar[0] + Tar[3] - Det[0], Det[0] + Det[3] - Tar[0]), Math.min(Tar[3], Det[3]));
        double areaY = Math.min(Math.min(Tar[1] + Tar[2] - Det[1], Det[1] + Det[2] - Tar[1]), Math.min(Tar[2], Det[2]));
        if (areaX > 0 && areaY > 0) return areaX * areaY;
        else return 0;
    }


    public double[] findTeamObjectPixels(int[] idCheck) {
        int interations = 50;
        double CoincidingPixelsLeft = 0;
        double CoincidingPixelsMiddle = 0;
        double totalLeftPixels = 0;
        double totalMiddlePixels = 0;
        double[] leftZone = {42, 133, 60, 60}; // origin x, origin y, height, width
        double[] middleZone = {194, 128, 43, 43};

        for (int i = 0; i < interations; i++) {
            HuskyLens.Block[] block = Camera.blocks();

            for (HuskyLens.Block value : block) {
                if (contains(idCheck, value.id)) {
                    totalLeftPixels = totalLeftPixels + leftZone[2] * leftZone[3];
                    totalMiddlePixels = totalMiddlePixels + middleZone[2] * middleZone[3];
                    CoincidingPixelsLeft = CoincidingPixelsLeft + coincidingArea(leftZone, new double[] {value.left, value.top, value.height, value.width});
                    CoincidingPixelsMiddle = CoincidingPixelsMiddle + coincidingArea(middleZone, new double[] {value.left, value.top, value.height, value.width});
                }
            }
        }

        if (CoincidingPixelsLeft < 100 * interations && CoincidingPixelsMiddle < 100 * interations) { // right
            return new double[]{2, (totalLeftPixels + totalMiddlePixels - CoincidingPixelsLeft - CoincidingPixelsMiddle) / (totalLeftPixels + totalMiddlePixels)};
        } else if (CoincidingPixelsLeft / totalLeftPixels >= CoincidingPixelsMiddle / totalMiddlePixels) {
            return new double[]{0, CoincidingPixelsLeft / totalLeftPixels}; // left
        } else {
            return new double[]{1, CoincidingPixelsMiddle / totalMiddlePixels}; // middle
        }
    }


    public void dropPixelOnBackboard() {
        Claw.setPosition(0 + ClawOffset);
        methodSleep(250);
        while (VFBLeft.getCurrentPosition() >= -3800) {
            VFBRight.setPower(-1);
            VFBLeft.setPower(-1);
        }
        VFBRight.setPower(0);
        VFBLeft.setPower(0);

        //methodSleep(250);

        Claw.setPosition(0.15 + ClawOffset);
        methodSleep(1000);
        Claw.setPosition(0 + ClawOffset);
        methodSleep(500);

        while (VFBLeft.getCurrentPosition() <= -20) {
            VFBRight.setPower(1);
            VFBLeft.setPower(1);
        }
        VFBRight.setPower(0);
        VFBLeft.setPower(0);
    }

}
