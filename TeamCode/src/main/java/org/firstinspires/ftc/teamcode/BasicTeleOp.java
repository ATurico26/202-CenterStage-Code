package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.List;


@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;


        // HuskyLens
        robot.Camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);


        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double LastTime = mRuntime.time();

        double ClawOffset = 0;
        int Virtual4BarHold = robot.VFBLeft.getCurrentPosition();
        double VFBPower = 0;
        double RobotRotation = 0;
        double RelativeRotation = 0; // Rotation relative to starting rotation
        double AbsoluteX = 0;
        double AbsoluteY = 0;
        boolean AbsoluteSetting = false;
        double TurnControl = 0;
        double RotationChange = 0;
        boolean ClawMovingToStorage = false;


        // RoadRunner
        List<Integer> lastTrackingEncPositions = new ArrayList<>();
        List<Integer> lastTrackingEncVels = new ArrayList<>();
        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels);

        //Starting Position
        myLocalizer.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(90)));


        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {

            // Get Current Pose from Road Runner
            myLocalizer.update();
            Pose2d myPose = myLocalizer.getPoseEstimate();


            //Absolute Driving
            RobotRotation = -1 * (360.0 / 190) * ((robot.LF.getCurrentPosition() + robot.RF.getCurrentPosition()) * RobotHardware.encoderBad) / (2 * RobotHardware.encoderTicksPer360) + RotationChange;
            //RobotRotation = 0.25 * ((1 / TurnRatio) * (robot.LF.getCurrentPosition() + robot.LB.getCurrentPosition()
                    //- robot.RF.getCurrentPosition() - robot.RB.getCurrentPosition())) + RotationChange;
            RelativeRotation = 2 * Math.signum(RobotRotation) * ((Math.abs(0.5 * RobotRotation) - Math.floor(Math.abs(0.5 * RobotRotation))) - (Math.floor(Math.abs(RobotRotation)) - 2 * Math.floor(0.5 * Math.abs(RobotRotation))));

            if (Math.abs(gamepad1.right_stick_x) < 0.05 && Math.abs(gamepad1.right_stick_y) < 0.05) {
                AbsoluteX = Math.cos(Math.PI * RelativeRotation) * (-0.4 * Math.abs(gamepad2.left_stick_x) * gamepad2.left_stick_x) +
                        Math.sin(Math.PI * RelativeRotation) * (-0.4 * Math.abs(gamepad2.left_stick_y) * gamepad2.left_stick_y);
                AbsoluteY = -1 * Math.sin(Math.PI * RelativeRotation) * (-0.4 * Math.abs(gamepad2.left_stick_x) * gamepad2.left_stick_x) +
                        Math.cos(Math.PI * RelativeRotation) * (-0.4 * Math.abs(gamepad2.left_stick_y) * gamepad2.left_stick_y);
            } else {
                AbsoluteX = Math.cos(Math.PI * RelativeRotation) * (-1 * Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x) +
                        Math.sin(Math.PI * RelativeRotation) * (-1 * Math.abs(gamepad1.right_stick_y) * gamepad1.right_stick_y);
                AbsoluteY = -1 * Math.sin(Math.PI * RelativeRotation) * (-1 * Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x) +
                        Math.cos(Math.PI * RelativeRotation) * (-1 * Math.abs(gamepad1.right_stick_y) * gamepad1.right_stick_y);
            }

            // Auto rotate to face virtual4Bar towards the board
            if (gamepad1.right_stick_button) {
                if(RotationChange == 0.5) {
                    if (RelativeRotation > -0.35 && RelativeRotation < 0.5) TurnControl = -1;
                    else if (RelativeRotation <= -0.35 && RelativeRotation > -0.45) TurnControl = -0.2;
                    else if (RelativeRotation <= -0.45 && RelativeRotation > -0.55) TurnControl = 0; //stop turning
                    else if (RelativeRotation <= -0.55 && RelativeRotation > -0.65) TurnControl = 0.2;
                    else TurnControl = 1;
                } else if(RotationChange == 0) {
                    if (RelativeRotation > 0.15 && RelativeRotation < 1) TurnControl = -1;
                    else if (RelativeRotation <= 0.15 && RelativeRotation > 0.05) TurnControl = -0.2;
                    else if (RelativeRotation >= -0.05 && RelativeRotation < 0.05) TurnControl = 0; //stop turning
                    else if (RelativeRotation >= -0.15 && RelativeRotation < -0.05) TurnControl = 0.2;
                    else TurnControl = 1;
                } else {
                    if (RelativeRotation < 0.35 && RelativeRotation > -0.5) TurnControl = -1;
                    else if (RelativeRotation >= 0.35 && RelativeRotation < 0.45) TurnControl = -0.2;
                    else if (RelativeRotation >= 0.45 && RelativeRotation < 0.55) TurnControl = 0; //stop turning
                    else if (RelativeRotation >= 0.55 && RelativeRotation < 0.65) TurnControl = 0.2;
                    else TurnControl = 1;
                }
            } else TurnControl = Math.abs(gamepad1.left_stick_x) * gamepad1.left_stick_x;

            robot.driveWithControllers(AbsoluteX, AbsoluteY, TurnControl, 1 - 0.6 * gamepad1.left_trigger);


            // Absolute Driving off
            //robot.driveWithControllers(-1 * Math.abs(gamepad1.right_stick_x) * gamepad1.right_stick_x, -1 * Math.abs(gamepad1.right_stick_y) * gamepad1.right_stick_y, TurnControl, 1 - 0.6 * gamepad1.left_trigger);


            // Reset Absolute Driving encoders
            if(gamepad1.dpad_right && !AbsoluteSetting) {
                //CurrentCoords[0] = 0.38;
                RotationChange = 0.5;
                //robot.resetDriveEncoders();
                AbsoluteSetting = true;
            } else if(gamepad1.dpad_left && !AbsoluteSetting) {
                //CurrentCoords[0] = 5.62;
                RotationChange = -0.5;
                //robot.resetDriveEncoders();
                AbsoluteSetting = true;
            } else if(gamepad1.dpad_up && !AbsoluteSetting) {
                //CurrentCoords[2] = 0;

                RotationChange = 0;
                //robot.resetDriveEncoders();
                AbsoluteSetting = true;
            } if(gamepad1.dpad_down) {
                //CurrentCoords[1] = 0.38;
                robot.resetDriveEncoders();
                AbsoluteSetting = true;
            } else if(!gamepad1.dpad_up && !gamepad1.dpad_right && !gamepad1.dpad_left && !gamepad1.dpad_down) AbsoluteSetting = false;


            // Virtual Four Bar
            if (gamepad2.right_stick_y < 0) VFBPower = (-1 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
            else if (gamepad2.right_stick_y >= 0) VFBPower = (1 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
            else VFBPower = 0;

            robot.VFBRight.setPower(VFBPower);
            robot.VFBLeft.setPower(VFBPower);


            // Intake
            if (gamepad2.right_trigger > 0.05) robot.Intake.setPower(0.75 * gamepad2.right_trigger);
            else if(gamepad2.left_trigger > 0.05) robot.Intake.setPower(-1 * gamepad2.left_trigger);
            else if(gamepad1.right_trigger > 0.05) robot.Intake.setPower(0.75 * gamepad1.right_trigger);
            else robot.Intake.setPower(0);


            // Drone launcher
            if(gamepad1.a) robot.DroneLauncher.setPosition(0);
            else if (gamepad1.b) {
                robot.DroneLauncher.setPosition(0.5);
                // 0.5 second wait to allow mechanism to fire - might hold any last inputs
                sleep(500);
            }


            // Claw
            if(gamepad2.y) robot.Claw.setPosition(0 + ClawOffset);
            else if (gamepad2.x) robot.Claw.setPosition(0.35 + ClawOffset);


            telemetry.addData("FPS:", Math.round((1 / (mRuntime.time() - LastTime)) * 1000));
            LastTime = mRuntime.time();
            telemetry.addData("X:", myPose.getX());
            telemetry.addData("Y:", myPose.getY());
            telemetry.addData("heading:", myPose.getHeading());
            telemetry.addData("Relative Rotation:", (RelativeRotation * 180));
            telemetry.addData("Claw:", robot.Claw.getPosition());
            telemetry.addData("VFB:", robot.VFBLeft.getCurrentPosition());

            // Distance Sensor Telemetry
            telemetry.addData("Distance left:", robot.LeftSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance right:", robot.RightSensor.getDistance(DistanceUnit.INCH));
            if (robot.LeftSensor.getDistance(DistanceUnit.INCH) < 35) telemetry.addLine("Object at middle");
            else if (robot.RightSensor.getDistance(DistanceUnit.INCH) < 35 && robot.RightSensor.getDistance(DistanceUnit.INCH) > 19.5) telemetry.addLine("Object at right");
            else telemetry.addLine("Object at left");

            // HuskyLens Telemetry
            HuskyLens.Block[] block = robot.Camera.blocks();
            telemetry.addData("HuskyLens block count:", block.length);
            for (int i = 0; i < block.length; i++) {
                telemetry.addLine("ID:" + (block[i].id) + " X/Y:" + (block[i].x) + ", " + (block[i].y) + " h:" + (block[i].height) + " w:" + (block[i].width) + " origin:" + (block[i].left) + ", " + (block[i].top));
            }

            telemetry.update();
        }
    }
}


class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;


    public final BNO055IMU imu;


    public final DcMotor RF, RB, LF, LB;


    public final DcMotor Intake, VFBRight, VFBLeft;


    public final Servo Claw, DroneLauncher;


    public final DistanceSensor RightSensor, LeftSensor;


    public final HuskyLens Camera;


    // PID for VFB setup
    double IntegralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    private double lastError = 0;
    ElapsedTime PIDtimer = new ElapsedTime();


    static final double C1 = 3.4470252727, C2 = 2.7227136331, mmToTile = 0.00168248199744, EncoderTickToMM = 0.024,
            encoderBad = (1.0 * 24765) / (1.0 * 4900), // (tiles supposed to go * ticks supposed to go) / (distance told to go * encoder ticks actually gone)
            encoderTicksPer360 = 2.08784254045 * 24765 * (140.0 / 90); // encoder wheel distance for 360 degrees rotation in tile lengths * supposed encoder ticks in a tile



    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        RF = hardwareMap.get(DcMotor.class, "RF"); // Right Encoder
        RB = hardwareMap.get(DcMotor.class, "RB");
        LF = hardwareMap.get(DcMotor.class, "LF"); // Left Encoder
        LB = hardwareMap.get(DcMotor.class, "LB"); // Back Encoder
        Intake = hardwareMap.get(DcMotor.class, "IN");
        VFBRight = hardwareMap.get(DcMotor.class, "VFBRight");
        VFBLeft = hardwareMap.get(DcMotor.class, "VFBLeft");
        DroneLauncher = hardwareMap.get(Servo.class, "Launcher");
        Claw = hardwareMap.get(Servo.class, "Claw");


        Camera = hardwareMap.get(HuskyLens.class, "HuskyLens");
        // HuskyLens
        telemetry.addData("HuskyLens knock:", Camera.knock());


        LeftSensor = hardwareMap.get(DistanceSensor.class, "LeftSensor");
        RightSensor = hardwareMap.get(DistanceSensor.class, "RightSensor");

        //RF.setDirection(DcMotor.Direction.FORWARD);
        //RB.setDirection(DcMotor.Direction.FORWARD);
        //LF.setDirection(DcMotor.Direction.REVERSE);
        //LB.setDirection(DcMotor.Direction.REVERSE);
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



    public void resetDriveEncoders() {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


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
            // Wait the set amount of time before stopping motors
        }
    }


    public double[] teamObjectPosition() { // (0 = right, 1 = middle, 2 = left), confidence percentage
        double[] allSensorData = {0, 0, 0}; // Middle, Right, Left
        int totalCounts = 30;

        for (int i = 0; i < totalCounts; i++) {
            if (LeftSensor.getDistance(DistanceUnit.INCH) < 35) allSensorData[0]++;
            else if (RightSensor.getDistance(DistanceUnit.INCH) < 35) allSensorData[1]++;
            else allSensorData[2]++;
        }
        if (allSensorData[0] >= allSensorData[1] && allSensorData[0] >= allSensorData[2]) {
            return new double[]{0, allSensorData[0] / totalCounts};
        } else if (allSensorData[1] >= allSensorData[2]) {
            return new double[]{1, allSensorData[1] / totalCounts};
        } else {
            return new double[]{2, allSensorData[2] / totalCounts};
        }
    }


    public void driveWithControllers(double strafe, double forward, double turn, double throttle) {
        double max_power = Math.max(1, Math.max(Math.max(
                Math.abs(-forward - strafe - turn), // LF
                Math.abs(-forward + strafe - turn) // LB
        ), Math.max(
                Math.abs(forward + strafe - turn), // RF
                Math.abs(forward - strafe - turn) // RB
        )));
        strafe /= max_power;
        forward /= max_power;
        turn /= max_power;
        LF.setPower(throttle * (-forward - strafe - turn));
        LB.setPower(throttle * (-forward + strafe - turn));
        RF.setPower(throttle * (forward + strafe - turn));
        RB.setPower(throttle * (forward - strafe - turn));
    }


    public double PIDControl(double reference, double state) {
        double error = reference - state;
        IntegralSum += error * PIDtimer.seconds();
        double derivative = (error - lastError) / PIDtimer.seconds();
        lastError = error;

        PIDtimer.reset();

        return (error * Kp) + (derivative * Kd) + (IntegralSum * Ki);
    }


    public void VFBGoToPosition(double position){
        double power = PIDControl(position, VFBLeft.getCurrentPosition());
        VFBRight.setPower(power);
        VFBLeft.setPower(power);
    }

}


