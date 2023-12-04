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


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



@TeleOp(name="TeleOp", group="Iterative Opmode")
public class BasicTeleOp extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;


        ElapsedTime mRuntime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        double LastTime = mRuntime.time();

        double ClawOffset = 0;
        int Virtual4BarHold = robot.VirtualFourBar.getCurrentPosition();
        double RobotRotation = 0;
        double RelativeRotation = 0; // Rotation relative to starting rotation
        double AbsoluteX = 0;
        double AbsoluteY = 0;
        boolean AbsoluteSetting = false;
        double TurnControl = 0;
        double RotationChange = 0;
        boolean ClawMovingToStorage = false;
        double[] CurrentCoords = robot.updateCoords(new double[]{3.5, 0.38, 0, -robot.LF.getCurrentPosition(), robot.RF.getCurrentPosition(), robot.LB.getCurrentPosition()});

        waitForStart();



        while (opModeIsActive() && !isStopRequested()) {

            CurrentCoords = robot.updateCoords(CurrentCoords);

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
            if (Math.abs(gamepad2.right_stick_y) > 0.05) {
                robot.VirtualFourBar.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                // prevent moving the arm past breaking point
                if (gamepad2.right_stick_y < 0) robot.VirtualFourBar.setPower(-1 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
                else if (gamepad2.right_stick_y >= 0) robot.VirtualFourBar.setPower(1 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));

                Virtual4BarHold = robot.VirtualFourBar.getCurrentPosition();
                //if (robot.VirtualFourBar.getCurrentPosition() < -250 && robot.VirtualFourBar.getCurrentPosition() > -1050) robot.Claw.setPosition(0 + ClawOffset);
            } else if(gamepad2.dpad_up) {
                Virtual4BarHold = -1450;
                //if (robot.VirtualFourBar.getCurrentPosition() > -1150) robot.Claw.setPosition(0 + ClawOffset);
            } else if(gamepad2.dpad_left) {
                Virtual4BarHold = -1150;
                //if (robot.VirtualFourBar.getCurrentPosition() > -1150) robot.Claw.setPosition(0 + ClawOffset);
            } else if(gamepad2.dpad_down) {
                Virtual4BarHold = -25;
                //if (robot.VirtualFourBar.getCurrentPosition() < -250) {
                    //robot.Claw.setPosition(0 + ClawOffset);
                    //ClawMovingToStorage = true;
                //}
            }
            // If using preset to move virtual4bar back to storage, open claw
            //if (ClawMovingToStorage && robot.VirtualFourBar.getCurrentPosition() > -200) {
                //robot.Claw.setPosition(0.4 + ClawOffset);
                //ClawMovingToStorage = false;
            //}
            // Hold virtual4bar at current position or move and hold at preset position
            if (!(Math.abs(gamepad2.right_stick_y) > 0.05)) {
                robot.VirtualFourBar.setPower(0);
            }

            //if (!(Math.abs(gamepad2.right_stick_y) > 0.05)) {
                //if (robot.VirtualFourBar.getCurrentPosition() > -300 && robot.VirtualFourBar.getCurrentPosition() < -1100) robot.VirtualFourBar.setPower(0.3);
                //else robot.VirtualFourBar.setPower(1);
                //robot.VirtualFourBar.setTargetPosition(Virtual4BarHold);
                //robot.VirtualFourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //}


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
            //  && !(robot.VirtualFourBar.getCurrentPosition() < -250 && robot.VirtualFourBar.getCurrentPosition() > -1150)



            telemetry.addData("FPS:", Math.round((1 / (mRuntime.time() - LastTime)) * 1000));
            LastTime = mRuntime.time();
            telemetry.addData("X:", CurrentCoords[0]);
            telemetry.addData("Y:", CurrentCoords[1]);
            telemetry.addData("Z:", CurrentCoords[2]);
            telemetry.addData("Left:", CurrentCoords[3]);
            telemetry.addData("Right:", CurrentCoords[4]);
            telemetry.addData("Back:", CurrentCoords[5]);
            telemetry.addData("Relative Rotation", (RelativeRotation * 180));
            telemetry.addData("Claw Pos", robot.Claw.getPosition());
            telemetry.addData("VFB:", robot.VirtualFourBar.getCurrentPosition());
            telemetry.addData("Distance left", robot.LeftSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance right", robot.RightSensor.getDistance(DistanceUnit.INCH));
            if (robot.LeftSensor.getDistance(DistanceUnit.INCH) < 35) telemetry.addLine("Object at middle");
            else if (robot.RightSensor.getDistance(DistanceUnit.INCH) < 35 && robot.RightSensor.getDistance(DistanceUnit.INCH) > 19.5) telemetry.addLine("Object at right");
            else telemetry.addLine("Object at left");
            telemetry.update();
        }
    }
}


class RobotHardware {

    public final HardwareMap map;
    public final Telemetry telemetry;


    public final BNO055IMU imu;


    public final DcMotor RF, RB, LF, LB;


    public final DcMotor Intake, VirtualFourBar;


    public final Servo Claw, DroneLauncher;


    public final DistanceSensor RightSensor, LeftSensor;


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
        VirtualFourBar = hardwareMap.get(DcMotor.class, "VFB");
        DroneLauncher = hardwareMap.get(Servo.class, "Launcher");
        Claw = hardwareMap.get(Servo.class, "Claw");


        LeftSensor = hardwareMap.get(DistanceSensor.class, "LeftSensor");
        RightSensor = hardwareMap.get(DistanceSensor.class, "RightSensor");

        //RF.setDirection(DcMotor.Direction.FORWARD);
        //RB.setDirection(DcMotor.Direction.FORWARD);
        //LF.setDirection(DcMotor.Direction.REVERSE);
        //LB.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        VirtualFourBar.setDirection(DcMotor.Direction.REVERSE);

        Claw.setDirection(Servo.Direction.FORWARD);
        DroneLauncher.setDirection(Servo.Direction.FORWARD);


        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VirtualFourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        VirtualFourBar.setTargetPosition(VirtualFourBar.getCurrentPosition());


        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VirtualFourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        VirtualFourBar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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


    public void moveDirection(double direction, double turn, double throttle, double time) {
        // tile = ~23.4 inches

        double forward = throttle * Math.cos(Math.PI * (direction / 180));
        double strafe = throttle * -Math.sin(Math.PI * (direction / 180));
        double max_power = Math.max(1, Math.max(Math.max(
                forward - strafe + turn, // LF
                -forward + strafe + turn // LB
        ), Math.max(
                forward + strafe - turn, // RF
                -forward - strafe - turn // RB
        )));
        strafe /= max_power;
        forward /= max_power;
        turn /= max_power;
        LF.setPower(0.99 * (forward - strafe + turn)); // numbers equalize motor power differences
        LB.setPower(1 * (-forward + strafe + turn)); // worst motor
        RF.setPower(0.9 * (forward + strafe - turn));
        RB.setPower(0.91 * (-forward - strafe - turn));

        long timeLong = Math.round(time * 1000);

        methodSleep(timeLong);

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }


    public double[] updateCoords(double[] CurrentCoords) {
        double e1 = (-(LF.getCurrentPosition() * encoderBad) - CurrentCoords[3]) * EncoderTickToMM;
        double e2 = ((RF.getCurrentPosition() * encoderBad) - CurrentCoords[4]) * EncoderTickToMM;
        double e3 = ((LB.getCurrentPosition() * encoderBad) - CurrentCoords[5]) * EncoderTickToMM;
        double ChangeInRotation = 0; //((e1 - e2) / (2 * C1));

        return new double[]{((e3 + ChangeInRotation * C2) * Math.cos((Math.PI * CurrentCoords[2]) / 180) + ((e1 + e2) / 2) * Math.sin((Math.PI * CurrentCoords[2]) / 180) + (CurrentCoords[0] / mmToTile)) * mmToTile, // X
                (-1 * (e3 + ChangeInRotation * C2) * Math.sin((Math.PI * CurrentCoords[2]) / 180) + ((e1 + e2) / 2) * Math.cos((Math.PI * CurrentCoords[2]) / 180) + (CurrentCoords[1] / mmToTile)) * mmToTile, // Y
                (ChangeInRotation + CurrentCoords[2]), // Rotation Z
                -LF.getCurrentPosition() * encoderBad, RF.getCurrentPosition() * encoderBad, LB.getCurrentPosition() * encoderBad}; // Last Encoder Positions
    }


    public double[] driveToTile(double[] CurrentCoords, double[] TargetCoords, double Speed) {
        CurrentCoords = updateCoords(CurrentCoords);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double Distance = Math.sqrt(Math.pow(TargetCoords[0] - CurrentCoords[0], 2) + Math.pow(TargetCoords[1] - CurrentCoords[1], 2));
        double maxPower = 0, maxSpeed = 0;

        while (Distance > 0.05) { //  || Math.abs(angleDifference(CurrentCoords[2], TargetCoords[2])) > 5
            CurrentCoords = updateCoords(CurrentCoords);
            Distance = Math.sqrt(Math.pow(TargetCoords[0] - CurrentCoords[0], 2) + Math.pow(TargetCoords[1] - CurrentCoords[1], 2));

            // Tell motors what to do
            maxPower = Math.min(1, Math.max(Math.max(Math.abs(TargetCoords[0] - CurrentCoords[0]), Math.abs(TargetCoords[1] - CurrentCoords[1])), 0.0001));
            maxSpeed = Math.min(1, Math.max((Distance / 0.8), 0.25)); // Distance in Tiles when motors start slowing down, minimum speed
            driveWithControllers(-1 * ((TargetCoords[0] - CurrentCoords[0]) / maxPower) * maxSpeed,
                    ((TargetCoords[1] - CurrentCoords[1]) / maxPower) * maxSpeed,
                    0 * 0.5 * TargetCoords[2], // 0.5 * (angleDifference(TargetCoords[2], CurrentCoords[2]) / 180)
                    Speed);

            telemetry.addData("Distance", Distance);
            telemetry.addData("Angle Diff:", angleDifference(CurrentCoords[2], TargetCoords[2]));
            telemetry.addData("X:", CurrentCoords[0]);
            telemetry.addData("Y:", CurrentCoords[1]);
            telemetry.addData("Z:", CurrentCoords[2]);
            telemetry.addData("X Power", -1 * ((TargetCoords[0] - CurrentCoords[0]) / maxPower) * maxSpeed);
            telemetry.addData("Y Power", ((TargetCoords[1] - CurrentCoords[1]) / maxPower) * maxSpeed);
            telemetry.addData("Z Power",0.5 * (angleDifference(TargetCoords[2], CurrentCoords[2]) / 180) * maxSpeed);
            telemetry.update();
        }

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);

        // Prevents updating encoders while robot is still moving
        methodSleep(300);

        return updateCoords(CurrentCoords);
    }


    public double[] turnDegrees(double[] CurrentCoords, double degrees) {

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double leftStart = LF.getCurrentPosition() * encoderBad;
        double rightStart = RF.getCurrentPosition() * encoderBad;
        double targetRotation = Math.abs(encoderTicksPer360 * (degrees / 360));

        while (Math.abs(LF.getCurrentPosition() * encoderBad - leftStart) < targetRotation && Math.abs(RF.getCurrentPosition() * encoderBad - rightStart) < targetRotation) {
            driveWithControllers(0, 0, Math.signum(degrees), 0.5);
        }

        driveWithControllers(0, 0, 0, 0);

        methodSleep(300);

        //CurrentCoords[2] = angleDifference(0, degrees + CurrentCoords[2]);
        return CurrentCoords;
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
}


