package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="ArmDown", group="Iterative Opmode")
public class ArmDown extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        Gamepad driver = gamepad1, operator = gamepad2;

        double ClawOffset = robot.ClawOffset;
        double VFBPower = 0;

        double[] distanceSensorChecks = robot.calibrateDistanceSensors();

        telemetry.addData("Left Sensor Check:", distanceSensorChecks[0]);
        telemetry.addData("Right Sensor Check:", distanceSensorChecks[1]);
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            // Virtual Four Bar

            //Initial Set Power
            if (gamepad2.right_stick_y < 0) VFBPower = (-0.5 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
            else if (gamepad2.right_stick_y >= 0) VFBPower = (0.5 * Math.abs(Math.pow(gamepad2.right_stick_y, 3)));
            else VFBPower = 0;

            robot.VFBRight.setPower(VFBPower);
            robot.VFBLeft.setPower(VFBPower);



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


            telemetry.addData("VFB Pos:", robot.VFBLeft.getCurrentPosition());
            telemetry.addData("VFB Vel:", robot.VFBLeft.getVelocity());
            telemetry.addData("Claw:", robot.Claw.getPosition());

            // Distance Sensor Telemetry
            telemetry.addData("Distance left:", robot.LeftSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance right:", robot.RightSensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Adjusted Left:", robot.LeftSensor.getDistance(DistanceUnit.INCH) - distanceSensorChecks[0] + robot.DistanceSensorError / 2);
            telemetry.addData("Adjusted Right:", robot.RightSensor.getDistance(DistanceUnit.INCH) - distanceSensorChecks[1] + robot.DistanceSensorError / 2);
            if (robot.LeftSensor.getDistance(DistanceUnit.INCH) > distanceSensorChecks[0]) telemetry.addLine("Object at right");
            else if (robot.RightSensor.getDistance(DistanceUnit.INCH) > distanceSensorChecks[1]) telemetry.addLine("Object at left");
            else telemetry.addLine("Object at middle");

            telemetry.update();


        }


    }

}