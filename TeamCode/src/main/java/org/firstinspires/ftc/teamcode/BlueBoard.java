package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "BlueBoard", group = "Iterative Opmode")
public class BlueBoard extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap, telemetry);

        double[] CurrentCoords = new double[]{-3.5, 0.38, 0, robot.LF.getCurrentPosition(), robot.RF.getCurrentPosition(), robot.LB.getCurrentPosition()};

        robot.VirtualFourBar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean middle = false;
        boolean right = false;

        waitForStart();

        //Close claw
        robot.Claw.setPosition(0);

        sleep(500);


        // Find where the team object is, move, and place pixel
        if (robot.LeftSensor.getDistance(DistanceUnit.INCH) < 35) {
            telemetry.addLine("Object at middle");
            telemetry.update();
            middle = true;

            sleep(500);

            CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-3.5, 1.66, 0}, 0.75);

            sleep(500);

            CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-3.5, 1.4, 0}, 0.75);

            CurrentCoords[0] = -1.4;
            CurrentCoords[1] = -3.5;

        }
        else if (robot.RightSensor.getDistance(DistanceUnit.INCH) < 35 && robot.RightSensor.getDistance(DistanceUnit.INCH) > 19.5) {
            telemetry.addLine("Object at right");
            telemetry.update();
            right = true;

            sleep(500);

            CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-3.46, 1.5, 0}, 0.75);

            sleep(300);

            //turning towards board puts the pixel in correct place

            CurrentCoords[0] = -1.5;
            CurrentCoords[1] = -3.46;

        }
        else {
            telemetry.addLine("Object at left");
            telemetry.update();

            sleep(500);

            CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-4.35, 1.28, 0}, 0.9);

            CurrentCoords = robot.turnDegrees(CurrentCoords, -6);

            sleep(500);

            CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-4.35, 1.05, 0}, 0.9);

            CurrentCoords[0] = -1.0;
            CurrentCoords[1] = -4.3;

        }

        CurrentCoords = robot.turnDegrees(CurrentCoords, 90);

        sleep(250);


        //Drive to board

        if (middle) {
            CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-2.35, -5.25, 0}, 0.8);
        } else if (right) {
            CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-2.5, -5.25, 0}, 0.8);
        } else {
            CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-2, -5.25, 0}, 0.8);
        }

        robot.VirtualFourBar.setPower(-1);
        sleep(1350);
        robot.VirtualFourBar.setPower(0);
        sleep(500);
        robot.Claw.setPosition(0.4);

        sleep(750);
        robot.Claw.setPosition(0);
        sleep(750);

        robot.VirtualFourBar.setPower(1);
        sleep(1350);
        robot.VirtualFourBar.setPower(0);
        sleep(500);

        CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{CurrentCoords[0], -4.8, 0}, 0.9);

        CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-1.2, -4.8, 0}, 0.9);

        CurrentCoords = robot.turnDegrees(CurrentCoords, 2);

        CurrentCoords = robot.driveToTile(CurrentCoords, new double[]{-1.3, -5.6, 0}, 0.9);

        sleep(500);


        telemetry.addLine("End of Autonomous");
        telemetry.update();

    }

}
