// Move forward/backward based on D-Pad(up/down) - Constant Speed

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@TeleOp(name = "Gyrotest", group = "teamcode")

public class GyroTest extends TeleopArjun {

    public double getAngleZ() {
        Orientation angles;
        angles = myGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public double getAngleY() {
        Orientation angles;
        angles = myGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
    }
    public double getAngleX() {
        Orientation angles;
        angles = myGyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }
    public void GyroTurn(double degrees, double power) {
        if(degrees < 0) {
            if(getAngleZ() <= degrees) {
                rotateRobot(0);
            } else {
                rotateRobot(power);
            }
        } else if(degrees > 0) {
            if(getAngleZ() >= degrees) {
                rotateRobot(0);
            } else {
                rotateRobot(-power);
            }
        } else {
            rotateRobot(0);
        }
    }
    public void runOpMode() {
        my_init();
        waitForStart();
              

        while(opModeIsActive()) {
            GyroTurn(90, 0.17);
            telemetry.addData("Current Z Degrees", getAngleZ());
            telemetry.addData("Current Y Degrees", getAngleY());
            telemetry.addData("Current X Degrees", getAngleX());
            telemetry.update();
        }
    }
  

}