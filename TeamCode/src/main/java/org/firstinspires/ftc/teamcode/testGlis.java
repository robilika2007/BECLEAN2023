package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testGlis extends LinearOpMode {

    DcMotorEx glis;
    Servo joint;
    Servo cleste;

    public static double jDefault = 0.5; //clestele paralel cu pamantu
    public static double jDrop = 1; //pozitie de drop

    public void runOpMode()
    {
        glis = this.hardwareMap.get(DcMotorEx.class, "glis");
        joint = this.hardwareMap.get(Servo.class, "joint");
        cleste = this.hardwareMap.get(Servo.class, "cleste");

        glis.setDirection(DcMotorSimple.Direction.FORWARD); //maybe need change

        waitForStart();

        while(opModeIsActive())
        {
            joint.setPosition(jDefault);

            if(gamepad1.left_trigger != 0)
            {
                glis.setPower(-0.3);
            }
            if(gamepad1.right_trigger != 0)
            {
                glis.setPower(0.3);
            }
            telemetry.addData("poz ", glis.getCurrentPosition());
            telemetry.update();
        }

    }
}
