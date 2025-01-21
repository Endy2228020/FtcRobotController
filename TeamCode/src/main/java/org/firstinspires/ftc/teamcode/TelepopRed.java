package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;
import java.util.Timer;

/*

 */

@TeleOp(name="TelepopRed", group="LAGRANGE")
//@Disabled
public class TelepopRed extends LinearOpMode implements Inter{

    //Железо
    private DcMotor m1, m2, m3, m4;
    //private DigitalChannel touch;

    //Переменные моторов
    private double test=0;
    private double zm1, zm2, zm3, zm4;
    private Servo teleServo;    // Renamed from tele to be more descriptive
    private Servo hookServo;    // Added proper servo object
    private Servo flipServo;    // Fixed from wrong name

    private static final double SERVO_OPEN = 0.5;  // Открытая позиция
    private static final double SERVO_CLOSE = 1.0; // Закрытая позиция

    boolean switchA = false, switchY = false, switchX = false;

    boolean openFlip = false, openHook = false, openTele = false;

//    private double telePosition = SERVO_OPEN;
//    private double hookPosition = SERVO_OPEN;
    private double flipPosition = 0.5;
    private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0;
    private double moment_diff_serv, moment_diff_switch, moment_diff_free;
    private boolean auto_mode = true, free_mode = false;
    private double a, turn;
    int telescopePos = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private double lamp=0;
    private int height;
    File telescopeFile = AppUtil.getInstance().getSettingsFile("telescopeFile.txt"); //Файл с позицией телескопа
    private int svob=0;

    //Инициализируем железо
    public void initC() {
        //Инициализация
        m1 = hardwareMap.get(DcMotor.class, "m1");
        m2 = hardwareMap.get(DcMotor.class, "m2");
        m3 = hardwareMap.get(DcMotor.class, "m3");
        m4 = hardwareMap.get(DcMotor.class, "m4");


        teleServo = hardwareMap.get(Servo.class, "tele");
        hookServo = hardwareMap.get(Servo.class, "hook");
        flipServo = hardwareMap.get(Servo.class, "flip");



//        teleServo.setPosition(telePosition);
//        hookServo.setPosition(hookPosition);
        flipServo.setPosition(flipPosition);
        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        // En1 = hardwareMap.get(DcMotor.class, "En1");
        // En2 = hardwareMap.get(DcMotor.class, "En2");
        // En3 = hardwareMap.get(DcMotor.class, "En3");

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        //En1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // En2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //En3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        // En1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // En2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // En3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        //  En1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // En2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //En3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void runOpMode() {

        class CalcThread implements Runnable {
            private Thread c;
            private boolean running;

            public void run() {
                telemetry.addLine("Calc thread running");
                telemetry.update();

                try {
                    m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
                    // En1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // En2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // En3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    while (!isStopRequested() & opModeIsActive()) {


                        double leftStickY = -gamepad1.left_stick_y;
                        double leftStickX = gamepad1.left_stick_x;
                        turn = gamepad1.right_stick_x;
                        //ТЕЛЕЖКА


                        //Коэффицент скорости робота
                        if (gamepad1.left_trigger < 0.5) {
                            a = 0.7 / 1;
                        } else if (gamepad1.left_trigger > 0.5) {
                            a = 10 / 1;
                        }
//                        if(gamepad2.right_stick_y <0.5){
//                            m5.setPower(gamepad2.right_stick_y);
//                        }

                        //Поворот


                        //Мощность моторов
                        zm1 =  (Range.clip((-leftStickX - leftStickY - turn) * a, -1, 1));
                        if (zm1 > -0.05 && zm1 < 0.05) {
                            zm1 = 0;
                        }

                        zm2 = Range.clip((leftStickX - leftStickY - turn) * a, -1, 1);
                        if (zm2 > -0.05 && zm2 < 0.05) {
                            zm2 = 0;
                        }

                        zm3 = (Range.clip((-leftStickX + leftStickY  - turn) * a, -1, 1));
                        if (zm3 > -0.05 && zm3 < 0.05) {
                            zm3 = 0;
                        }

                        zm4 = Range.clip((leftStickX + leftStickY - turn) * a, -1, 1);
                        if (zm4 > -0.05 && zm4 < 0.05) {
                            zm4 = 0;
                        }


                        //ТЕЛЕСКОП


                        //Захват "прямоугольничков"


                        moment_diff_serv = runtime.milliseconds() - last_moment_serv;
                        moment_diff_switch = runtime.milliseconds() - last_moment_switch;


                        if (gamepad1.a && !switchA) {
                            openFlip = !openFlip;
                            switchA = true;
                        }
                        if(!gamepad1.a && switchA){
                            switchA = false;
                        }


                        if (gamepad1.x && !switchX) {
                            openTele = !openTele;
                            switchX = true;
                        }
                        if(!gamepad1.x && switchX){
                            switchX = false;
                        }


                        if (gamepad1.y && !switchY) {
                            openHook = !openHook;
                            switchY = true;
                        }
                        if(!gamepad1.y && switchY){
                            switchY = false;
                        }

                    }

                } catch (Exception e) {
                    telemetry.addLine("Calc thread interrupted");
                    telemetry.update();
                }
            }
            public void start_c() {
                if (c == null) {
                    c = new Thread(this, "Calc thread");
                    c.start();
                }
            }
        }

        //Инициализация
        initC();

        waitForStart();

        //Запуск подпроцессов
        CalcThread C1 = new CalcThread();
        C1.start_c();

        //ОСНОВНАЯ ПРОГРАММА

        while(opModeIsActive() & !isStopRequested()) {

            m1.setPower(zm1);//слева спереди
            m2.setPower(zm2);//справа спереди
            m3.setPower(zm3);//слева сзади
            m4.setPower(zm4);//справа сздади

           //En3.setPower(lamp);

            if(openHook){
                hookServo.setPosition(0.5);
            }else {
                hookServo.setPosition(1);
            }


            if(openFlip){
                flipServo.setPosition(0.6);
            }else {
                flipServo.setPosition(0.1);
            }


            if(openTele){
                teleServo.setPosition(0.0);
            }else {
                teleServo.setPosition(0.8);
            }

            telemetry.addData("Состояние тригера", gamepad1.left_trigger);
            telemetry.addData("коэфицент скорости", a);
            telemetry.addData("Положение серво", height);
            telemetry.addData("Захват", hookServo.getPosition());
            telemetry.addData("Flip", flipServo.getPosition());
            telemetry.addData("Телескоп", teleServo.getPosition());
            telemetry.addData("Стик1 X", gamepad1.left_stick_x);
            telemetry.addData("Стик1 Y", gamepad1.left_stick_y);
            telemetry.addData("Стик2 X", gamepad2.right_stick_x);
            telemetry.addData("Стик2 Y", gamepad2.right_stick_y);
            telemetry.addData("Уровень", height);
            // Выводим значения в телеметрию

            telemetry.update();

        };
        ReadWriteFile.writeFile(telescopeFile, Integer.toString(telescopePos));
    }
}