import java.util.ArrayList;
import java.util.List;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class AutonomyTwo extends Robot {
    private int timeStep;
    private DistanceSensor[] distanceSensor;
    private Motor leftMotor;
    private Motor rightMotor;
    private Camera camera;
    private LED[] leds;

    protected void pause(double sec) {
        double current1 = this.getTime();
        double current2 = this.getTime();
        while (current2 < (current1 + sec)) {
            current2 = this.getTime();
            this.step(1);
        }
    }

    public AutonomyTwo() {
        timeStep = 128;  // set the control time step

        // Sensors initialization 
        distanceSensor = new DistanceSensor[8];
        String[] sensorNames = {
                "ps0", "ps1", "ps2", "ps3",
                "ps4", "ps5", "ps6", "ps7"
        };

        for (int i = 0; i < 8; i++) {
            distanceSensor[i] = this.getDistanceSensor(sensorNames[i]);
            distanceSensor[i].enable(timeStep);
        }

        // Camera
        camera = this.getCamera("camera");
        camera.enable(timeStep);
        camera.recognitionEnable(timeStep);

        // Motors
        leftMotor = this.getMotor("left wheel motor");
        rightMotor = this.getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY);
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        leftMotor.setVelocity(0.0);
        rightMotor.setVelocity(0.0);

        // LEDs
        leds = new LED[10];
        String[] ledsNames = {
                "led0", "led1", "led2", "led3",
                "led4", "led5", "led6", "led7",
                "led8", "led9"
        };
        for (int i = 0; i < 10; i++) {
            leds[i] = this.getLED(ledsNames[i]);
        }
    }

    protected double[] readDistanceSensorValues() {
        double[] psValues = new double[8];
        for (int i = 0; i < 8 ; i++) {
            psValues[i] = distanceSensor[i].getValue();
        }
        return psValues;
    }

    protected void move(double left, double right) {
        double max = 6.28;  // Maximum speed
        leftMotor.setVelocity(left * max / 100);
        rightMotor.setVelocity(right * max / 100);
    }

    protected void setLED(int num, boolean on) {
        if (num < 10) {
            leds[num].set(on ? 1 : 0);
        }
    }

    protected List<CameraRecognitionObject> cameraDetection() {
        ArrayList<CameraRecognitionObject> detected = new ArrayList<>();
        int nb = camera.getRecognitionNumberOfObjects();
        if (nb > 0) {
            CameraRecognitionObject[] objects = camera.getRecognitionObjects();
            for (CameraRecognitionObject object : objects) {
                detected.add(object);
            }
        }
        return detected;
    }

    protected CameraRecognitionObject targetDetected(List<CameraRecognitionObject> detected) {
        for (CameraRecognitionObject ob : detected) {
            if (ob.getModel().compareTo("cible") == 0)
                return ob;
        }
        return null;        
    }

    protected List<CameraRecognitionObject> otherRobotsDetected(List<CameraRecognitionObject> detected) {
        ArrayList<CameraRecognitionObject> robots = new ArrayList<>();
        for (CameraRecognitionObject ob : detected) {
            if (ob.getModel().compareTo("e-puck") == 0)
                robots.add(ob);
        }
        return robots;        
    }
//
    public void run() {
        while (step(timeStep) != -1) {
            double[] psValues = readDistanceSensorValues();
            boolean obstacleFront = psValues[0] > 80.0 || psValues[7] > 80.0;
            boolean obstacleLeft = psValues[5] > 80.0 || psValues[6] > 80.0;
            boolean obstacleRight = psValues[1] > 80.0 || psValues[2] > 80.0;

            if (obstacleFront) {
                if (obstacleLeft && !obstacleRight) {
                    move(100.0, -100.0);  // Maximum right turn
                    //pause(0.5);  // Reduced pause for faster adjustment
                } else if (obstacleRight && !obstacleLeft) {
                    move(-100.0, 100.0);  // Maximum left turn
                   // pause(0.5);  // Reduced pause
                } else {
                    if (Math.random() < 0.5) {
                        move(100.0, -100.0);
                       // pause(0.5);
                    } else {
                        move(-100.0, 100.0);
                       // pause(0.5);
                    }
                }
            } else {
                move(100.0, 100.0);  // Max forward speed
            }

            List<CameraRecognitionObject> detectedObjects = cameraDetection();
            CameraRecognitionObject target = targetDetected(detectedObjects);

            if (target != null) {
                double[] position = target.getPosition();
                double targetX = position[0];
                double targetY = position[1];

                if (targetX < -0.1) {
                    move(100.0, 80.0);  // Turn left towards target
                } else if (targetX > 0.1) {
                    move(80.0, 100.0);  // Turn right towards target
                } else {
                    move(100.0, 100.0);  // Move forward to target
                }

                  if (targetY < 0.1) {
                    move(0.0, 0.0);  // Stop near the target
                    System.out.println("Target reached!");
                }
            }

            List<CameraRecognitionObject> otherRobots = otherRobotsDetected(detectedObjects);
            if (!otherRobots.isEmpty()) {
                System.out.println("Other robots detected, adjusting path.");
                move(-100.0, -100.0);  // Reverse to avoid collision
            }
        }
    }

    public static void main(String[] args) {
        AutonomyTwo controller = new AutonomyTwo();
        controller.run();
    }
}
