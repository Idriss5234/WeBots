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

    public AutonomyTwo() {
        timeStep = 128;  // set the control time step

        // Sensors initialization 
        distanceSensor = new DistanceSensor[8];
        String[] sensorNames = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
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
        String[] ledsNames = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};
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

    // Méthode pour calculer la distance euclidienne entre deux points en 2D (X, Z)
    protected double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
    }

    public void run() {
        // Position actuelle du robot (peut être modifiée en fonction des informations que vous avez)
        double robotX = 0.0;
        double robotY = 0.0;

        while (step(timeStep) != -1) {
            double[] psValues = readDistanceSensorValues();
            boolean obstacleFront = psValues[0] > 80.0 || psValues[7] > 80.0;
            boolean obstacleLeft = psValues[5] > 80.0 || psValues[6] > 80.0;
            boolean obstacleRight = psValues[1] > 80.0 || psValues[2] > 80.0;

            if (obstacleFront) {
                if (obstacleLeft && !obstacleRight) {
                    move(100.0, -100.0);  // Maximum right turn
                } else if (obstacleRight && !obstacleLeft) {
                    move(-100.0, 100.0);  // Maximum left turn
                } else {
                    if (Math.random() < 0.5) {
                        move(100.0, -100.0);
                    } else {
                        move(-100.0, 100.0);
                    }
                }
            } else {
                move(100.0, 100.0);  // Max forward speed
            }

            // Détection des objets avec la caméra
            List<CameraRecognitionObject> detectedObjects = cameraDetection();
            CameraRecognitionObject target = targetDetected(detectedObjects);

            if (target != null) {
                double[] targetPosition = target.getPosition();
                double targetX = targetPosition[0];
                double targetY = targetPosition[1];

                System.out.println("Target detected: X = " + targetX + " Y = " + targetY);

                // Calculer la distance entre le robot et la cible
                double distanceToTarget = calculateDistance(robotX, robotY, targetX, targetY);

                System.out.println("Distance to target: " + distanceToTarget);

                // Ajuster la direction vers la cible
                if (targetY < -0.1) {
                    move(100.0, 80.0);  // Tourner à gauche vers la cible
                } else if (targetY > 0.1) {
                    move(80.0, 100.0);  // Tourner à droite vers la cible
                } else {
                    move(100.0, 100.0);  // Avancer vers la cible
                }

                // S'arrêter si la distance à la cible est inférieure à un seuil
                if (distanceToTarget < 0.1) {  // Seuil pour s'arrêter près de la cible
                    move(0.0, 0.0);  // S'arrêter
                    System.out.println("Target reached!");
                }
            }
        }
    }

    public static void main(String[] args) {
        AutonomyTwo controller = new AutonomyTwo();
        controller.run();
    }
}
