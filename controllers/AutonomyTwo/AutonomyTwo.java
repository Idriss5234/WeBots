import java.util.ArrayList;
import java.util.List;

import com.cyberbotics.webots.controller.Camera;
import com.cyberbotics.webots.controller.CameraRecognitionObject;
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.LED;
import com.cyberbotics.webots.controller.Motor;
import com.cyberbotics.webots.controller.Robot;

public class AutonomyTwo extends Robot {
    private int timeStep; // Temps d'intervalle pour les étapes de contrôle
    private DistanceSensor[] distanceSensor; // Tableau pour stocker les capteurs de distance
    private Motor leftMotor; // Moteur de la roue gauche
    private Motor rightMotor; // Moteur de la roue droite
    private Camera camera; // Caméra pour la reconnaissance d'objets
    private LED[] leds; // Tableau pour stocker les LEDs

    public AutonomyTwo() {
        timeStep = 128;  // Définir l'intervalle de temps pour le contrôle                                              
        // Initialisation des capteurs de distance
        distanceSensor = new DistanceSensor[8];
        String[] sensorNames = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
        for (int i = 0; i < 8; i++) {
            distanceSensor[i] = this.getDistanceSensor(sensorNames[i]); // Obtenir le capteur par son nom
            distanceSensor[i].enable(timeStep); // Activer le capteur avec l'intervalle de temps
        }

        // Initialisation de la caméra
        camera = this.getCamera("camera");
        camera.enable(timeStep); // Activer la caméra
        camera.recognitionEnable(timeStep); // Activer la reconnaissance d'objets

        // Initialisation des moteurs
        leftMotor = this.getMotor("left wheel motor");
        rightMotor = this.getMotor("right wheel motor");
        leftMotor.setPosition(Double.POSITIVE_INFINITY); // Position infinie pour permettre un contrôle manuel de la vitesse
        rightMotor.setPosition(Double.POSITIVE_INFINITY);
        leftMotor.setVelocity(0.0); // Initialiser la vitesse à 0
        rightMotor.setVelocity(0.0);

        // Initialisation des LEDs
        leds = new LED[10];
        String[] ledsNames = {"led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9"};
        for (int i = 0; i < 10; i++) {
            leds[i] = this.getLED(ledsNames[i]); // Obtenir la LED par son nom
        }
    }

    // Méthode pour lire les valeurs des capteurs de distance
    protected double[] readDistanceSensorValues() {
        double[] psValues = new double[8]; // Tableau pour stocker les valeurs des capteurs
        for (int i = 0; i < 8 ; i++) {
            psValues[i] = distanceSensor[i].getValue(); // Lire la valeur de chaque capteur
        }
        return psValues;
    }

    // Méthode pour définir la vitesse des moteurs
    protected void move(double left, double right) {
        double max = 6.28;  // Vitesse maximale du moteur                
        leftMotor.setVelocity(left * max / 100); // Calculer et définir la vitesse pour la roue gauche
        rightMotor.setVelocity(right * max / 100); // Calculer et définir la vitesse pour la roue droite
    }

    // Méthode pour détecter les objets via la caméra
    protected List<CameraRecognitionObject> cameraDetection() {
        ArrayList<CameraRecognitionObject> detected = new ArrayList<>(); // Liste pour stocker les objets détectés
        int nb = camera.getRecognitionNumberOfObjects(); // Nombre d'objets détectés
        if (nb > 0) {
            CameraRecognitionObject[] objects = camera.getRecognitionObjects(); // Obtenir les objets détectés
            for (CameraRecognitionObject object : objects) {
                detected.add(object); // Ajouter chaque objet à la liste
            }
        }
        return detected;
    }

    // Méthode pour vérifier si un objet cible est détecté
    protected CameraRecognitionObject targetDetected(List<CameraRecognitionObject> detected) {
        for (CameraRecognitionObject ob : detected) {
            if (ob.getModel().compareTo("cible") == 0) // Vérifier si le modèle de l'objet est "cible"
                return ob; // Retourner l'objet cible
        }
        return null;        
    }

    // Méthode pour calculer la distance euclidienne entre deux points en 2D (X, Z)
    protected double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2)); // Calculer la distance avec la formule euclidienne
    }

    public void run() {
        // Position actuelle du robot (peut être modifiée en fonction des informations que vous avez)
        double robotX = 0.0; // Coordonnée X initiale
        double robotY = 0.0; // Coordonnée Y initiale

        while (step(timeStep) != -1) { // Boucle principale du contrôle
            double[] psValues = readDistanceSensorValues(); // Lire les valeurs des capteurs
            boolean obstacleFront = psValues[0] > 80.0 || psValues[7] > 80.0; // Détection d'obstacle à l'avant
            boolean obstacleLeft = psValues[5] > 80.0 || psValues[6] > 80.0; // Détection d'obstacle à gauche
            boolean obstacleRight = psValues[1] > 80.0 || psValues[2] > 80.0; // Détection d'obstacle à droite

            // Logique pour éviter les obstacles
            if (obstacleFront) {
                if (obstacleLeft && !obstacleRight) {
                    move(100.0, -100.0);  // Tourner fortement à droite
                } else if (obstacleRight && !obstacleLeft) {
                    move(-100.0, 100.0);  // Tourner fortement à gauche
                } else {
                    if (Math.random() < 0.5) { // Choisir une direction aléatoire si bloqué
                        move(100.0, -100.0);
                    } else {
                        move(-100.0, 100.0);
                    }
                }
            } else {
                move(100.0, 100.0);  // Avancer à pleine vitesse si aucun obstacle
            }

            // Détection des objets avec la caméra
            List<CameraRecognitionObject> detectedObjects = cameraDetection();
            CameraRecognitionObject target = targetDetected(detectedObjects); // Vérifier si la cible est détectée

            if (target != null) {
                double[] targetPosition = target.getPosition(); // Obtenir la position de la cible
                double targetX = targetPosition[0]; // Coordonnée X de la cible
                double targetY = targetPosition[1]; // Coordonnée Y de la cible

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
        AutonomyTwo controller = new AutonomyTwo(); // Créer une instance du contrôleur
        controller.run(); // Exécuter la méthode principale
    }
}
