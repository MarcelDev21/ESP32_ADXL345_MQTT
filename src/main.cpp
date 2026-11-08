
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <stdio.h>

// =====================================================
// 1. CONFIGURATION DU CAPTEUR ADXL345
// =====================================================

#define SDA_PIN 21
#define SCL_PIN 22
#define ADXL345_ADDRESS 0x53

Adafruit_ADXL345_Unified accel =Adafruit_ADXL345_Unified(12345);

// =====================================================
// 2. CONFIGURATION WIFI
// =====================================================

const char* ssid = "DIGIFIBRA-efF3";
const char* password = "zS{&Tb|)4bC|";

// =====================================================
// 3. CONFIGURATION MQTT
// =====================================================

const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;

const char* topic = "tp/esp32/mpu6050/orientation/test-esp32-6842";

WiFiClient espClient;
PubSubClient client(espClient);

// =====================================================
// 4. ORIENTATIONS ET STRUCTURE DES MESSAGES
// =====================================================

// orientacíon
enum Orientation : uint8_t {
    UP,
    DOWN,
    LEFT,
    RIGHT
};

// orientacíon, acceleracíon x, accelaration y, accelaration z
struct MessageOrientation {
    Orientation orientation;
    float ax;
    float ay;
    float az;
};

// =====================================================
// 5. VARIABLES GLOBALES
// =====================================================

QueueHandle_t fileOrientations = nullptr;

Orientation derniereOrientation = UP;
bool premiereMesure = true;

// Compteurs d'erreurs //Contadores de errores
uint32_t nombreMessagesPerdus = 0;
uint32_t nombreErreursCapteur = 0;
uint32_t nombreErreursPublication = 0;
uint32_t nombreTentativesWiFi = 0;
uint32_t nombreTentativesMQTT = 0;

// Délais de reconnexion //Plazos de reconexión
const unsigned long DELAI_WIFI = 10000;
const unsigned long DELAI_MQTT = 5000;

unsigned long derniereTentativeWiFi = 0;
unsigned long derniereTentativeMQTT = 0;

// =====================================================
// 6. CONVERSION DE L'ORIENTATION EN TEXTE
// =====================================================

//convertir un valor de orientación en texto
const char* nomOrientation(Orientation orientation) {

    switch (orientation) {

        case UP:
            return "UP";

        case DOWN:
            return "DOWN";

        case LEFT:
            return "LEFT";

        case RIGHT:
            return "RIGHT";

        default:
            return "UNKNOWN";
    }
}

// =====================================================
// 7. VERIFICATION DE LA PRESENCE DU CAPTEUR I2C
// =====================================================

// Comprobar si el sensor ADXL345 se detecta correctamente y se comunica con el ESP32 a través del bus I2C.
bool capteurPresent() {

    Wire.beginTransmission(ADXL345_ADDRESS);

    return Wire.endTransmission() == 0;
}

// =====================================================
// 8. LECTURE DE L'ACCELEROMETRE
// =====================================================

//Leer los tres valores de aceleración (X, Y y Z) del sensor ADXL345 y, a continuación, comprobar si los valores obtenidos son correctos.
bool lireAccelerometre(float &ax, float &ay, float &az) {

    // Vérifier que le capteur répond sur le bus I2C
    if (!capteurPresent()) {
        return false;
    }

    //créer une variable appelée event qui va recevoir les informations mesurées par le capteur ADXL345
    sensors_event_t event;

    // Lecture des accélérations
    //Cette instruction demande à la bibliothèque de récupérer les mesures du capteur ADXL345.
    accel.getEvent(&event);

    ax = event.acceleration.x;
    ay = event.acceleration.y;
    az = event.acceleration.z;

    // Vérifier que les valeurs sont exploitables
    if (!isfinite(ax) || !isfinite(ay) || !isfinite(az)) {
        return false;
    }

    return true;
}

// =====================================================
// 9. DETECTION DE L'ORIENTATION
// =====================================================

// Retourne true si une orientation nette est détectée.
// Sinon retourne false : l'orientation précédente est conservée.
/*bool detecterOrientation(float ax, float ay, float az,Orientation &orientation) {

    const float seuil = 7.0f;

    // Axe Z dominant
    if (fabsf(az) >= fabsf(ax) && fabsf(az) >= fabsf(ay) && fabsf(az) > seuil) {
        orientation = (az > 0) ? UP : DOWN;
        return true;
    }

    // Axe X dominant
    if (fabsf(ax) >= fabsf(ay) &&
        fabsf(ax) >= fabsf(az) &&
        fabsf(ax) > seuil) {

        orientation = (ax > 0) ? RIGHT : LEFT;
        return true;
    }

    // Aucune orientation nette
    return false;
}*/

bool detecterOrientation(float ax, float ay, float az, Orientation &orientation) {

    const float seuil = 7.0f;

    // Si Z est l'axe dominant
    if (fabsf(az) > seuil && fabsf(az) >= fabsf(ax) && fabsf(az) >= fabsf(ay)) {
        if (az > 0) {
            orientation = UP;
        } else {
            orientation = DOWN;
        }
        return true;
    }

    // Si X est l'axe dominant
    if (fabsf(ax) > seuil && fabsf(ax) >= fabsf(ay) && fabsf(ax) >= fabsf(az)) {
        if (ax > 0) {
            orientation = RIGHT;
        } else {
            orientation = LEFT;
        }
        return true;
    }

    return false;
}

// =====================================================
// 10. TACHE FREERTOS : LECTURE DU CAPTEUR
// =====================================================

void tacheCapteur(void *parameter) {

    float ax, ay, az;

    uint8_t erreursConsecutives = 0;

    while (true) {

        // Lire le capteur
        if (!lireAccelerometre(ax, ay, az)) {

            nombreErreursCapteur++;
            erreursConsecutives++;

            Serial.println("[ERREUR] Lecture ADXL345 impossible !");

            Serial.print("[CAPTEUR] Nombre d'erreurs : ");
            Serial.println(nombreErreursCapteur);

            // Après plusieurs échecs, tenter de réinitialiser
            if (erreursConsecutives >= 5) {

                Serial.println( "[CAPTEUR] Tentative de reinitialisation..." );

                if (accel.begin()) {

                    accel.setRange(ADXL345_RANGE_2_G);

                    Serial.println( "[CAPTEUR] ADXL345 reinitialise !" );

                } else {

                    Serial.println("[CAPTEUR] Reinitialisation impossible !");
                }

                erreursConsecutives = 0;
            }

            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        erreursConsecutives = 0;

        // Déterminer l'orientation
        Orientation orientation;

        // Position intermédiaire : on garde l'orientation précédente
        if (!detecterOrientation(ax, ay, az, orientation)) {
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        // Enviar solo si cambia la orientación // Envoyer uniquement si l'orientation change
        if (premiereMesure ||
            orientation != derniereOrientation) {

            premiereMesure = false;
            derniereOrientation = orientation;

            MessageOrientation message;

            message.orientation = orientation;
            message.ax = ax;
            message.ay = ay;
            message.az = az;

            // Envoyer dans la queue
            if (xQueueSend( fileOrientations, &message, 0 ) == pdPASS) {
                Serial.print("[CAPTEUR] Orientation : ");
                Serial.println(nomOrientation(orientation));

            } else {

                nombreMessagesPerdus++;

                Serial.println("[ERREUR] Queue pleine !");

                Serial.print("[ERREUR] Messages perdus : ");
                Serial.println(nombreMessagesPerdus);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =====================================================
// 11. GESTION DE LA RECONNEXION WIFI
// =====================================================

void gererWiFi() {

// comprabado que el wifi esta conectado
    if (WiFi.status() == WL_CONNECTED) {
        return;
    }

    unsigned long maintenant = millis();

    // Ne pas recommencer trop souvent
    if (maintenant - derniereTentativeWiFi >= DELAI_WIFI) {

        derniereTentativeWiFi = maintenant;
        nombreTentativesWiFi++;

        Serial.println("[Wi-Fi] Connexion perdue !"); // connexion perdida
        Serial.println("[Wi-Fi] Nouvelle tentative..."); // nueva tentativa

        WiFi.reconnect();

        Serial.print("[Wi-Fi] Tentative numero : "); // tentativa numero ...
        Serial.println(nombreTentativesWiFi);
    }
}

// =====================================================
// 12. GESTION DE LA RECONNEXION MQTT
// =====================================================

void gererMQTT() {

    //  MQTT no puede funcionar sin Wi-Fi //MQTT ne peut pas fonctionner sans Wi-Fi
    if (WiFi.status() != WL_CONNECTED) {
        return;
    }

    //comprueba el estado de la conexión entre tu ESP32 y el broker MQTT
    if (client.connected()) {
        return;
    }

    unsigned long maintenant = millis();

    // Attendre avant une nouvelle tentative
    if (maintenant - derniereTentativeMQTT < DELAI_MQTT) {
        return;
    }

    derniereTentativeMQTT = maintenant;
    nombreTentativesMQTT++;

    Serial.println("[MQTT] Connexion au broker...");

    char clientId[40];

    snprintf( clientId, sizeof(clientId), "ESP32-ADXL345-%lu", (unsigned long)millis() );

    if (client.connect(clientId)) {

        Serial.println("[MQTT] Connexion reussie !");

    } else {

        Serial.print("[MQTT] Echec. Code : ");
        Serial.println(client.state());

        Serial.print("[MQTT] Tentative numero : ");
        Serial.println(nombreTentativesMQTT);
    }
}

// =====================================================
// 13. TACHE FREERTOS : COMMUNICATION MQTT
// =====================================================

void tacheMQTT(void *parameter) {

    MessageOrientation message;

    // Conserver le message en attente si publication échoue
    bool messageEnAttente = false;

    while (true) {

        // Vérifier et gérer le Wi-Fi
        gererWiFi();

        // Vérifier et gérer MQTT
        gererMQTT();

        // Entretenir la connexion MQTT
        if (client.connected()) {

            client.loop();

            // Récupérer un message uniquement si aucun
            // autre message n'est en attente de publication
            if (!messageEnAttente) {

                if (xQueueReceive( fileOrientations, &message, pdMS_TO_TICKS(50) ) == pdPASS) {
                    messageEnAttente = true;
                }
            }

            // Publier le message en attente
            if (messageEnAttente) {
                const char* orientation =nomOrientation(message.orientation);

                if (client.publish(topic, orientation)) {

                    Serial.print("[MQTT] Message publie : ");
                    Serial.println(orientation);

                    messageEnAttente = false;

                } else {

                    nombreErreursPublication++;

                    Serial.println("[MQTT] Echec de publication !"  );

                    Serial.print( "[MQTT] Nombre d'echecs : " );

                    Serial.println(nombreErreursPublication);
                }
            }

        } else {

            // Si MQTT est indisponible, les messages restent
            // dans la queue, sauf si celle-ci arrive à saturation.
            vTaskDelay(pdMS_TO_TICKS(50));
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =====================================================
// 14. SETUP
// =====================================================

void setup() {

    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("====================================");
    Serial.println(" ESP32 - ADXL345 - FREERTOS - MQTT");
    Serial.println("====================================");

    // Initialisation I2C
    Wire.begin(SDA_PIN, SCL_PIN);

    // Initialisation du capteur
    Serial.println("[INIT] Initialisation ADXL345...");

    if (!accel.begin()) {
        Serial.println("[ERREUR] ADXL345 non detecte !" );
        while (true) {
            delay(1000);
        }
    }

    accel.setRange(ADXL345_RANGE_2_G);

    Serial.println("[INIT] ADXL345 initialise !");

    // Initialisation Wi-Fi sans bloquer le setup
    // inicializamos el wifi
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);

    Serial.println("[Wi-Fi] Demande de connexion envoyee.");

    // Configuration MQTT
    client.setServer(mqtt_server, mqtt_port);
    client.setKeepAlive(30);
    client.setSocketTimeout(3);

    // Création de la queue
    // create queue
    fileOrientations = xQueueCreate( 5, sizeof(MessageOrientation));
    if (fileOrientations == nullptr) {
        Serial.println( "[ERREUR] Creation de la queue impossible !" );
        while (true) {
            delay(1000);
        }
    }

    Serial.println("[INIT] Queue creee avec succes !");

    // Création de la tâche capteur
    //Creación de la tarea del sensor
    BaseType_t resultat1 = xTaskCreate(
        tacheCapteur,
        "TacheCapteur",
        4096,
        nullptr,
        1,
        nullptr
    );

    // Création de la tâche MQTT
    // créacion de la tarea mqtt
    BaseType_t resultat2 = xTaskCreate(
        tacheMQTT,
        "TacheMQTT",
        6144,
        nullptr,
        1,
        nullptr
    );

    // Vérifier la création des tâches
    // comprobamos
    if (resultat1 != pdPASS || resultat2 != pdPASS) {

        Serial.println( "[ERREUR] Creation d'une tache impossible !" );

        while (true) {
            delay(1000);
        }
    }

    Serial.println("[INIT] Les deux taches FreeRTOS sont creees !");
}

// =====================================================
// 15. LOOP
// =====================================================

void loop() {

    vTaskDelay(pdMS_TO_TICKS(1000));
}
