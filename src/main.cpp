#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Incluir arquivo de configuração
#include "config.h"

// Clientes WiFi e MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Configuração NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3 * 3600);

// Definição dos pinos
#define DHT_INTERNO_PIN 4
#define DHT_EXTERNO_PIN 32
#define LDR_PIN 36
#define LED_STATUS 2
#define VENT_1_PIN 26
#define VENT_2_PIN 27
#define EXAUST_1_PIN 14
#define EXAUST_2_PIN 12
#define AQUEC_PIN 13

// Configurações PWM
const int CANAL_PWM_VENT1 = 0;
const int CANAL_PWM_VENT2 = 1;
const int CANAL_PWM_EXAUST1 = 2;
const int CANAL_PWM_EXAUST2 = 3;
const int FREQ_PWM = 5000;
const int RESOLUCAO_PWM = 8;

// Limites de operação
float TEMP_MAX = 70.0;
float TEMP_MIN = 30.0;
float TEMP_TARGET = 50.0;
float UMID_MAX = 60.0;
float UMID_MIN = 5.0;

// Estrutura para limites de alarme
struct LimitesAlarme {
    float tempMax = 90.0;
    float tempMin = 40.0;
    float umidMax = 80.0;
    float umidMin = 15.0;
} limites;

// Variáveis de estado
bool aquecimentoLigado = false;
bool ventiladoresLigados = false;
bool exaustoresLigados = false;
bool alarmeAtivo = false;
bool alarmeSoado = false;

// Variáveis de tempo
unsigned long tempoUltimaAtualizacao = 0;
unsigned long ultimaLeituraDHT1 = 0;
unsigned long ultimaLeituraDHT2 = 0;
const unsigned long INTERVALO_ATUALIZACAO = 3000;  // 5 segundos
const unsigned long INTERVALO_LEITURA_DHT = 2500;  // 2.5 segundos
unsigned long lastMsg = 0;
const long interval = 5000;  // Intervalo de envio de dados (5 segundos)

// Configuração dos sensores DHT22
#define DHTTYPE DHT22
DHT dhtInterno(DHT_INTERNO_PIN, DHTTYPE);
DHT dhtExterno(DHT_EXTERNO_PIN, DHTTYPE);

// Configuração do display LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Variáveis para armazenar leituras dos sensores
float tempInterna = 0;
float umidInterna = 0;
float tempExterna = 0;
float umidExterna = 0;
int luminosidade = 0;

// Protótipos das funções
void lerSensores();
void controlarTemperatura();
void controlarUmidade();
void atualizarDisplay();
void setupWiFi();
void reconnectMQTT();
void enviarDadosSerial();
void callback(char* topic, byte* payload, unsigned int length);
void enviarDadosMQTT();

/**
 * Realiza a leitura dos sensores com intervalo entre cada sensor
 */
void lerSensores() {
    unsigned long tempoAtual = millis();
    
    // Leitura do sensor interno
    if (tempoAtual - ultimaLeituraDHT1 >= INTERVALO_LEITURA_DHT) {
        tempInterna = dhtInterno.readTemperature();
        umidInterna = dhtInterno.readHumidity();
        ultimaLeituraDHT1 = tempoAtual;
    }
    
    // Leitura do sensor externo
    if (tempoAtual - ultimaLeituraDHT2 >= INTERVALO_LEITURA_DHT) {
        tempExterna = dhtExterno.readTemperature();
        umidExterna = dhtExterno.readHumidity();
        ultimaLeituraDHT2 = tempoAtual;
    }
    
    // Leitura do sensor de luminosidade
    luminosidade = analogRead(LDR_PIN);
}

/**
 * Funções de Controle
 */
void controlarVelocidadeVentiladores(int velocidade) {
    ledcWrite(CANAL_PWM_VENT1, velocidade);
    ledcWrite(CANAL_PWM_VENT2, velocidade);
}

void controlarVelocidadeExaustores(int velocidade) {
    ledcWrite(CANAL_PWM_EXAUST1, velocidade);
    ledcWrite(CANAL_PWM_EXAUST2, velocidade);
}

void verificarAlarmes() {
    bool alarmeTemperatura = (tempInterna > limites.tempMax) || (tempInterna < limites.tempMin);
    bool alarmeUmidade = (umidInterna > limites.umidMax) || (umidInterna < limites.umidMin);

    alarmeAtivo = alarmeTemperatura || alarmeUmidade;

    if (alarmeAtivo && !alarmeSoado) {
        Serial.println("### ALARME ###");
        alarmeSoado = true;
    } else if (!alarmeAtivo) {
        alarmeSoado = false;
    }
}

// Histerese para evitar oscilações
const float HISTERESE = 2.0;  // Zona morta de ±2°C

// Estrutura para controle PID
struct PIDController {
    float kp = 2.0;    // Ganho proporcional
    float ki = 0.5;    // Ganho integral
    float kd = 1.0;    // Ganho derivativo
    float lastError = 0.0;
    float integral = 0.0;
    unsigned long lastTime = 0;
} pidTemp;

// Função para calcular saída PID
int calcularPID(float setpoint, float input, PIDController &pid) {
    unsigned long now = millis();
    float deltaTime = (now - pid.lastTime) / 1000.0; // Converter para segundos
    
    if (pid.lastTime == 0) {
        pid.lastTime = now;
        return 0;
    }
    
    float error = setpoint - input;
    pid.integral += error * deltaTime;
    float derivative = (error - pid.lastError) / deltaTime;
    
    // Anti-windup para o termo integral
    pid.integral = constrain(pid.integral, -100, 100);
    
    float output = (pid.kp * error) + (pid.ki * pid.integral) + (pid.kd * derivative);
    
    pid.lastError = error;
    pid.lastTime = now;
    
    // Converter saída para faixa PWM (0-255)
    return constrain(map(output, -100, 100, 0, 255), 0, 255);
}

// Função para determinar necessidade de aquecimento baseada em histerese
bool necessitaAquecimento(float temperatura) {
    static bool aquecendo = false;
    
    if (aquecendo) {
        // Se está aquecendo, continua até atingir temperatura alvo + histerese
        if (temperatura >= TEMP_TARGET + HISTERESE) {
            aquecendo = false;
        }
    } else {
        // Se não está aquecendo, só começa se temperatura cair abaixo do alvo - histerese
        if (temperatura <= TEMP_TARGET - HISTERESE) {
            aquecendo = true;
        }
    }
    
    return aquecendo;
}

// Função para calcular velocidade dos ventiladores baseada na diferença de temperatura
int calcularVelocidadeVentiladores(float tempAtual) {
    float diffTemp = abs(TEMP_TARGET - tempAtual);
    
    // Se temperatura está muito próxima do alvo, mantém ventilação mínima
    if (diffTemp < HISTERESE) {
        return 64;  // ~25% da velocidade máxima
    }
    // Se temperatura está muito distante do alvo, ventilação máxima
    else if (diffTemp > 10) {
        return 255;  // 100% da velocidade
    }
    // Caso contrário, ajusta proporcionalmente
    else {
        return map(diffTemp, HISTERESE, 10, 64, 255);
    }
}

void controlarTemperatura() {
    // Calcula saída do controlador PID
    int saidaPID = calcularPID(TEMP_TARGET, tempInterna, pidTemp);
    
    // Verifica necessidade de aquecimento usando histerese
    bool precisaAquecer = necessitaAquecimento(tempInterna);
    
    // Calcula velocidade ideal dos ventiladores
    int velocidadeVent = calcularVelocidadeVentiladores(tempInterna);
    
    // Condições de segurança
    if (tempInterna >= TEMP_MAX) {
        digitalWrite(AQUEC_PIN, LOW);
        velocidadeVent = 255;  // Ventilação máxima
        aquecimentoLigado = false;
    }
    else if (tempInterna <= TEMP_MIN) {
        digitalWrite(AQUEC_PIN, HIGH);
        velocidadeVent = 128;  // Ventilação média
        aquecimentoLigado = true;
    }
    else {
        // Controle normal baseado na histerese
        digitalWrite(AQUEC_PIN, precisaAquecer ? HIGH : LOW);
        aquecimentoLigado = precisaAquecer;
        
        // Ajusta velocidade dos ventiladores baseado no PID se aquecimento estiver ligado
        if (precisaAquecer) {
            velocidadeVent = saidaPID;
        }
    }
    
    // Atualiza estado dos ventiladores
    ventiladoresLigados = (velocidadeVent > 0);
    controlarVelocidadeVentiladores(velocidadeVent);
}

// Função modificada para controle de umidade
void controlarUmidade() {
    int velocidadeExaust = 0;
    
    // Adiciona histerese no controle de umidade
    static bool exaustando = false;
    const float HISTERESE_UMID = 5.0;  // Histerese de 5% para umidade
    
    if (exaustando) {
        if (umidInterna <= UMID_MIN + HISTERESE_UMID) {
            exaustando = false;
        }
        velocidadeExaust = 255;  // Mantém exaustão máxima enquanto estiver acima do limite
    } else {
        if (umidInterna >= UMID_MAX - HISTERESE_UMID) {
            exaustando = true;
            velocidadeExaust = 255;
        } else {
            // Controle proporcional quando dentro dos limites
            velocidadeExaust = map(umidInterna, UMID_MIN, UMID_MAX, 0, 128);
        }
    }
    
    exaustoresLigados = (velocidadeExaust > 0);
    controlarVelocidadeExaustores(velocidadeExaust);
}

void atualizarDisplay() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(String(timeClient.getHours()) + ":" + String(timeClient.getMinutes()));
    lcd.setCursor(7, 0);
    lcd.print(alarmeAtivo ? "ALARME!" : "OK");
}

/**
 * Configuração inicial do sistema
 */
void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando sistema...");
    
    // Inicialização do LCD
    lcd.init();
    lcd.backlight();
    lcd.print("Iniciando...");

    // Inicialização dos sensores com delay entre eles
    Serial.println("Iniciando sensor interno...");
    dhtInterno.begin();
    delay(1000);  // Aguarda 1 segundo
    Serial.println("Iniciando sensor externo...");
    dhtExterno.begin();
    delay(1000);  // Aguarda 1 segundo

    // Configuração dos pinos
    pinMode(VENT_1_PIN, OUTPUT);
    pinMode(VENT_2_PIN, OUTPUT);
    pinMode(EXAUST_1_PIN, OUTPUT);
    pinMode(EXAUST_2_PIN, OUTPUT);
    pinMode(AQUEC_PIN, OUTPUT);
    pinMode(LED_STATUS, OUTPUT);
    pinMode(LDR_PIN, INPUT);

    // Configuração dos canais PWM
    ledcSetup(CANAL_PWM_VENT1, FREQ_PWM, RESOLUCAO_PWM);
    ledcSetup(CANAL_PWM_VENT2, FREQ_PWM, RESOLUCAO_PWM);
    ledcSetup(CANAL_PWM_EXAUST1, FREQ_PWM, RESOLUCAO_PWM);
    ledcSetup(CANAL_PWM_EXAUST2, FREQ_PWM, RESOLUCAO_PWM);

    ledcAttachPin(VENT_1_PIN, CANAL_PWM_VENT1);
    ledcAttachPin(VENT_2_PIN, CANAL_PWM_VENT2);
    ledcAttachPin(EXAUST_1_PIN, CANAL_PWM_EXAUST1);
    ledcAttachPin(EXAUST_2_PIN, CANAL_PWM_EXAUST2);

    // Conecta ao WiFi
    setupWiFi();
    
    // Configura MQTT
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);
    
    Serial.println("Sistema Iniciado!");
    lcd.clear();
    lcd.print("Sistema Pronto!");
    delay(2000);
}

void loop() {
    // Verifica se a conexão MQTT é bem-sucedida
    if (!client.connected()) {
        reconnectMQTT();
    }
    client.loop();
    
    unsigned long now = millis();
    if (now - lastMsg > interval) {
        lastMsg = now;
        
        // Leitura dos sensores
        lerSensores();
        
        // Controle do sistema
        controlarTemperatura();
        controlarUmidade();
        
        // Atualização das interfaces
        atualizarDisplay();
        enviarDadosMQTT();
        
        // Envia para o serial
        enviarDadosSerial();
    }
    
    // Pisca LED de status
    digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
}

void setupWiFi() {
    delay(10);
    Serial.println();
    Serial.print("Conectando a ");
    Serial.println(ssid);
    
    lcd.clear();
    lcd.print("Conectando WiFi");
    
    WiFi.begin(ssid, password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        lcd.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi conectado");
    Serial.println("IP: ");
    Serial.println(WiFi.localIP());
}

void reconnectMQTT() {
    while (!client.connected()) {
        Serial.print("Conectando ao MQTT...");
        String clientId = "ESP32Client-";
        clientId += String(random(0xffff), HEX);
        
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
            Serial.println("conectado");
            client.subscribe(topic_command);
            client.publish(topic_status, "online");
        } else {
            Serial.print("falhou, rc=");
            Serial.print(client.state());
            Serial.println(" tentando novamente em 5 segundos");
            delay(5000);
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    
    Serial.print("Mensagem recebida [");
    Serial.print(topic);
    Serial.print("] ");
    Serial.println(message);
    
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
    
    if (error) {
        Serial.print("deserializeJson() falhou: ");
        Serial.println(error.c_str());
        return;
    }
    
    // Processa comandos recebidos
    if (doc.containsKey("temp_min")) {
        TEMP_MIN = doc["temp_min"];
    }
    if (doc.containsKey("temp_max")) {
        TEMP_MAX = doc["temp_max"];
    }
    if (doc.containsKey("umid_max")) {
        UMID_MAX = doc["umid_max"];
    }
}

void enviarDadosMQTT() {
    StaticJsonDocument<300> doc;
    
    doc["temp_int"] = tempInterna;
    doc["umid_int"] = umidInterna;
    doc["temp_ext"] = tempExterna;
    doc["umid_ext"] = umidExterna;
    doc["luz"] = luminosidade;
    doc["aquec"] = aquecimentoLigado;
    doc["vent"] = ventiladoresLigados;
    doc["exaust"] = exaustoresLigados;
    doc["temp_target"] = TEMP_TARGET;
    doc["pid_output"] = pidTemp.lastError;
    
    char buffer[300];
    serializeJson(doc, buffer);
    
    client.publish(topic_data, buffer);
}

// Adicione essa função após as outras definições

void enviarDadosSerial() {
    Serial.println("\n--- Dados dos Sensores ---");
    Serial.print("Temperatura Interna: ");
    Serial.print(tempInterna);
    Serial.println(" °C");

    Serial.print("Umidade Interna: ");
    Serial.print(umidInterna);
    Serial.println(" %");

    Serial.print("Temperatura Externa: ");
    Serial.print(tempExterna);
    Serial.println(" °C");

    Serial.print("Umidade Externa: ");
    Serial.print(umidExterna);
    Serial.println(" %");

    Serial.print("Luminosidade: ");
    Serial.println(luminosidade);
    
    Serial.print("Aquecimento: ");
    Serial.println(aquecimentoLigado ? "Ligado" : "Desligado");
    
    Serial.print("Ventiladores: ");
    Serial.println(ventiladoresLigados ? "Ligado" : "Desligado");
    
    Serial.print("Exaustores: ");
    Serial.println(exaustoresLigados ? "Ligado" : "Desligado");

    Serial.println("--------------------------");
}