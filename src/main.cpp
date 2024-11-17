#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "config.h"

// Definição dos pinos
#define DHT_INTERNO_PIN 4
#define DHT_EXTERNO_PIN 32
#define LDR_PIN 36
#define VENT_1_PIN 26
#define VENT_2_PIN 27
#define EXAUST_1_PIN 14
#define EXAUST_2_PIN 12
#define AQUEC_PIN 13
#define LED_STATUS      2   // LED de status do sistema
#define LED_AQUEC       17  // LED indicador do aquecedor
#define LED_VENT1       18  // LED indicador do ventilador 1
#define LED_VENT2       5   // LED indicador do ventilador 2
#define LED_EXAUST1     19  // LED indicador do exaustor 1
#define LED_EXAUST2     23  // LED indicador do exaustor 2
#define LED_ALARM       21  // LED indicador de alarme

// Configurações PWM
const int CANAL_PWM_VENT1 = 0;
const int CANAL_PWM_VENT2 = 1;
const int CANAL_PWM_EXAUST1 = 2;
const int CANAL_PWM_EXAUST2 = 3;
const int FREQ_PWM = 5000;
const int RESOLUCAO_PWM = 8;

// Limites de operação
float TEMP_MAX = 100.0;
float TEMP_MIN = 30.0;
float TEMP_TARGET = 50.0;
float UMID_MAX = 90.0;
float UMID_MIN = 5.0;

// Estrutura para limites de alarme
struct LimitesAlarme {
    float tempMax = 80.0;
    float tempMin = 40.0;
    float umidMax = 80.0;
    float umidMin = 10.0;
} limites;

// Variáveis de estado
bool aquecimentoLigado = false;
bool ventiladoresLigados = false;
bool exaustoresLigados = false;
bool alarmeAtivo = false;

// Variáveis de tempo
unsigned long ultimaLeituraDHT1 = 0;
unsigned long ultimaLeituraDHT2 = 0;
const unsigned long INTERVALO_ATUALIZACAO = 3000;  // 5 segundos
const unsigned long INTERVALO_LEITURA_DHT = 2500;  // 2.5 segundos
unsigned long lastMsg = 0;
const long interval = 5000;  // Intervalo de envio de dados (5 segundos)

// Variáveis para controle de velocidade
int velocidadeVentiladores = 0;  // Velocidade atual dos ventiladores
int velocidadeExaustores = 0;   // Velocidade atual dos exaustores

// Estrutura para controle de padrões de pisca
struct BlinkPattern {
    unsigned long interval;
    unsigned long lastChange;
    bool state;
    bool enabled;
};

// Padrões de pisca para cada LED
BlinkPattern ledPatterns[] = {
    {500, 0, false, true},   // STATUS
    {250, 0, false, false},  // AQUEC
    {250, 0, false, false},  // VENT1
    {250, 0, false, false},  // VENT2
    {250, 0, false, false},  // EXAUST1
    {250, 0, false, false},  // EXAUST2
    {250, 0, false, false}   // ALARM
};

// Array com os pinos dos LEDs
const int ledPins[] = {
    LED_STATUS, LED_AQUEC, LED_VENT1, LED_VENT2, 
    LED_EXAUST1, LED_EXAUST2, LED_ALARM
};

// Enumeração para facilitar o acesso aos LEDs
enum LedIndex {
    LED_IDX_STATUS = 0,
    LED_IDX_AQUEC = 1,
    LED_IDX_VENT1 = 2,
    LED_IDX_VENT2 = 3,
    LED_IDX_EXAUST1 = 4,
    LED_IDX_EXAUST2 = 5,
    LED_IDX_ALARM = 6
};

// Clientes WiFi e MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Configuração NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3 * 3600);

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
void setupWiFi();
void reconnectMQTT();
void enviarDadosSerial();
void callback(char* topic, byte* payload, unsigned int length);
void enviarDadosMQTT();

// Função para inicializar os LEDs
void setupLEDs() {
    for (int i = 0; i < sizeof(ledPins)/sizeof(ledPins[0]); i++) {
        pinMode(ledPins[i], OUTPUT);
        digitalWrite(ledPins[i], LOW);
    }
}

// Função para atualizar o estado de um LED específico
void updateLED(LedIndex index, bool enabled, unsigned long interval = 0) {
    ledPatterns[index].enabled = enabled;
    if (interval > 0) {
        ledPatterns[index].interval = interval;
    }
}

// Função para atualizar todos os LEDs
void updateLEDs() {
    unsigned long currentMillis = millis();
    
    for (int i = 0; i < sizeof(ledPins)/sizeof(ledPins[0]); i++) {
        BlinkPattern& pattern = ledPatterns[i];
        
        if (!pattern.enabled) {
            digitalWrite(ledPins[i], LOW);
            continue;
        }
        
        // Se é um LED que deve ficar fixo (interval = 0)
        if (pattern.interval == 0) {
            digitalWrite(ledPins[i], HIGH);
            continue;
        }
        
        // Atualiza o estado do LED de acordo com o padrão de pisca
        if (currentMillis - pattern.lastChange >= pattern.interval) {
            pattern.state = !pattern.state;
            digitalWrite(ledPins[i], pattern.state);
            pattern.lastChange = currentMillis;
        }
    }
}

// Função para atualizar estados dos LEDs baseado no estado do sistema
void updateSystemLEDs() {
    // Status do sistema sempre piscando
    updateLED(LED_IDX_STATUS, true, 500);
    
    // Aquecimento
    updateLED(LED_IDX_AQUEC, aquecimentoLigado, 0);  // Intervalo 0 = LED fixo
    
    // Ventiladores - Velocidade do pisca proporcional à velocidade
    updateLED(LED_IDX_VENT1, ventiladoresLigados, ventiladoresLigados ? map(velocidadeVentiladores, 0, 255, 500, 100) : 0);
    updateLED(LED_IDX_VENT2, ventiladoresLigados, ventiladoresLigados ? map(velocidadeVentiladores, 0, 255, 500, 100) : 0);
    
    // Exaustores - Velocidade do pisca proporcional à velocidade
    updateLED(LED_IDX_EXAUST1, exaustoresLigados, exaustoresLigados ? map(velocidadeExaustores, 0, 255, 500, 100) : 0);
    updateLED(LED_IDX_EXAUST2, exaustoresLigados, exaustoresLigados ? map(velocidadeExaustores, 0, 255, 500, 100) : 0);
    
    // Alarme - LED piscando apenas quando ativo
    updateLED(LED_IDX_ALARM, alarmeAtivo, alarmeAtivo ? 100 : 0);
}

 // Realiza a leitura dos sensores com intervalo entre cada sensor
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

 // Funções de Controle
void controlarVelocidadeVentiladores(int velocidade) {
    velocidadeVentiladores = velocidade;
    ledcWrite(CANAL_PWM_VENT1, velocidade);
    ledcWrite(CANAL_PWM_VENT2, velocidade);
}

void controlarVelocidadeExaustores(int velocidade) {
    velocidadeExaustores = velocidade;
    ledcWrite(CANAL_PWM_EXAUST1, velocidade);
    ledcWrite(CANAL_PWM_EXAUST2, velocidade);
}

void verificarAlarmes() {
    bool alarmeTemperatura = (tempInterna > limites.tempMax) || (tempInterna < limites.tempMin);
    bool alarmeUmidade = (umidInterna > limites.umidMax) || (umidInterna < limites.umidMin);

    // Atualiza estado do alarme
    alarmeAtivo = alarmeTemperatura || alarmeUmidade;
    
    // Se houver mudança no estado do alarme
    static bool estadoAnterior = false;
    if (alarmeAtivo != estadoAnterior) {
        estadoAnterior = alarmeAtivo;
        // Envia dados apenas quando há mudança de estado
        enviarDadosSerial();
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
 // Configuração inicial do sistema
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

    // Inicialização dos LEDs
    setupLEDs();
    updateLED(LED_IDX_STATUS, true, 500);

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

        enviarDadosMQTT();

        enviarDadosSerial();

        updateSystemLEDs();

        verificarAlarmes();
    }
    
    // Atualizar estados dos LEDs
    updateLEDs();
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
    
    // Adiciona timestamp Unix
    doc["time"] = timeClient.getEpochTime();
    
    // Dados dos sensores no formato esperado pela API
    doc["internalTemperature"] = tempInterna;
    doc["externalTemperature"] = tempExterna;
    doc["internalHumidity"] = umidInterna;
    doc["externalHumidity"] = umidExterna;
    doc["luminosity"] = luminosidade;
    
    char buffer[300];
    serializeJson(doc, buffer);
    
    client.publish(topic_data, buffer);
}

void enviarDadosSerial() {
    Serial.println("\n--- Status do Sistema ---");
    Serial.print("Hora: ");
    Serial.print(timeClient.getHours());
    Serial.print(":");
    Serial.println(timeClient.getMinutes());
    if (alarmeAtivo) {
        Serial.println("### ALARME ATIVO ###");
        if (tempInterna > limites.tempMax) {
            Serial.println("Temperatura acima do limite!");
        }
        if (tempInterna < limites.tempMin) {
            Serial.println("Temperatura abaixo do limite!");
        }
        if (umidInterna > limites.umidMax) {
            Serial.println("Umidade acima do limite!");
        }
        if (umidInterna < limites.umidMin) {
            Serial.println("Umidade abaixo do limite!");
        }
    } else {
        Serial.println("Sistema OK");
    }

    Serial.println("--- Dados dos Sensores ---");
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
    
    Serial.println("--- Estado dos Atuadores ---");
    Serial.print("Aquecimento: ");
    Serial.println(aquecimentoLigado ? "Ligado" : "Desligado");
    
    Serial.print("Ventiladores: ");
    Serial.print(ventiladoresLigados ? "Ligado" : "Desligado");
    if (ventiladoresLigados) {
        Serial.print(" (Velocidade: ");
        Serial.print(map(velocidadeVentiladores, 0, 255, 0, 100));
        Serial.println("%)");
    } else {
        Serial.println();
    }
    
    Serial.print("Exaustores: ");
    Serial.print(exaustoresLigados ? "Ligado" : "Desligado");
    if (exaustoresLigados) {
        Serial.print(" (Velocidade: ");
        Serial.print(map(velocidadeExaustores, 0, 255, 0, 100));
        Serial.println("%)");
    } else {
        Serial.println();
    }
    Serial.println("--------------------------");
}