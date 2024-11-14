#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>

// Configurações do WiFi
const char* ssid = "Wokwi-GUEST";
const char* password = "";

// Configuração NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3 * 3600);

// Definição dos pinos - Modificado para melhor compatibilidade
#define DHT_INTERNO_PIN 4      // Mantido o pino original do interno
#define DHT_EXTERNO_PIN 32     // Alterado para pino 32 (mais estável para segundo DHT)
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
float TEMP_MAX = 40.0;
float TEMP_MIN = 20.0;
float UMID_MAX = 80.0;
float UMID_MIN = 20.0;

// Estrutura para limites de alarme
struct LimitesAlarme {
    float tempMax = 65.0;
    float tempMin = 35.0;
    float umidMax = 80.0;
    float umidMin = 5.0;
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
const unsigned long INTERVALO_ATUALIZACAO = 5000;  // 5 segundos
const unsigned long INTERVALO_LEITURA_DHT = 2500;  // 2.5 segundos

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

/**
 * Realiza a leitura dos sensores com intervalo entre cada sensor
 */
void lerSensores() {
    unsigned long tempoAtual = millis();
    bool leituraAtualizada = false;
    
    // Leitura do sensor interno
    if (tempoAtual - ultimaLeituraDHT1 >= INTERVALO_LEITURA_DHT) {
        float tempI = dhtInterno.readTemperature();
        float umidI = dhtInterno.readHumidity();
        
        if (!isnan(tempI) && !isnan(umidI)) {
            tempInterna = tempI;
            umidInterna = umidI;
            leituraAtualizada = true;
            Serial.println("Sensor Interno Atualizado - T: " + String(tempInterna) + "°C, U: " + String(umidInterna) + "%");
        } else {
            Serial.println("Falha na leitura do sensor DHT interno!");
        }
        ultimaLeituraDHT1 = tempoAtual;
    }
    
    // Leitura do sensor externo (com delay adicional)
    if (tempoAtual - ultimaLeituraDHT2 >= INTERVALO_LEITURA_DHT + 1000) {
        float tempE = dhtExterno.readTemperature();
        float umidE = dhtExterno.readHumidity();
        
        if (!isnan(tempE) && !isnan(umidE)) {
            tempExterna = tempE;
            umidExterna = umidE;
            leituraAtualizada = true;
            Serial.println("Sensor Externo Atualizado - T: " + String(tempExterna) + "°C, U: " + String(umidExterna) + "%");
        } else {
            Serial.println("Falha na leitura do sensor DHT externo!");
        }
        ultimaLeituraDHT2 = tempoAtual;
    }
    
    // Leitura do sensor de luminosidade
    luminosidade = analogRead(LDR_PIN);
    
    if (leituraAtualizada) {
        Serial.println("Luminosidade: " + String(luminosidade));
    }
}

// Funções de Controle
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

void controlarTemperatura() {
    int velocidadeVent = 0;

    if (tempInterna < TEMP_MIN) {
        digitalWrite(AQUEC_PIN, HIGH);
        velocidadeVent = 255;
        aquecimentoLigado = true;
        ventiladoresLigados = true;
    } else if (tempInterna > TEMP_MAX) {
        digitalWrite(AQUEC_PIN, LOW);
        velocidadeVent = 255;
        aquecimentoLigado = false;
        ventiladoresLigados = true;
    } else {
        digitalWrite(AQUEC_PIN, LOW);
        velocidadeVent = 128;
        aquecimentoLigado = false;
        ventiladoresLigados = true;
    }

    controlarVelocidadeVentiladores(velocidadeVent);
}

void controlarUmidade() {
    int velocidadeExaust = 0;

    if (umidInterna > UMID_MAX) {
        velocidadeExaust = 255;
        exaustoresLigados = true;
    } else if (umidInterna < UMID_MIN) {
        velocidadeExaust = 0;
        exaustoresLigados = false;
    } else {
        velocidadeExaust = 128;
        exaustoresLigados = true;
    }

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

    // Conexão WiFi
    WiFi.begin(ssid, password);
    Serial.print("Conectando ao WiFi");
    int tentativas = 0;
    while (WiFi.status() != WL_CONNECTED && tentativas < 20) {
        delay(500);
        Serial.print(".");
        tentativas++;
    }
    Serial.println();
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi conectado!");
        timeClient.begin();
    } else {
        Serial.println("Falha na conexão WiFi!");
    }

    Serial.println("Sistema iniciado!");
    lcd.clear();
}

void loop() {
    unsigned long tempoAtual = millis();

    // Atualização do tempo NTP se conectado
    if (WiFi.status() == WL_CONNECTED) {
        timeClient.update();
    }

    // Execução das rotinas de controle no intervalo definido
    if (tempoAtual - tempoUltimaAtualizacao >= INTERVALO_ATUALIZACAO) {
        lerSensores();
        verificarAlarmes();
        controlarTemperatura();
        controlarUmidade();
        atualizarDisplay();

        // Log de status do sistema
        Serial.println("\n--- Status do Sistema ---");
        Serial.println("Temperatura Interna: " + String(tempInterna) + "°C");
        Serial.println("Umidade Interna: " + String(umidInterna) + "%");
        Serial.println("Temperatura Externa: " + String(tempExterna) + "°C");
        Serial.println("Umidade Externa: " + String(umidExterna) + "%");
        Serial.println("Luminosidade: " + String(luminosidade));
        Serial.println("Status Aquecimento: " + String(aquecimentoLigado ? "ON" : "OFF"));
        Serial.println("Status Ventiladores: " + String(ventiladoresLigados ? "ON" : "OFF"));
        Serial.println("Status Exaustores: " + String(exaustoresLigados ? "ON" : "OFF"));
        Serial.println("----------------------");

        tempoUltimaAtualizacao = tempoAtual;
        digitalWrite(LED_STATUS, !digitalRead(LED_STATUS));
    }
}