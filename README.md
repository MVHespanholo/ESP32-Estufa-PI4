# ESP32-Estufa-PI4
Circuito simulado no Wokwi de uma estufa de desidratação

[![Status do Projeto](https://img.shields.io/badge/Status-Em%20Desenvolvimento-yellow)](https://github.com/MVHespanholo/ESP32-Estufa-PI4)
[![Licença](https://img.shields.io/badge/Licença-MIT-blue.svg)](LICENSE)

## 📋 Índice

- [Visão Geral](#-visão-geral)
- [Funcionalidades](#-funcionalidades)
- [Requisitos de Hardware](#-requisitos-de-hardware)
- [Esquema de Conexões](#-esquema-de-conexões)
- [Configuração do Ambiente](#-configuração-do-ambiente)
- [Instalação](#-instalação)
- [Sistema de Controle](#-sistema-de-controle)
- [Comunicação MQTT](#-comunicação-mqtt)
- [Indicadores LED](#-indicadores-led)
- [Parâmetros de Configuração](#-parâmetros-de-configuração)
- [Monitoramento](#-monitoramento)
- [Resolução de Problemas](#-resolução-de-problemas)
- [Contribuição](#-contribuição)
- [Licença](#-licença)

## 🌟 Visão Geral

Este projeto implementa um sistema de controle automatizado para uma estufa de desidratação, utilizando um ESP32 como microcontrolador principal. O sistema monitora e controla temperatura, umidade e ventilação, garantindo condições ideais para o processo de desidratação.

### Características Principais:
- Controle PID de temperatura com múltiplos sensores
- Sistema de ventilação e exaustão modulado por PWM
- Interface LCD para visualização em tempo real
- Monitoramento de luminosidade ambiente
- Sistema de alarme para condições críticas
- Sincronização de tempo via NTP
- Conectividade WiFi e MQTT com Adafruit IO

## 🎯 Funcionalidades

### Controle de Ambiente
- **Temperatura**: 
  - Monitoramento interno e externo com sensores DHT22
  - Controle PID para maior precisão
  - Sistema de aquecimento com histerese
- **Umidade**: 
  - Controle automático através de sistema de ventilação
  - Histerese de 5% para estabilidade
- **Ventilação**: 
  - Sistema PWM com 4 ventiladores
  - Velocidade proporcional à demanda
  - Controle independente de entrada e exaustão
- **Aquecimento**: 
  - Controle PID do elemento aquecedor
  - Proteção contra superaquecimento
  - LED indicador de status

### Sistema de Monitoramento
- Display LCD I2C 16x2 para visualização de dados
- Sistema de 7 LEDs indicadores
- Sensor de luminosidade (LDR)
- Log detalhado via porta serial
- Monitoramento remoto via MQTT

## 🔧 Requisitos de Hardware

### Componentes Principais
- 1x ESP32 DevKit V1
- 2x Sensores DHT22
- 1x Display LCD 16x2 I2C
- 1x Sensor LDR
- 4x Ventiladores com controle PWM
- 1x Elemento aquecedor
- 7x LEDs indicadores

### Componentes Auxiliares
- Resistores:
  - 7x 220Ω (LEDs)
  - 1x 10kΩ (LDR)
- Fonte de alimentação 12V
- Cabos e conectores

## 📡 Esquema de Conexões

### Pinagem ESP32
- **Sensores DHT22**:
  - Interno: GPIO4
  - Externo: GPIO32
- **Display LCD I2C**:
  - SDA: GPIO21
  - SCL: GPIO22
- **Controle**:
  - Ventilador 1: GPIO26 (PWM)
  - Ventilador 2: GPIO27 (PWM)
  - Exaustor 1: GPIO14 (PWM)
  - Exaustor 2: GPIO12 (PWM)
  - Aquecedor: GPIO13
- **LEDs**:
  - Status: GPIO2
  - Aquecedor: GPIO17
  - Ventilador 1: GPIO18
  - Ventilador 2: GPIO5
  - Exaustor 1: GPIO19
  - Exaustor 2: GPIO23
  - Alarme: GPIO21
- **Sensores**:
  - LDR: GPIO36 (ADC)

## ⚙️ Configuração do Ambiente

### Requisitos de Software
- PlatformIO IDE
- Driver USB CP210x (ESP32)
- Bibliotecas:
  ```ini
  lib_deps =
      adafruit/DHT sensor library
      adafruit/Adafruit Unified Sensor
      marcoschwartz/LiquidCrystal_I2C
      knolleary/PubSubClient
      bblanchon/ArduinoJson
      arduino-libraries/NTPClient
  ```

### Configurações do PlatformIO
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200

build_flags = 
    -DCORE_DEBUG_LEVEL=5
    -DCONFIG_ARDUHAL_LOG_COLORS=1
```

## 📥 Instalação

1. Clone o repositório
```bash
git clone https://github.com/MVHespanholo/ESP32-Estufa-PI4
```

2. Copie e configure o arquivo de configuração
```bash
cp config.example.h config.h
# Edite config.h com suas credenciais
```

3. Configure as credenciais em `config.h`:
```cpp
// Configurações do WiFi
const char* ssid = "SEU_SSID";
const char* password = "SUA_SENHA";

// Configurações do MQTT
const char* mqtt_user = "SEU_USUARIO_ADAFRUIT";
const char* mqtt_password = "SUA_CHAVE_ADAFRUIT";
```

## 🎮 Sistema de Controle

### Controle PID de Temperatura
```cpp
struct PIDController {
    float kp = 2.0;    // Ganho proporcional
    float ki = 0.5;    // Ganho integral
    float kd = 1.0;    // Ganho derivativo
    float lastError = 0.0;
    float integral = 0.0;
} pidTemp;
```

### Controle de Umidade
- Sistema com histerese de 5%
- Controle proporcional dos exaustores
- Velocidade adaptativa baseada na diferença

## 📡 Comunicação MQTT

### Tópicos
- `username/feeds/estufa.dados`: Dados dos sensores
- `username/feeds/estufa.comandos`: Recebimento de comandos
- `username/feeds/estufa.status`: Status do sistema

### Formato dos Dados
```json
{
    "time": 1234567890,
    "internalTemperature": 45.2,
    "externalTemperature": 25.1,
    "internalHumidity": 30.5,
    "externalHumidity": 65.0,
    "luminosity": 850
}
```

## 💡 Indicadores LED

| LED | Cor | Função | Estado |
|-----|-----|--------|---------|
| STATUS | Amarelo | Status do Sistema | Piscando: Sistema operacional |
| AQUEC | Vermelho | Aquecedor | Fixo: Aquecedor ligado |
| VENT1 | Verde | Ventilador 1 | Piscando: Velocidade proporcional |
| VENT2 | Verde | Ventilador 2 | Piscando: Velocidade proporcional |
| EXAUST1 | Branco | Exaustor 1 | Piscando: Velocidade proporcional |
| EXAUST2 | Branco | Exaustor 2 | Piscando: Velocidade proporcional |
| ALARM | Vermelho | Alarme | Piscando rápido: Condição crítica |

## ⚡ Parâmetros de Configuração

### Limites de Operação
```cpp
float TEMP_MAX = 70.0;  // Temperatura máxima
float TEMP_MIN = 30.0;  // Temperatura mínima
float TEMP_TARGET = 50.0;  // Temperatura alvo
float UMID_MAX = 60.0;  // Umidade máxima
float UMID_MIN = 5.0;   // Umidade mínima
```

### Limites de Alarme
```cpp
struct LimitesAlarme {
    float tempMax = 90.0;
    float tempMin = 40.0;
    float umidMax = 80.0;
    float umidMin = 15.0;
} limites;
```

## 📊 Monitoramento

### Serial Monitor
- Baud rate: 115200
- Log detalhado:
  ```
  --- Status do Sistema ---
  Hora: HH:MM
  Temperatura Interna: XX.X°C
  Umidade Interna: XX.X%
  Temperatura Externa: XX.X°C
  Umidade Externa: XX.X%
  Luminosidade: XXXX
  --- Estado dos Atuadores ---
  Aquecimento: ON/OFF
  Ventiladores: XX% (Velocidade)
  Exaustores: XX% (Velocidade)
  ```

### Display LCD
- Linha 1: Status do sistema e temperatura
- Linha 2: Umidade e estado dos atuadores

## 🔍 Resolução de Problemas

### Problemas Comuns

1. **Sistema não Conecta ao WiFi**
   - Verifique as credenciais em `config.h`
   - Confirme a força do sinal
   - Reinicie o ESP32

2. **Sensores DHT com Leituras Incorretas**
   - Verifique a alimentação (3.3V)
   - Confirme os resistores pull-up
   - Aumente o intervalo entre leituras

3. **Controle PID Instável**
   - Ajuste os ganhos (kp, ki, kd)
   - Verifique o posicionamento dos sensores
   - Aumente a histerese se necessário

### LEDs de Diagnóstico
- LED Status piscando lento: Sistema normal
- LED Status piscando rápido: Erro de conexão
- LED Alarme piscando: Condição crítica

## 🤝 Contribuição

1. Faça um Fork do projeto
2. Crie uma Branch para sua Feature (`git checkout -b feature/AmazingFeature`)
3. Commit suas mudanças (`git commit -m 'Add some AmazingFeature'`)
4. Push para a Branch (`git push origin feature/AmazingFeature`)
5. Abra um Pull Request

## 📄 Licença

Este projeto está sob a licença MIT. Veja o arquivo [LICENSE](LICENSE) para mais detalhes.

---

Desenvolvido por [Marcos Vinicius Hespanholo](https://github.com/MVHespanholo)