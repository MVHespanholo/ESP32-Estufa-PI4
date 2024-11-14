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
- [Parâmetros de Configuração](#-parâmetros-de-configuração)
- [Monitoramento](#-monitoramento)
- [Resolução de Problemas](#-resolução-de-problemas)
- [Contribuição](#-contribuição)
- [Licença](#-licença)

## 🌟 Visão Geral

Este projeto implementa um sistema de controle automatizado para uma estufa de desidratação, utilizando um ESP32 como microcontrolador principal. O sistema monitora e controla temperatura, umidade e ventilação, garantindo condições ideais para o processo de desidratação.

### Características Principais:
- Controle de temperatura e umidade com múltiplos sensores
- Sistema de ventilação e exaustão modulado por PWM
- Interface LCD para visualização em tempo real
- Monitoramento de luminosidade ambiente
- Sistema de alarme para condições fora dos parâmetros
- Sincronização de tempo via NTP
- Conectividade WiFi para monitoramento remoto

## 🎯 Funcionalidades

### Controle de Ambiente
- **Temperatura**: Monitoramento interno e externo com sensores DHT22
- **Umidade**: Controle automático através de sistema de ventilação
- **Ventilação**: Sistema PWM com 4 ventiladores (2 para entrada, 2 para exaustão)
- **Aquecimento**: Controle de elemento aquecedor para manutenção da temperatura

### Sistema de Monitoramento
- Display LCD I2C 16x2 para visualização de dados
- LED indicador de status do sistema
- Sensor de luminosidade (LDR)
- Log detalhado via porta serial

### Parâmetros de Operação
- Temperatura: 20°C a 40°C (configurável)
- Umidade: 20% a 80% (configurável)
- Limites de alarme personalizáveis
- Intervalos de atualização ajustáveis

## 🔧 Requisitos de Hardware

### Componentes Principais
- 1x ESP32 DevKit V1
- 2x Sensores DHT22
- 1x Display LCD 16x2 I2C
- 1x Sensor LDR
- 4x Ventiladores 12V
- 1x Elemento aquecedor
- LEDs indicadores

### Componentes Auxiliares
- Resistores:
  - 5x 220Ω (LEDs)
  - 1x 10kΩ (LDR)
- Fonte de alimentação 12V
- Cabos e conectores

## 📡 Esquema de Conexões

### Pinagem ESP32
- **Sensores DHT22**:
  - Interno: GPIO4
  - Externo: GPIO16
- **Display LCD I2C**:
  - SDA: GPIO21
  - SCL: GPIO22
- **Controle**:
  - Ventilador 1: GPIO26 (PWM)
  - Ventilador 2: GPIO27 (PWM)
  - Exaustor 1: GPIO14 (PWM)
  - Exaustor 2: GPIO12 (PWM)
  - Aquecedor: GPIO13
- **Sensores**:
  - LDR: GPIO36 (ADC)
  - LED Status: GPIO2

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
      arduino-libraries/NTPClient
  ```

### Configurações do PlatformIO
```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```

## 📥 Instalação

1. Clone o repositório
```bash
git clone https://github.com/MVHespanholo/ESP32-Estufa-PI4
```

2. Abra o projeto no PlatformIO IDE

3. Instale as dependências
```bash
pio lib install
```

4. Configure o WiFi
```cpp
const char* ssid = "seu-wifi";
const char* password = "sua-senha";
```

5. Compile e faça o upload para o ESP32

## ⚡ Parâmetros de Configuração

### Limites de Operação
```cpp
float TEMP_MAX = 40.0;  // Temperatura máxima
float TEMP_MIN = 20.0;  // Temperatura mínima
float UMID_MAX = 80.0;  // Umidade máxima
float UMID_MIN = 20.0;  // Umidade mínima
```

### Limites de Alarme
```cpp
struct LimitesAlarme {
    float tempMax = 65.0;
    float tempMin = 35.0;
    float umidMax = 80.0;
    float umidMin = 5.0;
} limites;
```

## 📊 Monitoramento

### Serial Monitor
- Baud rate: 115200
- Formato de log:
  ```
  --- Status do Sistema ---
  Temperatura Interna: XX.X°C
  Umidade Interna: XX.X%
  Temperatura Externa: XX.X°C
  Umidade Externa: XX.X%
  Luminosidade: XXXX
  Status Aquecimento: ON/OFF
  Status Ventiladores: ON/OFF
  Status Exaustores: ON/OFF
  ```

### Display LCD
- Linha 1: Hora atual e status do sistema
- Linha 2: Temperatura e umidade atual

## 🔍 Resolução de Problemas

### Problemas Comuns e Soluções

1. **Falha na Leitura dos Sensores DHT**
   - Verifique a alimentação (3.3V)
   - Confirme os resistores pull-up
   - Aumente o intervalo entre leituras
   - Verifique a integridade dos cabos

2. **Display LCD não Inicializa**
   - Confirme o endereço I2C (padrão: 0x27)
   - Verifique as conexões SDA/SCL
   - Teste a alimentação do módulo

3. **Ventiladores não Funcionam**
   - Verifique a configuração PWM
   - Confirme a alimentação 12V
   - Teste a continuidade dos cabos

### Códigos de Erro
- Falhas consecutivas são registradas no Serial Monitor
- Alarmes são indicados no display LCD
- LED de status pisca em caso de erro

## 🤝 Contribuição

1. Faça um Fork do projeto
2. Crie uma Branch para sua Feature (`git checkout -b feature/AmazingFeature`)
3. Commit suas mudanças (`git commit -m 'Add some AmazingFeature'`)
4. Push para a Branch (`git push origin feature/AmazingFeature`)
5. Abra um Pull Request

## 📄 Licença

Este projeto está sob a licença MIT. Veja o arquivo [LICENSE](LICENSE) para mais detalhes.

---

Desenvolvido por [Marcos Vinicius](https://github.com/MVHespanholo)