#include <WiFi.h>
#include <PubSubClient.h> // Biblioteca para MQTT

// Defina as credenciais da sua rede Wi-Fi
const char* ssid = "Nome_Wifi"; // Substitua pelo nome da sua rede Wifi
const char* password = "Senha_Wifi"; // Substitua pela sua senha da sua rede Wifi

// Configurações do MQTT
const char* mqttServer = "Endereço_Broker"; // Substitua pelo endereço do seu broker MQTT
const int mqttPort = 1883; // Porta do broker MQTT geralmente 1883

WiFiClient espClient; // Cliente Wi-Fi
PubSubClient client(espClient); // Cliente MQTT

int PinoAnalogico = 34; // Define o pino 34 como Pino Analógico do sensor
int PinoDigital = 12; // Define pino D12 como Pino Digital do Sensor 
int Rele = 14; // Pino Digital D14 como Relé
int LedVermelho = 26; // Pino Digital D26 para o LED Vermelho
int LedVerde = 27; // Pino Digital D27 para o LED Verde

void setup() {
    Serial.begin(9600); 
    pinMode(Rele, OUTPUT); // Declara o Relé como Saída Digital 
    pinMode(LedVermelho, OUTPUT); // Declara o LED Vermelho como Saída Digital
    pinMode(LedVerde, OUTPUT); // Declara o LED Verde como Saída Digital
    pinMode(PinoDigital, INPUT);

    // Conecta ao Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Conectado ao Wi-Fi");

    // Configura o servidor MQTT
    client.setServer(mqttServer, mqttPort);
}

void reconnect() {
    // Loop até que esteja conectado
    while (!client.connected()) {
        Serial.print("Tentando conexão MQTT...");
        // Tenta conectar
        if (client.connect("ESP32Client")) { // ID do cliente MQTT
            Serial.println("Conectado ao broker MQTT");
        } else {
            Serial.print("Falha na conexão, rc=");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void loop() {
    if (!client.connected()) {
        reconnect(); // Conecta se não estiver conectado
    }
    client.loop(); // Mantém a conexão MQTT

    int ValAnalogIn = analogRead(PinoAnalogico); 
    int Porcento = map(ValAnalogIn, 733, 595, 0, 100); // Altere os valores para deixar o sensor de umidade de solo calibrado
    Porcento = constrain(Porcento, 0, 100); // Porcentagem não passa de 100% e nem menos de 0%
    
    // Publicar a umidade no tópico
    String mensagem = String(Porcento);
    client.publish("SensorUmidade", mensagem.c_str()); // Publica a umidade no tópico "SensorUmidade. É possivel mudar o nome do tópico"

    // Imprime a porcentagem no monitor serial
    Serial.print("Umidade: ");
    Serial.print(Porcento);
    Serial.println("%");

    // Controle do relé e dos LEDs com base na umidade
    if (Porcento <= 60) {
        Serial.println("Irrigando Planta");
        digitalWrite(Rele, LOW); // Liga o relé
        digitalWrite(LedVermelho, HIGH); // Liga o led vermelho para informar que a bomba de água está ativa
        digitalWrite(LedVerde, LOW); // Led verde fica desligado
        client.publish("Bomba", "1"); // Publica que a bomba está ligada
    } else {
        Serial.println("Planta Irrigada");
        digitalWrite(Rele, HIGH); //Desliga o relé
        digitalWrite(LedVermelho, LOW); // Led vermelho fica desligado
        digitalWrite(LedVerde, HIGH); // Liga o led verde para informar que a bomba não está ativa
        client.publish("Bomba", "0"); // Publica que a bomba está desligada
    }

    delay(1000); // Aguarda 1 segundo antes da próxima leitura, possivel mudar o delay para menos mensagens
}