#include <Servo.h>
#include "HX711.h"
#include <FreeRTOS.h>
#include <task.h>

// Configuração dos pinos dos servos
#define PIN_SERVO1 2
#define PIN_SERVO2 3
#define PIN_SERVO3 4
#define PIN_SERVO4 5
#define PIN_SERVO5 6
#define PIN_SERVO6 7

// Configuração dos pinos do HX711
#define LOADCELL1_DOUT_PIN 8
#define LOADCELL1_SCK_PIN 9
#define LOADCELL2_DOUT_PIN 10
#define LOADCELL2_SCK_PIN 11
#define BUTTON_TARA1 12
#define BUTTON_TARA2 13

#define CALIBVAL1 380013
#define CALIBVAL2 -400330
#define PESOBASE 32

// Valores iniciais de tara (ZEROVAL) para cada balança
long ZEROVAL1 = 364969; 
long ZEROVAL2 = -385649;

long long BUFF1[50] = {0}; //Buffer da balanca 1
long long BUFF2[50] = {0}; //Buffer da balanca 2
byte BUFFL = sizeof(BUFF1) / sizeof(BUFF1[0]);

HX711 scale1;
HX711 scale2;

Servo servo1, servo2, servo3, servo4, servo5, servo6;

TaskHandle_t taskMovimentoHandle;
TaskHandle_t taskLeituraHandle;

void setup() {
  Serial.begin(115200);

  // Inicialização das balanças
  scale1.begin(LOADCELL1_DOUT_PIN, LOADCELL1_SCK_PIN);
  scale2.begin(LOADCELL2_DOUT_PIN, LOADCELL2_SCK_PIN);
  pinMode(BUTTON_TARA1, INPUT_PULLUP);
  pinMode(BUTTON_TARA2, INPUT_PULLUP);

  // Inicialização dos servos
  servo1.attach(PIN_SERVO1);
  servo2.attach(PIN_SERVO2);
  servo3.attach(PIN_SERVO3);
  servo4.attach(PIN_SERVO4);
  servo5.attach(PIN_SERVO5);
  servo6.attach(PIN_SERVO6);

  // Posição inicial dos servos
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
  servo6.write(80);

  // Criação das tarefas
  xTaskCreate(taskLeitura, "Leitura Balancas", 1000, NULL, 1, &taskLeituraHandle);
  xTaskCreate(taskMovimento, "Movimento Braço", 1000, NULL, 1, &taskMovimentoHandle);
  
  Serial.println("Sistema iniciado.");
}

// Função para movimentar um servo de forma não bloqueante
void moverServo(Servo& servo, int posicao, unsigned long tempoEspera) {
  static unsigned long ultimoTempo = 0;
  static bool emMovimento = false;

  if (!emMovimento) {
    servo.write(posicao);
    ultimoTempo = millis();
    emMovimento = true;
  } else if (millis() - ultimoTempo >= tempoEspera) {
    emMovimento = false; // Movimento concluído
  }
}

// Função para movimentar o braço para a peça escolhida
void moverParaPeca(int pos1, int pos2, int pos3, int pos4, int pos5, int pos6, const char* peca) {
  Serial.print("Movimentando para: ");
  Serial.println(peca);

  moverServo(servo1, pos1, 2000);
  moverServo(servo2, pos2, 2000);
  moverServo(servo3, pos3, 2000);
  moverServo(servo4, pos4, 2000);
  moverServo(servo5, pos5, 2000);
  moverServo(servo6, pos6, 2000);

  // Descarte da peça
  moverServo(servo3, 130, 2000);
  moverServo(servo1, 90, 2000);
  moverServo(servo2, 90, 2000);
  moverServo(servo3, 170, 2000);
  moverServo(servo4, 170, 2000);
  moverServo(servo5, 30, 2000);
  moverServo(servo6, 80, 2000);


  // Retorno à posição inicial
  moverServo(servo1, 90, 2000);
  moverServo(servo2, 90, 2000);
  moverServo(servo3, 90, 2000);
  moverServo(servo4, 90, 2000);
  moverServo(servo5, 90, 2000);
  moverServo(servo6, 80, 2000);

  Serial.println("Retorno à posição inicial concluído.");
}

// Função para coleta de dados das balanças
void taskLeitura(void *pvParameters) {
  while (1) {
    // Coleta de dados das balanças
    long RAW1 = ReadRAW(scale1, BUFF1);
    long RAW2 = ReadRAW(scale2, BUFF2);
    float peso1 = ConvertVal(RAW1, ZEROVAL1, CALIBVAL1);
    float peso2 = ConvertVal(RAW2, ZEROVAL2, CALIBVAL2);
    int qtd1 = DetectarQuantidade(peso1, 32.0, 3.0);
    int qtd2 = DetectarQuantidade(peso2, 25.0, 3.0);

    // Exibição dos dados
    Serial.print("Balança 1 - Peso: ");
    Serial.print(peso1);
    Serial.print("g, Qtd: ");
    Serial.println(qtd1);
    Serial.print("Balança 2 - Peso: ");
    Serial.print(peso2);
    Serial.print("g, Qtd: ");
    Serial.println(qtd2);
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);  
  }
}

// Função para movimentar o braço com a escolha de peça
void taskMovimento(void *pvParameters) {
  while (1) {
    // Verificar se há uma escolha da peça
    if (Serial.available()) {
      char escolha = Serial.read();
      switch (escolha) {
        case '1':
          moverParaPeca(25, 90, 165, 170, 45, 25, "Peça 1");
          break;
        case '2':
          moverParaPeca(25, 100, 160, 170, 30, 25, "Peça 2");
          break;
        case '3':
          moverParaPeca(1, 90, 170, 170, 40, 25, "Peça 3");
          break;
        case '4':
          moverParaPeca(1, 105, 165, 170, 25, 20, "Peça 4");
          break;
        default:
          Serial.println("Escolha inválida.");
          break;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);  
  }
}

// Função para conversão de peso
float ConvertVal(long RAWVAL, long ZEROVAL, long CALIBVAL) {
  float unit = (float)PESOBASE / (float)(ZEROVAL - CALIBVAL);
  return (float)(ZEROVAL - RAWVAL) * unit;
}

// Função para cálculo de média
long CalculateAverage(long long* BUFF) {
  long long ACC = 0;
  for (short i = 0; i < BUFFL; i++) {
    ACC += BUFF[i];
  }
  return (long)(ACC / BUFFL);
}

// Função para tara
void Tara(HX711& scale, long& ZEROVAL, int buttonPin) {
  Serial.print("Tara no botão ");
  Serial.println(buttonPin);
  while (!scale.is_ready());
  ZEROVAL = scale.read();
  Serial.println("Tara concluída.");
}

// Função para detecção de quantidade
int DetectarQuantidade(float pesoDetectado, float pesoPorItem, float margem) {
  for (int qtd = 1; qtd <= 2; qtd++) {
    float pesoMin = qtd * pesoPorItem - qtd * margem;
    float pesoMax = qtd * pesoPorItem + qtd * margem;
    if (pesoDetectado >= pesoMin && pesoDetectado <= pesoMax) {
      return qtd;
    }
  }
  return 0;
}

// Função para leitura do RAW
long ReadRAW(HX711& scale, long long* BUFF) {
  while (!scale.is_ready());
  long reading = scale.read();
  for (short i = 0; i < BUFFL - 1; i++) {
    BUFF[i] = BUFF[i + 1];
  }
  BUFF[BUFFL - 1] = reading;
  return CalculateAverage(BUFF);
}
