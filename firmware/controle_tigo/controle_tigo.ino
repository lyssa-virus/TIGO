#include <Encoder.h>

// =====================================================
// 1. PINAGEM DOS MOTORES - ROBÔ TIGO (LADO DIREITO INVERTIDO)
// =====================================================
// Roda Dianteira Esquerda (Front Left - FL)
const uint8_t PWM_FL  = 12;
const uint8_t DIR1_FL = 34;
const uint8_t DIR2_FL = 35;

// Roda Traseira Esquerda (Rear Left - RL)
const uint8_t PWM_RL  = 8;
const uint8_t DIR1_RL = 37;
const uint8_t DIR2_RL = 36;

// Roda Dianteira Direita (Front Right - FR) - Pinos DIR invertidos
const uint8_t PWM_FR  = 6;
const uint8_t DIR1_FR = 42;
const uint8_t DIR2_FR = 43;

// Roda Traseira Direita (Rear Right - RR) - Pinos DIR invertidos
const uint8_t PWM_RR  = 5;
const uint8_t DIR1_RR = A5;
const uint8_t DIR2_RR = A4;

// =====================================================
// 2. PINAGEM DOS ENCODERS (SHIELD MOEBIUS NO MEGA 2560)
// =====================================================
// O pino A de cada encoder deve ficar em um pino de interrupção externa do Mega:
// Pinos de interrupção no Mega: 2 (INT4), 3 (INT5), 18 (INT3), 19 (INT2)
const uint8_t ENC_A_FL = 18;
const uint8_t ENC_B_FL = 31;

const uint8_t ENC_A_RL = 19;
const uint8_t ENC_B_RL = 38;

const uint8_t ENC_A_FR = 3;
const uint8_t ENC_B_FR = 49;

const uint8_t ENC_A_RR = 2;
const uint8_t ENC_B_RR = 48;

// Instâncias dos 4 encoders
Encoder encFL(ENC_A_FL, ENC_B_FL);
Encoder encFR(ENC_A_FR, ENC_B_FR);
Encoder encRL(ENC_A_RL, ENC_B_RL);
Encoder encRR(ENC_A_RR, ENC_B_RR);

// Inversões de sentido dos encoders:
// Se uma roda ao girar para frente der contagem negativa, altere o sinal correspondente (-1 ou 1)
const int8_t ENC_DIR_FL = 1;
const int8_t ENC_DIR_FR = -1;  // Lado direito fisicamente invertido
const int8_t ENC_DIR_RL = 1;
const int8_t ENC_DIR_RR = -1;  // Lado direito fisicamente invertido

// =====================================================
// 3. PARÂMETROS GEOMÉTRICOS E MECÂNICOS DO CHASSI
// =====================================================
const float r  = 0.041;   // Raio da roda (m)
const float lx = 0.082;   // Metade da distância entre eixos (X)
const float ly = 0.106;   // Metade da distância entre eixos (Y)
const float K  = lx + ly; // 0.188 m

const float W_MAX_RODA = 34.55; // Limite angular estimado (rad/s)
const float TICKS_PER_REV = 1319.1; // Ticks por volta completa da roda
const float RAD_POR_TICK = (2.0 * 3.14159265359) / TICKS_PER_REV;

// =====================================================
// 4. PARÂMETROS DE CONTROLE PID
// =====================================================
// Ganhos padrão (ajustáveis nos testes práticos)
float Kp = 4.0;
float Ki = 10.0;
float Kd = 0.02;

const int PWM_MIN = 20; // Zona morta mínima para movimentar os motores
const float LIMITE_INTEGRAL = 100.0; // Anti-windup

struct PIDMotor {
    float integral;
    float erroAnterior;

    float calcular(float setpoint, float medido, float dt) {
        float erro = setpoint - medido;

        // Proporcional
        float p = Kp * erro;

        // Integral com anti-windup
        integral += erro * dt;
        integral = constrain(integral, -LIMITE_INTEGRAL, LIMITE_INTEGRAL);
        float i = Ki * integral;

        // Derivativo
        float d = 0.0;
        if (dt > 0.001) {
            d = Kd * (erro - erroAnterior) / dt;
        }
        erroAnterior = erro;

        return p + i + d;
    }

    void reset() {
        integral = 0.0;
        erroAnterior = 0.0;
    }
};

PIDMotor pidFL, pidFR, pidRL, pidRR;

// Estado dos comandos de velocidade (m/s e rad/s)
float vx = 0.0;
float vy = 0.0;
float wz = 0.0;

// Armazenamento dos ticks anteriores para cálculo de velocidade
long ultimosTicksFL = 0;
long ultimosTicksFR = 0;
long ultimosTicksRL = 0;
long ultimosTicksRR = 0;

unsigned long tempoUltimoComando = 0;
unsigned long tempoUltimoPID = 0;
const unsigned long TIMEOUT_MS = 500;
const unsigned long PID_INTERVAL_MS = 50; // 50ms = 20Hz (sincronizado com o nó ROS)

// =====================================================
// 5. CINEMÁTICA INVERSA MECANUM
// =====================================================
void calcularCinematicaInversa(float v_x, float v_y, float w_z, 
                               float &w_fl, float &w_fr, 
                               float &w_rl, float &w_rr) {
    w_fl = (v_x - v_y - K * w_z) / r;  // FL
    w_fr = (v_x + v_y + K * w_z) / r;  // FR
    w_rl = (v_x + v_y - K * w_z) / r;  // RL
    w_rr = (v_x - v_y + K * w_z) / r;  // RR
}

// =====================================================
// 6. ATUAÇÃO DAS PONTES H
// =====================================================
void acionarMotor(uint8_t pin_pwm, uint8_t pin_dir1, uint8_t pin_dir2, float pwm_cmd) {
    int pwm_val = constrain((int)abs(pwm_cmd), 0, 255);

    // Vence a zona morta se houver comando significativo
    if (pwm_val > 0 && pwm_val < PWM_MIN) {
        pwm_val = PWM_MIN;
    }

    if (pwm_cmd > 0.5) {
        digitalWrite(pin_dir1, LOW);
        digitalWrite(pin_dir2, HIGH);
        analogWrite(pin_pwm, pwm_val);
    } else if (pwm_cmd < -0.5) {
        digitalWrite(pin_dir1, HIGH);
        digitalWrite(pin_dir2, LOW);
        analogWrite(pin_pwm, pwm_val);
    } else {
        digitalWrite(pin_dir1, LOW);
        digitalWrite(pin_dir2, LOW);
        analogWrite(pin_pwm, 0);
    }
}

void pararMotores() {
    acionarMotor(PWM_FL, DIR1_FL, DIR2_FL, 0);
    acionarMotor(PWM_FR, DIR1_FR, DIR2_FR, 0);
    acionarMotor(PWM_RL, DIR1_RL, DIR2_RL, 0);
    acionarMotor(PWM_RR, DIR1_RR, DIR2_RR, 0);

    pidFL.reset();
    pidFR.reset();
    pidRL.reset();
    pidRR.reset();
}

// =====================================================
// 7. SETUP
// =====================================================
void setup() {
    Serial.begin(115200);
    Serial.setTimeout(10); // Evita travamento na leitura da serial

    pinMode(PWM_FL, OUTPUT); pinMode(DIR1_FL, OUTPUT); pinMode(DIR2_FL, OUTPUT);
    pinMode(PWM_RL, OUTPUT); pinMode(DIR1_RL, OUTPUT); pinMode(DIR2_RL, OUTPUT);
    pinMode(PWM_FR, OUTPUT); pinMode(DIR1_FR, OUTPUT); pinMode(DIR2_FR, OUTPUT);
    pinMode(PWM_RR, OUTPUT); pinMode(DIR1_RR, OUTPUT); pinMode(DIR2_RR, OUTPUT);

    pararMotores();

    tempoUltimoComando = millis();
    tempoUltimoPID = millis();
}

// =====================================================
// 8. LOOP PRINCIPAL
// =====================================================
void loop() {
    // --- 1. RECEPÇÃO DE COMANDOS PELA SERIAL ---
    if (Serial.available() > 0) {
        String linha = Serial.readStringUntil('\n');
        linha.trim();
        
        if (linha.startsWith("V,")) {
            int v1 = linha.indexOf(',');
            int v2 = linha.indexOf(',', v1 + 1);
            int v3 = linha.indexOf(',', v2 + 1);
            
            if (v1 > 0 && v2 > 0 && v3 > 0) {
                vx = linha.substring(v1 + 1, v2).toFloat();
                vy = linha.substring(v2 + 1, v3).toFloat();
                wz = linha.substring(v3 + 1).toFloat();
                tempoUltimoComando = millis();
            }
        } else if (linha.startsWith("S")) {
            // Comando de parada de segurança
            vx = 0.0;
            vy = 0.0;
            wz = 0.0;
            tempoUltimoComando = millis();
        }
    }

    // --- 2. TIMEOUT DE SEGURANÇA ---
    if (millis() - tempoUltimoComando > TIMEOUT_MS) {
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
    }

    // --- 3. CICLO DE CONTROLE PID E ENVIO DE ENCODERS (20 Hz) ---
    unsigned long agora = millis();
    if (agora - tempoUltimoPID >= PID_INTERVAL_MS) {
        float dt = (agora - tempoUltimoPID) / 1000.0;
        tempoUltimoPID = agora;

        // Leitura atual dos encoders brutos (com ajuste de direção)
        long ticksFL = encFL.read() * ENC_DIR_FL;
        long ticksFR = encFR.read() * ENC_DIR_FR;
        long ticksRL = encRL.read() * ENC_DIR_RL;
        long ticksRR = encRR.read() * ENC_DIR_RR;

        // Cálculo da variação de pulsos (delta)
        long dTicksFL = ticksFL - ultimosTicksFL;
        long dTicksFR = ticksFR - ultimosTicksFR;
        long dTicksRL = ticksRL - ultimosTicksRL;
        long dTicksRR = ticksRR - ultimosTicksRR;

        ultimosTicksFL = ticksFL;
        ultimosTicksFR = ticksFR;
        ultimosTicksRL = ticksRL;
        ultimosTicksRR = ticksRR;

        // Velocidade medida real em cada roda (rad/s)
        float w_fl_medido = (dTicksFL * RAD_POR_TICK) / dt;
        float w_fr_medido = (dTicksFR * RAD_POR_TICK) / dt;
        float w_rl_medido = (dTicksRL * RAD_POR_TICK) / dt;
        float w_rr_medido = (dTicksRR * RAD_POR_TICK) / dt;

        // Velocidades desejadas pela cinemática inversa
        float w_fl_alvo, w_fr_alvo, w_rl_alvo, w_rr_alvo;
        calcularCinematicaInversa(vx, vy, wz, w_fl_alvo, w_fr_alvo, w_rl_alvo, w_rr_alvo);

        // Se o robô deve ficar parado
        if (abs(vx) < 0.001 && abs(vy) < 0.001 && abs(wz) < 0.001) {
            pararMotores();
        } else {
            // Feedforward: PWM teórico direto a partir da velocidade desejada
            float ff_fl = (w_fl_alvo / W_MAX_RODA) * 255.0;
            float ff_fr = (w_fr_alvo / W_MAX_RODA) * 255.0;
            float ff_rl = (w_rl_alvo / W_MAX_RODA) * 255.0;
            float ff_rr = (w_rr_alvo / W_MAX_RODA) * 255.0;

            // PID corrige o erro entre velocidade alvo e medida
            float pid_fl = pidFL.calcular(w_fl_alvo, w_fl_medido, dt);
            float pid_fr = pidFR.calcular(w_fr_alvo, w_fr_medido, dt);
            float pid_rl = pidRL.calcular(w_rl_alvo, w_rl_medido, dt);
            float pid_rr = pidRR.calcular(w_rr_alvo, w_rr_medido, dt);

            // Comando final = Feedforward + Correção PID
            float cmd_fl = constrain(ff_fl + pid_fl, -255.0, 255.0);
            float cmd_fr = constrain(ff_fr + pid_fr, -255.0, 255.0);
            float cmd_rl = constrain(ff_rl + pid_rl, -255.0, 255.0);
            float cmd_rr = constrain(ff_rr + pid_rr, -255.0, 255.0);

            acionarMotor(PWM_FL, DIR1_FL, DIR2_FL, cmd_fl);
            acionarMotor(PWM_FR, DIR1_FR, DIR2_FR, cmd_fr);
            acionarMotor(PWM_RL, DIR1_RL, DIR2_RL, cmd_rl);
            acionarMotor(PWM_RR, DIR1_RR, DIR2_RR, cmd_rr);
        }

        // --- 4. ENVIO DOS ENCODERS PARA O ROS ---
        // Formato esperado pelo nó MecanumSerialBridge: "E,fl,fr,rl,rr\n"
        Serial.print("E,");
        Serial.print(ticksFL);
        Serial.print(",");
        Serial.print(ticksFR);
        Serial.print(",");
        Serial.print(ticksRL);
        Serial.print(",");
        Serial.println(ticksRR);
    }
}
