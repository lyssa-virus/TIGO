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
const uint8_t DIR1_FR = 42; // Invertido (era 43)
const uint8_t DIR2_FR = 43; // Invertido (era 42)

// Roda Traseira Direita (Rear Right - RR) - Pinos DIR invertidos
const uint8_t PWM_RR  = 5;
const uint8_t DIR1_RR = A5; // Invertido (era A4)
const uint8_t DIR2_RR = A4; // Invertido (era A5)

// =====================================================
// 2. PARÂMETROS GEOMÉTRICOS E MECÂNICOS DO CHASSI
// =====================================================
const float r  = 0.041;   // Raio da roda (m)
const float lx = 0.082;   // Metade da distância entre eixos (X)
const float ly = 0.106;   // Metade da distância entre eixos (Y)
const float K  = lx + ly; // 0.188 m

const float W_MAX_RODA = 34.55; // Limite angular estimado (rad/s)

float vx = 0.0;
float vy = 0.0;
float wz = 0.0;

unsigned long tempoUltimoComando = 0;
const unsigned long TIMEOUT_MS = 500;

// =====================================================
// 3. CINEMÁTICA INVERSA MECANUM
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
// 4. ATUAÇÃO DAS PONTES H
// =====================================================
void acionarMotor(uint8_t pin_pwm, uint8_t pin_dir1, uint8_t pin_dir2, float w_roda) {
    int pwm_val = constrain(map(abs(w_roda) * 1000, 0, W_MAX_RODA * 1000, 0, 255), 0, 255);

    if (w_roda > 0.01) {
        digitalWrite(pin_dir1, LOW);
        digitalWrite(pin_dir2, HIGH);
        analogWrite(pin_pwm, pwm_val);
    } else if (w_roda < -0.01) {
        digitalWrite(pin_dir1, HIGH);
        digitalWrite(pin_dir2, LOW);
        analogWrite(pin_pwm, pwm_val);
    } else {
        digitalWrite(pin_dir1, LOW);
        digitalWrite(pin_dir2, LOW);
        analogWrite(pin_pwm, 0);
    }
}

void aplicarVelocidades(float w_fl, float w_fr, float w_rl, float w_rr) {
    acionarMotor(PWM_FL, DIR1_FL, DIR2_FL, w_fl);
    acionarMotor(PWM_FR, DIR1_FR, DIR2_FR, w_fr);
    acionarMotor(PWM_RL, DIR1_RL, DIR2_RL, w_rl);
    acionarMotor(PWM_RR, DIR1_RR, DIR2_RR, w_rr);
}

// =====================================================
// 5. SETUP
// =====================================================
void setup() {
    Serial.begin(115200);

    pinMode(PWM_FL, OUTPUT); pinMode(DIR1_FL, OUTPUT); pinMode(DIR2_FL, OUTPUT);
    pinMode(PWM_RL, OUTPUT); pinMode(DIR1_RL, OUTPUT); pinMode(DIR2_RL, OUTPUT);
    pinMode(PWM_FR, OUTPUT); pinMode(DIR1_FR, OUTPUT); pinMode(DIR2_FR, OUTPUT);
    pinMode(PWM_RR, OUTPUT); pinMode(DIR1_RR, OUTPUT); pinMode(DIR2_RR, OUTPUT);

    aplicarVelocidades(0, 0, 0, 0);
}

// =====================================================
// 6. LOOP PRINCIPAL
// =====================================================
void loop() {
    if (Serial.available() > 0) {
        String linha = Serial.readStringUntil('\n');
        
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
        }
    }

    if (millis() - tempoUltimoComando > TIMEOUT_MS) {
        vx = 0.0;
        vy = 0.0;
        wz = 0.0;
    }

    float w_fl, w_fr, w_rl, w_rr;
    calcularCinematicaInversa(vx, vy, wz, w_fl, w_fr, w_rl, w_rr);
    aplicarVelocidades(w_fl, w_fr, w_rl, w_rr);

    delay(10);
}
