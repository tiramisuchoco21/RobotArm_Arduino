#define F_CPU 16000000L
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

// 서보모터 핀 정의
#define SERVO_PIN1 PB5 // 핀 11 (Timer1의 OC1A)
#define SERVO_PIN2 PB6 // 핀 12 (Timer1의 OC1B)
#define SERVO_PIN3 PB7 // 핀 13 (Timer1의 OC1C)

// 스위치 핀1 정의
#define SWITCH_PIN1 PB4 // 스위치 핀1 (핀 10)

// 스위치 핀2 정의(인터럽트 방식)
#define set_bit(value, bit) (_SFR_BYTE(value) |= _BV(bit))
#define clear_bit(value, bit) (_SFR_BYTE(value) &= ~_BV(bit))

// PWM 정의
#define ROTATION_DELAY 15 // 서보 이동 시간 딜레이
#define PWM_MIN 2000 // 최소 펄스 폭 (2ms)
#define PWM_MAX 4600 // 최대 펄스 폭 (4.6ms)
#define PWM_MID 3300 // 중앙 펄스 폭 (3.3ms)
#define PWM_PERIOD 20000 // PWM 주기 (20ms)

// 조이스틱 정의
#define ADC_CHANNEL_X 0 // A0 채널 (조이스틱 X축)
#define ADC_CHANNEL_Y 1 // A1 채널 (조이스틱 Y축)
#define ADC_THRESHOLD 513 // 중앙 기준값
#define STEP_SIZE 10 // PWM 값의 조정 크기

volatile uint8_t led_state = 1; // LED 상태 전환

// INT0 인터럽트
ISR(INT0_vect) {
  led_state = !led_state; // LED 상태 전환
}

// INT0 인터럽트 초기화
void INT0_init(void) {
  EIMSK |= (1 << INT0); // INT0 인터럽트 허용
  EICRA |= (1 << ISC01); // 하강 에지에서 인터럽트 발생
  sei(); // 전역적으로 인터럽트 허용
}

// Timer1 초기화 (PWM 제어)
void timer_counter_1_init() {
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1) | (1 << COM1C1); // Fast PWM, 비반전 출력
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11); // 분주 8
  ICR1 = PWM_PERIOD; // TOP 값
}

// ADC 초기화
void ADC_init() {
  ADMUX = (1 << REFS0); // AVcc 기준 전압
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1); // ADC 활성화, 프리스케일러 64
}

// ADC 값 읽기
uint16_t ADC_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F); // 채널 선택
  ADCSRA |= (1 << ADSC); // 변환 시작
  while (ADCSRA & (1 << ADSC)); // 변환 완료 대기
  return ADC; // 변환된 값 반환
}

int main(void) {
  // PWM 핀 설정
  DDRB |= (1 << SERVO_PIN1) | (1 << SERVO_PIN2) | (1 << SERVO_PIN3);

  // 스위치 핀 입력 설정 및 풀업 저항 활성화
  DDRB &= ~(1 << SWITCH_PIN1);
  PORTB |= (1 << SWITCH_PIN1);

  // LED 핀 출력 설정
  INT0_init(); // INT0 인터럽트 설정
  clear_bit(DDRD, 0); // 버튼 연결 핀을 입력으로 설정
  set_bit(PORTD, 0); // 내장 풀업 저항 사용 설정
  set_bit(DDRE, 3); // LED 연결 핀을 출력으로 설정

  timer_counter_1_init(); // Timer1 초기화
  ADC_init(); // ADC 초기화
  INT0_init(); // INT0 인터럽트 초기화

  uint16_t pwm_value1 = PWM_MID;
  uint16_t pwm_value2 = PWM_MID;
  uint16_t pwm_value3 = PWM_MID;

  while (1) {
    // 스위치2 (인터럽트 방식 활용)
    if (!led_state) {
      set_bit(PORTE, 3);
    } else {
      clear_bit(PORTE, 3);
    }
    
    // 스위치1 상태 확인
    uint8_t switch_pressed1 = !(PINB & (1 << SWITCH_PIN1));

    // 조이스틱 값 읽기
    uint16_t joystick_value_x = ADC_read(ADC_CHANNEL_X);
    uint16_t joystick_value_y = ADC_read(ADC_CHANNEL_Y);
    
    // 스위치1
    if (!switch_pressed1) { // 스위치를 누르지 않을 경우, 2축 회전의 모터가 동작
      if (joystick_value_x > ADC_THRESHOLD + 10) {
        pwm_value1 += STEP_SIZE;
        if (pwm_value1 > PWM_MAX) pwm_value1 = PWM_MAX;
      } else if (joystick_value_x < ADC_THRESHOLD - 10) {
        pwm_value1 -= STEP_SIZE;
        if (pwm_value1 < PWM_MIN) pwm_value1 = PWM_MIN;
      }
      
      if (joystick_value_y > ADC_THRESHOLD + 10) {
        pwm_value3 += STEP_SIZE;
        if (pwm_value3 > PWM_MAX) pwm_value3 = PWM_MAX;
      } else if (joystick_value_y < ADC_THRESHOLD - 10) {
        pwm_value3 -= STEP_SIZE;
        if (pwm_value3 < PWM_MIN) pwm_value3 = PWM_MIN;
      }
    } else { // 스위치를 누를 경우, 그립(Gripper)의 모터 모드로 전환
      if (joystick_value_x > ADC_THRESHOLD + 10) {
        pwm_value2 += STEP_SIZE;
        if (pwm_value2 > PWM_MAX) pwm_value2 = PWM_MAX;
      } else if (joystick_value_x < ADC_THRESHOLD - 10) {
        pwm_value2 -= STEP_SIZE;
        if (pwm_value2 < PWM_MIN) pwm_value2 = PWM_MIN;
      }
    }
    
    OCR1A = pwm_value1;
    OCR1B = pwm_value2;
    OCR1C = pwm_value3;
    
    _delay_ms(ROTATION_DELAY);
  }
  return 0;
}
