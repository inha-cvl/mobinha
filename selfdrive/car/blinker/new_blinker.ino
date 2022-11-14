#define ENA 2
#define IN1 3
#define IN2 4
#define ENB 7
#define IN3 5
#define IN4 6

class Blinker {

private:

    int cur_state_ = 0; 
    //0: middle, 1: left, 2: right
    double angle_to_tick_ = 1 / 1.8;
    int left_angle_, right_angle_;
    bool isNotMoving_ = true;

    void turn_ccw_() {
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
        delay(10);
        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,HIGH);
        delay(10);
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,HIGH);
        delay(10);
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
        delay(10);
    };

    void turn_cw_() {
        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
        delay(10);

        digitalWrite(IN1,HIGH);
        digitalWrite(IN2,LOW);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,HIGH);
        delay(10);

        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,LOW);
        digitalWrite(IN4,HIGH);
        delay(10);

        digitalWrite(IN1,LOW);
        digitalWrite(IN2,HIGH);
        digitalWrite(IN3,HIGH);
        digitalWrite(IN4,LOW);
        delay(10);
    };


public:

    Blinker(int left_angle, int right_angle) {
        left_angle_ = left_angle;
        right_angle_ = right_angle;
    };

    void set_blink_middle() {
        cur_state_ = 0;
        if (cur_state_ == 1) { // left
            set_blink_right();
        }
        else if (cur_state_ == 2) { //right
            set_blink_left();
        } 
    };

    void set_blink_left() {
        cur_state_= 1;
        int tick = left_angle_ * angle_to_tick_;
        for (int i = 0; i < tick; ++i) {
            Serial.print("leftleft");
            turn_ccw_();
        }
    }

    void set_blink_right() {
        cur_state_= 2;
        int tick = right_angle_ * angle_to_tick_;
        for (int i = 0; i < tick; ++i) {
            turn_cw_();
        }
            
    }
    bool isNotMoving() {
        return isNotMoving_;
    }
    void set_isNotMoving(bool not_moving) {
        isNotMoving_ = not_moving;
    }
};


void setup() {
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  digitalWrite(ENA,HIGH);
  digitalWrite(ENB,HIGH);
}


Blinker blinker(10, 10);

int state = 1;
void loop() {
    if (blinker.isNotMoving()) {
        if (state == 0){
            blinker.set_blink_middle();
        } else if (state == 1) {
            blinker.set_blink_left();
        } else if (state == 2) {
            blinker.set_blink_right();
        };
        blinker.set_isNotMoving(false);
    }
    //blinker.set_blink_left();
    delay(100);

}
