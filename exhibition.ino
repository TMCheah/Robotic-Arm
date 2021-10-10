//Author - TM Cheah
#include <Servo.h>

//====DEFINE====
//servo motor
#define	BASE_HOME 90	
#define	MOTOR1_HOME 60		
#define	MOTOR1_UP 80		
#define	MOTOR1_DOWN 47
#define	MOTOR2_HOME 80		
#define	MOTOR2_UP 70		
#define	MOTOR2_DOWN 110
#define	MOTOR3_HOME 120		
#define	MOTOR3_UP 130	
#define	MOTOR3_DOWN 135	
#define GRIP 0
#define UNGRIP 180

//Color Sensor
#define S0 31
#define S1 29
#define S2 26
#define S3 28
#define sensorOut 24
#define sensoronoff 30
#define LEDon 27

//Infrared Sensor
#define IRPower 22
#define IROut 23

//interrupt pin
#define STOP 19

//trigger start pin
#define START 34

//DECLARATION and INITIATION
//servo motor
Servo motor1;	//up, down, forward, backward
Servo motor2;	//up, down, forward, backward
Servo motor3;	//adjust the end effector
Servo motorBase;//responsible for left right
Servo motorClaw;//end effector 

//color sensor
int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

//intterupt
bool intterupted = false;
bool itemIn = false;

//trigger start pin
int readStart = 1;

void setup()
{
	//setup serial monitor
	Serial.begin(9600);
	delay(10);
	
	//setup servo motor
	motor1.attach(6);
	motor2.attach(5);
	motor3.attach(4);
	motorBase.attach(3);
	motorClaw.attach(2);
	delay(100);

	//setup for infrared sensor
	pinMode(IRPower, OUTPUT);
	pinMode(IROut, INPUT);
	digitalWrite(IROut, LOW);
	
	//setup for color sensor
	pinMode(S0, OUTPUT);
	pinMode(S1, OUTPUT);
	pinMode(S2, OUTPUT);
	pinMode(S3, OUTPUT);
	pinMode(sensoronoff, OUTPUT);
	pinMode(sensorOut, INPUT);
	pinMode(LEDon, OUTPUT);
	
	// Setting frequency scaling to 20%
	digitalWrite(S0,HIGH);
	digitalWrite(S1,LOW);
	
	//setup for interrupt pin
	//change the interrupt mode after deciding the button mode
	//RISING to trigger when the pin goes from low to high
	//FALLING for when the pin goes from high to low
	pinMode(STOP, INPUT);
	attachInterrupt(digitalPinToInterrupt(STOP), interruptMe, RISING);
	
	//setup for trigger start pin
	pinMode(START, INPUT_PULLUP);

	//initialize robot location to home
	home();
	
	//comment line 89 if interrupt is not set up yet;
	itemIn = true;
}

void home()
{
	ungrip();
	
	double mB = (double)motorBase.read();
	double m1 = (double)motor1.read();
	double m2 = (double)motor2.read();
	double m3 = (double)motor3.read();
	
  	int diff_mB = BASE_HOME - (int)mB;
	int diff_m1 = MOTOR1_HOME - (int)m1;
	int diff_m2 = MOTOR2_HOME - (int)m2;
	int diff_m3 = MOTOR3_HOME - (int)m3;
	
  	//int max = max(abs(diff_mB), max(abs(diff_m1),max(abs(diff_m2), abs(diff_m3))));
  	int max = getMax(diff_mB, diff_m1, diff_m2, diff_m3);
	
	double step_mB = (double)diff_mB/(double)max;
	double step_m1 = (double)diff_m1/(double)max;
	double step_m2 = (double)diff_m2/(double)max;
	double step_m3 = (double)diff_m3/(double)max;
	
	int delayTime = 15;
	
  	for(int i = 0; i < max; i++)
	{
		mB = mB+step_mB;
      	m1 = m1+step_m1;
		m2 = m2+step_m2;
		m3 = m3+step_m3;
		
		motorBase.write((int)mB);
		motor1.write((int)m1);
		motor2.write((int)m2);
		motor3.write((int)m3);
		
		if(i > max*0.75)
          delayTime+=2;
     	else if(i > max*0.5)
          delayTime++;
	  
		delay(delayTime);
	}
}

void moveTo(int getMB, int getM1, int getM2, int getM3)
{
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	double mB = (double)motorBase.read();
	double m1 = (double)motor1.read();
	double m2 = (double)motor2.read();
	double m3 = (double)motor3.read();
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	
	int diff_mB = getMB - (int)mB;
	int diff_m1 = getM1 - (int)m1;
	int diff_m2 = getM2 - (int)m2;
	int diff_m3 = getM3 - (int)m3;
	
	int max = getMax(diff_mB, diff_m1, diff_m2, diff_m3);
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	
	double step_mB = (double)diff_mB/(double)max;
	double step_m1 = (double)diff_m1/(double)max;
	double step_m2 = (double)diff_m2/(double)max;
	double step_m3 = (double)diff_m3/(double)max;
	
	int delayTime = 15;
	
  	for(int i = 0; i < max; i++)
	{
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		mB = mB+step_mB;
      	m1 = m1+step_m1;
		m2 = m2+step_m2;
		m3 = m3+step_m3;
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		motorBase.write((int)mB);
		motor1.write((int)m1);
		motor2.write((int)m2);
		motor3.write((int)m3);
		
		if(i > max*0.75)
          delayTime+=2;
     	else if(i > max*0.5)
          delayTime++;
	  
		delay(delayTime);
	}
	
}

void up()
{
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	double mB = (double)motorBase.read();
	double m1 = (double)motor1.read();
	double m2 = (double)motor2.read();
	double m3 = (double)motor3.read();
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
  	int diff_mB = 0;
	int diff_m1 = MOTOR1_UP - (int)m1;
	int diff_m2 = MOTOR2_UP - (int)m2;
	int diff_m3 = MOTOR3_UP - (int)m3;
	
  	//int max = max(abs(diff_mB), max(abs(diff_m1),max(abs(diff_m2), abs(diff_m3))));
  	int max = getMax(diff_mB, diff_m1, diff_m2, diff_m3);
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	double step_mB = (double)diff_mB/(double)max;
	double step_m1 = (double)diff_m1/(double)max;
	double step_m2 = (double)diff_m2/(double)max;
	double step_m3 = (double)diff_m3/(double)max;
	
	int delayTime = 15;
	
  	for(int i = 0; i < max; i++)
	{
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		mB = mB+step_mB;
      	m1 = m1+step_m1;
		m2 = m2+step_m2;
		m3 = m3+step_m3;
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		
		motorBase.write((int)mB);
		motor1.write((int)m1);
		motor2.write((int)m2);
		motor3.write((int)m3);
		
		if(i > max*0.75)
          delayTime+=2;
     	else if(i > max*0.5)
          delayTime++;
	  
		delay(delayTime);
	}
}

void down()
{
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	double mB = (double)motorBase.read();
	double m1 = (double)motor1.read();
	double m2 = (double)motor2.read();
	double m3 = (double)motor3.read();
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
  	int diff_mB = 0;
	int diff_m1 = MOTOR1_DOWN - (int)m1;
	int diff_m2 = MOTOR2_DOWN - (int)m2;
	int diff_m3 = MOTOR3_DOWN - (int)m3;
	
  	//int max = max(abs(diff_mB), max(abs(diff_m1),max(abs(diff_m2), abs(diff_m3))));
  	int max = getMax(diff_mB, diff_m1, diff_m2, diff_m3);
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	double step_mB = (double)diff_mB/(double)max;
	double step_m1 = (double)diff_m1/(double)max;
	double step_m2 = (double)diff_m2/(double)max;
	double step_m3 = (double)diff_m3/(double)max;
	
	int delayTime = 15;
	
  	for(int i = 0; i < max; i++)
	{
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		mB = mB+step_mB;
      	m1 = m1+step_m1;
		m2 = m2+step_m2;
		m3 = m3+step_m3;
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		
		motorBase.write((int)mB);
		motor1.write((int)m1);
		motor2.write((int)m2);
		motor3.write((int)m3);
		
		if(i > max*0.75)
          delayTime+=2;
     	else if(i > max*0.5)
          delayTime++;
	  
		delay(delayTime);
	}
}


void moveBase(int getMB)
{
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	double mB = (double)motorBase.read();
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
  	
	int diff_mB = getMB - mB;
	
  	//int max = max(abs(diff_mB), max(abs(diff_m1),max(abs(diff_m2), abs(diff_m3))));
  	
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}

	double step_mB = (double)diff_mB/(double)abs(diff_mB);
	
	int delayTime = 15;
	
  	for(int i = 0; i < abs(diff_mB); i++)
	{
		mB = mB+step_mB;
      	
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		
		motorBase.write((int)mB);
		
		if(i > abs(diff_mB)*0.75)
          delayTime+=2;
     	else if(i > abs(diff_mB)*0.5)
          delayTime++;
	  
		delay(delayTime);
	}
	delay(650);
}

void scan(int m1, int m2, int m3, int mB)
{
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	moveTo(mB, m1, m2, m3);
}

void grab(int m1, int m2, int m3, int mB)
{
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	moveTo(mB, m1, m2, m3);
	delay(1000);
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	grip();
}

void dropZone(char color)
{
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	if(color == 'r')
		moveBase(156);
	else if(color == 'g')
		moveBase(140);
	else if(color == 'b')
		moveBase(125);
}

void grip()
{
	motorClaw.write(GRIP);
}

void ungrip()
{
	motorClaw.write(UNGRIP);
}

int getMax(int diff_mB, int diff_m1, int diff_m2, int diff_m3)
{
	return max(abs(diff_mB), max(abs(diff_m1),max(abs(diff_m2), abs(diff_m3))));
}

int getRed()
{
	int redColor = 0;
	
	digitalWrite(S2,LOW);
	digitalWrite(S3,LOW);
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return 0;
	}
	// Reading the output frequency
	
	for(int i = 0; i < 3; i++)
	{
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		redColor += pulseIn(sensorOut, LOW);
		delay(30);
	}
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return 0;
	}
	
	redFrequency = (int) (redColor/3);
	
	delay(10);
	return redFrequency;
}

int getGreen()
{
	int greenColor = 0;
	
	digitalWrite(S2,HIGH);
	digitalWrite(S3,HIGH);
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return 0;
	}
	// Reading the output frequency
	
	for(int i = 0; i < 3; i++)
	{
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		greenColor += pulseIn(sensorOut, LOW);
		delay(30);
	}
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return 0;
	}
	
	greenFrequency = (int) (greenColor/3);
	
	delay(10);
	return greenFrequency;
}

int getBlue()
{
	int blueColor = 0;
	
	digitalWrite(S2,LOW);
	digitalWrite(S3,HIGH);

	if(intterupted)
	{
		Serial.println("interrupted");
		return 0;
	}
	// Reading the output frequency
	for(int i = 0; i < 3; i++)
	{
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		blueColor += pulseIn(sensorOut, LOW);
		delay(30);
	}
	
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return 0;
	}
	
	blueFrequency = (int) (blueColor/3);
	
	delay(10);
	return blueFrequency;
}

char getColor()
{
	int red, green, blue;
	
	//Set sensor on and LED on
	digitalWrite(sensoronoff, HIGH);
	digitalWrite(LEDon, HIGH);
	delay(500);
	
	if(intterupted)
	{
		Serial.println("interrupted");
		return ' ';
	}
	red = getRed();
	green = getGreen();
	blue = getBlue();
	if(intterupted)
	{
		Serial.println("interrupted");
		return ' ';
	}
	//flashing effect
	digitalWrite(LEDon, LOW);
	delay(200);
	digitalWrite(LEDon, HIGH);
	delay(100);
	digitalWrite(LEDon, LOW);
	delay(200);
	
	Serial.print("R: ");
	Serial.print(red);
	Serial.print(" G: ");
	Serial.print(green);
	Serial.print(" B: ");
	Serial.println(blue);
	
	if((red+green+blue)>10000)
		return 'n';
	else if(red < green && red < blue)
		return 'r';
	else if(green < red && green < blue)
		return 'g';
	else
		return 'b';
}

void process(int m1, int m2, int m3, int mB)
{
	char color;
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	grab(m1, m2, m3, mB);
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	color = getColor();
  Serial.print("Color: ");
  Serial.println(color);
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	up();
	delay(350);
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	dropZone(color);
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	down();
	delay(350);
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	ungrip();
  delay(350);
	up();
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	delay(350);
	if(intterupted)
	{
		Serial.println("interrupted");
		return;
	}
	up();
}



void loop()
{
	intterupted = false;
	
	readStart = digitalRead(START);

	while(itemIn) //&& readStart == 0)
	{
    delay(2000);
		int detectObj = 0;
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		//home
		Serial.println("Going home");
		home();
		delay(1000);
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		//scan 1
		Serial.println("Scan location 1");
		up();
		delay(350);
		moveBase(55);
		scan(65,55,105,55);
    delay(200);
		digitalWrite(IRPower, HIGH);	//turn on infrared
		delay(100);
		detectObj = digitalRead(IROut);	
		digitalWrite(IRPower, LOW);		//turn off infrared
		if(detectObj == 0)
		{
      Serial.println("object detected");
			if(intterupted)
			{
				Serial.println("interrupted");
				break;
			}
			process(55,87,120,55);
		}
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		//scan 2
		Serial.println("Scan location 2");
		up();
		delay(350);
		moveBase(29);
		scan(65,55,105,29);
    delay(200);
		digitalWrite(IRPower, HIGH);	//turn on infrared
		delay(100);
		detectObj = digitalRead(IROut);	
		digitalWrite(IRPower, LOW);		//turn off infrared
		if(detectObj == 0)
		{
			Serial.println("object detected");
			if(intterupted)
			{
				Serial.println("interrupted");
				break;
			}
			process(57,80,115,29);
		}
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		//scan 3
    Serial.println("Scan location 3");
		up();
		delay(350);
		moveBase(60);		
		scan(50,99,127,60);
    delay(200);
		digitalWrite(IRPower, HIGH);	//turn on infrared
		delay(100);
		detectObj = digitalRead(IROut);	
		digitalWrite(IRPower, LOW);		//turn off infrared
		if(detectObj == 0)
		{
			Serial.println("object detected");
			if(intterupted)
			{
				Serial.println("interrupted");
				break;
			}
			process(40,123,135,60);
		}
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		//scan 4
    Serial.println("Scan location 4");
		up();
		delay(350);
		moveBase(42);
		scan(58,80,120,42);
    delay(200);
		digitalWrite(IRPower, HIGH);	//turn on infrared
		delay(100);
		detectObj = digitalRead(IROut);	
		digitalWrite(IRPower, LOW);		//turn off infrared
		if(detectObj == 0)
		{
			Serial.println("object detected");
			if(intterupted)
			{
				Serial.println("interrupted");
				break;
			}
			process(49,104,128,42);
		}
		
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
		//scan 5
    Serial.println("Scan location 5");
		up();
		delay(350);
		moveBase(24);
		scan(54,88,115,24);
    delay(200);
		digitalWrite(IRPower, HIGH);	//turn on infrared
		delay(100);
		detectObj = digitalRead(IROut);	
		digitalWrite(IRPower, LOW);		//turn off infrared
		if(detectObj == 0)
		{
			Serial.println("object detected");
			if(intterupted)
			{
				Serial.println("interrupted");
				break;
			}
			process(40,120,135,24);
		}
		
		itemIn = false;
		Serial.println("Going home");
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
    up();
    delay(350);
    moveBase(BASE_HOME);
		home();
		if(intterupted)
		{
			Serial.println("interrupted");
			break;
		}
	}
}

void interruptMe()
{
  Serial.println("get interrupt LOL");
  itemIn = true;
  intterupted = true;
  readStart = 1;
  home();
}
