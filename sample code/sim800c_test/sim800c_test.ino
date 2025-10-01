#include <SoftwareSerial.h>

SoftwareSerial SIM800C(18, 17); // RX, TX

void setup() {
    Serial.begin(115200);
    SIM800C.begin(9600);

    // Power on the module
    pinMode(48, OUTPUT);
    digitalWrite(48, LOW);
    delay(1000); // PWRKEY needs to be low for at least 1 second
    digitalWrite(48, HIGH);
    delay(5000); // Wait for the module to initialize

    // Test AT communication
    SIM800C.println("AT");
    delay(1000);
    print_response();
    check_info();
    delay(1000);
}

void loop() {
    
  // Send an SMS
  send_sms();  
  delay(5000);
}

void check_info(){
  SIM800C.println("AT");
  delay(1000);
  print_response();
  SIM800C.println("AT+CSQ");//check signal stngth
  delay(1000);
  print_response();
  SIM800C.println("AT+CCID");//print sim card number
  delay(1000);
  print_response();
  SIM800C.println("AT+CREG?");//check for the registered network
  delay(1000);
  print_response();  
  SIM800C.println("ATI");//print module name and number
  delay(1000);
  print_response();
  SIM800C.println("AT+CBC");//battery percentage
  delay(1000);
  print_response();
}
  
void print_response(){
  while (SIM800C.available()){
    Serial.write(SIM800C.read());
  }
}

void send_sms()
{
  SIM800C.println("AT+CMGF=1"); // Configuring TEXT mode
  delay(1000);
  print_response;
  SIM800C.println("AT+CMGS=\"+xxxxxxxxxxxx\"");//change xxxxxxxxxxx with phone number to sms
  delay(1000);
  print_response;
  SIM800C.print("Edge Board Test SMS!"); //text content
  delay(1000);
  print_response;
  SIM800C.write(26);
}
