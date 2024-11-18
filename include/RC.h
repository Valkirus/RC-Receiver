struct RC_Data_Package {
  byte joy1_X;
  byte joy1_Y;
  byte joy1_Button;
  byte joy2_X;
  byte joy2_Y;
  byte joy2_Button;
  byte slider1;
  byte slider2;
  byte pushButton1;
  byte pushButton2;
};


/* struct Hexapod_Data_Package {
  float current_sensor_value;
}; */

//Hexapod_Data_Package hex_data;

extern RC_Data_Package rc_data;

void SetupRC();
void RC_DisplayData();
void RC_ResetData();
//void initializeHexPayload();

bool GetData();
