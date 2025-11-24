typedef struct {
  byte nodeID;
  byte payloadID;  
  float temp;
  float voltage;
  float humidity;
  unsigned long readoutms;
} BasePayLoad;

typedef struct {
  byte nodeID;
  byte payloadID;  
  unsigned long amb_als;
  unsigned long amb_ir;
  float uv;
  unsigned long readoutms;
} SI1145PayLoad;

typedef struct {
  byte nodeID;
  byte payloadID;  
  float lux;
  float voltage;
  unsigned long readoutms;
} BH1750Payload;

typedef struct {
  byte nodeID;
  byte payloadID;  
  int analogPayLoad;
  float voltage;  
} AnalogPayload;
