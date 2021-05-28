void Debugln(String string)
{
  #ifdef DEBUG_MODE
  Serial.println(string);
  #endif
}

void Debug(String string)
{
  #ifdef DEBUG_MODE
  Serial.print(string);
  #endif
}

void Debugln(float val)
{
  #ifdef DEBUG_MODE
  Serial.println(val);
  #endif
}

void Debug(float val)
{
  #ifdef DEBUG_MODE
  Serial.print(val);
  #endif
}
