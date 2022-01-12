template<class T>
class AverageValue {

public:
  AverageValue(uint32_t valueNum);
  ~AverageValue();
  void push(T value);
  T average();
protected:

private:
  T *values;
  uint32_t valueNum;
  uint32_t currentPosition = 0;
};

template<class T>
AverageValue<T>::AverageValue(uint32_t valueNum) {
  this->valueNum = valueNum;
  this->values = (T *) malloc(sizeof(T) * valueNum);
  this->currentPosition = 0;
  for (uint32_t i = 0; i < this->valueNum; i++) {
    this->values[i] = 0;
  }
}

template<class T>
AverageValue<T>::~AverageValue() {
  free(this->values);
}

template<class T>
void AverageValue<T>::push(T value) {
  this->values[this->currentPosition] = value;
  this->currentPosition = (this->currentPosition + 1) % this->valueNum;
}

template<class T>
T AverageValue<T>::average() {
  T sum = 0;
  for (uint32_t i = 0; i < this->valueNum; i++) {
    sum += this->values[i];
  }
  return (sum / (T)this->valueNum);
}

const long MAX_VALUES_NUM = 30; 
AverageValue<float> averageValue1(MAX_VALUES_NUM);

float current1;

float getCurrent(float sensorValue){
  float current;
  current=((sensorValue/1024.0)*5.0-2.5)/0.17;
  return current;
}

//////////////////////////////////////////current sensor protection//////////////////////


//void currentSensor_setup()
//{
//  Serial.begin(115200);
//  
//  _zeroCurrentSet=false;
//  setZeroCurrent();
//  delay(100);
//}

const int currSense_pin = A0;
bool enable_currentSafe = true;//change to true to activate
float safe_current = 3.5;


float currentSensor_loop()
{
  averageValue1.push(analogRead(currSense_pin));
  
  current1=abs(((averageValue1.average()/1024.0)*5.0-2.5)/0.17);

  return current1;
}



bool currSafe_check()
{
  float curr = currentSensor_loop();
  
  if (curr > safe_current && enable_currentSafe)
    return false;//not safe
  else 
    return true;//safe
}
