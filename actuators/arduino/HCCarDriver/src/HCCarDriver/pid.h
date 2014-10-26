#ifndef __PDCONTROLLER_H
#define __PDCONTROLLER_H

class PDController {
public:
  PDController() {
    P = 0;
    D = 0;
    setValue = 0;
    lastError = 0;
    output = 0;
    clampValues = false;
  }
  
  PDController(float p, float d) {
    P = p;
    D = d;
    setValue = 0;
    lastError = 0;
    output = 0;
    clampValues = false;
  }
  
  void setDesiredValue(float value) {
    setValue = value;
  }
  
  void setRange(float minimum, float maximum) {
    minVal = minimum;
    maxVal = maximum;
    clampValues = true;
  }
  
  void update(float measurement) {
    
    float error = setValue - measurement;
    float dError = error - lastError;
    lastError = error;
    
    output += P * error + D * dError;
    if(clampValues)
      output = min(maxVal, max(minVal, output));
  }
  
  float getOutput() {
    return output;
  }
  
  void setOutput(float val) {
    output = val;
  }
  
private:
  float P, D;
  float setValue;
  float lastError;
  float output;
  float maxVal;
  float minVal;
  bool clampValues;
};

#endif //__PDCONTROLLER_H
