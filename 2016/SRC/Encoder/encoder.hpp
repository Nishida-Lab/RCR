#ifndef __ENCODER__
#define __ENCODER__

#include <iostream>

using namespace std;

typedef pair<int, int> Enc;

class Encoder{
public:

  Encoder();
  ~Encoder();

  void GetCnt(long &front_cnt, long &rear_cnt);

private:
  Enc rear_pin;
  Enc front_pin;
};

#endif
