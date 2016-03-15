/*
 * MMA7760.h
 * Library for accelerometer_MMA7760
 *
 * Copyright (c) 2013 seeed technology inc.
 * Author        :   FrankieChu
 * Create Time   :   Jan 2013
 * Change Log    :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

//#include <Wire.h>
#include "application.h"
#include "MMA7660.h"

/*Function: Write a byte to the register of the MMA7660*/
void MMA7660::write(uint8_t _register, uint8_t _data) 
{
  Wire.begin();
  Wire.beginTransmission(MMA7660_ADDR);
  Wire.write(_register);
  Wire.write(_data);
  Wire.endTransmission();
}
/*Function: Read a byte from the regitster of the MMA7660*/
uint8_t MMA7660::read(uint8_t _register) 
{
  uint8_t data_read;
  Wire.begin();
  Wire.beginTransmission(MMA7660_ADDR);
  Wire.write(_register);
  Wire.endTransmission();
  Wire.beginTransmission(MMA7660_ADDR);
  Wire.requestFrom(MMA7660_ADDR,1);
  while(Wire.available())
  {
    data_read = Wire.read();
  }
  Wire.endTransmission();
  return data_read;
}

// populate lookup table based on the MMA7660 datasheet at http://www.farnell.com/datasheets/1670762.pdf
void MMA7660::initAccelTable() {
  int8_t i;
  float val, valZ;
  int8_t xx;
  signed char vali;

  accLookup[0].xyAngle = 0.00;
  accLookup[1].xyAngle = 2.69;
  accLookup[2].xyAngle = 5.38;
  accLookup[3].xyAngle = 8.08;
  accLookup[4].xyAngle = 10.81;
  accLookup[5].xyAngle = 13.55;
  accLookup[6].xyAngle = 16.33;
  accLookup[7].xyAngle = 19.16;
  accLookup[8].xyAngle = 20.02;
  accLookup[9].xyAngle = 24.95;
  accLookup[10].xyAngle = 27.95;
  accLookup[11].xyAngle = 31.04;
  accLookup[12].xyAngle = 34.23;
  accLookup[13].xyAngle = 37.54;
  accLookup[14].xyAngle = 41.01;
  accLookup[15].xyAngle = 44.68;
  accLookup[16].xyAngle = 48.59;
  accLookup[17].xyAngle = 52.83;
  accLookup[18].xyAngle = 57.54;
  accLookup[19].xyAngle = 62.95;
  accLookup[20].xyAngle = 69.64;
  accLookup[21].xyAngle = 79.86;

  for (i=0;i<64;i++) {
	vali = i;
    xx = ((char)(vali<<2));
	accLookup[i].g = xx / 4 / 21.00;
  }

  for (i = 0; i < 22; i++) {
    accLookup[i].zAngle = 90.00 - accLookup[i].xyAngle;
  }

  for (i = 63; i > 42; i--) {
    accLookup[i].xyAngle = 0.00 - accLookup[64-i].xyAngle;
    accLookup[i].zAngle = 0.00 - accLookup[64-i].zAngle;
  }

  for (i = 22; i < 43; i++) {
    accLookup[i].xyAngle = 255;
    accLookup[i].zAngle = 255;
  }
}

void MMA7660::init()
{
  initAccelTable();
  setMode(MMA7660_STAND_BY);
  setSampleRate(AUTO_SLEEP_32);
  setMode(MMA7660_ACTIVE);
}

void MMA7660::init(uint8_t interrupts)
{
  initAccelTable();
  setMode(MMA7660_STAND_BY);
  setSampleRate(AUTO_SLEEP_32);
  write(MMA7660_INTSU, interrupts);
  setMode(MMA7660_ACTIVE);
}
void MMA7660::setMode(uint8_t mode) {
  write(MMA7660_MODE,mode);
}
void MMA7660::setSampleRate(uint8_t rate) {
  write(MMA7660_SR,rate);
}

/*Function: Get the contents of the registers in the MMA7660*/
/*          so as to calculate the acceleration.            */
void MMA7660::getXYZ(int8_t *x,int8_t *y,int8_t *z)
{
  signed char val[3];
  int8_t xx, yy, zz;
  int count;
  bool error;

  do {
    error = false;
    count = 0;
    while(Wire.available() > 0)
      Wire.read();
    Wire.requestFrom(MMA7660_ADDR,3);
    while(Wire.available()) {
      if(count < 3) {
        val[count] = Wire.read();        
        if (0x40 & val[count] == 0x40) { // alert bit is set, data is garbage and we have to start over.
          error = true;
          break;
        }
      }
      count++;
    }
  } while (error);
  xx = ((char)(val[0]<<2));
  yy = ((char)(val[1]<<2));
  zz = ((char)(val[2]<<2));
  *x = xx / 4;
  *y = yy / 4;
  *z = zz / 4;
}

void MMA7660::getAcceleration(float *ax,float *ay,float *az)
{
    int8_t x,y,z;
    getXYZ(&x,&y,&z);
    *ax = x/21.00;
    *ay = y/21.00;
    *az = z/21.00;
}

void MMA7660::getAcceleration(MMA7660_ACC_DATA *data) {
  unsigned char val[3];
  int count;
  bool error;

  do {
    error = false;
    count = 0;

    while(Wire.available() > 0) {
      Wire.read();
    }

    Wire.requestFrom(MMA7660_ADDR, 3);
    while(Wire.available()) {
      if (count < 3) {
        val[count] = Wire.read();
        if (0x40 & val[count] == 0x40) { // alert bit is set, data is garbage and we have to start over.
          error = true;
          break;
        }
      }
      count++;
    }
  } while (error);

  (*data).x = accLookup[val[0]];
  (*data).y = accLookup[val[1]];
  (*data).z = accLookup[val[2]];
}

void MMA7660::getAllData(MMA7660_DATA *data) {
  int count = 0;
  uint8_t val[11] = {0};

  while (Wire.available() > 0) {
    Wire.read();
  }

  Wire.requestFrom(MMA7660_ADDR, 11);
  while (Wire.available()) {
    if (count < 11) {
      val[count] = Wire.read();
    }
    count++;
  }

  data->X = val[0];
  data->Y = val[1];
  data->Z = val[2];
  data->TILT = val[3];
  data->SRST = val[4];
  data->SPCNT = val[5];
  data->INTSU = val[6];
  data->MODE = val[7];
  data->SR = val[8];
  data->PDET = val[9];
  data->PD = val[10];
}