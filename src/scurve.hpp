#pragma once
#include <cstdio>
#include <iostream>
#include <vector>
#include <cmath>
#include <complex>
using namespace std::complex_literals;

//S curve generator
//This would have been way, waaaay easier to do discretely.
//But I felt like making a continuous s curve.

struct Slice {
  double endTime;
  double yTranslationNext;
  double defIntegral;
};

struct InfiniteSCurve {
  double velLimit;
  double accLimit;
  double jrkLimit;
  bool hasAnAccLimiter = true;
  std::vector<Slice> slices;

  double calcFwdJerkLimit(double time) {
    return jrkLimit * time * time;
  }

  double calcRevJerkLimit(double time) {
    if(hasAnAccLimiter) {
      time -= slices[1].endTime;
    } else {
      time -= slices[0].endTime;
    }
    time -= slices[0].endTime;
    return velLimit - jrkLimit * time * time;
  }

  double calcAccLimit(double time) {
    time -= slices[0].endTime;
    time *= accLimit;
    time += slices[0].yTranslationNext;
    return time;
  }

  InfiniteSCurve() {}

  InfiniteSCurve(double v, double a, double j):
  velLimit(v), accLimit(a), jrkLimit(j) {
    printf("Attempting to define end of first Jerk Limiter.\n");
    Slice fwdJerkSlice;
    fwdJerkSlice.endTime = (a/(2*j));
    fwdJerkSlice.yTranslationNext = calcFwdJerkLimit(fwdJerkSlice.endTime);
    fwdJerkSlice.defIntegral = calcFwdJerkLimit(fwdJerkSlice.endTime) * fwdJerkSlice.endTime / 3.0;
    printf("OK, fwdJerkSlice placed at %f,%f, with a definite integral of %f.\n",
      fwdJerkSlice.endTime,
      fwdJerkSlice.yTranslationNext,
      fwdJerkSlice.defIntegral
    );
    if(fwdJerkSlice.yTranslationNext > (v/2.0) || fwdJerkSlice.yTranslationNext < 0) {
      printf("An Acceleration Limiter is not needed; maxing the jerk never exceeds the accel limit. Curve will have no linear component.\n");
      hasAnAccLimiter = false;
      //Overwrite the endTime
      fwdJerkSlice.endTime = sqrt(v/(2.0*j));
      fwdJerkSlice.defIntegral = calcFwdJerkLimit(fwdJerkSlice.endTime) * fwdJerkSlice.endTime / 3.0;
      Slice revJerkSlice;
      revJerkSlice.endTime = 2.0 * fwdJerkSlice.endTime;
      revJerkSlice.defIntegral = v * fwdJerkSlice.endTime - fwdJerkSlice.defIntegral;
      printf("OK, revJerkSlice placed at %f,<const v>, with a definite integral of %f.\n",
        revJerkSlice.endTime,
        revJerkSlice.defIntegral
      );
      slices.push_back(fwdJerkSlice);
      slices.push_back(revJerkSlice);
    } else {
      printf("Attempting to place an Acceleration Limiter.\n");
      Slice accSlice;
      accSlice.endTime = v/a;
      //The definite integral of a portion of a line is the area of a quadrilateral.
      //(side_1 + side_2) / (2 * dt)
      slices.push_back(fwdJerkSlice);
      accSlice.defIntegral = (accSlice.endTime - fwdJerkSlice.endTime) * (calcAccLimit(fwdJerkSlice.endTime) + calcAccLimit(accSlice.endTime)) / 2.0;
      printf("OK, accSlice placed at %f,<not used>, with a definite integral of %f.\n",
        accSlice.endTime,
        accSlice.defIntegral
      );
      printf("Attempting to place a Reverse Jerk Limiter.\n");
      Slice revJerkSlice;
      revJerkSlice.endTime = v/a + fwdJerkSlice.endTime;
      revJerkSlice.defIntegral = v * fwdJerkSlice.endTime - fwdJerkSlice.defIntegral;
      //revJerkSlice.defIntegral = (fwdJerkSlice.yTranslationNext * fwdJerkSlice.endTime - fwdJerkSlice.defIntegral) + (v - fwdJerkSlice.yTranslationNext) * fwdJerkSlice.endTime;
      printf("OK, revJerkSlice placed at %f,<const v>, with a definite integral of %f.\n",
        revJerkSlice.endTime,
        revJerkSlice.defIntegral
      );
      slices.push_back(accSlice);
      slices.push_back(revJerkSlice);
    }
  }
  double calc(double num) {
    //Find most appropriate slice number.
    int slice = 0;
    for(auto& sliceObj: slices) {
      if(num > sliceObj.endTime) {
        slice++;
      } else break;
    }
    if(slice == 0) {
      return calcFwdJerkLimit(num);
    }
    if(!hasAnAccLimiter) {
      slice++;
    }
    if(slice == 1) {
      return calcAccLimit(num);
    }
    if(slice == 2) {
      return calcRevJerkLimit(num);
    }
    //if(slice == 3) {
    return velLimit;
    //}
  }

  double calcTimeForPos(double pos) {
    //Find most appropriate slice number.
    int slice = 0;
    double c = 0;
    double tStart = 0;
    for(auto& sliceObj: slices) {
      if(pos > (c + sliceObj.defIntegral)) {
        c += sliceObj.defIntegral;
        tStart = sliceObj.endTime;
        slice++;
      } else break;
    }
    //This lets us assume that c = 0 below.
    pos -= c;
    auto v = velLimit;
    auto a = accLimit;
    auto j = jrkLimit;
    if(slice == 0) {
      //Knowing that this is jx^2, the integral is
      //jx^3/3. x = jy^3/3, 3x = jy^3, 3x/j = y^3,
      //y = cube_root(3x/j)
      return std::pow(3 * pos / jrkLimit, 1.0/3);
    }
    if(!hasAnAccLimiter) {
      slice++;
    }
    if(slice == 1) {
      //Knowing that this is ax  + s[0].y,
      //the integral is (ax^2)/2 + (s[0].y)x.
      //The coeffcient of the x^1 term will be stored in z.
      auto z = slices[0].yTranslationNext;
      //The integral is now (ax^2)/2 + zx.
      //In our domain, we're using the right side of this
      //parabola, starting with the second root.
      //x = (ay^2)/2 + zy
      //Cheating with wolframalpha, y is isolated.
      //y = (sqrt(2 a x + z^2) - z)/a and a!=0
      //(There is also a second solution, but it is extraneous to our domain.)
      return (sqrt(2*a*pos + z*z) - z)/a + tStart;
    }
    if(slice == 2) {
      //Solutions:
      //b is riiii below, to save space
      //(y == z + (3 2^(1/3) v)/b + b/(3 2^(1/3) j) && j != 0) (Outside domain)
      //(y == z - (3 (1 + I Sqrt[3]) v)/(2^(2/3) b) - ((1 - I Sqrt[3]) b)/(6 2^(1/3) j) && j != 0) (Outside domain)
      //(y == z - (3 (1 - I Sqrt[3]) v)/(2^(2/3) b) - ((1 + I Sqrt[3]) b)/(6 2^(1/3) j) && j != 0) <-- This one
      //(y == x/v && j == 0 && v != 0) J shouldn't be 0.
      auto p = std::complex(pos, 0.0);
      auto ai = std::complex(0.0, 1.0);
      auto z = std::complex(slices[0].endTime, 0.0);
      auto riiii = -27*j*j*j*z*z*z + 81*j*j*v*z - 81*j*j*p;
      auto sqroot = std::sqrt(riiii * riiii - 2916*j*j*j*v*v*v);
      auto qroot = std::pow(riiii + sqroot, 1/3.0);
      auto qr2 = std::pow(2, 1/3.0);
      auto dqr2 = std::pow(2, 2/3.0);
      auto sol = z - (3 * v * (1.0 - std::sqrt(3)*ai))/(dqr2 * qroot) - ((1.0 + std::sqrt(3)*ai) * qroot)/(6*qr2 * j);
      return sol.real() + tStart;
    }
    //if(slice == 3) {
    //This case easy af. Just divide by v.
    return pos / v + tStart;
    //}
  }

  double calcPosForTime(double time) {
    //Still integrals, but no inverses (yay).
    //Find most appropriate slice number.
    int slice = 0;
    double c = 0;
    double tStart = 0;
    for(auto& sliceObj: slices) {
      if(time > sliceObj.endTime) {
        c += sliceObj.defIntegral;
        tStart = sliceObj.endTime;
        slice++;
      } else break;
    }
    time -= tStart;
    auto v = velLimit;
    auto a = accLimit;
    auto j = jrkLimit;
    if(slice == 0) {
      //Knowing that this is jx^2, the integral is
      //jx^3/3 + c.
      return jrkLimit * time*time*time / 3.0;
    }
    if(!hasAnAccLimiter) {
      slice++;
    }
    if(slice == 1) {
      //Knowing that this is ax  + s[0].y,
      //the integral is (ax^2)/2 + (s[0].y)x.
      //The coeffcient of the x^1 term will be stored in z.
      auto z = slices[0].yTranslationNext;
      //The integral is now (ax^2)/2 + zx.
      return a * time*time / 2.0 + z * time + c;
    }
    if(slice == 2) {
      //This is v - jx^2. However, pos = 0 isn't x = 0.
      //It's whatever s[n-2].endTime - s[n-1].endTime is.
      //However, it's a bad idea to translate the function.
      //We'll get a function that's impossible to integrate.
      //Instead, the bounds for indefinite integration will be translated.
      //The integral of v - jx^2 is vx - (jx^3)/3.
      //Evaluate for slice[0].endTime, then
      // evaluate for time.
      //Subtract the second from the first, add c, return.
      auto beginLoc = slices[0].endTime;
      auto timeLoc = slices[0].endTime - time;
      auto timeLocInt  = v *  timeLoc - (j *  timeLoc* timeLoc* timeLoc)/3.0;
      auto beginLocInt = v * beginLoc - (j * beginLoc*beginLoc*beginLoc)/3.0;
      //x = v * z - (j*z^3)/3 - v * (z - y) + (j*(z - y)^3)/3
      return beginLocInt - timeLocInt + c;
    }
    //This case easy af. Just multiply by v.
    return time * v + c;
  }

  double calcAccForTime(double time) {
    //Finally we get to do some derivatives, the easy stuff.
    //Find most appropriate slice number.
    int slice = 0;
    for(auto& sliceObj: slices) {
      if(time > sliceObj.endTime) {
        slice++;
      } else break;
    }
    if(slice == 0) {
      //jx^2, d/dx gives 2jx.
      return 2 * jrkLimit * time;
    }
    if(!hasAnAccLimiter) {
      slice++;
    }
    if(slice == 1) {
      //constant slope of a
      return accLimit;
    }
    if(slice == 2) {
      //subtract endTime of this slice.
      time -= slices[slices.size() - 1].endTime;
      //Time will be negative. Return -2jx.
      return -2 * jrkLimit * time;
    }
    //if(slice == 3) {
    //constant slope of 0
    return 0;
    //}
  }

  double minPos() {
    double sum = 0;
    for(auto& slice: slices) {
      sum += slice.defIntegral;
    }
    return sum;
  }
  double minHalfWidth() {
    return slices[slices.size() - 1].endTime;
  }
};

class SCurve {
  public:
  InfiniteSCurve underlying;
  const double velLimit;
  const double accLimit;
  const double jrkLimit;
  const double distance;
  double tWidth;
  SCurve(double v, double a, double j, double d):
  velLimit(v), accLimit(a), jrkLimit(j), distance(d) {
    printf("Trying a curve as-given...\n");
    underlying = InfiniteSCurve(v, a, j);
    if(underlying.minPos() * 2 > d) {
      printf("That didn't work, trying an acc-limiter-cut.\n");
      double newVelLimit = (j * sqrt((a * (a*a*a + 16*d*j*j))/(j*j)) - (a*a))/(4 * j);
      printf("velLimit from %f to %f\n", v, newVelLimit);
      underlying = InfiniteSCurve(newVelLimit, a, j);
      if(!underlying.hasAnAccLimiter) {
        newVelLimit = std::pow((d/2.0)*sqrt(2*j),2.0/3.0);
        printf("velLimit from %f to %f\n", v, newVelLimit);
        underlying = InfiniteSCurve(newVelLimit, a, j);
      }
      printf("Should be 0: %f\n", (underlying.minPos() * 2) - d);
      printf("Should be 1: %f\n", (underlying.minPos() * 2) / d);
      tWidth = underlying.minHalfWidth() * 2;
    } else {
      printf("That worked.\n");
      tWidth = underlying.minHalfWidth() * 2 + (d - underlying.minPos() * 2)/v;
      printf("Distance = %f, tWidth = %f\n", d, tWidth);
    }
  }

  double calc(double num) {
    if(num < 0 || num > tWidth) return 0;
    if(num > tWidth / 2) return underlying.calc(tWidth - num);
    return underlying.calc(num);
  }

  double calcPosForTime(double num) {
    if(num < 0) return 0;
    if(num > tWidth) return distance;
    if(num > tWidth / 2) return distance - underlying.calcPosForTime(tWidth - num);
    return underlying.calcPosForTime(num);
  }

  double calcAccForTime(double num) {
    if(num < 0 || num > tWidth) return 0;
    if(num > tWidth / 2) return -underlying.calcAccForTime(tWidth - num);
    return underlying.calcAccForTime(num);
  }

  double calcTimeForPos(double pos) {
    if(pos < 0) return 0;
    if(pos > distance) return tWidth;
    if(pos > distance / 2) return tWidth - underlying.calcTimeForPos(distance - pos);
    return underlying.calcTimeForPos(pos);
  }

  double timingWidth() {
    return tWidth;
  }
};
