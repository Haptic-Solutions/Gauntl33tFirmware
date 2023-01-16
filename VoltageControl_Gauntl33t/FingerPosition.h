#ifndef FINGERPOSITION_H
#define FINGERPOSITION_H
namespace Gauntl33t
{
  namespace MATH
  {
  //const float PI = 3.1415926535897932384626433832795;
  const float PI2 = PI * 2.0;
  }

  class FingerPosition
  {
    private:
    float mMaxAngle_Cumulative;
    float mMinAngle_Cumulative;
    float mCurAngle_Cumulative;
    float mPrevAngle_Cumulative;
    float mPrevAngle;
    public:
    //Initialize finger position with starting sensor angle.
    FingerPosition(float aStartAngle);
    //Call every frame with new angle so that the position is acturate.
    void SensorUpdatePos(float aAngle);
    //return normalized value between 0.0 and 1.0 of finger position.
    float GetFingerPosition();
  };
}
#endif